# -*- coding: utf-8 -*-
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import re
import sys
from ament_index_python.packages import get_package_share_directory

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

try:
    import ikfastpy
except ImportError:
    print("Error: No se pudo importar la librería 'ikfastpy'.")
    sys.exit(1)


class RobotDataPublisher(Node):
    def __init__(self):
        super().__init__('robot_data_publisher')

        self.start_time = False
        self.ur3_solutions = []
        self.endo_joints = []

        # --- Publicadores ---
        self.ur3_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.ur3_msg = JointState()
        self.ur3_msg.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        self.endo_pub = self.create_publisher(JointState, '/endowrist', 10)
        self.endo_msg = JointState()
        self.endo_msg.header.frame_id = 'base_link'
        self.endo_msg.name = ['shaft', 'wrist', 'jaw_dx', 'jaw_sx']

        try:
            self.get_logger().info('Cargando datos desde archivo de sutura...')

            package_share_dir = get_package_share_directory('code')
            suture_path = os.path.join(package_share_dir, 'puntos_sutura.txt')

            # 1) Cargar puntos de sutura (en metros)
            self.suture_points = self.load_suture_points(suture_path)
            if not self.suture_points:
                raise ValueError("No se cargaron puntos válidos desde el archivo de sutura.")

            # 2) Ajustar al marco del robot en Unity
            robot_pos_unity = np.array([-0.55, 0.188, -0.465])  # [m]
            self.suture_points = [tuple(np.array(p) - robot_pos_unity) for p in self.suture_points]

            # 3) Configuración inicial del robot (sliders en grados)
            initial_joint_deg = np.array([-45, 10, 51, 0, 90, 0])
            initial_joint_rad = np.deg2rad(initial_joint_deg)

            # Publicar esta posición inicial para colocar el robot antes de la sutura
            self.publish_initial_position(initial_joint_rad)

            # 4) Calcular orientación del efector desde FK de la configuración inicial
            ur_kin = ikfastpy.PyKinematics()
            fk_list = ur_kin.forward(initial_joint_rad.tolist())  # 12 valores (3x4)
            fk_3x4 = np.array(fk_list).reshape(3, 4)
            R_from_initial_pose = fk_3x4[:, :3]  # Rotación 3x3

            # 5) Orientación alternativa que ya dio soluciones antes
            R_alt = np.array([[1, 0, 0],
                              [0, 0, -1],
                              [0, 1, 0]])

            self.orientations = [R_from_initial_pose, R_alt]

            # 6) Endowrist fijo
            self.endo_joints = np.zeros((len(self.suture_points), 7))

            # 7) IK con referencia en la orientación calculada desde la pose inicial
            self.get_logger().info('Calculando IK con orientación de referencia inicial...')
            self.ur3_solutions = self.calculate_ur3_ik_multi_orient(self.suture_points)

            if not self.ur3_solutions:
                self.get_logger().error("No se encontró ninguna solución IK válida. Deteniendo nodo.")
                self.destroy_node()
                rclpy.shutdown()
                return

        except (FileNotFoundError, ValueError) as e:
            self.get_logger().error(f"Error: {str(e)}")
            self.destroy_node()
            rclpy.shutdown()
            return

        self.num_points = min(len(self.ur3_solutions), len(self.endo_joints))
        self.publish_index = 0
        self.timer = self.create_timer(0.5, self.publish_callback)
        self.start_time = True


    def publish_initial_position(self, joint_values_rad):
        """Coloca el robot en la pose inicial antes de la sutura."""
        self.ur3_msg.header.stamp = self.get_clock().now().to_msg()
        self.ur3_msg.position = joint_values_rad.tolist()
        self.ur3_pub.publish(self.ur3_msg)
        self.get_logger().info(f"📍 Robot posicionado en referencia inicial: {np.rad2deg(joint_values_rad)}°")


    def load_suture_points(self, filename):
        """Lee coordenadas de sutura desde un archivo y genera posiciones XYZ interpoladas."""
        points = []
        with open(filename, 'r') as f:
            lines = f.readlines()

        entrada = None
        salida = None

        for line in lines:
            if "Entrada" in line:
                entrada = tuple(map(float, re.findall(r"[-\d.]+", line)))
            elif "Salida" in line:
                salida = tuple(map(float, re.findall(r"[-\d.]+", line)))

                if entrada and salida:
                    num_steps = 5
                    for t in np.linspace(0, 1, num_steps):
                        x = entrada[0] + t * (salida[0] - entrada[0])
                        y = entrada[1] + t * (salida[1] - entrada[1])
                        z = entrada[2] + t * (salida[2] - entrada[2])
                        points.append((x, y, z))
                    entrada, salida = None, None
        return points


    def calculate_ur3_ik_multi_orient(self, points):
        """Intenta varias orientaciones para cada punto."""
        ur_kin = ikfastpy.PyKinematics()
        n_joints = ur_kin.getDOF()
        solutions = []
        last_solution = np.zeros(n_joints)

        for i, (x, y, z) in enumerate(points):
            found_solution = False

            for idx, Rm in enumerate(self.orientations):
                pose_matrix = np.eye(4)
                pose_matrix[:3, :3] = Rm
                pose_matrix[0, 3] = x
                pose_matrix[1, 3] = y
                pose_matrix[2, 3] = z

                pose_3x4_flat = pose_matrix[:3, :].flatten().tolist()
                joint_configs = ur_kin.inverse(pose_3x4_flat)

                if joint_configs:
                    n_solutions = len(joint_configs) // n_joints
                    joint_configs = np.array(joint_configs).reshape(n_solutions, n_joints)
                    best_solution = min(joint_configs, key=lambda cfg: np.linalg.norm(cfg - last_solution))
                    solutions.append(best_solution)
                    last_solution = best_solution
                    found_solution = True
                    self.get_logger().info(f"✅ Punto {i+1}: orientación usada = {'REF-INICIAL' if idx==0 else 'ALT-4'}")
                    break

            if not found_solution:
                self.get_logger().warn(f"❌ Punto {i+1}: sin solución en ninguna orientación.")

        return solutions


    def publish_callback(self):
        if self.publish_index >= self.num_points:
            self.get_logger().info('Secuencia terminada. Reiniciando...')
            self.publish_index = 0

        if not self.ur3_solutions:
            self.get_logger().error("No hay soluciones IK válidas, deteniendo nodo.")
            self.destroy_node()
            rclpy.shutdown()
            return

        current_ur3_q = self.ur3_solutions[self.publish_index]
        self.ur3_msg.header.stamp = self.get_clock().now().to_msg()
        self.ur3_msg.position = current_ur3_q.tolist()
        self.ur3_pub.publish(self.ur3_msg)

        current_endo_q = self.endo_joints[self.publish_index]
        shaft_pos = current_endo_q[2]
        wrist_pos = current_endo_q[4]
        jaw_pos = current_endo_q[6]

        self.endo_msg.header.stamp = self.get_clock().now().to_msg()
        self.endo_msg.position = [shaft_pos, wrist_pos, jaw_pos, -jaw_pos]
        self.endo_pub.publish(self.endo_msg)

        self.publish_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = RobotDataPublisher()
    if node.start_time:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
