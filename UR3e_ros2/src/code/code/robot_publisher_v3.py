# -*- coding: utf-8 -*-
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import re
import sys
from ament_index_python.packages import get_package_share_directory

# Agrega el path del directorio actual para que pueda encontrar ikfastpy
# Se asume que el solucionador ikfast para el UR3/UR5 está en este directorio.
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

try:
    import ikfastpy
except ImportError:
    print("Error: No se pudo importar la librería 'ikfastpy'.")
    print("Asegúrate de que esté instalada y de que el archivo del solucionador compilado")
    print("se encuentre en el mismo directorio que este script.")
    sys.exit(1)

from ament_index_python.packages import get_package_share_directory

class RobotDataPublisher(Node):
    def __init__(self):
        super().__init__('robot_data_publisher')

        self.start_time = False
        self.ur3_solutions = []
        self.endo_joints = []

        # --- Publicadores y mensajes ---
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
            self.get_logger().info('Cargando datos desde los archivos...')

            package_share_dir = get_package_share_directory('code')
            pfr_path = os.path.join(package_share_dir, 'pfr_poses_4x4.txt')
            joint_path = os.path.join(package_share_dir, 'joint_positions.txt')

            self.pfr_poses = self.load_pfr_poses(pfr_path)
            self.endo_joints = self.load_joint_positions(joint_path)

            if not self.pfr_poses:
                raise ValueError("No se cargaron poses válidas desde el archivo de poses.")
            if len(self.endo_joints) == 0:
                raise ValueError("No se cargaron posiciones válidas desde el archivo de articulaciones.")

            self.get_logger().info(f'Se cargaron {len(self.pfr_poses)} poses del UR3.')
            self.get_logger().info(f'Se cargaron {len(self.endo_joints)} posiciones del Endowrist.')

            # --- Matriz de desplazamiento (offset/calibración) ---
            displacement_mm = np.array([
                [1, 0, 0, 279.360498],
                [0, 1, 0, -308.113605],
                [0, 0, 1, 322.060891],
                [0, 0, 0, 1.0]
            ])
            self.displacement_matrix_m = displacement_mm.copy()
            self.displacement_matrix_m[:3, 3] /= 1000.0
            self.get_logger().info(f"Matriz de desplazamiento (m):\n{self.displacement_matrix_m}")

            # --- Aplicar transformación a cada pose ---
            transformed_poses = []
            for pose in self.pfr_poses:
                new_pose = self.displacement_matrix_m @ pose
                transformed_poses.append(new_pose)
            self.pfr_poses = transformed_poses
            self.get_logger().info(f'Desplazamiento aplicado a {len(self.pfr_poses)} poses.')

            # --- Cálculo IK ---
            self.get_logger().info('Calculando cinemática inversa para las poses del UR3...')
            self.ur3_solutions = self.calculate_ur3_ik(self.pfr_poses)
            self.get_logger().info('Cálculo de IK completado.')

        except (FileNotFoundError, ValueError) as e:
            self.get_logger().error(f"Error durante la carga de datos: {str(e)}")
            self.destroy_node()
            rclpy.shutdown()
            return

        if not self.ur3_solutions or not self.endo_joints.any():
            self.get_logger().warn('No se cargaron datos válidos. Apagando nodo.')
            self.destroy_node()
            rclpy.shutdown()
            return

        self.num_points = min(len(self.ur3_solutions), len(self.endo_joints))
        self.get_logger().info(f'Se publicarán {self.num_points} puntos de datos sincronizados.')

        self.publish_index = 0
        self.timer = self.create_timer(0.05, self.publish_callback)
        self.start_time = True


    def load_pfr_poses(self, filename):
        """Carga una lista de matrices de transformación 4x4 desde un archivo de texto."""
        with open(filename, 'r') as f:
            content = f.read()
        
        # Usar una expresión regular para encontrar bloques de 4x4 números
        pose_blocks = re.findall(r'--- Pose \d+ ---\n(.*?)(?=\n--- Pose|\Z)', content, re.S)
        
        poses = []
        for block in pose_blocks:
            # Limpiar el bloque y convertirlo en una matriz numpy
            lines = block.strip().split('\n')
            matrix_data = [list(map(float, line.split())) for line in lines]
            if len(matrix_data) == 4 and all(len(row) == 4 for row in matrix_data):
                poses.append(np.array(matrix_data))
        return poses

    def load_joint_positions(self, filename):
        """Carga las posiciones articulares desde un archivo de texto, ignorando el encabezado."""
        return np.loadtxt(filename, comments='#', delimiter=',')

    def calculate_ur3_ik(self, poses):
        """Calcula la cinemática inversa para una lista de poses del efector final."""
        # Inicializar cinemática del UR5 (compatible con UR3 para los 6 ejes)
        try:
            ur_kin = ikfastpy.PyKinematics()
            n_joints = ur_kin.getDOF()
        except NameError:
             self.get_logger().error("ikfastpy no está definido. La librería no se importó correctamente.")
             return []


        solutions = []
        # Posición inicial de referencia (home o la primera solución encontrada)
        last_solution = np.zeros(n_joints) 

        for i, pose in enumerate(poses):
            # Convertir la matriz de pose a la lista de 12 elementos que espera ikfast
            pose_3x4_flat = pose[:3, :].flatten().tolist()
            
            # Obtener todas las posibles soluciones de IK
            joint_configs = ur_kin.inverse(pose_3x4_flat)
            
            if not joint_configs:
                self.get_logger().warn(f"No se encontró solución de IK para la pose {i+1}.")
                continue

            # Organizar las soluciones en una matriz
            n_solutions = len(joint_configs) // n_joints
            joint_configs = np.array(joint_configs).reshape(n_solutions, n_joints)
            
            # --- Lógica para seleccionar la "mejor" solución ---
            # Se elige la solución más cercana a la configuración articular anterior
            # para asegurar un movimiento suave del robot.
            best_solution = None
            min_dist = float('inf')
            
            for config in joint_configs:
                dist = np.linalg.norm(config - last_solution)
                if dist < min_dist:
                    min_dist = dist
                    best_solution = config
            
            if best_solution is not None:
                solutions.append(best_solution)
                last_solution = best_solution # Actualizar la última solución para la siguiente iteración
            else:
                 self.get_logger().warn(f"No se pudo seleccionar una solución válida para la pose {i+1}.")


        return solutions

    def publish_callback(self):
        """
        Publica los datos de las articulaciones del UR3 y del Endowrist.
        Se ejecuta en cada tick del temporizador.
        """
        if self.publish_index >= self.num_points:
            self.get_logger().info('Se han publicado todos los puntos. Reiniciando secuencia.')
            self.publish_index = 0

        # --- UR3: aplicar offset angular ---
        offset_deg = np.array([0, -30, -30, -90, 0, 0])
        offset_rad = np.deg2rad(offset_deg)

        current_ur3_q = self.ur3_solutions[self.publish_index] + offset_rad
        self.ur3_msg.header.stamp = self.get_clock().now().to_msg()
        self.ur3_msg.position = current_ur3_q.tolist()
        self.ur3_pub.publish(self.ur3_msg)
        self.get_logger().info(f'Publicando en /joint_states: {np.round(current_ur3_q, 3)}')

        # --- Endowrist ---
        current_endo_q = self.endo_joints[self.publish_index]
        shaft_pos = current_endo_q[2]
        wrist_pos = current_endo_q[4]
        jaw_pos = current_endo_q[6]

        self.endo_msg.header.stamp = self.get_clock().now().to_msg()
        self.endo_msg.position = [shaft_pos, wrist_pos, jaw_pos, -jaw_pos]
        self.endo_pub.publish(self.endo_msg)
        self.get_logger().info(f'Publicando en /endowrist: {[round(p, 3) for p in self.endo_msg.position]}')

        self.publish_index += 1


def main(args=None):
    rclpy.init(args=args)
    
    robot_data_publisher = RobotDataPublisher()
    
    # Solo hacer spin si el nodo se inicializó correctamente
    if robot_data_publisher.start_time:
      try:
          rclpy.spin(robot_data_publisher)
      except KeyboardInterrupt:
          pass
      finally:
          # Destruir el nodo explícitamente
          robot_data_publisher.destroy_node()
          rclpy.shutdown()

if __name__ == '__main__':
    main()