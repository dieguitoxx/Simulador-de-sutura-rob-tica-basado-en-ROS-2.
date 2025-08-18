# -*- coding: utf-8 -*-
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import sys
import re
import io

# Agrega el path del directorio actual para que pueda encontrar ikfastpy
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

# Verificar si ikfastpy está disponible
IKFAST_AVAILABLE = True
try:
    import ikfastpy
except ImportError:
    IKFAST_AVAILABLE = False

class RobotDataPublisher(Node):
    """
    Este nodo carga datos de posición para un robot UR3 y un endoscopio desde variables internas,
    calcula la cinemática inversa para el UR3 y publica los datos disponibles.
    El nodo es resiliente a fallos: si los datos del UR3 fallan, seguirá publicando los del Endowrist y viceversa.
    """
    def __init__(self):
        super().__init__('robot_data_publisher')
        
        # --- Datos integrados en el código ---
        self.joint_positions_data = """
2.17159265, 0.13000000, 0.15000000, 1.61159265, 1.15000000, 0.46000000, 0.00000000
2.16706425, 0.13087108, 0.15001714, 1.61646035, 1.15062609, 0.45907458, 0.00000000
2.16270361, 0.13175363, 0.15003463, 1.62116474, 1.15123875, 0.45813921, 0.00000000
2.15850950, 0.13264719, 0.15005246, 1.62570698, 1.15183785, 0.45719412, 0.00000000
2.15448056, 0.13355131, 0.15007062, 1.63008836, 1.15242324, 0.45623955, 0.00000000
2.15061531, 0.13446553, 0.15008912, 1.63431028, 1.15299479, 0.45527573, 0.00000000
2.14691219, 0.13538939, 0.15010794, 1.63837422, 1.15355238, 0.45430291, 0.00000000
2.14336952, 0.13632247, 0.15012709, 1.64228178, 1.15409585, 0.45332132, 0.00000000
2.13998556, 0.13726431, 0.15014656, 1.64603463, 1.15462511, 0.45233122, 0.00000000
        """
        self.pfr_poses_data = "..." # (Datos de poses omitidos por brevedad, son los mismos)
        # ... (el resto de los datos de las poses van aquí) ...
        
        # --- Estados de preparación independientes ---
        self.is_ready = False       # Estado general del nodo
        self.ur3_ready = False      # Estado de los datos del UR3
        self.endo_ready = False     # Estado de los datos del Endowrist
        
        # --- Publicadores y Mensajes ---
        self.ur3_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.ur3_msg = JointState()
        self.ur3_msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.endo_pub = self.create_publisher(JointState, '/endowrist', 10)
        self.endo_msg = JointState()
        self.endo_msg.header.frame_id = 'base_link'
        self.endo_msg.name = ['shaft', 'wrist', 'jaw_dx', 'jaw_sx']

        # --- Inicialización de variables ---
        self.pfr_poses = []
        self.endo_joints = []
        self.ur3_solutions = []
        self.publish_index = 0
        self.num_points = 0

        # Intentar inicializar el nodo
        if self.initialize_node():
            self.is_ready = True
            self.timer = self.create_timer(1.0, self.publish_callback)
            self.get_logger().info('Nodo inicializado. Comenzando publicación de datos disponibles.')
        else:
            self.get_logger().fatal('Falló la inicialización de TODOS los publicadores. El nodo no puede funcionar.')

    def initialize_node(self):
        """Inicializa cada componente (UR3, Endowrist) de forma independiente."""
        self.initialize_endo_data()
        self.initialize_ur3_data()
        
        # El nodo está listo si al menos uno de los dos subsistemas lo está.
        if not self.ur3_ready and not self.endo_ready:
            self.get_logger().error("No se pudieron cargar los datos ni para el UR3 ni para el Endowrist.")
            return False
        
        # El número total de puntos será el máximo de los dos para que el bucle no termine antes de tiempo.
        self.num_points = max(len(self.ur3_solutions), len(self.endo_joints))
        self.get_logger().info(f'Secuencia de publicación tendrá {self.num_points} pasos.')
        return True

    def initialize_endo_data(self):
        """Carga y prepara los datos del Endowrist."""
        try:
            self.get_logger().info("Intentando cargar datos del Endowrist...")
            self.endo_joints = np.loadtxt(io.StringIO(self.joint_positions_data.strip()), comments='#', delimiter=',')
            self.get_logger().info(f'Carga exitosa: {len(self.endo_joints)} puntos para Endowrist.')
            self.endo_ready = True
        except Exception as e:
            self.get_logger().error(f'FALLO al cargar datos del Endowrist: {e}')
            self.endo_ready = False

    def initialize_ur3_data(self):
        """Carga, calcula IK y prepara los datos del UR3."""
        try:
            self.get_logger().info("Intentando cargar datos del UR3...")
            # Cargar Poses
            self.pfr_poses = self.load_pfr_poses_from_string(self.pfr_poses_data)
            if not self.pfr_poses:
                raise ValueError("No se pudieron parsear las matrices de pose desde la cadena de texto.")
            self.get_logger().info(f"Se cargaron {len(self.pfr_poses)} poses para el UR3.")

            # Calcular Cinemática Inversa
            if not IKFAST_AVAILABLE:
                self.get_logger().warn("Librería 'ikfastpy' no disponible. No se publicarán datos del UR3.")
                self.ur3_ready = False
                return

            self.get_logger().info("Calculando cinemática inversa para las poses del UR3...")
            self.ur3_solutions = self.calculate_ur3_ik(self.pfr_poses)

            if not self.ur3_solutions:
                self.get_logger().warn("No se encontraron soluciones de IK para ninguna pose. No se publicarán datos del UR3.")
                self.ur3_ready = False
                return

            self.get_logger().info(f"Cálculo de IK exitoso: {len(self.ur3_solutions)} soluciones encontradas para UR3.")
            self.ur3_ready = True

        except Exception as e:
            self.get_logger().error(f'FALLO durante la inicialización del UR3: {e}')
            self.ur3_ready = False

    def load_pfr_poses_from_string(self, content):
        """Carga poses desde una cadena de texto."""
        # ... (esta función no cambia)
        poses = []
        pose_blocks = re.findall(r'--- Pose \d+ ---\n(.*?)(?=\n--- Pose|\Z)', content, re.S)
        for block in pose_blocks:
            lines = block.strip().split('\n')
            matrix_data = [list(map(float, line.split())) for line in lines]
            if len(matrix_data) == 4 and all(len(row) == 4 for row in matrix_data):
                poses.append(np.array(matrix_data))
        return poses

    def calculate_ur3_ik(self, poses):
        """Calcula la cinemática inversa para una lista de poses del efector final."""
        # ... (esta función no cambia)
        try:
            ur_kin = ikfastpy.PyKinematics()
            n_joints = ur_kin.getDOF()
        except Exception as e:
            self.get_logger().error(f"Error al inicializar ikfastpy: {e}")
            return []
        
        solutions = []
        last_solution = np.zeros(n_joints)
        for i, pose in enumerate(poses):
            pose_3x4_flat = pose[:3, :].flatten().tolist()
            joint_configs = ur_kin.inverse(pose_3x4_flat)
            if not joint_configs:
                self.get_logger().warn(f"No se encontró solución de IK para la pose {i+1}.")
                continue
            n_solutions = len(joint_configs) // n_joints
            joint_configs = np.array(joint_configs).reshape(n_solutions, n_joints)
            best_solution = min(joint_configs, key=lambda x: np.linalg.norm(x - last_solution))
            solutions.append(best_solution)
            last_solution = best_solution
        return solutions

    def publish_callback(self):
        """Publica los datos disponibles para cada subsistema."""
        if not self.is_ready:
            return
            
        if self.publish_index >= self.num_points:
            self.get_logger().info('Se han publicado todos los puntos. Reiniciando secuencia.')
            self.publish_index = 0

        now = self.get_clock().now().to_msg()
        
        # --- Publicar estado del UR3 (si está listo y hay datos para este índice) ---
        if self.ur3_ready and self.publish_index < len(self.ur3_solutions):
            current_ur3_q = self.ur3_solutions[self.publish_index]
            self.ur3_msg.header.stamp = now
            self.ur3_msg.position = current_ur3_q.tolist()
            self.ur3_pub.publish(self.ur3_msg)
            self.get_logger().info(f'Publicando en /joint_states: {np.round(current_ur3_q, 3)}')
        
        # --- Publicar estado del Endowrist (si está listo y hay datos para este índice) ---
        if self.endo_ready and self.publish_index < len(self.endo_joints):
            current_endo_q = self.endo_joints[self.publish_index]
            if len(current_endo_q) >= 7:
                shaft_pos = current_endo_q[2]
                wrist_pos = current_endo_q[4]
                jaw_pos = current_endo_q[6]
                self.endo_msg.header.stamp = now
                self.endo_msg.position = [shaft_pos, wrist_pos, jaw_pos, -jaw_pos]
                self.endo_pub.publish(self.endo_msg)
                self.get_logger().info(f'Publicando en /endowrist: {[round(p, 3) for p in self.endo_msg.position]}')
            else:
                self.get_logger().warn(f'Fila {self.publish_index} de Endowrist con datos insuficientes, omitiendo publicación.')

        self.publish_index += 1

def main(args=None):
    rclpy.init(args=args)
    robot_data_publisher = None
    try:
        robot_data_publisher = RobotDataPublisher()
        if robot_data_publisher.is_ready:
            rclpy.spin(robot_data_publisher)
    except KeyboardInterrupt:
        if robot_data_publisher:
            robot_data_publisher.get_logger().info("Interrupción por teclado recibida. Cerrando nodo...")
    except Exception as e:
        print(f"Error fatal al crear o ejecutar el nodo: {e}")
    finally:
        if robot_data_publisher:
            robot_data_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()