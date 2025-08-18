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
    print("Advertencia: No se pudo importar la librería 'ikfastpy'.")
    print("El nodo funcionará sin cálculos de cinemática inversa.")

class RobotDataPublisher(Node):
    """
    Este nodo carga datos de posición para un robot UR3 y un endoscopio,
    aplica una transformación de desplazamiento a las poses del UR3,
    calcula la cinemática inversa y publica los datos en ROS 2.
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
       
        self.pfr_poses_data = """
--- Pose 1 ---
-0.56052947 -0.82488571 -0.07328212 0.01832053
0.81792524 -0.56529953 0.10693335 -0.02673334
-0.12963414 0.00000000 0.99156189 -0.24789047
0.00000000 0.00000000 0.00000000 1.00000000
--- Pose 2 ---
-0.55675622 -0.82743715 -0.07328214 0.01832053
0.82036140 -0.56155833 0.10797874 -0.02699469
-0.13049782 0.00000000 0.99144860 -0.24786215
0.00000000 0.00000000 0.00000000 1.00000000
--- Pose 3 ---
-0.55310916 -0.82987803 -0.07329876 0.01832469
0.82268551 -0.55794485 0.10902338 -0.02725584
-0.13137277 0.00000000 0.99133304 -0.24783326
0.00000000 0.00000000 0.00000000 1.00000000
--- Pose 4 ---
-0.54958857 -0.83221081 -0.07333199 0.01833300
0.82490005 -0.55445935 0.11006699 -0.02751675
-0.13225854 0.00000000 0.99121525 -0.24780381
0.00000000 0.00000000 0.00000000 1.00000000
--- Pose 5 ---
-0.54619452 -0.83443793 -0.07338179 0.01834545
0.82700749 -0.55110193 0.11110930 -0.02777733
-0.13315466 0.00000000 0.99109527 -0.24777382
0.00000000 0.00000000 0.00000000 1.00000000
--- Pose 6 ---
-0.54292694 -0.83656184 -0.07344816 0.01836204
0.82901030 -0.54787252 0.11215005 -0.02803751
-0.13406068 0.00000000 0.99097312 -0.24774328
0.00000000 0.00000000 0.00000000 1.00000000
--- Pose 7 ---
-0.53978560 -0.83858493 -0.07353108 0.01838277
0.83091092 -0.54477088 0.11318897 -0.02829724
-0.13497615 0.00000000 0.99084885 -0.24771221
0.00000000 0.00000000 0.00000000 1.00000000
--- Pose 8 ---
-0.53677010 -0.84050961 -0.07363050 0.01840763
0.83271176 -0.54179664 0.11422578 -0.02855645
-0.13590063 0.00000000 0.99072247 -0.24768062
0.00000000 0.00000000 0.00000000 1.00000000
--- Pose 9 ---
-0.53387995 -0.84233821 -0.07374641 0.01843660
0.83441521 -0.53894929 0.11526023 -0.02881506
-0.13683368 0.00000000 0.99059404 -0.24764851
0.00000000 0.00000000 0.00000000 1.00000000
        """

        ### SECCIÓN NUEVA: Definición de la matriz de desplazamiento ###
        # Matriz de desplazamiento (offset) definida con valores en mm.
        displacement_mm = np.array([
            [1, 0, 0, 279.360498],
            [0, 1, 0, -308.113605],
            [0, 0, 1, 322.060891],
            [0, 0, 0, 1.0]
        ])
       
        # Copiamos la matriz y convertimos su parte de traslación (última columna) a metros.
        # La cinemática inversa y ROS trabajan en metros.
        self.displacement_matrix_m = displacement_mm.copy()
        self.displacement_matrix_m[:3, 3] /= 1000.0
        self.get_logger().info(f"Matriz de desplazamiento en metros cargada:\n{self.displacement_matrix_m}")
        ### FIN DE SECCIÓN NUEVA ###

        # Inicializar estado del nodo
        self.is_ready = False
       
        # --- Publicadores y Mensajes ---
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
            self.get_logger().info('Nodo inicializado correctamente. Comenzando publicación.')
        else:
            self.get_logger().error('Falló la inicialización del nodo.')

    def initialize_node(self):
        """Inicializa el nodo cargando datos, aplicando offset y calculando IK."""
        try:
            if not self.load_data_from_variables():
                return False
               
            self.get_logger().info(f'Se cargaron {len(self.pfr_poses)} poses originales del UR3.')
            self.get_logger().info(f'Se cargaron {len(self.endo_joints)} posiciones del Endowrist.')

            ### SECCIÓN MODIFICADA: Aplicar el desplazamiento a cada pose ###
            self.get_logger().info('Aplicando matriz de desplazamiento a todas las poses...')
            transformed_poses = []
            for pose in self.pfr_poses:
                # La nueva pose es el resultado de la multiplicación de la matriz de desplazamiento
                # por la pose original. El operador '@' es para la multiplicación de matrices en NumPy.
                new_pose = self.displacement_matrix_m @ pose
                transformed_poses.append(new_pose)
           
            # Actualizamos la lista de poses con las poses ya transformadas.
            self.pfr_poses = transformed_poses
            self.get_logger().info(f'Desplazamiento aplicado. {len(self.pfr_poses)} poses actualizadas.')
            ### FIN DE SECCIÓN MODIFICADA ###

            if IKFAST_AVAILABLE and len(self.pfr_poses) > 0:
                self.get_logger().info('Calculando cinemática inversa para las poses del UR3...')
                self.ur3_solutions = self.calculate_ur3_ik(self.pfr_poses)
                self.get_logger().info('Cálculo de IK completado.')
            else:
                if not IKFAST_AVAILABLE:
                    self.get_logger().warn('ikfastpy no disponible. Usando poses dummy para UR3.')
                self.ur3_solutions = [np.zeros(6) for _ in range(len(self.pfr_poses))]

            self.num_points = min(len(self.ur3_solutions), len(self.endo_joints))
           
            if self.num_points == 0:
                self.get_logger().error('No hay datos sincronizados para publicar.')
                return False
           
            self.get_logger().info(f'Se publicarán {self.num_points} puntos de datos sincronizados.')
            return True
           
        except Exception as e:
            self.get_logger().error(f'Error durante la inicialización: {e}')
            return False

    def load_data_from_variables(self):
        """Carga datos desde las variables de texto internas."""
        try:
            self.get_logger().info('Cargando datos desde variables internas...')
           
            self.pfr_poses = self.load_pfr_poses_from_string(self.pfr_poses_data)
            self.endo_joints = np.loadtxt(io.StringIO(self.joint_positions_data.strip()), comments='#', delimiter=',')
           
            return True
           
        except Exception as e:
            self.get_logger().error(f'Error al cargar datos desde variables: {e}')
            return False

    def load_pfr_poses_from_string(self, content):
        """Carga poses desde una cadena de texto."""
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
        if not IKFAST_AVAILABLE:
            return []
           
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
           
            best_solution = None
            min_dist = float('inf')
           
            for config in joint_configs:
                dist = np.linalg.norm(config - last_solution)
                if dist < min_dist:
                    min_dist = dist
                    best_solution = config
           
            if best_solution is not None:
                solutions.append(best_solution)
                last_solution = best_solution
            else:
                self.get_logger().warn(f"No se pudo seleccionar una solución válida para la pose {i+1}.")

        return solutions

    def publish_callback(self):
        """Publica los datos de las articulaciones del UR3 y del Endowrist."""
        if not self.is_ready:
            return
           
        if self.publish_index >= self.num_points:
            self.get_logger().info('Se han publicado todos los puntos. Reiniciando secuencia.')
            self.publish_index = 0

        current_ur3_q = self.ur3_solutions[self.publish_index]
        self.ur3_msg.header.stamp = self.get_clock().now().to_msg()
        self.ur3_msg.position = current_ur3_q.tolist()
        self.ur3_pub.publish(self.ur3_msg)
        
        # Vector en grados que deseas sumar
        offset_degrees = np.array([0, -90, -40, 0, 30, 0])
        # Conversión a radianes
        offset_radians = np.deg2rad(offset_degrees)


        # Sumar los offsets
        current_ur3_q = current_ur3_q+offset_radians
        self.get_logger().info(f'Publicando en /joint_states: {np.round(current_ur3_q, 3)}')

        current_endo_q = self.endo_joints[self.publish_index]
       
        if len(current_endo_q) >= 7:
            shaft_pos = current_endo_q[2]
            wrist_pos = current_endo_q[4]
            jaw_pos = current_endo_q[6]
        else:
            shaft_pos, wrist_pos, jaw_pos = 0.0, 0.0, 0.0
            self.get_logger().warn(f'Fila {self.publish_index} tiene datos insuficientes: {len(current_endo_q)} columnas')

        self.endo_msg.header.stamp = self.get_clock().now().to_msg()
        self.endo_msg.position = [shaft_pos, wrist_pos, jaw_pos, -jaw_pos]
        self.endo_pub.publish(self.endo_msg)
        self.get_logger().info(f'Publicando en /endowrist: {[round(p, 3) for p in self.endo_msg.position]}')

        self.publish_index += 1

def main(args=None):
    rclpy.init(args=args)
   
    robot_data_publisher = None
    try:
        robot_data_publisher = RobotDataPublisher()
       
        if robot_data_publisher.is_ready:
            try:
                rclpy.spin(robot_data_publisher)
            except KeyboardInterrupt:
                robot_data_publisher.get_logger().info("Interrupción por teclado recibida. Cerrando nodo...")
        else:
            robot_data_publisher.get_logger().error("El nodo no está listo. Terminando.")
           
    except Exception as e:
        print(f"Error al crear el nodo: {e}")
    finally:
        if robot_data_publisher:
            robot_data_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()