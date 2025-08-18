# -*- coding: utf-8 -*-
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from ament_index_python.packages import get_package_share_directory

class RobotDataPublisher(Node):
    def __init__(self):
        super().__init__('robot_data_publisher')

        self.start_time = False
        self.ur3_joints = []
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
        self.endo_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        try:
            self.get_logger().info('Cargando datos desde los archivos...')

            package_share_dir = get_package_share_directory('code')
            ur3_path = os.path.join(package_share_dir, 'ur3_joints.txt')
            endo_path = os.path.join(package_share_dir, 'endowrist_joints.txt')

            # Cargar datos de ambos archivos
            self.ur3_joints = self.load_joint_positions(ur3_path)
            self.endo_joints = self.load_joint_positions(endo_path)

            if len(self.ur3_joints) == 0:
                raise ValueError("No se cargaron posiciones válidas del archivo UR3.")
            if len(self.endo_joints) == 0:
                raise ValueError("No se cargaron posiciones válidas del archivo Endowrist.")

            self.get_logger().info(f'Se cargaron {len(self.ur3_joints)} posiciones del UR3.')
            self.get_logger().info(f'Se cargaron {len(self.endo_joints)} posiciones del Endowrist.')

        except (FileNotFoundError, ValueError) as e:
            self.get_logger().error(f"Error durante la carga de datos: {str(e)}")
            self.destroy_node()
            rclpy.shutdown()
            return

        # Verificar que ambos archivos tengan datos
        if len(self.ur3_joints) == 0 or len(self.endo_joints) == 0:
            self.get_logger().warn('No se cargaron datos válidos. Apagando nodo.')
            self.destroy_node()
            rclpy.shutdown()
            return

        # Usar el mínimo número de puntos para sincronización
        self.num_points = min(len(self.ur3_joints), len(self.endo_joints))
        self.get_logger().info(f'Se publicarán {self.num_points} puntos de datos sincronizados.')

        self.publish_index = 0
        # Timer para publicar cada 50ms (20Hz)
        self.timer = self.create_timer(0.05, self.publish_callback)
        self.start_time = True

    def load_joint_positions(self, filename):
        """
        Carga las posiciones articulares desde un archivo de texto.
        Ignora el encabezado (líneas que empiezan con #) y la última columna (q6).
        Retorna solo las primeras 6 columnas (q0-q5).
        """
        try:
            # Cargar datos completos
            full_data = np.loadtxt(filename, comments='#', delimiter=',')
           
            # Si es un array 1D (una sola fila), convertir a 2D
            if full_data.ndim == 1:
                full_data = full_data.reshape(1, -1)
           
            # Tomar solo las primeras 6 columnas (ignorar q6)
            joint_data = full_data[:, :6]
           
            self.get_logger().info(f'Archivo {filename}: {joint_data.shape[0]} filas, {joint_data.shape[1]} articulaciones')
            return joint_data
           
        except Exception as e:
            self.get_logger().error(f"Error cargando {filename}: {str(e)}")
            return np.array([])

    def publish_callback(self):
        """
        Publica los datos de las articulaciones del UR3 y del Endowrist sincronizadamente.
        Se ejecuta en cada tick del temporizador.
        """
        if self.publish_index >= self.num_points:
            self.get_logger().info('Se han publicado todos los puntos. Reiniciando secuencia.')
            self.publish_index = 0

        # --- Publicar UR3 ---
        current_ur3_q = self.ur3_joints[self.publish_index]
        self.ur3_msg.header.stamp = self.get_clock().now().to_msg()
        self.ur3_msg.position = current_ur3_q.tolist()
        self.ur3_pub.publish(self.ur3_msg)
       
        # --- Publicar Endowrist ---
        current_endo_q = self.endo_joints[self.publish_index]
        self.endo_msg.header.stamp = self.get_clock().now().to_msg()
        self.endo_msg.position = current_endo_q.tolist()
        self.endo_pub.publish(self.endo_msg)

        # Log de información
        self.get_logger().info(
            f'Punto {self.publish_index + 1}/{self.num_points} - '
            f'UR3: {np.round(current_ur3_q, 3)} | '
            f'Endo: {np.round(current_endo_q, 3)}'
        )

        self.publish_index += 1

def main(args=None):
    rclpy.init(args=args)
   
    robot_data_publisher = RobotDataPublisher()
   
    # Solo hacer spin si el nodo se inicializó correctamente
    if robot_data_publisher.start_time:
        try:
            rclpy.spin(robot_data_publisher)
        except KeyboardInterrupt:
            self.get_logger().info('Interrumpido por el usuario.')
        finally:
            # Destruir el nodo explícitamente
            robot_data_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()