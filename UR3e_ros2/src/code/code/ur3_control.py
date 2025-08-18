import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import math

class UR3Control(Node):
    def __init__(self):
        super().__init__('ur3_control')
        
        # Publicadores para /joint_states y /joint_states2
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_state_pub2 = self.create_publisher(JointState, '/joint_states2', 10)
        
        # Configurar mensajes
        self.joint_state_msg = JointState()
        self.joint_state_msg2 = JointState()
        self.joint_state_msg.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.joint_state_msg2.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Posiciones iniciales (en radianes)
        self.home_position = [-3.4907, -1.1730, -1.5708, -0.9802, 1.5708, 0.0]
        self.home_position2 = [0.1745, -1.1730, -1.5708, -0.9802, 1.5708, 0.0]
        
        # Inicializar los robots en la posición de reposo
        self.send_joint_state(self.home_position, self.joint_state_pub, self.joint_state_msg)
        self.send_joint_state(self.home_position2, self.joint_state_pub2, self.joint_state_msg2)
        
        # Ejecutar la lógica principal en un bucle infinito
        self.execute_trajectories_loop()
    
    def send_joint_state(self, joint_positions, publisher, msg):
        # Configurar el mensaje JointState
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = joint_positions
        msg.velocity = [1.0] * 6  # Velocidades (ajustar según sea necesario)
        msg.effort = []  # Esfuerzo (vacío si no se usa)
        
        # Publicar el mensaje
        publisher.publish(msg)
        self.get_logger().info(f"Published joint positions: {joint_positions}")
    
    def tray_lineal(self, p1, p2, v):
        # p1 y p2 son matrices 1x3 con las coordenadas (x,y,z) de cada punto
        # v es la velocidad en mm/segundo

        # Calcula la distancia entre p1 y p2
        d = np.linalg.norm(np.array(p2) - np.array(p1))

        # Calcula el número de puntos intermedios necesarios para alcanzar la velocidad deseada
        n = int(np.ceil(d / (v / 1000)))

        # Calcula los puntos intermedios
        pts_intermedios = np.zeros((n, 3))
        for i in range(n):
            t = i / (n - 1)  # Interpolación lineal en el intervalo [0,1]
            pts_intermedios[i, :] = (1 - t) * np.array(p1) + t * np.array(p2)

        return pts_intermedios
    
    def md_circulo_v5nuevo(self, p1, p2, Rd):
        # Calcular punto medio con profundidad del 80% del radio de la aguja
        Pcentral = (np.array(p1) + np.array(p2)) / 2
        pcz = Pcentral[2] - Rd * 0.7
        p3 = np.array([Pcentral[0], Pcentral[1], pcz])

        P = np.column_stack((p1, p2, p3))

        # Get circle definition and 3D planar normal to it
        p0 = self.getCentre(p1, p2, p3)
        r = np.linalg.norm(p0 - p1)
        n0, idx = self.getNormal(p0, p1, p2, p3)

        # Vectors to describe the plane
        q1 = P[:, idx[0]]
        q2 = p0 + np.cross(n0, (p1 - p0))

        # Function to compute circle point at given angle
        def fc(a):
            return p0 + np.cos(a) * (q1 - p0) + np.sin(a) * (q2 - p0)

        # Get angles of the original points for the circle formula
        a1 = self.angleFromPoint(p0, p1, q1, q2)
        a2 = self.angleFromPoint(p0, p2, q1, q2)
        a3 = self.angleFromPoint(p0, p3, q1, q2)

        # Generar la trayectoria circular
        path_ar = self.path_Arc(fc, a2, a1)
        return path_ar.T  # Transponer para obtener la forma correcta

    def getCentre(self, p1, p2, p3):
        # Get centre of circle defined by 3D points 'p1','p2','p3'
        v1 = p2 - p1
        v2 = p3 - p1

        v11 = np.dot(v1, v1)
        v22 = np.dot(v2, v2)
        v12 = np.dot(v1, v2)

        b = 1 / (2 * (v11 * v22 - v12**2))
        k1 = b * v22 * (v11 - v12)
        k2 = b * v11 * (v22 - v12)

        p0 = p1 + k1 * v1 + k2 * v2
        return p0

    def getNormal(self, p0, p1, p2, p3):
        # Compute all 3 normals in case two points are colinear with centre
        n12 = np.cross((p1 - p0), (p2 - p0))
        n23 = np.cross((p3 - p0), (p2 - p0))
        n13 = np.cross((p3 - p0), (p1 - p0))

        n = np.column_stack((n12, n23, n13))
        n = n / np.sign(n[0, :])
        idx = np.where(~np.all(np.isnan(n), axis=1))[0]
        n = n[:, idx[0]]
        n0 = n / np.linalg.norm(n)
        return n0, idx

    def angleFromPoint(self, p0, p, q1, q2):
        # Get the circle angle for point 'p'
        comp = lambda a, b: np.dot(a, b) / np.linalg.norm(b)
        ang = np.arctan2(comp(p - p0, q2 - p0), comp(p - p0, q1 - p0))
        return ang

    def path_Arc(self, fc, a, b):
        # Plot circle arc between angles 'a' and 'b' for circle function 'fc'
        while a > b:
            a = a - 2 * np.pi  # Ensure we always go from a to b

        aa = np.linspace(a, b, 100)
        c = fc(aa)
        return c

    def execute_trajectories_loop(self):
        # Bucle infinito para ejecutar las trayectorias continuamente
        while rclpy.ok():
            # Definir puntos iniciales y finales para el primer robot
            pini_r = [0.5, 0.2, 0.3]  # Ajustar según sea necesario
            pfin_r = [0.6, 0.3, 0.4]  # Ajustar según sea necesario

            # Calcular trayectoria lineal para el primer robot
            pts_intermedios = self.tray_lineal(pini_r, pfin_r, v=1)

            # Enviar trayectoria lineal al primer robot
            for point in pts_intermedios:
                joint_positions = self.calculate_inverse_kinematics(point, [0.0, 0.0, 0.0, 1.0])
                self.send_joint_state(joint_positions, self.joint_state_pub, self.joint_state_msg)
                rclpy.spin_once(self, timeout_sec=0.01)

            # Definir puntos iniciales y finales para el segundo robot
            pini_r2 = [0.4, 0.1, 0.2]  # Ajustar según sea necesario
            pfin_r2 = [0.5, 0.2, 0.3]  # Ajustar según sea necesario

            # Calcular trayectoria lineal para el segundo robot
            pts_intermedios2 = self.tray_lineal(pini_r2, pfin_r2, v=1)

            # Enviar trayectoria lineal al segundo robot
            for point in pts_intermedios2:
                joint_positions2 = self.calculate_inverse_kinematics(point, [0.0, 0.0, 0.0, 1.0])
                self.send_joint_state(joint_positions2, self.joint_state_pub2, self.joint_state_msg2)
                rclpy.spin_once(self, timeout_sec=0.01)
    
    def calculate_inverse_kinematics(self, target_position, target_orientation):
        # Simulación de cinemática inversa (ajustar según el modelo del robot)
        # Aquí puedes usar una biblioteca como `kdl_kinematics` o `pykdl_utils`
        # Para este ejemplo, devolvemos una posición aleatoria
        return [
            target_position[0] * 0.5,
            target_position[1] * 0.5,
            target_position[2] * 0.5,
            0.0, 0.0, 0.0
        ]

def main(args=None):
    rclpy.init(args=args)
    ur3_control = UR3Control()
    rclpy.spin(ur3_control)
    ur3_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
