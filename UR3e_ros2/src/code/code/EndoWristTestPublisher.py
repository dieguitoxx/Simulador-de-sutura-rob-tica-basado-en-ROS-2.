# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


class EndoWristTestPublisher(Node):
    def __init__(self):
        super().__init__('endowrist_test_publisher')

        self.pub = self.create_publisher(JointState, '/endowrist', 10)

        self.msg = JointState()
        self.msg.header.frame_id = 'base_link'
        self.msg.name = ['shaft', 'wrist', 'jaw_dx', 'jaw_sx']

        self.time = 0.0
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.phase_duration = 4.0  # duración de cada fase (s)

    def timer_callback(self):
        self.time += 0.05

        # Movimiento base de avance y giro
        shaft = 0.5 * np.sin(self.time * 0.5)             # Avance/retraer
        wrist = np.deg2rad(45) * np.sin(self.time * 0.3)  # Giro +/- 45°

        # Determinar fase
        phase = int((self.time // self.phase_duration) % 2)

        if phase == 0:
            # --- Movimiento de lado a lado ---
            jaw = 1.5 * np.sin(self.time * 2.0)
            self.msg.position = [shaft, wrist, jaw, -jaw]
            fase_txt = "LATERAL"

        else:
            # --- Movimiento de apertura/cierre ---
            threshold = 0.03
            sine = np.sin(self.time * 2.0)
            positive_sine = (sine + 1.0) / 2.0
            jaw = threshold + (1.0 - threshold) * positive_sine
            self.msg.position = [shaft, wrist, jaw, jaw]
            fase_txt = "APERTURA"

        # Publicar
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

        self.get_logger().info(
            f"📤 Fase={fase_txt} | shaft={shaft:.3f} m | wrist={np.rad2deg(wrist):.1f}° | jaw_dx={self.msg.position[2]:.3f} rad | jaw_sx={self.msg.position[3]:.3f} rad"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EndoWristTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
