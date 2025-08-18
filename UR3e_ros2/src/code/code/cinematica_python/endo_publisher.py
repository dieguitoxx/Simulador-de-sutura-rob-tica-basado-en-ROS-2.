import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class EndoWristPublisher(Node):
    def __init__(self):
        super().__init__('endo_publisher')
        self.publisher_ = self.create_publisher(JointState, '/endowrist', 10)
        self.timer = self.create_timer(1.0, self.publish_joint_state)  # Publica cada segundo

    def publish_joint_state(self):
        msg = JointState()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = [0.1, 0.2, 0.3]  # Valores de ejemplo
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info('Publicando JointState: {}'.format(msg.position))

def main(args=None):
    rclpy.init(args=args)
    node = EndoWristPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
