import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class EndoWristSubscriber(Node):
    def __init__(self):
        super().__init__('endo_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/endowrist',
            self.listener_callback,
            10)
        self.subscription  # Para evitar advertencias de variable no utilizada

    def listener_callback(self, msg):
        self.get_logger().info('Recibido JointState: {}'.format(msg.position))

def main(args=None):
    rclpy.init(args=args)
    node = EndoWristSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
