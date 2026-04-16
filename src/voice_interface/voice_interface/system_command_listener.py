import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SystemCommandListener(Node):
    def __init__(self):
        super().__init__('system_command_listener')

        self.create_subscription(String, '/system_command', self.system_cb, 10)
        self.create_subscription(String, '/build_request', self.build_cb, 10)
        self.create_subscription(String, '/block_sequence', self.sequence_cb, 10)
        self.create_subscription(String, '/home_request', self.home_cb, 10)

        self.get_logger().info('System Command Listener started.')

    def system_cb(self, msg):
        self.get_logger().info(f'SYSTEM: {msg.data}')

    def build_cb(self, msg):
        self.get_logger().info(f'BUILD: {msg.data}')

    def sequence_cb(self, msg):
        self.get_logger().info(f'SEQUENCE: {msg.data}')

    def home_cb(self, msg):
        self.get_logger().info(f'HOME REQUEST: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SystemCommandListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()