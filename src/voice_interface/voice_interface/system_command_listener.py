# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String


# class SystemCommandListener(Node):
#     def __init__(self):
#         super().__init__('system_command_listener')

#         self.create_subscription(String, '/system_command', self.system_cb, 10)
#         self.create_subscription(String, '/build_request', self.build_cb, 10)
#         self.create_subscription(String, '/block_sequence', self.sequence_cb, 10)
#         self.create_subscription(String, '/home_request', self.home_cb, 10)

#         self.get_logger().info('System Command Listener started.')

#     def system_cb(self, msg):
#         self.get_logger().info(f'SYSTEM: {msg.data}')

#     def build_cb(self, msg):
#         self.get_logger().info(f'BUILD: {msg.data}')

#     def sequence_cb(self, msg):
#         self.get_logger().info(f'SEQUENCE: {msg.data}')

#     def home_cb(self, msg):
#         self.get_logger().info(f'HOME REQUEST: {msg.data}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = SystemCommandListener()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SystemCommandListener(Node):
    def __init__(self):
        super().__init__('system_command_listener')

        # Main shared command topic (GUI + Voice + Keyboard)
        self.create_subscription(
            String,
            '/brick_command',
            self.system_cb,
            10
        )

        # Optional system feedback topic
        self.create_subscription(
            String,
            '/system_status',
            self.status_cb,
            10
        )

        # Build / preset requests
        self.create_subscription(
            String,
            '/build_request',
            self.build_cb,
            10
        )

        # Custom sequence requests
        self.create_subscription(
            String,
            '/block_sequence',
            self.sequence_cb,
            10
        )

        self.get_logger().info(
            'System Command Listener started.'
        )

    def system_cb(self, msg):
        self.get_logger().info(
            f'COMMAND: {msg.data}'
        )

    def status_cb(self, msg):
        self.get_logger().info(
            f'STATUS: {msg.data}'
        )

    def build_cb(self, msg):
        self.get_logger().info(
            f'BUILD: {msg.data}'
        )

    def sequence_cb(self, msg):
        self.get_logger().info(
            f'SEQUENCE: {msg.data}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SystemCommandListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()