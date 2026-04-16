import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re


class CommandParserNode(Node):
    def __init__(self):
        super().__init__('command_parser_node')

        self.subscription = self.create_subscription(
            String,
            '/voice_command_raw',
            self.command_callback,
            10
        )

        self.system_pub = self.create_publisher(String, '/system_command', 10)
        self.build_pub = self.create_publisher(String, '/build_request', 10)
        self.sequence_pub = self.create_publisher(String, '/block_sequence', 10)

        self.get_logger().info('Command Parser Node started.')

    def command_callback(self, msg):
        raw = msg.data.lower().strip()
        self.get_logger().info(f'Received raw command: "{raw}"')

        if raw in ['start', 'begin', 'go']:
            self.publish_msg(self.system_pub, 'START')
            return

        if raw in ['stop', 'halt', 'cancel']:
            self.publish_msg(self.system_pub, 'STOP')
            return

        if raw in ['reset', 'home', 'go home']:
            self.publish_msg(self.system_pub, 'RESET')
            return

        if 'build tower' in raw:
            self.publish_msg(self.build_pub, 'BUILD_TOWER')
            return

        if 'build line' in raw:
            self.publish_msg(self.build_pub, 'BUILD_LINE')
            return

        if raw.startswith('sequence'):
            nums = re.findall(r'\d+', raw)
            if nums:
                sequence = ','.join(nums)
                self.publish_msg(self.sequence_pub, sequence)
            else:
                self.get_logger().warn('No numbers found in sequence command.')
            return

        self.get_logger().warn(f'Unknown command: "{raw}"')

    def publish_msg(self, publisher, text):
        msg = String()
        msg.data = text
        publisher.publish(msg)
        self.get_logger().info(f'Published parsed command: "{text}"')


def main(args=None):
    rclpy.init(args=args)
    node = CommandParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()