# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import re


# class CommandParserNode(Node):
#     def __init__(self):
#         super().__init__('command_parser_node')

#         self.subscription = self.create_subscription(
#             String,
#             '/voice_command_raw',
#             self.command_callback,
#             10
#         )

#         self.system_pub = self.create_publisher(String, '/system_command', 10)
#         self.build_pub = self.create_publisher(String, '/build_request', 10)
#         self.sequence_pub = self.create_publisher(String, '/block_sequence', 10)

#         self.get_logger().info('Command Parser Node started.')

#     def command_callback(self, msg):
#         raw = msg.data.lower().strip()
#         self.get_logger().info(f'Received raw command: "{raw}"')

#         if raw in ['start', 'begin', 'go']:
#             self.publish_msg(self.system_pub, 'START')
#             return

#         if raw in ['stop', 'halt', 'cancel']:
#             self.publish_msg(self.system_pub, 'STOP')
#             return

#         if raw in ['reset', 'home', 'go home']:
#             self.publish_msg(self.system_pub, 'RESET')
#             return

#         if 'build tower' in raw:
#             self.publish_msg(self.build_pub, 'BUILD_TOWER')
#             return

#         if 'build line' in raw:
#             self.publish_msg(self.build_pub, 'BUILD_LINE')
#             return

#         if raw.startswith('sequence'):
#             nums = re.findall(r'\d+', raw)
#             if nums:
#                 sequence = ','.join(nums)
#                 self.publish_msg(self.sequence_pub, sequence)
#             else:
#                 self.get_logger().warn('No numbers found in sequence command.')
#             return

#         self.get_logger().warn(f'Unknown command: "{raw}"')

#     def publish_msg(self, publisher, text):
#         msg = String()
#         msg.data = text
#         publisher.publish(msg)
#         self.get_logger().info(f'Published parsed command: "{text}"')


# def main(args=None):
#     rclpy.init(args=args)
#     node = CommandParserNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
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

        # Main shared command topic (GUI + Voice + Keyboard)
        self.system_pub = self.create_publisher(String, '/brick_command', 10)

        self.build_pub = self.create_publisher(String, '/build_request', 10)
        self.sequence_pub = self.create_publisher(String, '/block_sequence', 10)

        # Gate: GUI publishes /voice_enabled (Bool). When False, the mic
        # keeps listening but parsed commands are NOT forwarded onward.
        # Defaults to False — matches the GUI's startup state.
        self._voice_enabled = False
        self.create_subscription(
            Bool, '/voice_enabled', self._on_voice_enabled, 10
        )

        self.get_logger().info(
            'Command Parser Node started (voice disabled until GUI toggle).'
        )

    def _on_voice_enabled(self, msg: Bool) -> None:
        if msg.data != self._voice_enabled:
            self._voice_enabled = msg.data
            self.get_logger().info(
                f"Voice {'ENABLED' if msg.data else 'disabled'} via /voice_enabled."
            )

    def command_callback(self, msg):
        raw = msg.data.lower().strip()
        if not self._voice_enabled:
            self.get_logger().debug(
                f'Ignoring voice command "{raw}" — gate is off.'
            )
            return
        self.get_logger().info(f'Received raw command: "{raw}"')

        # -------- System commands --------
        if raw in ['start', 'begin', 'go']:
            self.publish_msg(self.system_pub, 'start')
            return

        if raw in ['pause', 'hold', 'wait']:
            self.publish_msg(self.system_pub, 'pause')
            return

        if raw in ['stop', 'halt', 'cancel']:
            self.publish_msg(self.system_pub, 'stop')
            return

        if raw in ['reset', 'home', 'go home']:
            self.publish_msg(self.system_pub, 'reset')
            return

        # -------- Build commands --------
        if 'build tower' in raw:
            self.publish_msg(self.build_pub, 'BUILD_TOWER')
            return

        if 'build line' in raw:
            self.publish_msg(self.build_pub, 'BUILD_LINE')
            return

        # -------- Colour selection --------
        colour_map = {
            'pick red': 'COLOUR_RED',
            'pick blue': 'COLOUR_BLUE',
            'pick black': 'COLOUR_BLACK',
        }

        if raw in colour_map:
            self.publish_msg(self.build_pub, colour_map[raw])
            return

        # -------- Custom sequence --------
        if raw.startswith('sequence'):
            nums = re.findall(r'\d+', raw)

            if nums:
                sequence = ','.join(nums)
                self.publish_msg(self.sequence_pub, sequence)
            else:
                self.get_logger().warn(
                    'No numbers found in sequence command.'
                )
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