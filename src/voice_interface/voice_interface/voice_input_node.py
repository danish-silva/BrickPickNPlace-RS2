import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr


class VoiceInputNode(Node):
    def __init__(self):
        super().__init__('voice_input_node')

        self.publisher_ = self.create_publisher(String, '/voice_command_raw', 10)

        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 100
        self.recognizer.dynamic_energy_threshold = False
        self.recognizer.pause_threshold = 0.8

        # Ubuntu / PulseAudio input
        self.mic = sr.Microphone(device_index=10)

        self.valid_commands = [
            'start',
            'stop',
            'reset',
            'build tower',
            'build line',
        ]

        self.get_logger().info('Voice Input Node started.')
        self.get_logger().info('Modes: [m] microphone, [k] keyboard')
        self.get_logger().info(
            'Valid mic commands: start, stop, reset, build tower, build line, sequence one two three'
        )
        self.get_logger().info('Keyboard can still be used for all commands.')

        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def publish_command(self, command_text: str):
        msg = String()
        msg.data = command_text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published raw command: "{msg.data}"')

    def normalize_command(self, text: str):
        text = text.lower().strip()

        alias_map = {
            # start
            'go': 'start',
            'begin': 'start',
            'star': 'start',
            'stark': 'start',

            # stop
            'halt': 'stop',
            'cancel': 'stop',
            'stock': 'stop',
            'top': 'stop',

            # reset
            'home': 'reset',
            'go home': 'reset',
            'recent': 'reset',
            'resent': 'reset',
            'resets': 'reset',
            'balls' : 'reset',

            # build tower
            'tower': 'build tower',
            'build our': 'build tower',
            'build power': 'build tower',
            'bill tower': 'build tower',

            # build line
            'line': 'build line',
            'build mine': 'build line',
            'build lion': 'build line',
            'bill line': 'build line',
        }

        if text in alias_map:
            return alias_map[text]

        if text in self.valid_commands:
            return text

        # Handle spoken sequence commands like:
        # "sequence one two three"
        # "sequence 1 2 3"
        # "sequence one 2 three"
        number_map = {
            'zero': '0',
            'one': '1',
            'two': '2',
            'to': '2',
            'too': '2',
            'three': '3',
            'four': '4',
            'for': '4',
            'five': '5',
            'six': '6',
            'seven': '7',
            'eight': '8',
            'ate': '8',
            'nine': '9',
        }

        if text.startswith('sequence '):
            tail = text[len('sequence '):].strip()
            parts = tail.split()

            converted = []
            for p in parts:
                if p.isdigit():
                    converted.append(p)
                elif p in number_map:
                    converted.append(number_map[p])
                else:
                    return None

            if converted:
                return 'sequence ' + ' '.join(converted)

        return None

    def listen_microphone(self):
        try:
            with self.mic as source:
                self.get_logger().info(
                    'Listening... say: start, stop, reset, build tower, build line, or sequence one two three'
                )
                audio = self.recognizer.listen(
                    source,
                    timeout=15,
                    phrase_time_limit=4
                )

            try:
                text = self.recognizer.recognize_google(audio).lower().strip()
                self.get_logger().info(f'Raw recognized speech: "{text}"')

                normalized = self.normalize_command(text)
                if normalized:
                    self.get_logger().info(f'Normalized command: "{normalized}"')
                    return normalized

                self.get_logger().warn('Speech recognized, but not matched to a valid command.')
                return None

            except sr.UnknownValueError:
                self.get_logger().warn('Speech was heard, but not understood.')
            except sr.RequestError as e:
                self.get_logger().error(f'Google recognizer request failed: {e}')
            except Exception as e:
                self.get_logger().error(f'Recognition exception: {type(e).__name__}: {e}')

        except sr.WaitTimeoutError:
            self.get_logger().warn('No speech detected in time.')
        except Exception as e:
            self.get_logger().error(f'Mic/listen exception: {type(e).__name__}: {e}')

        return None

    def input_loop(self):
        while rclpy.ok():
            try:
                mode = input("\nEnter [m] for microphone or [k] for keyboard: ").strip().lower()

                if mode == 'm':
                    self.get_logger().info(
                        'Get ready, then speak clearly right after the next message.'
                    )
                    spoken_text = self.listen_microphone()
                    if spoken_text:
                        self.publish_command(spoken_text)

                elif mode == 'k':
                    user_input = input("Type command: ").strip().lower()
                    normalized = self.normalize_command(user_input)
                    if normalized:
                        self.publish_command(normalized)
                    else:
                        self.get_logger().warn('Typed input is not a valid command.')

                else:
                    self.get_logger().warn('Invalid mode. Type m or k.')

            except EOFError:
                self.get_logger().warn('Input stream closed.')
                break
            except KeyboardInterrupt:
                self.get_logger().info('Stopping input loop.')
                break
            except Exception as e:
                self.get_logger().error(f'Input loop error: {type(e).__name__}: {e}')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import threading


# class VoiceInputNode(Node):
#     def __init__(self):
#         super().__init__('voice_input_node')
#         self.publisher_ = self.create_publisher(String, '/voice_command_raw', 10)

#         self.get_logger().info('Voice Input Node started.')
#         self.get_logger().info('Type commands like: start, stop, reset')

#         self.input_thread = threading.Thread(target=self.read_input_loop, daemon=True)
#         self.input_thread.start()
#     def read_input_loop(self):
#         while rclpy.ok():
#             user_input = input("Command: ").strip()

#             if user_input:
#                 msg = String()
#                 msg.data = user_input
#                 self.publisher_.publish(msg)
#                 self.get_logger().info(f'Published: {msg.data}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = VoiceInputNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
