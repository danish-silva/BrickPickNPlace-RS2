# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String


# class ResetExecutorNode(Node):
#     def __init__(self):
#         super().__init__('reset_executor_node')

#         self.create_subscription(
#             String,
#             '/system_command',
#             self.command_callback,
#             10
#         )

#         self.home_pub = self.create_publisher(String, '/home_request', 10)

#         self.get_logger().info("Reset Executor Node started")

#     def command_callback(self, msg):
#         if msg.data == "RESET":
#             self.get_logger().info("RESET received -> publishing HOME request")

#             home_msg = String()
#             home_msg.data = "HOME"
#             self.home_pub.publish(home_msg)

#             self.get_logger().info("Published /home_request = HOME")


# def main(args=None):
#     rclpy.init(args=args)
#     node = ResetExecutorNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ResetExecutorNode(Node):
    def __init__(self):
        super().__init__('reset_executor_node')

        self.subscription = self.create_subscription(
            String,
            '/system_command',
            self.command_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        # Standard UR3e joint order
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Safe home pose in radians
        self.home_joints = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]

        self.current_joint_map = {}
        self.have_joint_state = False

        self.get_logger().info('Reset Executor Node started.')
        self.get_logger().info('Waiting for RESET command...')

    def joint_state_callback(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            return

        self.current_joint_map = dict(zip(msg.name, msg.position))
        self.have_joint_state = True

    def command_callback(self, msg: String):
        if msg.data == 'RESET':
            self.get_logger().info('RESET received -> sending robot to HOME position')
            self.send_home_trajectory()

    def get_current_positions_in_order(self):
        if not self.have_joint_state:
            return None

        positions = []
        for joint in self.joint_names:
            if joint not in self.current_joint_map:
                self.get_logger().error(f'Missing joint state for {joint}')
                return None
            positions.append(self.current_joint_map[joint])

        return positions

    def send_home_trajectory(self):
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'FollowJointTrajectory action server not available. '
                'Check UR driver / controller startup.'
            )
            return

        current_positions = self.get_current_positions_in_order()
        if current_positions is None:
            self.get_logger().error('No valid joint states yet. Try again in a second.')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        # Point 1: current position at time 0
        start_point = JointTrajectoryPoint()
        start_point.positions = current_positions
        start_point.time_from_start = Duration(sec=0, nanosec=0)

        # Point 2: home position at time 6 sec
        goal_point = JointTrajectoryPoint()
        goal_point.positions = self.home_joints
        goal_point.time_from_start = Duration(sec=6, nanosec=0)

        goal_msg.trajectory.points = [start_point, goal_point]

        self.get_logger().info(f'Current joints: {current_positions}')
        self.get_logger().info(f'Home joints:    {self.home_joints}')
        self.get_logger().info('Sending safe 6-second home trajectory...')

        send_goal_future = self.trajectory_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error('No goal handle returned.')
            return

        if not goal_handle.accepted:
            self.get_logger().error('Home trajectory goal was rejected.')
            return

        self.get_logger().info('Home trajectory goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()

        if result is None:
            self.get_logger().error('No result returned from trajectory action.')
            return

        status = result.status
        error_code = result.result.error_code

        if status == 4:
            self.get_logger().info('Robot reached HOME position successfully.')
        else:
            self.get_logger().warn(
                f'Trajectory finished with status={status}, error_code={error_code}'
            )

    def feedback_callback(self, feedback_msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ResetExecutorNode()

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