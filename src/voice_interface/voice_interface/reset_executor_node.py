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
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient

# from std_msgs.msg import String
# from sensor_msgs.msg import JointState
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration


# class ResetExecutorNode(Node):
#     def __init__(self):
#         super().__init__('reset_executor_node')

#         self.subscription = self.create_subscription(
#             String,
#             '/system_command',
#             self.command_callback,
#             10
#         )

#         self.joint_state_sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state_callback,
#             10
#         )

#         self.trajectory_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/scaled_joint_trajectory_controller/follow_joint_trajectory'
#         )

#         # Standard UR3e joint order
#         self.joint_names = [
#             'shoulder_pan_joint',
#             'shoulder_lift_joint',
#             'elbow_joint',
#             'wrist_1_joint',
#             'wrist_2_joint',
#             'wrist_3_joint'
#         ]

#         # Safe home pose in radians
#         self.home_joints = [0.0, -1.5708, 0.00, -1.5708, 0.0, 0.0]

#         self.current_joint_map = {}
#         self.have_joint_state = False

#         self.get_logger().info('Reset Executor Node started.')
#         self.get_logger().info('Waiting for RESET command...')

#     def joint_state_callback(self, msg: JointState):
#         if len(msg.name) != len(msg.position):
#             return

#         self.current_joint_map = dict(zip(msg.name, msg.position))
#         self.have_joint_state = True

#     def command_callback(self, msg: String):
#         if msg.data == 'RESET':
#             self.get_logger().info('RESET received -> sending robot to HOME position')
#             self.send_home_trajectory()

#     def get_current_positions_in_order(self):
#         if not self.have_joint_state:
#             return None

#         positions = []
#         for joint in self.joint_names:
#             if joint not in self.current_joint_map:
#                 self.get_logger().error(f'Missing joint state for {joint}')
#                 return None
#             positions.append(self.current_joint_map[joint])

#         return positions

#     def send_home_trajectory(self):
#         if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error(
#                 'FollowJointTrajectory action server not available. '
#                 'Check UR driver / controller startup.'
#             )
#             return

#         current_positions = self.get_current_positions_in_order()
#         if current_positions is None:
#             self.get_logger().error('No valid joint states yet. Try again in a second.')
#             return

#         goal_msg = FollowJointTrajectory.Goal()
#         goal_msg.trajectory.joint_names = self.joint_names

#         # Point 1: current position at time 0
#         start_point = JointTrajectoryPoint()
#         start_point.positions = current_positions
#         start_point.time_from_start = Duration(sec=0, nanosec=0)

#         # Point 2: home position at time 6 sec
#         goal_point = JointTrajectoryPoint()
#         goal_point.positions = self.home_joints
#         goal_point.time_from_start = Duration(sec=6, nanosec=0)

#         goal_msg.trajectory.points = [start_point, goal_point]

#         self.get_logger().info(f'Current joints: {current_positions}')
#         self.get_logger().info(f'Home joints:    {self.home_joints}')
#         self.get_logger().info('Sending safe 6-second home trajectory...')

#         send_goal_future = self.trajectory_client.send_goal_async(
#             goal_msg,
#             feedback_callback=self.feedback_callback
#         )
#         send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()

#         if goal_handle is None:
#             self.get_logger().error('No goal handle returned.')
#             return

#         if not goal_handle.accepted:
#             self.get_logger().error('Home trajectory goal was rejected.')
#             return

#         self.get_logger().info('Home trajectory goal accepted.')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.result_callback)

#     def result_callback(self, future):
#         result = future.result()

#         if result is None:
#             self.get_logger().error('No result returned from trajectory action.')
#             return

#         status = result.status
#         error_code = result.result.error_code

#         if status == 4:
#             self.get_logger().info('Robot reached HOME position successfully.')
#         else:
#             self.get_logger().warn(
#                 f'Trajectory finished with status={status}, error_code={error_code}'
#             )

#     def feedback_callback(self, feedback_msg):
#         pass


# def main(args=None):
#     rclpy.init(args=args)
#     node = ResetExecutorNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient

# from std_msgs.msg import String
# from sensor_msgs.msg import JointState
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration


# class ResetExecutorNode(Node):
#     def __init__(self):
#         super().__init__('reset_executor_node')

#         self.create_subscription(String, '/system_command', self.command_callback, 10)
#         self.create_subscription(String, '/brick_command', self.command_callback, 10)
#         self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

#         self.trajectory_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/scaled_joint_trajectory_controller/follow_joint_trajectory'
#         )

#         self.joint_names = [
#             'shoulder_pan_joint',
#             'shoulder_lift_joint',
#             'elbow_joint',
#             'wrist_1_joint',
#             'wrist_2_joint',
#             'wrist_3_joint'
#         ]

#         # Real pendant home position: [0, -90, 0, -90, 0, 0]
#         self.home_joints = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]

#         # Small safe demo movement around home position
#         self.demo_joints = [
#             [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0],
#             [0.25, -1.45, 0.1, -1.57, 0.0, 0.0],
#             [-0.25, -1.45, -0.1, -1.57, 0.0, 0.0],
#             [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0],
#         ]

#         self.current_joint_map = {}
#         self.have_joint_state = False
#         self.active_goal_handle = None

#         self.get_logger().info('Reset Executor Node started.')
#         self.get_logger().info('Commands: START = demo movement, STOP = cancel, RESET = home')

#     def joint_state_callback(self, msg: JointState):
#         if len(msg.name) == len(msg.position):
#             self.current_joint_map = dict(zip(msg.name, msg.position))
#             self.have_joint_state = True

#     def command_callback(self, msg: String):
#         command = msg.data.strip().upper()

#         if command == 'START':
#             self.get_logger().info('START received -> running slow demo movement')
#             self.send_demo_trajectory()
#         elif command == 'STOP':
#             self.get_logger().info('STOP received -> cancelling and returning HOME')
#             self.cancel_active_goal()
#             self.send_home_trajectory()
#         elif command == 'PAUSE':
#             self.get_logger().info('PAUSE received -> cancelling active trajectory')
#             self.cancel_active_goal()

#         elif command == 'RESET':
#             self.get_logger().info('RESET received -> returning HOME')
#             self.send_home_trajectory()

#     def get_current_positions_in_order(self):
#         if not self.have_joint_state:
#             return None

#         positions = []
#         for joint in self.joint_names:
#             if joint not in self.current_joint_map:
#                 self.get_logger().error(f'Missing joint state for {joint}')
#                 return None
#             positions.append(self.current_joint_map[joint])

#         return positions

#     def send_demo_trajectory(self):
#         current_positions = self.get_current_positions_in_order()
#         if current_positions is None:
#             self.get_logger().error('No valid joint states yet.')
#             return

#         points = []

#         start_point = JointTrajectoryPoint()
#         start_point.positions = current_positions
#         start_point.time_from_start = Duration(sec=0)
#         points.append(start_point)

#         # Slow movement: 5 seconds between each point
#         time_sec = 5
#         for joints in self.demo_joints:
#             point = JointTrajectoryPoint()
#             point.positions = joints
#             point.time_from_start = Duration(sec=time_sec)
#             points.append(point)
#             time_sec += 5

#         self.send_trajectory(points, 'demo movement')

#     def send_home_trajectory(self):
#         current_positions = self.get_current_positions_in_order()
#         if current_positions is None:
#             self.get_logger().error('No valid joint states yet.')
#             return

#         start_point = JointTrajectoryPoint()
#         start_point.positions = current_positions
#         start_point.time_from_start = Duration(sec=0)

#         home_point = JointTrajectoryPoint()
#         home_point.positions = self.home_joints
#         home_point.time_from_start = Duration(sec=8)  # slower home movement

#         self.send_trajectory([start_point, home_point], 'home movement')

#     def send_trajectory(self, points, label):
#         if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error('Trajectory action server not available.')
#             return

#         goal_msg = FollowJointTrajectory.Goal()
#         goal_msg.trajectory.joint_names = self.joint_names
#         goal_msg.trajectory.points = points

#         self.get_logger().info(f'Sending {label} trajectory...')
#         future = self.trajectory_client.send_goal_async(
#             goal_msg,
#             feedback_callback=self.feedback_callback
#         )
#         future.add_done_callback(self.goal_response_callback)

#     def cancel_active_goal(self):
#         if self.active_goal_handle is None:
#             self.get_logger().warn('No active trajectory goal to cancel.')
#             return

#         cancel_future = self.active_goal_handle.cancel_goal_async()
#         cancel_future.add_done_callback(self.cancel_done_callback)

#     def cancel_done_callback(self, future):
#         self.get_logger().info('Cancel request sent.')
#         self.active_goal_handle = None

#     def goal_response_callback(self, future):
#         goal_handle = future.result()

#         if goal_handle is None:
#             self.get_logger().error('No goal handle returned.')
#             return

#         if not goal_handle.accepted:
#             self.get_logger().error('Trajectory goal rejected.')
#             return

#         self.active_goal_handle = goal_handle
#         self.get_logger().info('Trajectory goal accepted.')

#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.result_callback)

#     def result_callback(self, future):
#         result = future.result()
#         self.active_goal_handle = None

#         if result is None:
#             self.get_logger().error('No result returned.')
#             return

#         if result.status == 4:
#             self.get_logger().info('Trajectory completed successfully.')
#         else:
#             self.get_logger().warn(f'Trajectory finished with status={result.status}')

#     def feedback_callback(self, feedback_msg):
#         pass


# def main(args=None):
#     rclpy.init(args=args)
#     node = ResetExecutorNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#
#recent draft v 1
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient

# from std_msgs.msg import String, Float64MultiArray
# from sensor_msgs.msg import JointState
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration


# class ResetExecutorNode(Node):
#     def __init__(self):
#         super().__init__('reset_executor_node')

#         # Listen to keyboard/mic commands
#         self.create_subscription(
#             String,
#             '/system_command',
#             self.command_callback,
#             10
#         )

#         # Listen to GUI button commands
#         self.create_subscription(
#             String,
#             '/brick_command',
#             self.command_callback,
#             10
#         )

#         # Joint states
#         self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state_callback,
#             10
#         )

#         # Publish to teammate movement node
#         self.pose_pub = self.create_publisher(
#             Float64MultiArray,
#             '/ordered_pose_array',
#             10
#         )

#         # Home trajectory action client
#         self.trajectory_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/scaled_joint_trajectory_controller/follow_joint_trajectory'
#         )

#         self.joint_names = [
#             'shoulder_pan_joint',
#             'shoulder_lift_joint',
#             'elbow_joint',
#             'wrist_1_joint',
#             'wrist_2_joint',
#             'wrist_3_joint'
#         ]

#         # Confirmed pendant home pose
#         self.home_joints = [
#             0.0,
#             -1.5708,
#             0.0,
#             -1.5708,
#             0.0,
#             0.0
#         ]

#         self.current_joint_map = {}
#         self.have_joint_state = False
#         self.active_goal_handle = None

#         self.get_logger().info('Reset Executor Node started')
#         self.get_logger().info(
#             'START = teammate motion | STOP = cancel+home | '
#             'PAUSE = cancel | RESET = home'
#         )

#     # --------------------------------------------------
#     # Joint states
#     # --------------------------------------------------
#     def joint_state_callback(self, msg: JointState):
#         if len(msg.name) == len(msg.position):
#             self.current_joint_map = dict(zip(msg.name, msg.position))
#             self.have_joint_state = True

#     # --------------------------------------------------
#     # Main command callback
#     # --------------------------------------------------
#     def command_callback(self, msg: String):
#         command = msg.data.strip().upper()

#         if command == 'START':
#             self.get_logger().info(
#                 'START received -> sending pose sequence to motion teammate node'
#             )
#             self.send_motion_sequence()

#         elif command == 'STOP':
#             self.get_logger().info(
#                 'STOP received -> cancelling active goal and returning HOME'
#             )
#             self.cancel_active_goal()
#             self.send_home_trajectory()

#         elif command == 'PAUSE':
#             self.get_logger().info(
#                 'PAUSE received -> cancelling active goal'
#             )
#             self.cancel_active_goal()

#         elif command in ['RESET', 'HOME']:
#             self.get_logger().info(
#                 'RESET/HOME received -> returning HOME'
#             )
#             self.send_home_trajectory()

#     # --------------------------------------------------
#     # Publish to teammate movement package
#     # --------------------------------------------------
#     def send_motion_sequence(self):
#         msg = Float64MultiArray()

#         # 3 cartesian poses (x,y,z,r,p,y)
#         msg.data = [
#             0.3, 0.25, 0.3, 3.14159, 0.0, 0.0,
#             0.3, 0.25, 0.2, 3.14159, 0.0, 0.0,
#             0.3, 0.25, 0.3, 3.14159, 0.0, 0.0
#         ]

#         self.pose_pub.publish(msg)
#         self.get_logger().info('Published /ordered_pose_array')

#     # --------------------------------------------------
#     # Home motion
#     # --------------------------------------------------
#     def get_current_positions_in_order(self):
#         if not self.have_joint_state:
#             return None

#         positions = []

#         for joint in self.joint_names:
#             if joint not in self.current_joint_map:
#                 self.get_logger().error(f'Missing joint state for {joint}')
#                 return None

#             positions.append(self.current_joint_map[joint])

#         return positions

#     def send_home_trajectory(self):
#         if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error(
#                 'Trajectory action server not available'
#             )
#             return

#         current_positions = self.get_current_positions_in_order()

#         if current_positions is None:
#             self.get_logger().error(
#                 'No valid joint states yet'
#             )
#             return

#         goal_msg = FollowJointTrajectory.Goal()
#         goal_msg.trajectory.joint_names = self.joint_names

#         start_point = JointTrajectoryPoint()
#         start_point.positions = current_positions
#         start_point.time_from_start = Duration(sec=0)

#         home_point = JointTrajectoryPoint()
#         home_point.positions = self.home_joints
#         home_point.time_from_start = Duration(sec=8)

#         goal_msg.trajectory.points = [
#             start_point,
#             home_point
#         ]

#         self.get_logger().info('Sending HOME trajectory...')

#         future = self.trajectory_client.send_goal_async(
#             goal_msg,
#             feedback_callback=self.feedback_callback
#         )

#         future.add_done_callback(
#             self.goal_response_callback
#         )

#     # --------------------------------------------------
#     # Cancel motion
#     # --------------------------------------------------
#     def cancel_active_goal(self):
#         if self.active_goal_handle is None:
#             self.get_logger().warn(
#                 'No active trajectory goal to cancel'
#             )
#             return

#         cancel_future = self.active_goal_handle.cancel_goal_async()
#         cancel_future.add_done_callback(
#             self.cancel_done_callback
#         )

#     def cancel_done_callback(self, future):
#         self.get_logger().info('Cancel request sent')
#         self.active_goal_handle = None

#     # --------------------------------------------------
#     # Goal handling
#     # --------------------------------------------------
#     def goal_response_callback(self, future):
#         goal_handle = future.result()

#         if goal_handle is None:
#             self.get_logger().error(
#                 'No goal handle returned'
#             )
#             return

#         if not goal_handle.accepted:
#             self.get_logger().error(
#                 'Trajectory goal rejected'
#             )
#             return

#         self.active_goal_handle = goal_handle

#         self.get_logger().info(
#             'Trajectory goal accepted'
#         )

#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(
#             self.result_callback
#         )

#     def result_callback(self, future):
#         result = future.result()
#         self.active_goal_handle = None

#         if result is None:
#             self.get_logger().error(
#                 'No result returned'
#             )
#             return

#         if result.status == 4:
#             self.get_logger().info(
#                 'Trajectory completed successfully'
#             )
#         else:
#             self.get_logger().warn(
#                 f'Trajectory finished with status={result.status}'
#             )

#     def feedback_callback(self, feedback_msg):
#         pass


# def main(args=None):
#     rclpy.init(args=args)

#     node = ResetExecutorNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()

#         if rclpy.ok():
#             rclpy.shutdown()


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

        self.create_subscription(String, '/brick_command', self.command_callback, 10)
        self.create_subscription(String, '/system_command', self.command_callback, 10)
        self.create_subscription(String, '/home_request', self.command_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Original upright home position: [0, -90, 0, -90, 0, 0]
        self.home_joints = [0.8713, -1.4801, 0.1733, -0.2580, -1.5837, 5.5503]

        # Replace these later with your real sample joint positions
        self.demo_joints = [
                        self.home_joints,
                        [1.0665, -1.1645, 1.2737, -1.6398, -1.6139, 1.0875],
                        [1.3370, -1.3069, 1.4013, -1.6161, -1.5920, 1.3261],
                        self.home_joints,
                    ]

        self.current_joint_map = {}
        self.have_joint_state = False
        self.active_goal_handle = None

        self.get_logger().info('Reset Executor Node started.')
        self.get_logger().info('START = original home then demo | STOP = cancel + updated home | RESET/HOME = updated home')

    def joint_state_callback(self, msg: JointState):
        if len(msg.name) == len(msg.position):
            self.current_joint_map = dict(zip(msg.name, msg.position))
            self.have_joint_state = True

    def command_callback(self, msg: String):
        command = msg.data.strip().upper()

        if command == 'START':
            self.get_logger().info('START received -> moving to original home, then demo sequence')
            self.send_demo_trajectory()

        elif command == 'PAUSE':
            self.get_logger().info('PAUSE received -> cancelling active trajectory')
            self.cancel_active_goal()

        elif command == 'STOP':
            self.get_logger().info('STOP received -> cancelling and returning to updated home')
            self.cancel_active_goal()
            self.send_home_trajectory(self.home_joints)

        elif command in ['RESET', 'HOME']:
            self.get_logger().info('RESET/HOME received -> returning to updated home')
            self.send_home_trajectory(self.home_joints)

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

    def send_demo_trajectory(self):
        current_positions = self.get_current_positions_in_order()
        if current_positions is None:
            self.get_logger().error('No valid joint states yet.')
            return

        points = []

        start_point = JointTrajectoryPoint()
        start_point.positions = current_positions
        start_point.time_from_start = Duration(sec=0)
        points.append(start_point)

        time_sec = 6
        for joints in self.demo_joints:
            point = JointTrajectoryPoint()
            point.positions = joints
            point.time_from_start = Duration(sec=time_sec)
            points.append(point)
            time_sec += 6

        self.send_trajectory(points, 'START demo sequence')

    def send_home_trajectory(self, target_joints):
        current_positions = self.get_current_positions_in_order()
        if current_positions is None:
            self.get_logger().error('No valid joint states yet.')
            return

        start_point = JointTrajectoryPoint()
        start_point.positions = current_positions
        start_point.time_from_start = Duration(sec=0)

        home_point = JointTrajectoryPoint()
        home_point.positions = target_joints
        home_point.time_from_start = Duration(sec=8)

        self.send_trajectory([start_point, home_point], 'HOME movement')

    def send_trajectory(self, points, label):
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory action server not available.')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = points

        self.get_logger().info(f'Sending {label}...')
        future = self.trajectory_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def cancel_active_goal(self):
        if self.active_goal_handle is None:
            self.get_logger().warn('No active trajectory goal to cancel.')
            return

        cancel_future = self.active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        self.get_logger().info('Cancel request sent.')
        self.active_goal_handle = None

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error('No goal handle returned.')
            return

        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected.')
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info('Trajectory goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        self.active_goal_handle = None

        if result is None:
            self.get_logger().error('No result returned.')
            return

        if result.status == 4:
            self.get_logger().info('Trajectory completed successfully.')
        else:
            self.get_logger().warn(f'Trajectory finished with status={result.status}')

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