#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
import math

# Much simpler — use MoveGroupInterface which talks TO move_group
# rather than creating its own planning stack
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from rclpy.action import ActionClient

TARGET_JOINT_ANGLES = [
    math.radians(0),
    math.radians(-90),
    math.radians(90),
    math.radians(-90),
    math.radians(-90),
    math.radians(0),
    # math.radians(0),
    # math.radians(0),
    # math.radians(0),
    # math.radians(0),
    # math.radians(0),
    # math.radians(0),
]

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

class MoveToPosition(Node):
    def __init__(self):
        super().__init__("move_to_position")
        self._client = ActionClient(self, MoveGroup, "/move_action")

    def run(self):
        self.get_logger().info("Waiting for move_group action server...")
        self._client.wait_for_server()
        self.get_logger().info("Connected! Sending goal...")

        # Build joint constraints
        joint_constraints = []
        for name, angle in zip(JOINT_NAMES, TARGET_JOINT_ANGLES):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        # Build the request
        request = MotionPlanRequest()
        request.group_name = "ur_manipulator"
        request.goal_constraints.append(
            Constraints(joint_constraints=joint_constraints)
        )
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3

        # Send goal
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False  # True = plan only, False = plan + execute

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted, executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f"Done! Error code: {result.error_code.val}")
        # error_code 1 = SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPosition()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()