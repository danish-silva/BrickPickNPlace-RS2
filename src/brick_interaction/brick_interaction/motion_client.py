#!/usr/bin/env python3
"""
Async MoveGroup action client wrapper.

Sends motion goals to the /move_action server (provided by MoveIt2's move_group
node) and reports success/failure via a callback.

Key design: fully async callbacks — no spin_until_future_complete.
Calling spin_until_future_complete inside a node that is already being spun
by rclpy.spin() will deadlock. Instead we use add_done_callback so that
results are delivered on the existing executor.

Pattern copied from ur3e_motion/ur3e_motion/move_to_position.py with the
blocking calls replaced by async callbacks.
"""

import math
from typing import Callable

from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
)

PLANNING_GROUP = 'ur_manipulator'

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Safe home position — matches ur3e_motion/move_to_position.py
# Used as the stub target until real Cartesian pose goals are implemented
SAFE_HOME_ANGLES = [
    math.radians(0),
    math.radians(-90),
    math.radians(90),
    math.radians(-90),
    math.radians(-90),
    math.radians(0),
]


class MotionClient:
    """
    Wraps the /move_action ActionClient for use inside a spinning ROS node.

    Args:
        node:    The parent ROS2 node (used to create the ActionClient).
        on_done: Callback invoked with True on motion success, False on failure.
    """

    def __init__(self, node: Node, on_done: Callable[[bool], None]) -> None:
        self._node = node
        self._on_done = on_done
        self._client = ActionClient(node, MoveGroup, '/move_action')

    def send_pose_goal(self, pose: Pose) -> None:
        """
        Send a motion goal derived from the given brick pose.

        TODO: Replace the joint-space stub below with a proper Cartesian
              pose constraint once real brick poses arrive from perception.
              For now we ignore the pose argument and move to SAFE_HOME_ANGLES
              so the pipeline can be tested end-to-end without needing real
              brick coordinates.

        Args:
            pose: Target Cartesian pose for the end-effector (currently unused).
        """
        if not self._client.server_is_ready():
            self._node.get_logger().warn(
                '/move_action server not ready — is move_group running?'
            )
            self._on_done(False)
            return

        # --- STUB: move to safe home joint position ---
        # TODO: Build a Cartesian PositionConstraint + OrientationConstraint
        #       from the pose argument once perception is integrated.
        joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=angle,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for name, angle in zip(JOINT_NAMES, SAFE_HOME_ANGLES)
        ]

        request = MotionPlanRequest()
        request.group_name = PLANNING_GROUP
        request.goal_constraints.append(
            Constraints(joint_constraints=joint_constraints)
        )
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False  # plan AND execute

        self._node.get_logger().info('Sending MoveGroup goal...')
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------ #
    # Internal async callbacks                                             #
    # ------------------------------------------------------------------ #

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error('MoveGroup goal rejected!')
            self._on_done(False)
            return
        self._node.get_logger().info('MoveGroup goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        result = future.result().result
        # error_code.val == 1 means SUCCESS in MoveIt2
        success = (result.error_code.val == 1)
        self._node.get_logger().info(
            f'MoveGroup done — error_code={result.error_code.val} '
            f'({"SUCCESS" if success else "FAILURE"})'
        )
        self._on_done(success)
