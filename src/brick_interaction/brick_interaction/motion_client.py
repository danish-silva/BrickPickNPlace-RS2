#!/usr/bin/env python3
"""
Publisher wrapper for sending target poses to the move_to_position node.

Publishes a Float64MultiArray to ordered_pose_array, matching the
subscriber in ur3e_motion_cpp/src/move_to_position.cpp.
"""

import math
from typing import Callable

from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray


def _pose_to_flat_array(pose: Pose) -> list[float]:
    """Convert a Pose into [x, y, z, roll, pitch, yaw]."""
    q = pose.orientation
    # Quaternion to roll/pitch/yaw
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        roll,
        pitch,
        yaw,
    ]


class MotionClient:
    """
    Wraps a publisher to ordered_pose_array for use with the move_to_position node.

    Args:
        node:    The parent ROS2 node (used to create the publisher).
        on_done: Callback invoked with True on motion success, False on failure.
    """

    def __init__(self, node: Node, on_done: Callable[[bool], None]) -> None:
        self._node = node
        self._on_done = on_done
        self._pose_publisher = node.create_publisher(Float64MultiArray, 'ordered_pose_array', 10)
        self._completion_timer = None

    def send_pose_goal(self, pose: Pose) -> None:
        """
        Publish the target pose as a Float64MultiArray to ordered_pose_array.

        Args:
            pose: Target Cartesian pose for the end-effector.
        """
        msg = Float64MultiArray()
        msg.data = _pose_to_flat_array(pose)
        self._node.get_logger().info('Publishing pose to ordered_pose_array...')
        self._pose_publisher.publish(msg)
        # Assume success after publishing since move_to_position does not provide feedback.
        self._schedule_done(5.0)

    def send_pick_place_goal(
        self,
        brick_pose: Pose,
        target_pose: Pose,
        completion_delay_s: float = 1.0,
    ) -> None:
        """
        Publish one pick/place pair for ur3e_motion_mtc.

        The MTC node expects exactly 12 values:
        [brick x, y, z, roll, pitch, yaw, target x, y, z, roll, pitch, yaw].
        """
        msg = Float64MultiArray()
        msg.data = _pose_to_flat_array(brick_pose) + _pose_to_flat_array(target_pose)
        self._node.get_logger().info(
            'Publishing pick/place pair to ordered_pose_array...'
        )
        self._pose_publisher.publish(msg)
        self._schedule_done(completion_delay_s)

    def _schedule_done(self, delay_s: float) -> None:
        if self._completion_timer is not None:
            self._completion_timer.cancel()
            self._node.destroy_timer(self._completion_timer)
            self._completion_timer = None

        def _complete_once() -> None:
            if self._completion_timer is not None:
                self._completion_timer.cancel()
                self._node.destroy_timer(self._completion_timer)
                self._completion_timer = None
            self._on_done(True)

        self._completion_timer = self._node.create_timer(delay_s, _complete_once)
