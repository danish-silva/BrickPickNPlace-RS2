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
        self._node.create_timer(5.0, lambda: self._on_done(True))  # 5 seconds delay
