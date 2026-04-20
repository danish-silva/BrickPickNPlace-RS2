#!/usr/bin/env python3
"""
OnRobot RG2 gripper client for the pick-and-place system.

Controls the gripper by publishing desired finger width to the
/onrobot/finger_width_controller/commands topic. Since the RG2 driver
does not provide an action server, a one-shot ROS2 timer is used to
wait for the gripper to finish moving before signalling completion.

Same async callback pattern as MotionClient: on_done(success: bool).
"""

from typing import Callable

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# ── ROS2 topic published by the OnRobot RG2 ros2_control driver ──
GRIPPER_CMD_TOPIC = '/onrobot/finger_width_controller/commands'


class GripperClient:
    """
    Publishes open/close commands to the OnRobot RG2 gripper.

    Args:
        node:              Parent ROS2 node (used for publisher and timer).
        on_done:           Callback invoked with True after the wait period.
        open_width:        Finger width in metres for "open" (default 0.110).
        close_width:       Finger width in metres for "close" (default 0.0).
        wait_sec:          Seconds to wait after publishing a command (default 1.0).
    """

    def __init__(
        self,
        node: Node,
        on_done: Callable[[bool], None],
        open_width: float = 0.110,
        close_width: float = 0.0,
        wait_sec: float = 1.0,
    ) -> None:
        self._node = node
        self._on_done = on_done
        self._open_width = open_width
        self._close_width = close_width
        self._wait_sec = wait_sec

        self._pub = node.create_publisher(Float64MultiArray, GRIPPER_CMD_TOPIC, 10)
        self._timer = None

    # ── Public API ────────────────────────────────────────────────

    def open(self) -> None:
        """Open the gripper to open_width, then signal on_done after wait_sec."""
        self._node.get_logger().info(
            f'Gripper OPEN → {self._open_width:.3f} m'
        )
        self._publish_width(self._open_width)
        self._start_wait()

    def close(self) -> None:
        """Close the gripper to close_width, then signal on_done after wait_sec."""
        self._node.get_logger().info(
            f'Gripper CLOSE → {self._close_width:.3f} m'
        )
        self._publish_width(self._close_width)
        self._start_wait()

    # ── Internal helpers ──────────────────────────────────────────

    def _publish_width(self, width: float) -> None:
        msg = Float64MultiArray()
        msg.data = [width]
        self._pub.publish(msg)

    def _start_wait(self) -> None:
        """Start a one-shot timer; cancels any existing timer first."""
        if self._timer is not None:
            self._timer.cancel()
            self._node.destroy_timer(self._timer)
        self._timer = self._node.create_timer(self._wait_sec, self._timer_callback)

    def _timer_callback(self) -> None:
        """Fires once after wait_sec — destroy timer and signal completion."""
        self._timer.cancel()
        self._node.destroy_timer(self._timer)
        self._timer = None
        self._on_done(True)
