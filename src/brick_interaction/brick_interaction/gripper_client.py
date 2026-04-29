#!/usr/bin/env python3
"""
OnRobot RG2 gripper client for the pick-and-place system.

Controls the gripper by publishing the desired finger width to the
/finger_width_controller/commands topic.

The gripper is moved smoothly: instead of jumping from the current width
straight to the target, intermediate widths are published on a periodic
timer at a configurable speed (m/s). This makes the visualisation finger
motion look like a real RG2 closing/opening rather than instantly snapping
to position.

Same async callback pattern as MotionClient: on_done(success: bool)
is invoked when the gripper reaches its target. Manual variants
(manual_open/manual_close) intentionally do NOT invoke on_done so the
GUI can drive the gripper without advancing the pick-and-place cycle.
"""

from typing import Callable

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# ── ROS2 topic published by the OnRobot RG2 ros2_control controller ──
GRIPPER_CMD_TOPIC = '/finger_width_controller/commands'


class GripperClient:
    """
    Publishes smoothly-interpolated finger-width commands to the
    OnRobot RG2 gripper.

    Args:
        node:         Parent ROS2 node (used for the publisher and timer).
        on_done:      Callback invoked with True after a cycle-driven
                      open()/close() reaches its target.
        open_width:   Finger width in metres for "open"  (default 0.110).
        close_width:  Finger width in metres for "close" (default 0.0).
        speed:        Travel speed in metres/second (default 0.05).
        update_hz:    Publish rate of intermediate widths (default 20 Hz).
    """

    def __init__(
        self,
        node: Node,
        on_done: Callable[[bool], None],
        open_width: float = 0.110,
        close_width: float = 0.0,
        speed: float = 0.05,
        update_hz: float = 20.0,
    ) -> None:
        self._node = node
        self._on_done = on_done
        self._open_width = open_width
        self._close_width = close_width

        self._update_period = 1.0 / update_hz
        self._step_size = speed * self._update_period   # metres per tick

        # Assume the gripper starts in the open position.
        self._current_width = open_width
        self._target_width = open_width

        self._fire_done_on_complete = False
        self._timer = None

        self._pub = node.create_publisher(Float64MultiArray, GRIPPER_CMD_TOPIC, 10)

    # ── Cycle-driven API (fires on_done when target reached) ──────

    def open(self) -> None:
        """Smoothly open the gripper, then signal on_done(True)."""
        self._node.get_logger().info(
            f'Gripper OPEN → {self._open_width:.3f} m'
        )
        self._begin_move(self._open_width, fire_done=True)

    def close(self) -> None:
        """Smoothly close the gripper, then signal on_done(True)."""
        self._node.get_logger().info(
            f'Gripper CLOSE → {self._close_width:.3f} m'
        )
        self._begin_move(self._close_width, fire_done=True)

    # ── Manual (GUI) API — no on_done callback ────────────────────

    def manual_open(self) -> None:
        """Open the gripper without advancing the pick-and-place cycle."""
        self._node.get_logger().info('Gripper MANUAL OPEN')
        self._begin_move(self._open_width, fire_done=False)

    def manual_close(self) -> None:
        """Close the gripper without advancing the pick-and-place cycle."""
        self._node.get_logger().info('Gripper MANUAL CLOSE')
        self._begin_move(self._close_width, fire_done=False)

    # ── Internal helpers ──────────────────────────────────────────

    def _begin_move(self, target: float, fire_done: bool) -> None:
        """Start (or redirect) the smooth-motion timer toward target."""
        self._stop_timer()
        self._target_width = target
        self._fire_done_on_complete = fire_done
        self._publish_width(self._current_width)
        self._timer = self._node.create_timer(
            self._update_period, self._tick
        )

    def _tick(self) -> None:
        """One interpolation step toward the target width."""
        delta = self._target_width - self._current_width

        if abs(delta) <= self._step_size:
            self._current_width = self._target_width
            self._publish_width(self._current_width)
            self._stop_timer()
            if self._fire_done_on_complete:
                self._on_done(True)
            return

        self._current_width += self._step_size if delta > 0 else -self._step_size
        self._publish_width(self._current_width)

    def _stop_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._node.destroy_timer(self._timer)
            self._timer = None

    def _publish_width(self, width: float) -> None:
        msg = Float64MultiArray()
        msg.data = [width]
        self._pub.publish(msg)
