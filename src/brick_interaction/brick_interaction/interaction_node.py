#!/usr/bin/env python3
"""
Interaction and Execution node — Subsystem 3 (Pass level).

Responsibilities:
  - Receives start/pause/stop commands from the GUI on /brick_command
  - Manages system state via StateMachine
  - Selects a target brick pose (stub — hardcoded until perception is integrated)
  - Sends the pose to MotionClient which calls the MoveIt2 /move_action server
  - Publishes the current system state to /system_status for the GUI

Topics:
  Subscribes:  /brick_command      (std_msgs/String)          <- brick_gui
  Subscribes:  /brick_detections   (geometry_msgs/PoseArray)  <- perception (stub)
  Publishes:   /system_status      (std_msgs/String)          -> brick_gui
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray

from brick_interaction.state_machine import StateMachine, SystemState
from brick_interaction.motion_client import MotionClient


class BrickInteractionNode(Node):

    def __init__(self) -> None:
        super().__init__('brick_interaction')

        # Latched QoS: new subscribers immediately receive the last published value.
        # This means `ros2 topic echo /system_status --once` works even if the
        # state hasn't changed since startup.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- Publishers ---
        self._status_pub = self.create_publisher(String, '/system_status', latched_qos)

        # --- Subscribers ---
        self.create_subscription(
            String, '/brick_command', self._command_callback, 10
        )
        self.create_subscription(
            PoseArray, '/brick_detections', self._detection_callback, 1
        )

        # --- State machine ---
        # on_state_change fires _publish_status on every real transition
        self._sm = StateMachine(on_state_change=self._publish_status)

        # --- Motion client ---
        # on_done fires _on_motion_done when the MoveGroup action completes
        self._motion = MotionClient(self, on_done=self._on_motion_done)

        # --- Perception cache ---
        # Populated by _detection_callback when real perception is running.
        # TODO: replace stub in _get_target_pose() with this once perception works.
        self._latest_detections: PoseArray | None = None

        # Publish initial status so the GUI shows "idle" on startup
        self._publish_status(SystemState.IDLE)

        self.get_logger().info('BrickInteractionNode ready — waiting for commands.')

    # ------------------------------------------------------------------ #
    # Subscriber callbacks                                                 #
    # ------------------------------------------------------------------ #

    def _command_callback(self, msg: String) -> None:
        """
        Receives 'start', 'pause', or 'stop' from /brick_command.
        Routes to the state machine and kicks off the pick-and-place cycle
        when transitioning into RUNNING.
        """
        command = msg.data.strip().lower()
        prev_state = self._sm.state

        valid = self._sm.handle_command(command)
        if not valid:
            self.get_logger().warn(
                f'Command "{command}" ignored in state {prev_state.value}'
            )
            return

        # If we just entered RUNNING (from IDLE or PAUSED), start the cycle.
        # If we were already RUNNING (shouldn't happen, but guard anyway) skip.
        if self._sm.state == SystemState.RUNNING and prev_state != SystemState.RUNNING:
            self._start_pick_and_place_cycle()

    def _detection_callback(self, msg: PoseArray) -> None:
        """
        Caches the latest brick detections from the perception subsystem.
        Called whenever Danish's perception node publishes to /brick_detections.
        """
        self._latest_detections = msg

    # ------------------------------------------------------------------ #
    # Pick-and-place cycle                                                 #
    # ------------------------------------------------------------------ #

    def _get_target_pose(self) -> Pose:
        """
        Returns the pose the arm should move to for the pick action.

        STUB: Returns a hardcoded pose at a known safe location.
        TODO: Replace with real perception data:
            if self._latest_detections and self._latest_detections.poses:
                return self._latest_detections.poses[0]
        """
        pose = Pose()
        pose.position.x = 0.4   # metres in robot base frame
        pose.position.y = 0.2
        pose.position.z = 0.2
        pose.orientation.w = 1.0  # no rotation (identity quaternion)
        return pose

    def _start_pick_and_place_cycle(self) -> None:
        """
        Entry point for a single pick-and-place execution cycle.

        Gets the target pose and fires off the motion goal asynchronously.
        _on_motion_done() is called when the MoveGroup action completes.

        TODO: Extend this method into a multi-step sequence once gripper
              control is integrated:
                1. Move to approach pose (above brick)
                2. Open gripper
                3. Move to pick pose
                4. Close gripper
                5. Move to lift pose
                6. Move to place pose
                7. Open gripper
              This is the natural insertion point for a py_trees behaviour tree.
        """
        if self._sm.state != SystemState.RUNNING:
            return  # Guard: bail if paused between state check and here

        target_pose = self._get_target_pose()
        self.get_logger().info(
            f'Starting pick-and-place cycle — target: '
            f'({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, '
            f'{target_pose.position.z:.3f})'
        )
        self._motion.send_pose_goal(target_pose)

    # ------------------------------------------------------------------ #
    # Motion completion callback                                           #
    # ------------------------------------------------------------------ #

    def _on_motion_done(self, success: bool) -> None:
        """
        Called by MotionClient when the MoveGroup action finishes.
        Drives the state machine to COMPLETED or ERROR.
        """
        if success:
            self.get_logger().info('Pick-and-place cycle completed successfully.')
            self._sm.set_completed()
        else:
            self.get_logger().error('Pick-and-place cycle failed.')
            self._sm.set_error()

    # ------------------------------------------------------------------ #
    # Status publishing                                                    #
    # ------------------------------------------------------------------ #

    def _publish_status(self, state: SystemState) -> None:
        """
        Publishes the current state string to /system_status.
        Registered as the StateMachine on_state_change callback so it fires
        automatically on every real transition.
        """
        msg = String()
        msg.data = state.value
        self._status_pub.publish(msg)
        self.get_logger().info(f'System status -> {state.value}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BrickInteractionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
