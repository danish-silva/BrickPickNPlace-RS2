#!/usr/bin/env python3
"""
Interaction and Execution node — Subsystem 3 (Pass level).

Responsibilities:
  - Receives start/pause/stop commands from the GUI on /brick_command
  - Manages system state via StateMachine
  - Sorts detected bricks by distance from the robot arm base (closest first)
  - Sends each brick pose in order to MotionClient → MoveIt2 /move_action server
  - Publishes the current system state to /system_status for the GUI

Topics:
  Subscribes:  /brick_command        (std_msgs/String)          <- brick_gui
  Subscribes:  /brick_detections     (geometry_msgs/PoseArray)  <- perception
                                     TODO: update topic to /perception/brick_list
                                     when Danish's perception node is integrated
  Publishes:   /system_status        (std_msgs/String)          -> brick_gui
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray

from brick_interaction.state_machine import StateMachine, SystemState
from brick_interaction.motion_client import MotionClient
from brick_interaction.brick_sorter import (
    Brick,
    PlacementSlot,
    MOCK_BRICKS,
    PLACEMENT_SLOTS,
    ROBOT_BASE,
    sort_by_distance,
    format_sorted_summary,
)


# ===========================================================================
# INTEGRATION POINTS — edit this block when connecting to other subsystems
# ===========================================================================
#
# 1. MOTION PLANNING — ur3e_motion_cpp  *** ALREADY WIRED ***
#    Every pick and place pose is sent to the MoveIt2 action server via
#    MotionClient (brick_interaction/motion_client.py).
#
#    Action server : /move_action   (moveit_msgs/action/MoveGroup)
#    Provided by   : move_group node, launched alongside ur3e_motion_cpp
#
#    Verify live with:
#      ros2 action list                   → should show /move_action
#      ros2 action info /move_action      → shows server node + clients
#
#    Observable in logs once move_group is running:
#      "Sending MoveGroup goal..."
#      "MoveGroup goal accepted, executing..."
#      "MoveGroup done — error_code=1 (SUCCESS)"
#
#    Current stub  : send_pose_goal() plans to SAFE_HOME_ANGLES (joint-space),
#                    ignoring the Cartesian pose argument, so the pipeline can
#                    be tested end-to-end without real brick coordinates.
#    To complete   : Replace the joint-constraint block in motion_client.py
#                    with a PositionConstraint + OrientationConstraint built
#                    from the pose argument. No changes needed in this file.
#
# 2. VISION INPUT (brick detections from perception subsystem)
#    Subscriber topic  : /brick_detections        (current stub)
#    → change to       : /perception/brick_list   (when Danish's node is ready)
#    Message type      : geometry_msgs/PoseArray  → <custom perception msg>
#    Conversion        : update _bricks_to_use() to call
#                        bricks_from_perception(self._latest_detections)
#                        (see brick_sorter.py for the converter template)
#
# 3. PICK POSE OUTPUT (publish pick target for visualisation / downstream use)
#    Uncomment in __init__:
#    PICK_POSE_TOPIC  = '/interaction/pick_pose'   # geometry_msgs/PoseStamped
#    self._pick_pose_pub = self.create_publisher(PoseStamped, PICK_POSE_TOPIC, 10)
#    Then call self._pick_pose_pub.publish(stamped_pose) in _send_pick()
#
# 4. PLACEMENT POSE OUTPUT (publish placement target for visualisation / downstream)
#    Uncomment in __init__:
#    PLACE_POSE_TOPIC = '/interaction/place_pose'  # geometry_msgs/PoseStamped
#    self._place_pose_pub = self.create_publisher(PoseStamped, PLACE_POSE_TOPIC, 10)
#    Then call self._place_pose_pub.publish(stamped_pose) in _send_place()
#
# ===========================================================================


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
        # Populated by _detection_callback when real perception publishes.
        # TODO: update topic to /perception/brick_list and parse into Brick objects
        #       when Danish's perception node is ready (see _detection_callback).
        self._latest_detections: PoseArray | None = None

        # --- Brick pick queue ---
        # Sorted list of Brick objects for the current cycle, closest first.
        # Built in _start_pick_and_place_cycle() from either mock or real data.
        self._brick_queue: list[Brick] = []
        self._queue_index: int = 0

        # --- Pick-and-place phase tracker ---
        # Each brick requires two motion goals: 'pick' (move to brick) then
        # 'place' (move to placement slot). _phase records which step is active
        # so _on_motion_done knows what to do when the action completes.
        self._phase: str = 'pick'  # 'pick' | 'place'

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

        TODO: When Danish's perception node is ready:
          1. Change the subscriber topic to /perception/brick_list
          2. Change the message type to the custom brick list message
          3. Convert each entry to a Brick object and store as a list here
          4. In _start_pick_and_place_cycle, use self._latest_detections
             instead of MOCK_BRICKS
        """
        self._latest_detections = msg

    # ------------------------------------------------------------------ #
    # Pick-and-place cycle                                                 #
    # ------------------------------------------------------------------ #

    def _bricks_to_use(self) -> list[Brick]:
        """
        Returns the brick list for this cycle.

        Currently returns MOCK_BRICKS for demo purposes.
        TODO: When perception is integrated, replace with:
            return bricks_from_perception(self._latest_detections)
        """
        return list(MOCK_BRICKS)

    def _pose_from_xyztheta(self, x: float, y: float, z: float, theta: float) -> Pose:
        """
        Build a geometry_msgs/Pose from position + yaw.
        z is raised by 15 cm to approach from above.
        Orientation is a pure z-axis rotation (quaternion from yaw only).
        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z + 0.15  # 15 cm clearance above surface
        pose.orientation.z = math.sin(theta / 2.0)
        pose.orientation.w = math.cos(theta / 2.0)
        return pose

    def _brick_to_pose(self, brick: Brick) -> Pose:
        """Convert a Brick's pick position into a motion goal Pose."""
        return self._pose_from_xyztheta(brick.x, brick.y, brick.z, brick.theta)

    def _slot_to_pose(self, slot: PlacementSlot) -> Pose:
        """Convert a PlacementSlot's target position into a motion goal Pose."""
        return self._pose_from_xyztheta(slot.x, slot.y, slot.z, slot.theta)

    def _start_pick_and_place_cycle(self) -> None:
        """
        Loads the brick list, sorts by distance from the arm base, logs the
        full pick order, and begins the first pick-and-place step.

        Each brick goes through two motion goals:
          1. _send_pick()  — move to the brick's detected position
          2. _send_place() — move to the corresponding placement slot

        TODO: This method is the natural insertion point for a py_trees
              behaviour tree once gripper control is integrated.
        """
        if self._sm.state != SystemState.RUNNING:
            return

        # Sort bricks closest-to-furthest from the robot arm base
        bricks = sort_by_distance(self._bricks_to_use(), ROBOT_BASE)
        self._brick_queue = bricks
        self._queue_index = 0

        # Log the full sorted pick order — visible to tutor during demo
        for line in format_sorted_summary(bricks, ROBOT_BASE):
            self.get_logger().info(line)

        self._send_pick()

    def _send_pick(self) -> None:
        """
        Phase 1 of pick-and-place: send the motion goal for the current
        brick's detected position. Sets _phase = 'pick'.
        """
        if self._sm.state != SystemState.RUNNING:
            return

        self._phase = 'pick'
        brick = self._brick_queue[self._queue_index]
        slot = PLACEMENT_SLOTS[self._queue_index]
        total = len(self._brick_queue)
        self.get_logger().info(
            f'[{self._queue_index + 1}/{total}] PICK  '
            f'{brick.colour} {brick.size} '
            f'at ({brick.x:.3f}, {brick.y:.3f})  →  '
            f'PLACE at {slot.label} ({slot.x:.3f}, {slot.y:.3f})'
        )
        self._motion.send_pose_goal(self._brick_to_pose(brick))

    def _send_place(self) -> None:
        """
        Phase 2 of pick-and-place: send the motion goal for the placement
        slot that corresponds to the current brick. Sets _phase = 'place'.
        """
        if self._sm.state != SystemState.RUNNING:
            return

        self._phase = 'place'
        slot = PLACEMENT_SLOTS[self._queue_index]
        self.get_logger().info(
            f'[{self._queue_index + 1}/{len(self._brick_queue)}] '
            f'PLACE → {slot.label} ({slot.x:.3f}, {slot.y:.3f})'
        )
        self._motion.send_pose_goal(self._slot_to_pose(slot))

    # ------------------------------------------------------------------ #
    # Motion completion callback                                           #
    # ------------------------------------------------------------------ #

    def _on_motion_done(self, success: bool) -> None:
        """
        Called by MotionClient when the MoveGroup action finishes.

        Two-phase logic:
          pick  succeeded → send placement goal for the same brick
          place succeeded → advance to the next brick (or mark complete)
          either failed   → set error and stop the cycle
        """
        if not success:
            self.get_logger().error(
                f'Motion failed during {self._phase} of brick '
                f'{self._queue_index + 1}/{len(self._brick_queue)} '
                f'— stopping cycle.'
            )
            self._sm.set_error()
            return

        if self._phase == 'pick':
            # Pick motion done — now move to the placement slot
            self._send_place()
        else:
            # Place motion done — advance to the next brick
            self._queue_index += 1
            if self._queue_index < len(self._brick_queue):
                self._send_pick()
            else:
                self.get_logger().info(
                    f'All {len(self._brick_queue)} bricks placed — cycle complete.'
                )
                self._sm.set_completed()

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
