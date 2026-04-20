#!/usr/bin/env python3
"""
Interaction and Execution node — Subsystem 3 (Pass level).

Responsibilities:
  - Receives start/pause/stop commands from the GUI on /brick_command
  - Manages system state via StateMachine
  - Sorts detected bricks by distance from the robot arm base (closest first)
  - Executes an 8-step pick-and-place sequence per brick:
      1. Approach above brick   (arm moves at safe Z height)
      2. Descend to brick       (arm lowers to grab height)
      3. Grab                   (gripper closes)
      4. Retract with brick     (arm rises back to safe Z height)
      5. Approach above slot    (arm moves at safe Z height)
      6. Descend to slot        (arm lowers to release height)
      7. Release                (gripper opens)
      8. Retract                (arm rises back to safe Z height)
  - Publishes the current system state to /system_status for the GUI

Topics:
  Subscribes:  /brick_command        (std_msgs/String)              <- brick_gui
  Subscribes:  /brick_detections     (vision_msgs/Detection3DArray) <- perception
  Publishes:   /system_status        (std_msgs/String)              -> brick_gui
"""

import enum
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection3DArray

from brick_interaction.state_machine import StateMachine, SystemState
from brick_interaction.motion_client import MotionClient
from brick_interaction.gripper_client import GripperClient
from brick_interaction.brick_sorter import (
    Brick,
    PlacementSlot,
    MOCK_BRICKS,
    PLACEMENT_SLOTS,
    ROBOT_BASE,
    sort_by_distance,
    format_sorted_summary,
    bricks_from_detections,
)


# ===========================================================================
# CONFIGURABLE CONSTANTS — edit these to tune heights, gripper widths, etc.
# ===========================================================================

# Heights (metres above the brick/slot surface Z coordinate)
Z_APPROACH = 0.15       # safe clearance for lateral travel
Z_GRAB     = 0.005      # height above surface for grab / release

# Gripper finger widths (metres) — adjust for brick size
GRIPPER_OPEN_WIDTH  = 0.110   # fully open RG2
GRIPPER_CLOSE_WIDTH = 0.0     # fully closed
GRIPPER_WAIT_SEC    = 1.0     # seconds to wait after a grip command


# ===========================================================================
# PHASE ENUM — the 8-step sequence for each brick
# ===========================================================================

class Phase(enum.Enum):
    PICK_APPROACH  = 'pick_approach'    # move above brick
    PICK_DESCEND   = 'pick_descend'     # lower to grab height
    PICK_GRAB      = 'pick_grab'        # close gripper
    PICK_RETRACT   = 'pick_retract'     # rise back up
    PLACE_APPROACH = 'place_approach'   # move above slot
    PLACE_DESCEND  = 'place_descend'    # lower to release height
    PLACE_RELEASE  = 'place_release'    # open gripper
    PLACE_RETRACT  = 'place_retract'    # rise back up


_NEXT_PHASE = {
    Phase.PICK_APPROACH:  Phase.PICK_DESCEND,
    Phase.PICK_DESCEND:   Phase.PICK_GRAB,
    Phase.PICK_GRAB:      Phase.PICK_RETRACT,
    Phase.PICK_RETRACT:   Phase.PLACE_APPROACH,
    Phase.PLACE_APPROACH: Phase.PLACE_DESCEND,
    Phase.PLACE_DESCEND:  Phase.PLACE_RELEASE,
    Phase.PLACE_RELEASE:  Phase.PLACE_RETRACT,
    Phase.PLACE_RETRACT:  None,  # signals: advance to next brick
}


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
#    Current stub  : send_pose_goal() plans to SAFE_HOME_ANGLES (joint-space),
#                    ignoring the Cartesian pose argument, so the pipeline can
#                    be tested end-to-end without real brick coordinates.
#    To complete   : Replace the joint-constraint block in motion_client.py
#                    with a PositionConstraint + OrientationConstraint built
#                    from the pose argument. No changes needed in this file.
#
# 2. VISION INPUT — perception subsystem  *** WIRED ***
#    Subscriber topic  : /brick_detections  (vision_msgs/Detection3DArray)
#    Conversion        : bricks_from_detections() in brick_sorter.py converts
#                        Detection3DArray → list[Brick]
#    NOTE: Coordinates arrive in camera_color_optical_frame.
#          A TF transform to the robot base frame may be needed once
#          camera-to-robot calibration is finalized.
#
# 3. GRIPPER — OnRobot RG2  *** WIRED ***
#    Commands are published to /onrobot/finger_width_controller/commands
#    via GripperClient (brick_interaction/gripper_client.py).
#    Edit the constants above to adjust open/close widths and wait time.
#
# ===========================================================================


class BrickInteractionNode(Node):

    def __init__(self) -> None:
        super().__init__('brick_interaction')

        # Latched QoS: new subscribers immediately receive the last published value.
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
            Detection3DArray, '/brick_detections', self._detection_callback, 1
        )

        # --- State machine ---
        self._sm = StateMachine(on_state_change=self._publish_status)

        # --- Motion client ---
        self._motion = MotionClient(self, on_done=self._on_step_done)

        # --- Gripper client ---
        self._gripper = GripperClient(
            self,
            on_done=self._on_step_done,
            open_width=GRIPPER_OPEN_WIDTH,
            close_width=GRIPPER_CLOSE_WIDTH,
            wait_sec=GRIPPER_WAIT_SEC,
        )

        # --- Perception cache ---
        self._latest_detections: Detection3DArray | None = None

        # --- Brick pick queue ---
        self._brick_queue: list[Brick] = []
        self._queue_index: int = 0

        # --- Phase tracker ---
        self._phase: Phase = Phase.PICK_APPROACH

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

        if self._sm.state == SystemState.RUNNING and prev_state != SystemState.RUNNING:
            self._start_pick_and_place_cycle()

    def _detection_callback(self, msg: Detection3DArray) -> None:
        """Caches the latest brick detections from the perception subsystem."""
        self._latest_detections = msg
        self.get_logger().info(
            f'Received {len(msg.detections)} brick detection(s) from perception.'
        )

    # ------------------------------------------------------------------ #
    # Pick-and-place cycle                                                 #
    # ------------------------------------------------------------------ #

    def _bricks_to_use(self) -> list[Brick]:
        """
        Returns the brick list for this cycle.

        Uses real perception data if available, otherwise falls back
        to MOCK_BRICKS for testing without the camera.
        """
        if self._latest_detections is not None and self._latest_detections.detections:
            bricks = bricks_from_detections(self._latest_detections)
            self.get_logger().info(
                f'Using {len(bricks)} brick(s) from perception.'
            )
            return bricks
        self.get_logger().warn(
            'No perception data — falling back to MOCK_BRICKS.'
        )
        return list(MOCK_BRICKS)

    def _approach_pose(self, x: float, y: float, z: float, theta: float) -> Pose:
        """Pose at safe approach height above the target."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z + Z_APPROACH
        pose.orientation.z = math.sin(theta / 2.0)
        pose.orientation.w = math.cos(theta / 2.0)
        return pose

    def _surface_pose(self, x: float, y: float, z: float, theta: float) -> Pose:
        """Pose at grab/release height just above the surface."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z + Z_GRAB
        pose.orientation.z = math.sin(theta / 2.0)
        pose.orientation.w = math.cos(theta / 2.0)
        return pose

    def _start_pick_and_place_cycle(self) -> None:
        """
        Loads the brick list, sorts by distance from the arm base, logs the
        full pick order, and begins the first 8-step sequence.
        """
        if self._sm.state != SystemState.RUNNING:
            return

        bricks = sort_by_distance(self._bricks_to_use(), ROBOT_BASE)
        self._brick_queue = bricks
        self._queue_index = 0

        for line in format_sorted_summary(bricks, ROBOT_BASE):
            self.get_logger().info(line)

        self._phase = Phase.PICK_APPROACH
        self._execute_phase()

    def _execute_phase(self) -> None:
        """
        Dispatches the current phase to the appropriate action
        (arm motion or gripper command).
        """
        if self._sm.state != SystemState.RUNNING:
            return

        brick = self._brick_queue[self._queue_index]
        slot = PLACEMENT_SLOTS[self._queue_index]
        idx_str = f'[{self._queue_index + 1}/{len(self._brick_queue)}]'

        self.get_logger().info(f'{idx_str} {self._phase.value}')

        if self._phase == Phase.PICK_APPROACH:
            self._motion.send_pose_goal(
                self._approach_pose(brick.x, brick.y, brick.z, brick.theta))

        elif self._phase == Phase.PICK_DESCEND:
            self._motion.send_pose_goal(
                self._surface_pose(brick.x, brick.y, brick.z, brick.theta))

        elif self._phase == Phase.PICK_GRAB:
            self._gripper.close()

        elif self._phase == Phase.PICK_RETRACT:
            self._motion.send_pose_goal(
                self._approach_pose(brick.x, brick.y, brick.z, brick.theta))

        elif self._phase == Phase.PLACE_APPROACH:
            self._motion.send_pose_goal(
                self._approach_pose(slot.x, slot.y, slot.z, slot.theta))

        elif self._phase == Phase.PLACE_DESCEND:
            self._motion.send_pose_goal(
                self._surface_pose(slot.x, slot.y, slot.z, slot.theta))

        elif self._phase == Phase.PLACE_RELEASE:
            self._gripper.open()

        elif self._phase == Phase.PLACE_RETRACT:
            self._motion.send_pose_goal(
                self._approach_pose(slot.x, slot.y, slot.z, slot.theta))

    # ------------------------------------------------------------------ #
    # Step completion callback                                             #
    # ------------------------------------------------------------------ #

    def _on_step_done(self, success: bool) -> None:
        """
        Unified callback for both MotionClient and GripperClient.

        On success: advance to the next phase, or to the next brick if
        the 8-step sequence is complete.
        On failure: set error state and stop the cycle.
        """
        if not success:
            self.get_logger().error(
                f'Step failed during {self._phase.value} of brick '
                f'{self._queue_index + 1}/{len(self._brick_queue)} '
                f'— stopping cycle.'
            )
            self._sm.set_error()
            return

        next_phase = _NEXT_PHASE[self._phase]

        if next_phase is not None:
            self._phase = next_phase
            self._execute_phase()
        else:
            # All 8 steps complete for this brick — advance to next
            self._queue_index += 1
            if self._queue_index < len(self._brick_queue):
                self._phase = Phase.PICK_APPROACH
                self._execute_phase()
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
