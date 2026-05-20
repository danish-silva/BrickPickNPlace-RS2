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
  Subscribes:  /brick_detector/brick_pose (geometry_msgs/PoseStamped) <- brick_vision (camera frame, diagnostic)
  Subscribes:  /pickup_bricks        (vision_msgs/Detection3DArray) <- frame_transform_node (base_link)
  Subscribes:  /available_slots_base (geometry_msgs/PoseArray)      <- frame_transform_node (base_link)
  Publishes:   /snapshot_trigger     (std_msgs/Empty)               -> brick_vision (bootstrap only;
                                                                       motion node re-triggers thereafter)
  Publishes:   /ordered_pose_array   (std_msgs/Float64MultiArray)   -> ur3e_motion_mtc
  Publishes:   /system_status        (std_msgs/String)              -> brick_gui
"""

import enum
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
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
    choose_nearest_eligible_brick,
    choose_nearest_slot,
    sort_by_distance,
    format_sorted_summary,
    brick_from_pose_stamped,
    bricks_from_detections,
    slots_from_pose_array,
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
GRIPPER_SPEED       = 0.05    # m/s — how fast the fingers travel
GRIPPER_UPDATE_HZ   = 20.0    # publish rate of intermediate widths

# Current outputs from brick_vision's node name + private topics.
VISION_BRICK_POSE_TOPIC = '/brick_detector/brick_pose'        # camera frame (diagnostic only)
VISION_DETECTIONS_TOPIC = '/pickup_bricks'                    # base_link, via frame_transform_node
VISION_AVAILABLE_SLOTS_TOPIC = '/available_slots_base'        # base_link, via frame_transform_node
VISION_SCAN_TRIGGER_TOPIC = '/snapshot_trigger'

# MTC consumes one message containing exactly two poses: pick brick, place slot.
MOTION_MODE = 'mtc'  # 'mtc' for ur3e_motion_mtc, 'stepwise' for ur3e_motion_cpp
MTC_MOTION_COMPLETION_DELAY_S = 45.0

# Scan loop timing. The motion package's MTC task returns to camera_home after
# place, so each loop asks vision for a fresh snapshot from the scan position.
SCAN_TIMEOUT_S = 8.0
MAX_BRICKS_PER_RUN = 100

# Brick selection criteria. Empty allow-lists mean "accept any".
ELIGIBLE_COLOURS: list[str] = []
ELIGIBLE_SIZES: list[str] = []
MIN_DETECTION_CONFIDENCE = 0.30

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
# 1. MOTION PLANNING — ur3e_motion_mtc  *** WIRED ***
#    On "start", this node publishes one /ordered_pose_array message containing
#    [brick pose, target slot pose], exactly matching ur3e_motion_mtc's current
#    std_msgs/Float64MultiArray contract. MTC's task should finish by returning
#    to camera_home before this node requests the next vision scan.
#
# 2. VISION INPUT — perception subsystem  *** WIRED ***
#    Current topic     : /brick_detector/brick_pose (geometry_msgs/PoseStamped)
#                        Published by brick_vision. Contains the best brick's
#                        position and angle, but not colour/size metadata.
#    Brick list        : /brick_detector/detections  (vision_msgs/Detection3DArray)
#    Drop-zone slots   : /brick_detector/available_slots (geometry_msgs/PoseArray)
#    Scan trigger      : /snapshot_trigger (std_msgs/Empty)
#    Conversion        : bricks_from_detections() in brick_sorter.py converts
#                        Detection3DArray → list[Brick]
#    NOTE: Coordinates arrive in camera_color_optical_frame.
#          A TF transform to the robot base frame may be needed once
#          camera-to-robot calibration is finalized.
#
# 3. GRIPPER — OnRobot RG2  *** WIRED ***
#    Commands are published to /finger_width_controller/commands
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
        self._scan_trigger_pub = self.create_publisher(
            Empty, VISION_SCAN_TRIGGER_TOPIC, 10
        )

        # --- Subscribers ---
        self.create_subscription(
            String, '/brick_command', self._command_callback, 10
        )
        self.create_subscription(
            PoseStamped, VISION_BRICK_POSE_TOPIC, self._vision_pose_callback, 10
        )
        # Bridge outputs use TRANSIENT_LOCAL — match it so we receive the
        # latched snapshot as soon as we subscribe.
        self.create_subscription(
            Detection3DArray, VISION_DETECTIONS_TOPIC,
            self._detection_callback, latched_qos
        )
        self.create_subscription(
            PoseArray, VISION_AVAILABLE_SLOTS_TOPIC,
            self._available_slots_callback, latched_qos
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
            speed=GRIPPER_SPEED,
            update_hz=GRIPPER_UPDATE_HZ,
        )

        # --- Perception cache ---
        self._latest_detections: Detection3DArray | None = None
        self._latest_vision_pose: PoseStamped | None = None
        self._latest_available_slots: PoseArray | None = None
        self._fresh_detections = False
        self._fresh_available_slots = False
        self._waiting_for_scan = False
        self._scan_timeout_timer = None
        self._placed_count = 0

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
        Receives commands from /brick_command:
            'start' / 'pause' / 'stop'   → state machine
            'gripper_open' / 'gripper_close' → manual gripper jog (only
            allowed outside RUNNING so the cycle isn't disrupted).
        """
        command = msg.data.strip().lower()

        if command in ('gripper_open', 'gripper_close'):
            self._handle_manual_gripper(command)
            return

        prev_state = self._sm.state

        valid = self._sm.handle_command(command)
        if not valid:
            self.get_logger().warn(
                f'Command "{command}" ignored in state {prev_state.value}'
            )
            return

        if self._sm.state != SystemState.RUNNING:
            self._waiting_for_scan = False
            self._cancel_scan_timeout()

        if self._sm.state == SystemState.RUNNING and prev_state != SystemState.RUNNING:
            self._start_pick_and_place_cycle()

    def _handle_manual_gripper(self, command: str) -> None:
        """Drive the gripper from the GUI without touching the cycle state."""
        if self._sm.state == SystemState.RUNNING:
            self.get_logger().warn(
                f'Manual gripper command "{command}" ignored while cycle is running.'
            )
            return
        if command == 'gripper_open':
            self._gripper.manual_open()
        else:
            self._gripper.manual_close()

    def _detection_callback(self, msg: Detection3DArray) -> None:
        """Caches the latest brick detections from the perception subsystem."""
        self._latest_detections = msg
        if self._waiting_for_scan:
            self._fresh_detections = True
        self.get_logger().info(
            f'Received {len(msg.detections)} brick detection(s) from perception.'
        )
        self._try_process_fresh_scan()

    def _available_slots_callback(self, msg: PoseArray) -> None:
        """Caches the latest open drop-zone slots from brick_vision."""
        self._latest_available_slots = msg
        if self._waiting_for_scan:
            self._fresh_available_slots = True
        self.get_logger().info(
            f'Received {len(msg.poses)} available drop-zone slot(s) from perception.'
        )
        self._try_process_fresh_scan()

    def _vision_pose_callback(self, msg: PoseStamped) -> None:
        """Caches the latest best-brick pose published by brick_vision."""
        self._latest_vision_pose = msg
        brick = brick_from_pose_stamped(msg)
        self.get_logger().info(
            'Received brick_vision pose: '
            f'frame={msg.header.frame_id or "unknown"} '
            f'pos=({brick.x:.3f}, {brick.y:.3f}, {brick.z:.3f}) '
            f'theta={math.degrees(brick.theta):.1f} deg'
        )

    # ------------------------------------------------------------------ #
    # Pick-and-place cycle                                                 #
    # ------------------------------------------------------------------ #

    def _bricks_to_use(self, allow_fallback: bool = True) -> list[Brick]:
        """
        Returns the brick list for this cycle.

        Uses real perception data if available, otherwise falls back
        to MOCK_BRICKS for testing without the camera.
        """
        if self._latest_detections is not None:
            bricks = bricks_from_detections(self._latest_detections)
            self.get_logger().info(
                f'Using {len(bricks)} brick(s) from perception.'
            )
            return bricks
        if not allow_fallback:
            return []
        if self._latest_vision_pose is not None:
            self.get_logger().info(
                'Using latest brick_vision pose. Colour/size are unavailable '
                'on /brick_detector/brick_pose and will be marked unknown.'
            )
            return [brick_from_pose_stamped(self._latest_vision_pose)]
        self.get_logger().warn(
            'No perception data — falling back to MOCK_BRICKS.'
        )
        return list(MOCK_BRICKS)

    def _slots_to_use(self, allow_fallback: bool = True) -> list[PlacementSlot]:
        """
        Returns the current open drop-zone slots.

        Uses brick_vision's /brick_detector/available_slots when present,
        otherwise falls back to the calibrated/static slots for bench tests.
        """
        if self._latest_available_slots is not None:
            slots = slots_from_pose_array(self._latest_available_slots)
            self.get_logger().info(
                f'Using {len(slots)} open slot(s) from brick_vision.'
            )
            return slots
        if not allow_fallback:
            return []

        self.get_logger().warn(
            'No available_slots from brick_vision — falling back to PLACEMENT_SLOTS.'
        )
        return list(PLACEMENT_SLOTS)

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

    def _target_pose(self, x: float, y: float, z: float, theta: float) -> Pose:
        """Raw object/slot pose for MTC pick-and-place planning."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.z = math.sin(theta / 2.0)
        pose.orientation.w = math.cos(theta / 2.0)
        return pose

    def _start_pick_and_place_cycle(self) -> None:
        """
        Starts the scan-pick-place loop.

        MTC mode does one brick per scan:
          scan -> choose nearest eligible brick + nearest open slot -> motion
          -> MTC returns to camera_home -> scan again.
        """
        if self._sm.state != SystemState.RUNNING:
            return

        self._placed_count = 0

        if MOTION_MODE == 'mtc':
            self._request_fresh_scan()
            return

        # Legacy stepwise mode uses the currently cached brick list.
        bricks = sort_by_distance(self._bricks_to_use(), ROBOT_BASE)
        if not bricks:
            self.get_logger().error('No bricks available for pick-and-place.')
            self._sm.set_error()
            return

        self._brick_queue = bricks
        self._queue_index = 0

        for line in format_sorted_summary(bricks, ROBOT_BASE):
            self.get_logger().info(line)

        self._phase = Phase.PICK_APPROACH
        self._execute_phase()

    def _request_fresh_scan(self) -> None:
        """Wait for the next brick_vision snapshot.

        First call after entering RUNNING bootstraps the cycle by publishing
        /snapshot_trigger. Every subsequent cycle is triggered by the motion
        node once it finishes a pick-and-place task and returns to camera_home.
        """
        if self._sm.state != SystemState.RUNNING:
            return

        if self._placed_count >= MAX_BRICKS_PER_RUN:
            self.get_logger().warn(
                f'Stopping after MAX_BRICKS_PER_RUN={MAX_BRICKS_PER_RUN}.'
            )
            self._sm.set_completed()
            return

        self._waiting_for_scan = True
        self._fresh_detections = False
        self._fresh_available_slots = False
        self._latest_detections = None
        self._latest_available_slots = None

        # Scan timeout removed — wait indefinitely for fresh detections.
        self._cancel_scan_timeout()

        if self._placed_count == 0:
            self.get_logger().info(
                f'Bootstrapping cycle: publishing first /snapshot_trigger on '
                f'{VISION_SCAN_TRIGGER_TOPIC}.'
            )
            self._scan_trigger_pub.publish(Empty())
        else:
            self.get_logger().info(
                'Waiting for next /snapshot_trigger from motion node…'
            )

    def _try_process_fresh_scan(self) -> None:
        """Proceed once the current scan has both detections and available slots."""
        if (
            not self._waiting_for_scan
            or not self._fresh_detections
            or not self._fresh_available_slots
            or self._sm.state != SystemState.RUNNING
        ):
            return

        self._waiting_for_scan = False
        self._cancel_scan_timeout()

        bricks = sort_by_distance(self._bricks_to_use(allow_fallback=False), ROBOT_BASE)
        if not bricks:
            self.get_logger().info(
                f'No bricks in latest scan. Placed {self._placed_count} brick(s); cycle complete.'
            )
            self._sm.set_completed()
            return

        for line in format_sorted_summary(bricks, ROBOT_BASE):
            self.get_logger().info(line)

        self._publish_mtc_pick_place_pair(bricks, allow_slot_fallback=False)

    def _on_scan_timeout(self) -> None:
        """Stop the loop if vision does not answer a scan request."""
        self._cancel_scan_timeout()
        if not self._waiting_for_scan or self._sm.state != SystemState.RUNNING:
            return
        self._waiting_for_scan = False
        self.get_logger().error(
            'Timed out waiting for fresh brick_vision detections and available_slots.'
        )
        self._sm.set_error()

    def _cancel_scan_timeout(self) -> None:
        if self._scan_timeout_timer is not None:
            self._scan_timeout_timer.cancel()
            self.destroy_timer(self._scan_timeout_timer)
            self._scan_timeout_timer = None

    def _publish_mtc_pick_place_pair(
        self,
        bricks: list[Brick],
        allow_slot_fallback: bool = True,
    ) -> None:
        """Select one brick and one open slot, then publish the MTC contract."""
        brick = choose_nearest_eligible_brick(
            bricks,
            ROBOT_BASE,
            allowed_colours=ELIGIBLE_COLOURS,
            allowed_sizes=ELIGIBLE_SIZES,
            min_confidence=MIN_DETECTION_CONFIDENCE,
        )
        if brick is None:
            self.get_logger().error(
                'No brick met the configured criteria: '
                f'colours={ELIGIBLE_COLOURS or "any"}, '
                f'sizes={ELIGIBLE_SIZES or "any"}, '
                f'min_confidence={MIN_DETECTION_CONFIDENCE:.2f}'
            )
            self._sm.set_error()
            return

        slot = choose_nearest_slot(
            self._slots_to_use(allow_fallback=allow_slot_fallback),
            brick,
        )
        if slot is None:
            self.get_logger().error('No open drop-zone slot available.')
            self._sm.set_error()
            return

        self.get_logger().info(
            'Selected pick/place pair: '
            f'brick {brick.colour} {brick.size} '
            f'at ({brick.x:.3f}, {brick.y:.3f}, {brick.z:.3f}) '
            f'-> {slot.label} at ({slot.x:.3f}, {slot.y:.3f}, {slot.z:.3f})'
        )
        self._motion.send_pick_place_goal(
            self._target_pose(brick.x, brick.y, brick.z, brick.theta),
            self._target_pose(slot.x, slot.y, slot.z, slot.theta),
            completion_delay_s=MTC_MOTION_COMPLETION_DELAY_S,
        )

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
            if MOTION_MODE == 'mtc':
                self.get_logger().error('MTC pick/place command failed — stopping cycle.')
            else:
                self.get_logger().error(
                    f'Step failed during {self._phase.value} of brick '
                    f'{self._queue_index + 1}/{len(self._brick_queue)} '
                    f'— stopping cycle.'
                )
            self._sm.set_error()
            return

        if MOTION_MODE == 'mtc':
            self._placed_count += 1
            self.get_logger().info(
                f'MTC pick/place assumed complete for brick {self._placed_count}; requesting next scan.'
            )
            self._request_fresh_scan()
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
