#!/usr/bin/env python3
"""
Brick data model and sorting logic for the pick-and-place system.

Pure Python — no ROS imports. This keeps the sorting logic independently
testable and easy to extend.

Integration path (when Danish's perception node is ready):
    1. Subscribe to /perception/brick_list
    2. In the callback, convert each detected brick message into a Brick object
    3. Call sort_by_distance(bricks, ROBOT_BASE) to get the pick order
    See interaction_node.py _detection_callback for the integration point.
"""

import math
from dataclasses import dataclass


@dataclass
class Brick:
    """
    Represents a single detected LEGO brick.

    All coordinates are in the robot base frame (metres).
    theta is the yaw angle of the brick's long axis (radians).
    """
    x: float
    y: float
    z: float
    theta: float   # radians — orientation of brick's long axis
    colour: str    # 'red', 'blue', 'green', 'yellow'
    size: str      # '2x2', '2x4', '2x6', etc.


@dataclass
class PlacementSlot:
    """
    Predefined target location on the placement board.

    All coordinates are in the robot base frame (metres).
    theta is the desired yaw angle for the brick when placed (radians).
    label is a human-readable slot identifier used in log output.
    """
    x: float
    y: float
    z: float
    theta: float  # radians — desired orientation when placed
    label: str    # e.g. 'slot_1', 'slot_2'


# ---------------------------------------------------------------------------
# Reference point — robot arm base in the workspace frame
# TODO: Update to real UR3e base position once calibration is confirmed.
#       This is the point that all brick distances are measured from.
# ---------------------------------------------------------------------------
ROBOT_BASE = (0.0, 0.0)   # (x, y) in metres


# ---------------------------------------------------------------------------
# Mock brick data for Sprint 2 demo
# These are representative positions within the UR3e's reachable workspace
# (roughly 0.3–0.6 m from the base on a flat table).
#
# TODO: Replace MOCK_BRICKS with real perception output when integration is ready.
#       See brick_to_pose() below for how to convert a Brick into a Pose for motion.
# ---------------------------------------------------------------------------
MOCK_BRICKS = [
    Brick(x=0.45, y= 0.15, z=0.0, theta=0.0,           colour='red',    size='2x4'),
    Brick(x=0.30, y= 0.20, z=0.0, theta=math.pi / 4,   colour='blue',   size='2x2'),
    Brick(x=0.50, y=-0.10, z=0.0, theta=math.pi / 2,   colour='green',  size='2x4'),
    Brick(x=0.35, y= 0.30, z=0.0, theta=0.0,           colour='yellow', size='2x2'),
    Brick(x=0.55, y= 0.05, z=0.0, theta=math.pi / 6,   colour='red',    size='2x2'),
]


# ---------------------------------------------------------------------------
# Predefined placement slots — where picked bricks are placed on the board.
# Slots are arranged in a small grid on the far side of the workspace from
# the pick area (roughly x=0.10–0.20 m, y=0.35–0.45 m).
#
# TODO: Replace with real calibrated board positions once the physical
#       placement board is measured. Slot ordering should match the
#       intended build pattern.
# ---------------------------------------------------------------------------
PLACEMENT_SLOTS = [
    PlacementSlot(x=0.10, y=0.35, z=0.0, theta=0.0,           label='slot_1'),
    PlacementSlot(x=0.15, y=0.35, z=0.0, theta=0.0,           label='slot_2'),
    PlacementSlot(x=0.20, y=0.35, z=0.0, theta=math.pi / 2,   label='slot_3'),
    PlacementSlot(x=0.10, y=0.40, z=0.0, theta=0.0,           label='slot_4'),
    PlacementSlot(x=0.15, y=0.40, z=0.0, theta=math.pi / 2,   label='slot_5'),
]


# ===========================================================================
# VISION INTEGRATION — converts Detection3DArray → list[Brick]
# ===========================================================================
# Perception publishes: vision_msgs/Detection3DArray
#   - det.bbox.center.position   → x, y, z (metres, camera_color_optical_frame)
#   - det.bbox.center.orientation → quaternion
#   - det.bbox.size              → x (length), y (width), z (height) in metres
#   - det.results[0].hypothesis.class_id → colour string ("red", "blue", …)
#   - det.results[0].hypothesis.score    → detection confidence
# ===========================================================================


def _yaw_from_quaternion(q) -> float:
    """Extract yaw (z-axis rotation) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _size_label_from_bbox(size) -> str:
    """
    Convert bounding-box dimensions (metres) to a human-readable brick size.

    The perception bbox.size gives length (x) and width (y) in metres.
    LEGO stud pitch is ~8 mm, so divide each dimension by 0.008 and round
    to the nearest even integer to get stud counts.

    Falls back to 'unknown' if the dimensions don't map cleanly.
    """
    stud_x = round(size.x / 0.008)
    stud_y = round(size.y / 0.008)
    lo, hi = sorted([stud_x, stud_y])
    if lo >= 1 and hi >= 1:
        return f'{lo}x{hi}'
    return 'unknown'


def bricks_from_detections(msg) -> list[Brick]:
    """
    Convert a vision_msgs/Detection3DArray into a list of Brick objects.

    Args:
        msg: A vision_msgs/Detection3DArray published by the perception node.

    Returns:
        List of Brick objects with coordinates in the perception frame.
        NOTE: These may need a TF transform to the robot base frame
        before being used for motion planning.
    """
    bricks = []
    for det in msg.detections:
        if not det.results:
            continue
        colour = det.results[0].hypothesis.class_id
        p = det.bbox.center.position
        q = det.bbox.center.orientation
        bricks.append(Brick(
            x=p.x,
            y=p.y,
            z=p.z,
            theta=_yaw_from_quaternion(q),
            colour=colour,
            size=_size_label_from_bbox(det.bbox.size),
        ))
    return bricks


def _xy_distance(brick: Brick, reference: tuple) -> float:
    """Euclidean distance in the x-y plane between a brick and a reference point."""
    return math.hypot(brick.x - reference[0], brick.y - reference[1])


def sort_by_distance(bricks: list, reference: tuple = ROBOT_BASE) -> list:
    """
    Return a new list of Brick objects sorted closest-to-furthest from reference.

    Args:
        bricks:    List of Brick objects to sort.
        reference: (x, y) tuple — the point to measure distance from.
                   Defaults to ROBOT_BASE.

    Returns:
        A new sorted list (the input list is not modified).
    """
    return sorted(bricks, key=lambda b: _xy_distance(b, reference))


def format_sorted_summary(sorted_bricks: list, reference: tuple = ROBOT_BASE) -> list:
    """
    Return a list of human-readable strings describing the sorted pick order.
    Used for logging in interaction_node.py.

    Example output line:
        '  1. blue   2x2 | pos=( 0.300,  0.200) theta= 45.0° | dist=0.361m'
    """
    lines = [
        f'Sorted {len(sorted_bricks)} bricks by distance from arm base '
        f'({reference[0]:.1f}, {reference[1]:.1f}):'
    ]
    for i, brick in enumerate(sorted_bricks, start=1):
        dist = _xy_distance(brick, reference)
        theta_deg = math.degrees(brick.theta)
        lines.append(
            f'  {i}. {brick.colour:<6} {brick.size:<3} | '
            f'pos=({brick.x:6.3f}, {brick.y:6.3f}) '
            f'theta={theta_deg:5.1f}° | '
            f'dist={dist:.3f}m'
        )
    return lines
