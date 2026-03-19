#!/usr/bin/env python3
"""
move_to_position.py
-------------------
Moves the UR3e from its current position to a predefined target joint position
using MoveIt2's MoveGroupInterface via the moveit_commander Python bindings.

Run AFTER all three launch terminals are active.

Usage:
    ros2 run ur3e_motion move_to_position
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

import math


# ---------------------------------------------------------------------------
# Target joint angles (in RADIANS) for each of the 6 UR3e joints:
#
#   Joint order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
#
# This target puts the arm in a clear, open "salute" pose — safe for ursim.
# Adjust these values to whatever position you want.
# ---------------------------------------------------------------------------
TARGET_JOINT_ANGLES = [
    math.radians(0),      # shoulder_pan   — straight ahead
    math.radians(-90),    # shoulder_lift  — arm pointing up
    math.radians(90),     # elbow          — elbow bent 90°
    math.radians(-90),    # wrist_1        — wrist angled down
    math.radians(-90),    # wrist_2        — wrist rotated
    math.radians(0),      # wrist_3        — no end-effector spin
]

PLANNING_GROUP = "ur_manipulator"   # MoveIt planning group name for UR robots
PLANNING_TIME  = 10.0               # seconds allowed for motion planning


class MoveToPosition(Node):

    def __init__(self):
        super().__init__("move_to_position")
        self.logger = get_logger("move_to_position")

    def run(self):
        # ------------------------------------------------------------------
        # 1. Initialise MoveItPy — this connects to the running MoveIt stack
        # ------------------------------------------------------------------
        self.logger.info("Initialising MoveItPy …")
        robot = MoveItPy(node_name="move_to_position_moveitpy")
        arm   = robot.get_planning_component(PLANNING_GROUP)

        self.logger.info(f"Connected to planning group: '{PLANNING_GROUP}'")

        # ------------------------------------------------------------------
        # 2. Print current joint state so you can see where it started
        # ------------------------------------------------------------------
        robot_model  = robot.get_robot_model()
        robot_state  = RobotState(robot_model)
        robot_state.update()  # sync to live joint state

        current = robot_state.get_joint_group_positions(PLANNING_GROUP)
        formatted = [f"{math.degrees(v):.1f}°" for v in current]
        self.logger.info(f"Current joints: {formatted}")

        # ------------------------------------------------------------------
        # 3. Set the goal — joint-space target
        # ------------------------------------------------------------------
        goal_state = RobotState(robot_model)
        goal_state.update()
        goal_state.set_joint_group_positions(PLANNING_GROUP, TARGET_JOINT_ANGLES)

        arm.set_start_state_to_current_state()
        arm.set_goal_state(robot_state=goal_state)

        target_fmt = [f"{math.degrees(v):.1f}°" for v in TARGET_JOINT_ANGLES]
        self.logger.info(f"Target joints:  {target_fmt}")

        # ------------------------------------------------------------------
        # 4. Plan
        # ------------------------------------------------------------------
        self.logger.info("Planning …")
        plan_result = arm.plan()

        if not plan_result:
            self.logger.error("Planning FAILED — no trajectory found.")
            self.logger.error(
                "Check that: URSim is running, robot is not in fault, "
                "and the target angles are within joint limits."
            )
            return

        self.logger.info("Plan found! Executing …")

        # ------------------------------------------------------------------
        # 5. Execute
        # ------------------------------------------------------------------
        robot.execute(plan_result.trajectory, controllers=[])

        self.logger.info("Motion complete ✓")

        # ------------------------------------------------------------------
        # 6. Print final joint state for confirmation
        # ------------------------------------------------------------------
        robot_state.update()
        final = robot_state.get_joint_group_positions(PLANNING_GROUP)
        formatted_final = [f"{math.degrees(v):.1f}°" for v in final]
        self.logger.info(f"Final joints:   {formatted_final}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MoveToPosition()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()