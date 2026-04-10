# launch/move_to_position.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess

def get_param_str(node, param):
    result = subprocess.run(
        ["ros2", "param", "get", node, param, "--hide-type"],
        capture_output=True, text=True
    )
    return result.stdout.strip()

def generate_launch_description():

    # Pull robot_description params from move_group
    robot_description       = get_param_str("/move_group", "robot_description")
    robot_description_sem   = get_param_str("/move_group", "robot_description_semantic")

    # Pull kinematics sub-params individually
    kin_solver          = get_param_str("/move_group", "robot_description_kinematics.ur_manipulator.kinematics_solver")
    kin_attempts        = get_param_str("/move_group", "robot_description_kinematics.ur_manipulator.kinematics_solver_attempts")
    kin_search_res      = get_param_str("/move_group", "robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution")
    kin_timeout         = get_param_str("/move_group", "robot_description_kinematics.ur_manipulator.kinematics_solver_timeout")

    return LaunchDescription([
        Node(
            package="ur3e_motion_cpp",
            executable="move_to_position",
            output="screen",
            parameters=[{
                "robot_description":          robot_description,
                "robot_description_semantic": robot_description_sem,
                "robot_description_kinematics.ur_manipulator.kinematics_solver":                   kin_solver,
                "robot_description_kinematics.ur_manipulator.kinematics_solver_attempts":          kin_attempts,
                "robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution": kin_search_res,
                "robot_description_kinematics.ur_manipulator.kinematics_solver_timeout":           kin_timeout,
            }],
        )
    ])