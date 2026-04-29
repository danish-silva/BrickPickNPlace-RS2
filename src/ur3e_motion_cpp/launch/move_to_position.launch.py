from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import subprocess


def _coerce_param_value(value):
    if value is None:
        return None
    stripped = value.strip()
    if stripped == "":
        return None
    try:
        if "." not in stripped and "e" not in stripped.lower():
            return int(stripped)
        return float(stripped)
    except ValueError:
        return stripped


def get_param(node, param):
    result = subprocess.run(
        ["ros2", "param", "get", node, param, "--hide-type"],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        return None
    return _coerce_param_value(result.stdout)


def launch_setup(context, *args, **kwargs):
    planning_group = LaunchConfiguration("planning_group").perform(context)
    kinematics_group = LaunchConfiguration("kinematics_group").perform(context)

    # Pull robot_description params from the already-running move_group node.
    robot_description = get_param("/move_group", "robot_description")
    robot_description_sem = get_param("/move_group", "robot_description_semantic")
    if robot_description is None or robot_description_sem is None:
        raise RuntimeError(
            "Could not read robot_description from /move_group. "
            "Launch ur_onrobot_moveit_config first, then launch ur3e_motion_cpp."
        )

    parameters = {
        "planning_group": planning_group,
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_sem,
    }

    kinematics_keys = [
        "kinematics_solver",
        "kinematics_solver_attempts",
        "kinematics_solver_search_resolution",
        "kinematics_solver_timeout",
        "max_cache_size",
        "min_pose_distance",
        "min_joint_config_distance",
    ]
    for key in kinematics_keys:
        value = get_param(
            "/move_group",
            f"robot_description_kinematics.{kinematics_group}.{key}",
        )
        if value is not None:
            parameters[f"robot_description_kinematics.{kinematics_group}.{key}"] = value

    return [
        Node(
            package="ur3e_motion_cpp",
            executable="move_to_position",
            output="screen",
            parameters=[parameters],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "planning_group",
            default_value="ur_onrobot_manipulator",
            description="MoveIt planning group used by move_to_position.",
        ),
        DeclareLaunchArgument(
            "kinematics_group",
            default_value="ur_onrobot_manipulator",
            description="Group key under robot_description_kinematics.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
