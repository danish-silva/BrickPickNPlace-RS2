from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    start_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_onrobot_control"), "launch", "start_robot.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "onrobot_type": "rg2",
            "use_fake_hardware": "true",
            "robot_ip": "0.0.0.0",
            "initial_joint_controller": "joint_trajectory_controller",
            "launch_rviz": "false",
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_onrobot_moveit_config"), "launch", "ur_onrobot_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "onrobot_type": "rg2",
            "use_sim_time": "true",
            "launch_rviz": "true",
            "launch_servo": "false",
        }.items(),
    )

    arm_motion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur3e_motion_cpp"), "launch", "move_to_position.launch.py"]
            )
        ),
    )

    brick_interaction = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("brick_interaction"), "launch", "brick_interaction.launch.py"]
            )
        ),
    )

    return LaunchDescription(
        [
            start_robot,
            TimerAction(period=8.0, actions=[moveit]),
            TimerAction(period=15.0, actions=[arm_motion]),
            TimerAction(period=17.0, actions=[brick_interaction]),
        ]
    )
