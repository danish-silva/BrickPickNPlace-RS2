from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ur_moveit_config"),
                         "launch", "ur_moveit.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "launch_rviz": "false",
        }.items(),
    )

    move_node = Node(
        package="ur3e_motion",
        executable="move_to_position",
        output="screen",
    )

    return LaunchDescription([ur_moveit_launch, move_node])