from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='brick_gui',
            executable='brick_gui_node',
            name='brick_gui',
            output='screen',
        ),
    ])
