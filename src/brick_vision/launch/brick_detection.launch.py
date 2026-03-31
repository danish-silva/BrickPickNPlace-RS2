from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='brick_vision',
            executable='brick_detector',
            name='brick_detector',
            output='screen',
        ),
    ])
