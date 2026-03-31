from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='brick_interaction',
            executable='brick_interaction_node',
            name='brick_interaction',
            output='screen',
        ),
    ])
