"""
system.launch.py — bring up the Brick Pick'n'Place application stack,
assuming the UR driver and MoveIt+RViz are ALREADY running.

Launch sequence (relative to invocation):
  T+0    brick_vision (brick_detector)
  T+5s   frame_transform_node (camera → base_link bridge)
  T+10s  ur3e_motion_mtc
  T+25s  brick_interaction_node (state machine)
  T+30s  brick_gui_node
  T+33s  voice_interface/voice_input_node
  T+36s  voice_interface/command_parser_node

Run the UR driver and MoveIt+RViz in their own terminals BEFORE invoking
this launch file. The delays are time-based, not event-based — ROS launch
cannot block on a long-running node being "fully initialised". Increase
the per-stage delay if your machine is slow or you see startup
race-condition warnings.

Launch:
    ros2 launch brick_interaction system.launch.py
    ros2 launch brick_interaction system.launch.py voice:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ── Arguments ────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('voice',          default_value='true'),
        # Per-stage start delays (seconds from launch start).
        DeclareLaunchArgument('t_vision',       default_value='0.0'),
        DeclareLaunchArgument('t_bridge',       default_value='5.0'),
        DeclareLaunchArgument('t_motion',       default_value='10.0'),
        DeclareLaunchArgument('t_interaction',  default_value='25.0'),
        DeclareLaunchArgument('t_gui',          default_value='30.0'),
        DeclareLaunchArgument('t_voice_in',     default_value='33.0'),
        DeclareLaunchArgument('t_voice_parser', default_value='36.0'),
    ]

    voice = LaunchConfiguration('voice')

    # ── T+0: brick_vision ────────────────────────────────────────────────
    vision = TimerAction(
        period=LaunchConfiguration('t_vision'),
        actions=[Node(
            package='brick_vision',
            executable='brick_detector',
            output='screen',
        )],
    )

    # ── T+5s: frame_transform_node (bridge) ─────────────────────────────
    bridge = TimerAction(
        period=LaunchConfiguration('t_bridge'),
        actions=[Node(
            package='brick_interaction',
            executable='frame_transform_node',
            output='screen',
        )],
    )

    # ── T+10s: ur3e_motion_mtc ──────────────────────────────────────────
    motion = TimerAction(
        period=LaunchConfiguration('t_motion'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur3e_motion_mtc'),
                    'launch', 'ur3e_motion_mtc.launch.py',
                ])
            ]),
        )],
    )

    # ── T+25s: brick_interaction_node (state machine) ───────────────────
    interaction = TimerAction(
        period=LaunchConfiguration('t_interaction'),
        actions=[Node(
            package='brick_interaction',
            executable='brick_interaction_node',
            output='screen',
        )],
    )

    # ── T+30s: brick_gui_node ───────────────────────────────────────────
    gui = TimerAction(
        period=LaunchConfiguration('t_gui'),
        actions=[Node(
            package='brick_gui',
            executable='brick_gui_node',
            output='screen',
        )],
    )

    # ── T+33s, T+36s: voice interface ───────────────────────────────────
    voice_input = TimerAction(
        period=LaunchConfiguration('t_voice_in'),
        actions=[Node(
            package='voice_interface',
            executable='voice_input_node',
            output='screen',
            condition=IfCondition(voice),
        )],
    )

    voice_parser = TimerAction(
        period=LaunchConfiguration('t_voice_parser'),
        actions=[Node(
            package='voice_interface',
            executable='command_parser_node',
            output='screen',
            condition=IfCondition(voice),
        )],
    )

    return LaunchDescription(args + [
        vision,
        bridge,
        motion,
        interaction,
        gui,
        voice_input,
        voice_parser,
    ])
