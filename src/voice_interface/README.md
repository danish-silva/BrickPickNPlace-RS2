# Voice Interface Subsystem (UR3e)

This subsystem enables voice-based task-level control of a UR3e robotic arm using ROS2.

## Features
- Voice commands: Start, Stop, Reset
- Publishes system commands to `/system_command`
- Modular node-based architecture
- Integrated with MoveIt for robot motion

## Nodes

1. voice_input_node.py  
   - Captures microphone input  
   - Converts speech to text  

2. command_parser_node.py  
   - Parses commands (start/stop/reset)  
   - Publishes to `/system_command`  

3. system_command_listener.py  
   - Subscribes to `/system_command`  
   - Triggers actions  

4. reset_executor_node.py  
   - Moves UR3e to home position  

## Topics

- `/system_command` (std_msgs/String)

## How to Run

```bash
colcon build
source install/setup.bash
ros2 run voice_interface voice_input_node
ros2 run voice_interface command_parser_node
ros2 run voice_interface system_command_listener
ros2 run voice_interface reset_executor_node
