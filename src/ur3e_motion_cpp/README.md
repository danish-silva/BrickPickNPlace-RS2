Terminal 1:
source ~/ws_moveit2/install/setup.bash
source ~/ws_moveit2/config_override/setup_override.bash
ros2 run ur_client_library start_ursim.sh -m ur3e

Terminal 2:
cd ~/ws_moveit2
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
source ~/ws_moveit2/config_override/setup_override.bash
ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur3e   robot_ip:=192.168.56.101   launch_rviz:=false

Terminal 3:
cd ~/ws_moveit2
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
source ~/ws_moveit2/config_override/setup_override.bash
ros2 launch ur_moveit_config ur_moveit.launch.py   ur_type:=ur3e   launch_rviz:=true

Terminal 4:
cd ~/ws_moveit2
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
source ~/ws_moveit2/config_override/setup_override.bash
ros2 launch ur3e_motion_cpp move_to_position.launch.py


Test Publish:
ros2 topic pub --once /ordered_pose_array std_msgs/msg/Float64MultiArray "{
  data: [0.3, 0.25, 0.3, 3.14159, 0.0, 0.0,
         0.3, 0.25, 0.2, 3.14159, 0.0, 0.0,
         0.3, 0.25, 0.3, 3.14159, 0.0, 0.0,
        -0.3, 0.25, 0.3, 3.14159, 0.0, 0.0]
}"

