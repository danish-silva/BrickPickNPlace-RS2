# LeBrick n' Place

A vision-guided pick-and-place system that uses the UR3e robotic arm to detect, sort, and arrange LEGO bricks by colour and size into predefined designs.

> **41069 Robotics Studio 2** -- University of Technology Sydney  
> Team: **LeBrick James**  
> Client: Isira Wijegunawardana | Coach: Katie Powell

| Name | Student ID | Role |
|------|-----------|------|
| Danish Silva | 24638001 | Perception & Mapping |
| Hari Mahadevan | 24801045 | Interaction & Execution |
| Benjamin Costarella | 24834727 | Motion Planning & Control |
| Dheeraj Panjwani | 14275073 | Voice Interface & HRI |

---

## Project Overview

LeBrick n' Place is a ROS2-based vision-guided pick-and-place system that uses the UR3e robotic arm to detect, sort, and arrange custom LEGO bricks onto the placement board. Utilising an overhead Intel RealSense D435i depth camera, the system identifies the bricks colour, size, and pose, allowing the UR3e to pick-and-place them in the users desired arrangement. Users interact through a GUI that supports colour sorting, size sorting, horizontal/vertical placement, and preset build designs.

### Subsystems

| Subsystem | Package | Lead | Description |
|-----------|---------|------|-------------|
| Perception & Mapping | `brick_vision` | Danish Silva | Brick detection, colour/size classification, and 3D pose estimation via RealSense + OpenCV |
| Motion Planning & Control | `ur3e_motion_mtc` | Benjamin Costarella | Performs Inverse Kinematics, trajectory planning, singularity & obstacle avoidance using MoveIt2 to plan and execute pick-and-place tasks |
| Interaction & Execution | `brick_interaction` & `brick_gui` | Hari Mahadevan | Task sequencing, state management, GUI coordination, and error recovery |
| Voice Interface | Task Level Control via Voice Interface| Dheeraj Panjwani | Voice commands via microphone for system control and task selection |

### System Architecture

```
+---------------------+       +-------------------------+       +---------------------+
|   Perception &      | ----> |   Interaction &         | ----> |   Motion Planning   |
|   Mapping           |       |   Execution             |       |   & Control         |
|   (brick_vision)    |       |   (brick_interaction)   |       |   (ur3e_motion)     |
+---------------------+       +-------------------------+       +---------------------+
   Intel RealSense               GUI + Task Sequencing             UR3e + MoveIt2
   OpenCV + Depth                 Behaviour Tree                  Task Constructor
                                         ^
                                         |
                               +---------------------+
                               |   Voice Interface   |
                               | (Microphone + ROS2) |
                               +---------------------+
```

---

## Repository Structure (NOT SURE IF NEEDED FOR DOCUMENTATION)

```
BrickPickNPlace-RS2/
├── src/
│   ├── brick_vision/          # Perception & mapping (RealSense + OpenCV)
│   ├── brick_gui/             # User interface
│   ├── brick_interaction/     # Task sequencing & execution logic
│   └── ur3e_motion_mtc/       # Motion planning (MoveIt2 Task Constructor)
├── .gitignore
└── README.md
```

---

## Dependencies

### Hardware

| Component | Version |
|---|---|
| Robot | Universal Robots UR3e |
| Gripper | OnRobot RG2 |
| Camera | Intel RealSense D435i |
| LEGO Bricks | LeBrick n' Place Custom LEGO Board |
| LEGO Board | LeBrick n' Place Custom LEGO Bricks |

### Software

| Component | Version |
|---|---|
| Operating System | Ubuntu 22.04 |
| ROS2 | Humble Hawksbill |
| MoveIt2 | 2.5.9 |
| MoveIt Task Constructor | 0.1.3 |
| RViz2 | 11.2.26 |
| Python | 3.10+ |
| C++ | 17 |

---

## Installation

### Hardware 

| Component | Details |
|-----------|---------|
| Robot | UR3e with gripper attachment |
| Camera | Intel RealSense D435i, mounted overhead on a fixed stand |
| Workspace | White LEGO base plate |
| Bricks | Large LEGO bricks (4x2 = 100mm x 50mm, 15mm studs) |
| Voice | Computor/Headphones with Microphone |

**If we have any pics of the workspace setup would be good**

### Software 

```bash
# ROS 2 + MoveIt2
sudo apt install ros-humble-moveit ros-humble-ur-robot-driver ros-humble-cv-bridge

# Python packages
pip install opencv-python pyrealsense2 numpy

# Clone
git clone https://github.com/DanishUTS/BrickPickNPlace-RS2.git
cd BrickPickNPlace-RS2

# Build
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Install Build Tools

```bash
sudo apt update

sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-moveit-task-constructor-*
```

### Initialise rosdep

```bash
sudo rosdep init
rosdep update
```

### Install Workspace Dependencies

```bash
cd ~/ws_moveit2

rosdep install --from-paths src --ignore-src -r -y
```

---

## Launch Commands

**Terminal 1 — UR driver (connect to physical robot):**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  robot_ip:=0.0.0.0 \
  launch_rviz:=false
```

**Terminal 1 — UR driver (connect to Fake Hardware):**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=true launch_rviz:=false
```

**Terminal 2 — MoveIt + RViz:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2 launch_rviz:=true
```

**Terminal 3 - Brick Detection**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run brick_vision brick_detector
```

**Terminal 4 - Brick Interaction**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run brick_interaction brick_interaction_node
```

**Terminal 5 - GUI**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run brick_gui brick_gui_node
```

**Terminal 6 — Launch Motion Planning and Control Node**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3e_motion_mtc ur3e_motion_mtc.launch.py
```
**Terminal 7 — Voice Interface: Voice Input Node**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run voice_interface voice_input_node
```

**Terminal 8 — Voice Interface: Command Parser Node**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run voice_interface command_parser_node
```
**Terminal 9 — Voice Interface: System Command Listener Node**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run voice_interface system_command_listener
```
**Terminal 10 — Voice Interface: Reset Executor Node**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run voice_interface reset_executor_node
```

### Expected Outcome



---

## Troubleshooting & FAQs


---

