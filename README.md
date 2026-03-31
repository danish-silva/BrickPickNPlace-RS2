# LeBrick n' Place

A vision-guided pick-and-place system that uses the **UR3e** robotic arm to detect, sort, and arrange LEGO bricks by colour and size into predefined designs.

> **41069 Robotics Studio 2** -- University of Technology Sydney  
> Team: **LeBrick James**  
> Client: Isira Wijegunawardana | Coach: Katie Powell

---

## Overview

The system uses an overhead **Intel RealSense D435i** depth camera to detect LEGO bricks on a white workspace, identifies their colour, size (by stud count), and orientation, then commands the UR3e to pick and place them according to user-selected arrangements. Users interact through a GUI that supports colour sorting, size sorting, horizontal/vertical placement, and preset build designs.

### System Architecture

```
+---------------------+       +-------------------------+       +---------------------+
|   Perception &      | ----> |   Interaction &         | ----> |   Motion Planning   |
|   Mapping           |       |   Execution             |       |   & Control         |
|   (brick_vision)    |       |   (brick_interaction)   |       |   (ur3e_motion)     |
+---------------------+       +-------------------------+       +---------------------+
   Intel RealSense               GUI + Task Sequencing            UR3e + MoveIt2
   OpenCV + Depth                 Behaviour Tree                  IK + Trajectories
                                         ^
                                         |
                               +---------------------+
                               |   Voice Interface   |
                               |   (ESP32 + ROS2)    |
                               +---------------------+
```

### Subsystems

| Subsystem | Package | Lead | Description |
|-----------|---------|------|-------------|
| Perception & Mapping | `brick_vision` | Danish Silva | Brick detection, colour/size classification, and 3D pose estimation via RealSense + OpenCV |
| Motion Planning & Control | `ur3e_motion` / `ur3e_motion_cpp` | Benjamin Costarella | Inverse kinematics, trajectory planning, singularity & obstacle avoidance using MoveIt2 |
| Interaction & Execution | `brick_interaction` | Hari Mahadevan | Task sequencing, state management, GUI coordination, and error recovery |
| Voice Interface | ESP32 + ROS Bridge | Dheeraj Panjwani | Voice commands via ESP32 microphone for system control and task selection |

---

## Repository Structure

```
BrickPickNPlace-RS2/
├── src/
│   ├── brick_vision/          # Perception & mapping (RealSense + OpenCV)
│   ├── brick_gui/             # User interface
│   ├── brick_interaction/     # Task sequencing & execution logic
│   ├── ur3e_motion/           # Motion planning (Python / MoveIt2)
│   └── ur3e_motion_cpp/       # Motion planning (C++ / MoveIt2)
├── .gitignore
└── README.md
```

---

## Prerequisites

- **Ubuntu 22.04** with **ROS 2 Humble**
- **MoveIt2** (system install)
- **Intel RealSense D435i** camera
- **Python 3.10+**

### Install Dependencies

```bash
# ROS 2 + MoveIt2
sudo apt install ros-humble-moveit ros-humble-ur-robot-driver ros-humble-cv-bridge

# Python packages
pip install opencv-python pyrealsense2 numpy
```

---

## Build & Run

```bash
# Clone
git clone https://github.com/DanishUTS/BrickPickNPlace-RS2.git
cd BrickPickNPlace-RS2

# Build
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Launch Individual Subsystems

```bash
# Brick detection (standalone, no ROS required)
python3 src/brick_vision/brick_vision/brick_detector.py

# Brick detection (ROS 2 node)
ros2 launch brick_vision brick_detection.launch.py

# Motion planning
ros2 launch ur3e_motion move_to_position.launch.py
```

---

## Hardware Setup

| Component | Details |
|-----------|---------|
| Robot | UR3e with gripper attachment |
| Camera | Intel RealSense D435i, mounted overhead on a fixed stand |
| Workspace | White LEGO base plate |
| Bricks | Large LEGO bricks (4x2 = 100mm x 50mm, 15mm studs) |
| Voice | ESP32 with microphone module |

---

## Team

| Name | Student ID | Role |
|------|-----------|------|
| Danish Silva | 24638001 | Perception & Mapping |
| Hari Mahadevan | 24801045 | Interaction & Execution |
| Benjamin Costarella | 24834727 | Motion Planning & Control |
| Dheeraj Panjwani | 14275073 | Voice Interface & HRI |