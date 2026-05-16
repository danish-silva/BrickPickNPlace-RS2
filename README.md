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

## Voice Interface Subsystem

The Voice Interface subsystem provides task-level control of the robot through voice commands via microphone or keyboard input. It translates spoken or typed commands into ROS2 messages published to the shared `/brick_command` topic, which is consumed by the Interaction & Execution subsystem.

> **Note:** `reset_executor_node` is a **temporary execution layer** used for independent subsystem testing. It directly sends joint trajectories to the UR3e until full integration with `brick_interaction` is complete.

### Nodes

| Node | File | Description |
|------|------|-------------|
| `voice_input_node` | `voice_input_node.py` | Captures microphone or keyboard input and publishes raw text to `/voice_command_raw` |
| `command_parser_node` | `command_parser_node.py` | Subscribes to `/voice_command_raw`, normalises and maps input, publishes parsed commands to `/brick_command` |
| `system_command_listener` | `system_command_listener.py` | Monitors `/brick_command` and related topics — used for integration debugging and status logging |
| `reset_executor_node` | `reset_executor_node.py` | Temporary execution node — listens to `/brick_command` and directly executes joint trajectories on the UR3e for standalone testing |

### Topics

| Topic | Type | Publisher | Subscriber(s) | Description |
|-------|------|-----------|----------------|-------------|
| `/voice_command_raw` | `std_msgs/String` | `voice_input_node` | `command_parser_node` | Raw unprocessed voice or keyboard input |
| `/brick_command` | `std_msgs/String` | `command_parser_node` | `brick_interaction`, `reset_executor_node`, `system_command_listener` | Shared command bus used by all subsystems |
| `/build_request` | `std_msgs/String` | `command_parser_node` | `brick_interaction` | Preset build commands (BUILD_TOWER, BUILD_LINE) |
| `/block_sequence` | `std_msgs/String` | `command_parser_node` | `brick_interaction` | Custom brick placement sequence commands |
| `/joint_states` | `sensor_msgs/JointState` | UR driver | `reset_executor_node` | Current robot joint positions used for trajectory planning |
| `/system_status` | `std_msgs/String` | `brick_interaction` | `system_command_listener` | System status feedback (monitored for debugging) |

### Supported Commands

| Voice / Keyboard Input | Aliases | Published To | Published Value | Behaviour |
|------------------------|---------|--------------|-----------------|-----------|
| `start` | `begin`, `go` | `/brick_command` | `start` | Executes demo trajectory sequence |
| `pause` | `hold`, `wait` | `/brick_command` | `pause` | Cancels active trajectory |
| `stop` | `halt`, `cancel` | `/brick_command` | `stop` | Cancels trajectory and returns robot to home pose |
| `reset` | `home`, `go home` | `/brick_command` | `reset` | Returns robot to home pose |
| `build tower` | `tower` | `/build_request` | `BUILD_TOWER` | Triggers tower build preset |
| `build line` | `line` | `/build_request` | `BUILD_LINE` | Triggers line build preset |
| `sequence 1 2 3` | Spoken e.g. `sequence one two three` | `/block_sequence` | `1,2,3` | Custom brick placement sequence |

### Input Modes

| Mode | Key | Description |
|------|-----|-------------|
| Single microphone | `m` | Captures one spoken command, then returns to mode selection |
| Continuous microphone | `c` | Keeps listening and processing commands until Ctrl+C |
| Keyboard | `k` | Accepts typed commands — useful for testing without a microphone |

### Configuration

| Parameter | Default | File | Description |
|-----------|---------|------|-------------|
| `device_index` | `10` | `voice_input_node.py` | Microphone device index — change if mic not detected |
| `energy_threshold` | `100` | `voice_input_node.py` | Mic sensitivity — lower value = more sensitive |
| `pause_threshold` | `0.8` | `voice_input_node.py` | Seconds of silence before a command is finalised |
| `phrase_time_limit` | `4s` | `voice_input_node.py` | Maximum duration of a single spoken command |
| Home joints | `[0.8713, -1.4801, 0.1733, -0.2580, -1.5837, 5.5503]` | `reset_executor_node.py` | Robot home pose in radians (6 joints) |

---

## Repository Structure (NOT SURE IF NEEDED FOR DOCUMENTATION)

```
BrickPickNPlace-RS2/
├── src/
│   ├── brick_vision/          # Perception & mapping (RealSense + OpenCV)
│   ├── brick_gui/             # User interface
│   ├── brick_interaction/     # Task sequencing & execution logic
│   ├── ur3e_motion_mtc/       # Motion planning (MoveIt2 Task Constructor)
│   └── voice_interface/       # Voice & keyboard command interface
│       ├── voice_interface/
│       │   ├── voice_input_node.py
│       │   ├── command_parser_node.py
│       │   ├── system_command_listener.py
│       │   └── reset_executor_node.py
│       └── package.xml
├── .gitignore
└── README.md
```

---

## Dependencies

### Hardware

| Component | Details |
|---|---|
| Robot | Universal Robots UR3e |
| Gripper | OnRobot RG2 |
| Camera | Intel RealSense D435i |
| LEGO Bricks | LeBrick n' Place Custom LEGO Bricks |
| LEGO Board | LeBrick n' Place Custom LEGO Board |

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

### Hardware Setup

| Component | Details |
|-----------|---------|
| Robot | UR3e with gripper attachment |
| Camera | Intel RealSense D435i, mounted overhead on a fixed stand |
| Workspace | White LEGO base plate |
| Bricks | Large LEGO bricks (4x2 = 100mm x 50mm, 15mm studs) |
| Microphone | Computer headset or USB microphone |

> Add workspace setup photos here when available.

### Software

```bash
# ROS2 + MoveIt2 + UR driver
sudo apt install ros-humble-moveit ros-humble-ur-robot-driver ros-humble-cv-bridge

# Python packages
pip install opencv-python pyrealsense2 numpy

# Voice Interface dependencies
pip install SpeechRecognition pyaudio
```

> If `pip install pyaudio` fails, install the system dependency first:
> ```bash
> sudo apt install portaudio19-dev
> pip install pyaudio
> ```

```bash
# Clone the repository
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

**Terminal 3 — Brick Detection:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run brick_vision brick_detector
```

**Terminal 4 — Brick Interaction:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run brick_interaction brick_interaction_node
```

**Terminal 5 — GUI:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run brick_gui brick_gui_node
```

**Terminal 6 — Motion Planning and Control:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3e_motion_mtc ur3e_motion_mtc.launch.py
```

**Terminal 7 — Voice Interface: Voice Input Node:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run voice_interface voice_input_node
```

**Terminal 8 — Voice Interface: Command Parser Node:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run voice_interface command_parser_node
```

**Terminal 9 — Voice Interface: System Command Listener:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run voice_interface system_command_listener
```

**Terminal 10 — Voice Interface: Reset Executor Node:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run voice_interface reset_executor_node
```

---

## Expected Outcome

> Add photos/screenshots here — robot in action, RViz view, brick detection output, GUI.

---

## Troubleshooting & FAQs

### Voice Interface

**Microphone not detected / `device_index` error**

Run the following to list available audio devices and find your mic's index:
```bash
python3 -c "import speech_recognition as sr; print(sr.Microphone.list_microphone_names())"
```
Update `device_index` in `voice_input_node.py` to match your microphone.

---

**Command not recognised by Google Speech Recognition**

The system uses Google STT which requires an active internet connection.
Check your network is available and no firewall is blocking outbound requests.
Use keyboard mode `[k]` to bypass the mic and test the rest of the pipeline.

---

**Speech recognised but command not matched**

The node will log: `Speech recognized, but not matched to a valid command.`

Speak one of the supported commands clearly: `start`, `pause`, `stop`, `reset`, `build tower`, `build line`.
The node also accepts common aliases (e.g. `go`, `halt`, `home`) — see the Supported Commands table above.

---

**`reset_executor_node` — trajectory action server not available**

The UR driver (Terminal 1) must be running before launching `reset_executor_node`.
Check Terminal 1 for errors. If testing without a physical robot, ensure fake hardware is enabled:
```bash
use_fake_hardware:=true
```

---

**No joint states received**

`reset_executor_node` requires `/joint_states` from the UR driver to plan trajectories.
Confirm the driver launched successfully with:
```bash
ros2 topic echo /joint_states
```

---

**Commands not reaching `brick_interaction`**

Verify each step in the pipeline individually:
```bash
# Check voice input is publishing
ros2 topic echo /voice_command_raw

# Check parser is forwarding
ros2 topic echo /brick_command

# Check build commands
ros2 topic echo /build_request
```

---

**`pyaudio` install fails on Ubuntu**

Install the system dependency first:
```bash
sudo apt install portaudio19-dev
pip install pyaudio
```
