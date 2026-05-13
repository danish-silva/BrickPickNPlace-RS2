# Brick Interaction Subsystem

## Purpose

The `brick_interaction` package is the interaction and execution coordinator for the Brick Pick and Place system. It connects the GUI, vision, motion, and gripper subsystems into one controlled pick-and-place workflow.

The GUI is provided by the separate `brick_gui` package, but it is part of the same interaction/control section. The GUI publishes user commands to `/brick_command`, and `brick_interaction` receives those commands, runs the state machine, and publishes status updates back to the GUI on `/system_status`.

The subsystem is responsible for:

- Receiving user commands such as `start`, `pause`, `stop`, `gripper_open`, and `gripper_close`
- Managing the system state with a deterministic state machine
- Requesting fresh brick and slot data from the vision subsystem
- Selecting the nearest eligible brick and a suitable available placement slot
- Sending pick/place targets to the motion subsystem
- Sending gripper open/close commands to the RG2 gripper controller
- Publishing system status for the GUI

This package does not perform camera detection or MoveIt planning itself. Instead, it coordinates those subsystems through ROS 2 topics.

## Key Topics, Services, Files, and Nodes

### Main Nodes

```bash
brick_interaction
brick_gui
```

Run the interaction node directly:

```bash
ros2 run brick_interaction brick_interaction_node
```

Run the GUI directly:

```bash
ros2 run brick_gui brick_gui_node
```

### Main Files

| File | Purpose |
| --- | --- |
| `brick_interaction/interaction_node.py` | Main ROS 2 node. Handles commands, scan requests, pick/place selection, motion commands, gripper commands, and status publishing. |
| `brick_interaction/state_machine.py` | Pure Python state machine for `idle`, `running`, `paused`, `completed`, and `error`. |
| `brick_interaction/brick_sorter.py` | Brick and placement-slot data models, detection conversion, confidence/colour/size filtering, and nearest-brick/slot selection. |
| `brick_interaction/motion_client.py` | Publishes pick/place pose arrays to the motion subsystem on `/ordered_pose_array`. |
| `brick_interaction/gripper_client.py` | Publishes smooth RG2 finger-width commands to `/finger_width_controller/commands`. |
| `launch/brick_interaction.launch.py` | Launches the main `brick_interaction` node. |
| `../brick_gui/brick_gui/gui_node.py` | PyQt5 GUI node. Publishes Start/Pause/Stop commands and displays system status. |
| `../brick_gui/brick_gui/ui/main_window.ui` | Qt Designer layout for the GUI buttons, status badge, task label, and log window. |
| `../brick_gui/setup.py` | Installs the GUI node and `.ui` file so `ros2 run brick_gui brick_gui_node` can load the interface. |

### Services

This subsystem currently does not provide or call ROS services. Communication is topic-based.

## Inputs and Outputs

### Subscribed Inputs

| Topic | Type | Source | Purpose |
| --- | --- | --- | --- |
| `/brick_command` | `std_msgs/String` | GUI or manual CLI command | Receives `start`, `pause`, `stop`, `gripper_open`, and `gripper_close`. |
| `/brick_detector/detections` | `vision_msgs/Detection3DArray` | Vision subsystem | Provides detected bricks with pose, size, colour/class, and confidence. |
| `/brick_detector/available_slots` | `geometry_msgs/PoseArray` | Vision subsystem | Provides open placement/drop-zone slots. |
| `/brick_detector/brick_pose` | `geometry_msgs/PoseStamped` | Vision subsystem | Provides a fallback or best single brick pose. This topic does not include colour or size metadata. |

### Published Outputs

| Topic | Type | Destination | Purpose |
| --- | --- | --- | --- |
| `/snapshot_trigger` | `std_msgs/Empty` | Vision subsystem | Requests a fresh camera snapshot/scan. |
| `/ordered_pose_array` | `std_msgs/Float64MultiArray` | Motion subsystem | Sends pick and place target poses. |
| `/system_status` | `std_msgs/String` | GUI or monitoring tools | Publishes current state: `idle`, `running`, `paused`, `completed`, or `error`. |
| `/finger_width_controller/commands` | `std_msgs/Float64MultiArray` | RG2 gripper controller | Sends desired gripper finger width in metres. |

### GUI Interface

The GUI uses the same ROS interfaces as the command-line tests:

| GUI Control | Published Topic | Message |
| --- | --- | --- |
| Start button | `/brick_command` | `std_msgs/String` with `data: start` |
| Pause button | `/brick_command` | `std_msgs/String` with `data: pause` |
| Stop button | `/brick_command` | `std_msgs/String` with `data: stop` |

The GUI subscribes to `/system_status` and updates:

- Status badge: `idle`, `running`, `paused`, `completed`, or `error`
- Current task label
- Last update time
- Execution log

The current GUI does not expose `gripper_open` or `gripper_close` buttons. Those manual gripper commands can still be tested from the command line.

### Motion Output Format

In the current `mtc` mode, `/ordered_pose_array` contains exactly 12 values:

```text
[brick_x, brick_y, brick_z, brick_roll, brick_pitch, brick_yaw,
 slot_x,  slot_y,  slot_z,  slot_roll,  slot_pitch,  slot_yaw]
```

This is the contract expected by the `ur3e_motion_mtc` subsystem.

## How to Run or Test Independently

Before running commands, build and source the workspace:

```bash
colcon build --packages-select brick_interaction brick_gui
source install/setup.bash
```

Important: run each `ros2 topic echo` in its own separate terminal before publishing the command you want to observe. Most command and motion topics are not latched, so if you start echoing after a one-shot message has already been published, you may miss it.

### Terminal 1: Start the Interaction Node

```bash
ros2 run brick_interaction brick_interaction_node
```

Expected behaviour:

- The node starts as `brick_interaction`
- It publishes initial status `idle`
- It waits for GUI or CLI commands

### Terminal 2: Observe Scan Requests

Start this before sending `start`:

```bash
ros2 topic echo /snapshot_trigger
```

When the system starts, this terminal should print an empty message. This proves the interaction node is publishing scan requests to the vision subsystem.

### Terminal 3: Observe Motion Commands

Start this before sending `start`:

```bash
ros2 topic echo /ordered_pose_array
```

When the node receives fresh detections and slots, this terminal should print the selected pick/place pair. This proves the interaction node is publishing to the motion subsystem.

### Terminal 4: Observe Gripper Commands

Start this before testing manual gripper commands:

```bash
ros2 topic echo /finger_width_controller/commands
```

This proves the interaction node is publishing gripper width commands.

### Terminal 5: Start the GUI

```bash
ros2 run brick_gui brick_gui_node
```

Expected behaviour:

- The GUI window opens.
- The Start, Pause, and Stop buttons are visible.
- The status badge shows the latest system state.
- Clicking Start/Pause/Stop sends the matching command to the interaction node.

Simple GUI command check:

- Click **Start** and confirm the GUI status changes to `RUNNING` and Terminal 1 logs the `running` state.
- Click **Pause** while the system is running and confirm the GUI status changes to `PAUSED` and Terminal 1 logs the `paused` state.
- Click **Stop** and confirm the GUI status returns to `IDLE` and Terminal 1 logs the `idle` state.

### Terminal 6: Start Continuous Mock Available Slots

Use a continuous publisher instead of `--once` so the interaction node receives a fresh slot message after it enters the `running` state.

```bash
ros2 topic pub /brick_detector/available_slots geometry_msgs/msg/PoseArray "{poses: [{position: {x: 0.20, y: 0.40, z: 0.0}, orientation: {w: 1.0}}, {position: {x: 0.60, y: 0.40, z: 0.0}, orientation: {w: 1.0}}]}" --rate 1
```

This provides two open placement slots.

### Terminal 7: Start Continuous Mock Brick Detections

This example publishes two bricks. The blue brick is closer to `ROBOT_BASE = (0.0, 0.0)` than the red brick, so the interaction node should select the blue brick first.

```bash
ros2 topic pub /brick_detector/detections vision_msgs/msg/Detection3DArray "{detections: [{bbox: {center: {position: {x: 0.55, y: 0.25, z: 0.0}, orientation: {w: 1.0}}, size: {x: 0.032, y: 0.016, z: 0.012}}, results: [{hypothesis: {class_id: red, score: 0.90}}]}, {bbox: {center: {position: {x: 0.25, y: 0.05, z: 0.0}, orientation: {w: 1.0}}, size: {x: 0.032, y: 0.016, z: 0.012}}, results: [{hypothesis: {class_id: blue, score: 0.95}}]}]}" --rate 1
```

This proves the interaction node is subscribed to `/brick_detector/detections` and can process multiple candidate bricks.

The closest-brick check works because `brick_sorter.py` compares each brick's XY position to `ROBOT_BASE = (0.0, 0.0)`:

| Brick | Position | Distance from arm base | Expected order |
| --- | --- | --- | --- |
| Blue | `(0.25, 0.05)` | `sqrt(0.25^2 + 0.05^2) = 0.255 m` | 1st |
| Red | `(0.55, 0.25)` | `sqrt(0.55^2 + 0.25^2) = 0.604 m` | 2nd |

So the node should select the blue brick first. In Terminal 1, the node logs should show the blue brick above the red brick in the sorted summary. In Terminal 3, `/ordered_pose_array` should start with the blue brick pose:

```text
data:
- 0.25
- 0.05
- 0.0
- 0.0
- 0.0
- 0.0
...
```

The first six values are the selected brick pose: `x`, `y`, `z`, `roll`, `pitch`, and `yaw`. Since the blue brick is closer, the first two values should be approximately `0.25` and `0.05`, not `0.55` and `0.25`.

### Start the Cycle With the GUI

Once the observer terminals and mock vision publishers are running, click the **Start** button in the GUI.

Expected behaviour:

- Terminal 1 logs that the interaction node entered `running`.
- Terminal 2 receives a `/snapshot_trigger`.
- Terminal 3 receives an `/ordered_pose_array` motion command.
- The GUI status badge changes to `RUNNING`.

### Alternative: Start the Cycle Without the GUI

If you do not want to use the GUI, publish the same command manually:

```bash
ros2 topic pub --once /brick_command std_msgs/msg/String "{data: start}"
```

Expected behaviour:

- Terminal 1 logs that the interaction node entered `running`
- The node logs that it requested a fresh snapshot
- The node logs that it received available slots
- The node logs that it received brick detections
- The node prints a sorted brick summary
- The closest brick should appear first in the sorted summary
- The node logs the selected pick/place pair
- `/ordered_pose_array` receives 12 values

Expected proof in the terminals:

| Terminal | What it proves |
| --- | --- |
| Terminal 2, `/snapshot_trigger` | The node requests a fresh scan from vision. |
| Terminal 3, `/ordered_pose_array` | The node publishes a selected brick/slot pair to motion. |
| Terminal 5, GUI | The GUI sends commands and displays status. |
| Terminal 6, slots publisher | The node can subscribe to available placement slots. |
| Terminal 7, detections publisher | The node can subscribe to brick detections. |
| Terminal 1, node logs | The node sorted bricks and selected the nearest valid brick. |

After proving the motion command, send `stop` so the loop does not keep requesting scans:

```bash
ros2 topic pub --once /brick_command std_msgs/msg/String "{data: stop}"
```

Or click the **Stop** button in the GUI.

### Timeout Test

To prove failure handling, stop the mock vision publishers, then click Start in the GUI or send:

```bash
ros2 topic pub --once /brick_command std_msgs/msg/String "{data: start}"
```

If the vision topics do not respond, the node waits for `SCAN_TIMEOUT_S` seconds and then enters the `error` state.

Expected result: the GUI status changes to `ERROR`, and Terminal 1 logs the scan timeout.

This proves the subsystem can detect missing vision input and fail safely instead of continuing with stale data.

### Manual Gripper Test

Manual gripper commands are only accepted when the pick-and-place cycle is not running.

Start observing `/finger_width_controller/commands` in a separate terminal before sending these commands.

Close:

```bash
ros2 topic pub --once /brick_command std_msgs/msg/String "{data: gripper_close}"
```

Open:

```bash
ros2 topic pub --once /brick_command std_msgs/msg/String "{data: gripper_open}"
```

Expected behaviour:

- `gripper_open` publishes finger widths moving toward `GRIPPER_OPEN_WIDTH`
- `gripper_close` publishes finger widths moving toward `GRIPPER_CLOSE_WIDTH`
- Manual gripper commands do not advance the pick-and-place cycle

## Configurable Settings and Important Parameters

The current implementation uses constants in `interaction_node.py`. These are the main values to tune:

| Setting | Default | Meaning |
| --- | --- | --- |
| `Z_APPROACH` | `0.15` | Safe clearance height above a brick or slot in metres. |
| `Z_GRAB` | `0.005` | Grab/release height above the surface in metres. |
| `GRIPPER_OPEN_WIDTH` | `0.110` | RG2 fully open finger width in metres. |
| `GRIPPER_CLOSE_WIDTH` | `0.0` | RG2 closed finger width in metres. |
| `GRIPPER_SPEED` | `0.05` | Gripper finger travel speed in metres per second. |
| `GRIPPER_UPDATE_HZ` | `20.0` | Rate for publishing intermediate gripper widths. |
| `MOTION_MODE` | `'mtc'` | Uses the MTC pick/place interface. Legacy `'stepwise'` mode also exists in the code. |
| `MTC_MOTION_COMPLETION_DELAY_S` | `45.0` | Time to wait before assuming the MTC pick/place task is complete. |
| `SCAN_TIMEOUT_S` | `8.0` | Maximum time to wait for fresh vision detections and slots. |
| `MAX_BRICKS_PER_RUN` | `100` | Safety limit on bricks placed in one run. |
| `ELIGIBLE_COLOURS` | `[]` | Optional colour allow-list. Empty means any colour. |
| `ELIGIBLE_SIZES` | `[]` | Optional brick-size allow-list. Empty means any size. |
| `MIN_DETECTION_CONFIDENCE` | `0.30` | Minimum accepted detection confidence. |

`brick_sorter.py` also contains fallback/mock values:

| Setting | Purpose |
| --- | --- |
| `ROBOT_BASE` | XY reference point used for sorting bricks by distance. |
| `MOCK_BRICKS` | Representative mock brick list for testing without perception data. |
| `PLACEMENT_SLOTS` | Static fallback placement slots if no live slot data is available. |

Future improvement: expose these values through ROS parameters or a YAML configuration file so they can be tuned without editing source code.

## Expected Behaviour

When a `start` command is received:

1. The state machine transitions to `running`.
2. The node publishes `/snapshot_trigger`.
3. The node waits for fresh `/brick_detector/detections` and `/brick_detector/available_slots`.
4. Detections are converted into `Brick` objects.
5. Slots are converted into `PlacementSlot` objects.
6. Bricks are filtered by confidence, colour, and size if those filters are configured.
7. The nearest eligible brick is selected.
8. The nearest available slot is selected.
9. A pick/place pair is published on `/ordered_pose_array`.
10. After the configured MTC completion delay, the node requests another scan.
11. If no bricks remain, the state becomes `completed`.

Command behaviour:

| Current State | Command | Result |
| --- | --- | --- |
| `idle` | `start` | Begin scan and pick/place cycle. |
| `running` | `pause` | Transition to `paused`. |
| `running` | `stop` | Return to `idle`. |
| `paused` | `start` | Resume by entering `running`. |
| `paused` | `stop` | Return to `idle`. |
| `completed` | `start` | Start a new cycle. |
| `error` | `start` | Retry by starting a new cycle. |
| Any non-running state | `gripper_open` / `gripper_close` | Manually jog gripper. |

## Known Limitations and Assumptions

- The subsystem assumes that the vision coordinates are suitable for motion. Comments in the code note that a TF transform from `camera_color_optical_frame` to the robot base frame may be required once calibration is finalized.
- Motion completion is currently timer-based. `MotionClient` assumes the MTC task has completed after `MTC_MOTION_COMPLETION_DELAY_S`; it does not currently receive action feedback from the motion subsystem.
- Most configurable values are source-code constants rather than ROS parameters.
- `/brick_detector/brick_pose` is only a fallback/best-pose topic and does not include brick colour or size.
- In current `mtc` mode, the system sends one brick/slot pair per scan and then requests a new scan after the assumed motion completion.
- The node requires both fresh detections and fresh available slots after a scan trigger. If either is missing, it times out and enters `error`.
- The `stepwise` motion mode is present as legacy logic, but the current integrated mode is `mtc`.
- Static `MOCK_BRICKS` and `PLACEMENT_SLOTS` are useful for early testing, but final operation should use calibrated vision outputs.
- Manual gripper commands are ignored while the pick-and-place cycle is running to avoid disrupting the automated sequence.
- The GUI Pause button changes the interaction state and cancels scan waiting, but it does not cancel or pause an already-sent motion task because the current motion interface is publisher/timer-based rather than action-feedback-based.

## Troubleshooting

| Symptom | Likely Cause | What to Check |
| --- | --- | --- |
| Status becomes `error` after `start` | Vision did not publish both detections and available slots before timeout. | Echo `/brick_detector/detections`, `/brick_detector/available_slots`, and `/snapshot_trigger`. |
| No motion command appears | Missing detections, missing slots, or all detections filtered out. | Echo `/ordered_pose_array` and check node logs for filtering messages. |
| Gripper does not move | Gripper controller may not be running or topic name may not match. | Echo `/finger_width_controller/commands`. |
| Manual gripper command ignored | System is currently `running`. | Send `stop` first, then send `gripper_open` or `gripper_close`. |
| Pick/place target appears wrong | Input brick or slot coordinates may not match the motion subsystem's expected frame. | Check the values published on `/brick_detector/detections` and `/brick_detector/available_slots`. |
