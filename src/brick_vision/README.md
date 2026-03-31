# brick_vision -- Perception & Mapping

ROS 2 package for detecting LEGO bricks using an **Intel RealSense D435i** depth camera and **OpenCV**. Identifies bricks by their stud (peg) pattern, classifies colour, and outputs 3D pose estimates in the camera frame.

---

## How It Works

The detector uses a **peg-based** approach rather than simple shape matching, which eliminates false positives from random rectangular objects:

```
Camera Frame ──> Detect Circular Studs ──> Validate Size (15mm) ──> Group into Grid
                                                                         │
                 Output Brick Pose <── Identify Colour (HSV) <── Match Config (4x2)
```

### Detection Pipeline

1. **Capture** -- RealSense streams aligned colour + depth at 640x480 @ 30fps
2. **Workspace ROI** -- Crops to a calibrated region (saves on first run)
3. **Stud detection** -- Adaptive threshold + contour analysis finds circular blobs
4. **Size validation** -- Each blob's real-world diameter is checked against 15mm using depth data
5. **Spatial grouping** -- Studs spaced ~25mm apart (centre-to-centre) are grouped via BFS
6. **Brick classification** -- Groups are matched to known configs (currently 4x2 = 8 studs)
7. **Colour identification** -- HSV sampling around each stud determines brick colour
8. **3D pose output** -- Centre position deprojected to camera-frame metres, orientation from the stud cluster

### Supported Colours

| Colour | Detection Method |
|--------|-----------------|
| Red | HSV hue 0-10 + 170-179 |
| Orange | HSV hue 11-25 |
| Yellow | HSV hue 26-34 |
| Green | HSV hue 35-85 |
| Blue | HSV hue 86-130 |
| Purple | HSV hue 131-169 |
| Black | Low value (V < 60) |

> White is intentionally excluded -- the workspace surface is white.

### Brick Configurations

Currently supports **4x2** bricks (100mm x 50mm, 8 studs). Additional sizes can be added to the `BRICK_CONFIGS` list in `brick_detector.py`.

---

## Package Structure

```
brick_vision/
├── brick_vision/
│   ├── __init__.py
│   ├── brick_detector.py                  # Main detector (peg-based)
│   └── brick_detector_dimension_based.py  # Backup: dimension-based detector
├── config/
│   └── workspace_calibration.json         # Saved workspace ROI + surface depth
├── launch/
│   └── brick_detection.launch.py          # ROS 2 launch file
├── test/
│   └── test_copyright.py
├── resource/
│   └── brick_vision
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Installation

```bash
# System dependencies
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-geometry-msgs

# Python dependencies
pip install opencv-python pyrealsense2 numpy
```

---

## Usage

### Standalone (no ROS 2 required)

```bash
python3 src/brick_vision/brick_vision/brick_detector.py
```

On first run, you'll be prompted to **calibrate the workspace**:
1. A camera feed window opens
2. Click and drag to draw a rectangle around your workspace area
3. Press **Enter** to confirm (or **Esc** to skip)
4. Calibration is saved to `config/workspace_calibration.json`

### Keyboard Controls (standalone mode)

| Key | Action |
|-----|--------|
| `q` | Quit |
| `c` | Re-calibrate workspace ROI |
| `s` | Save a snapshot (PNG) |

### ROS 2 Node

```bash
# After colcon build + source install/setup.bash
ros2 run brick_vision brick_detector

# Or via launch file
ros2 launch brick_vision brick_detection.launch.py
```

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/detection_image` | `sensor_msgs/Image` | Annotated camera feed with detections drawn |
| `~/brick_pose` | `geometry_msgs/PoseStamped` | 3D pose of the highest-confidence brick (camera frame) |

---

## Detection Output

Each detected brick returns:

```python
{
    "center_px":    (320, 240),           # pixel coordinates
    "center_3d":    [0.05, -0.02, 0.35],  # metres, camera frame
    "size_px":      (80, 40),             # bounding box pixels
    "size_m":       (0.098, 0.049),       # real-world metres
    "angle":        45.0,                 # degrees
    "colour":       "orange",             # detected colour
    "colour_bgr":   (0, 140, 255),        # for visualisation
    "stud_count":   8,                    # number of studs found
    "brick_config": (4, 2),               # matched configuration
    "confidence":   0.92,                 # 0-1
}
```

---

## Tuning Parameters

Key constants at the top of `brick_detector.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `STUD_DIAMETER_M` | 0.015 | Expected stud diameter (metres) |
| `STUD_SPACING_M` | 0.024 | Centre-to-centre stud spacing (metres) |
| `STUD_DIAM_TOL` | 0.50 | Diameter tolerance (50%) |
| `STUD_SPACING_TOL` | 0.45 | Spacing tolerance for grouping (45%) |
| `MIN_CIRCULARITY` | 0.55 | Minimum blob circularity (1.0 = perfect circle) |
| `BLACK_V_MAX` | 60 | HSV value ceiling for black detection |

If bricks aren't being detected, try:
- **Increase `STUD_DIAM_TOL`** if studs are being rejected
- **Lower `MIN_CIRCULARITY`** if the camera angle makes studs look elliptical
- **Adjust `BLACK_V_MAX`** if black bricks are missed under bright lighting
- **Press `c`** to re-calibrate if the camera has moved

---

## Backup Detector

The file `brick_detector_dimension_based.py` is a backup that uses **contour dimensions** (aspect ratio + metric size) instead of stud detection. It's kept for reference but is less robust against noise.
