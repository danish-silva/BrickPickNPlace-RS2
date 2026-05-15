# brick_vision — Perception & Mapping

ROS 2 package for detecting LEGO bricks and analysing a build plate using an **Intel RealSense D435i** depth camera and **OpenCV**. Identifies bricks by their stud pattern, classifies colour, estimates 3D pose, and finds available placement slots on a 12×14-stud build zone.

---

## What the node does

For each captured frame the pipeline runs three stages:

1. **Brick detection** — find 4×2 LEGO bricks in the workspace ROI, identify their colour, compute centre + yaw in 3D camera coordinates.
2. **Build-zone analysis** — locate the 14-row × 12-column stud grid on the build plate, decide which studs are free vs occupied, slide a 4×2 window over the grid to find legal placement slots (with a 1-stud gap rule).
3. **Publish results** — bricks, free studs, and available slots are all published on dedicated topics for the interaction node to consume.

```
Camera ── stud blob detection ── size validation ── grid grouping ── 4×2 match
                                                                       │
                                                                       ▼
                                                          colour + pose + confidence
                                                                       │
                                                                       ▼
Build-zone ──► studs_px (14×12) ──► occupancy (4×2 oriented-box test) ──► slot search
```

### Two operating modes

| Mode | When to use | Frame rate |
|---|---|---|
| `on_trigger` (default) | Production. Vision waits for an `std_msgs/Empty` on `/snapshot_trigger` (e.g. when the arm reaches home and the workspace is clear), takes 5 frames, keeps the best one, publishes once. Results are **latched** so late subscribers immediately get the most recent snapshot. | Event-driven |
| `continuous` | Debug / tuning. Runs detection at ~15 Hz the whole time. | ~15 Hz |

### Two-stage calibration

Both calibrations are interactive (clicks in the OpenCV window) and saved to `~/.brick_vision/workspace_calibration.json`.

| Calibration | Trigger | What you click |
|---|---|---|
| **Workspace ROI** | `c` key | Drag a rectangle around the table area the camera should look at. Surface depth is sampled inside the ROI. |
| **Build-zone grid** | `b` key | Click the *top-left* stud centre, then the *bottom-right* stud centre, of the 12×14 build plate. The 168-stud grid is linearly interpolated between them. |

---

## ROS 2 interface

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `mode` | `"on_trigger"` | `"on_trigger"` or `"continuous"` |
| `trigger_topic` | `"/snapshot_trigger"` | Topic name to subscribe to in `on_trigger` mode |
| `snapshot_frames` | `5` | Capture N frames per trigger, publish the best one |
| `show_preview` | `true` | Show the OpenCV window. Set `false` for headless runs |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `/snapshot_trigger` | `std_msgs/Empty` | (on_trigger mode only) Take one snapshot now. Topic name is overridable via the `trigger_topic` parameter. |

### Published topics

All published under the node namespace `/brick_detector/`. In `on_trigger` mode the four result topics use **`TRANSIENT_LOCAL`** QoS (latched, depth 1) — a subscriber that starts after a snapshot immediately receives the most recent result.

| Topic | Type | Purpose |
|---|---|---|
| `~/detection_image` | `sensor_msgs/Image` | Annotated camera frame: brick boxes + colour labels, build-zone perimeter, free studs (green dots), occupied studs (red X), available 4×2 slots (cyan rectangles), status line. Not latched. |
| `~/detections` | `vision_msgs/Detection3DArray` | **All** detected bricks (pickup + placed). Useful for rviz / debugging. One entry per brick, binding `pose + size + colour + confidence`. Pose orientation encodes yaw as a quaternion (roll/pitch = 0). Colour string is in `results[0].hypothesis.class_id`. |
| `~/pickup_detections` | `vision_msgs/Detection3DArray` | **Main consumer topic for the interaction node.** Only bricks whose pixel centre falls *outside* the calibrated build-zone rectangle — i.e. bricks waiting to be picked. Same message structure as `~/detections`. If the build zone isn't calibrated, this is identical to `~/detections`. |
| `~/placed_detections` | `vision_msgs/Detection3DArray` | Bricks already on the build plate (pixel centre inside the build-zone rectangle, ½-stud margin). Diagnostic only — the interaction node should ignore these. |
| `~/brick_pose` | `geometry_msgs/PoseStamped` | Convenience — first **pickup-side** brick only. Use `~/pickup_detections` for real consumption. |
| `~/free_studs` | `geometry_msgs/PoseArray` | Centre of every free stud on the 12×14 grid. Diagnostic. |
| `~/available_slots` | `geometry_msgs/PoseArray` | **Placement-target topic.** Midpoint of each legal 4×2 area (free + 1-stud gap respected). Orientation encodes the slot's yaw (long axis along X vs Y). |

All headers stamp `frame_id = "camera_color_optical_frame"` — consumers should transform to `base_link` via `tf2`.

---

## Brick detection details

### Pipeline

1. **Capture** — RealSense streams aligned colour + depth at 640×480 @ 30 fps.
2. **Workspace ROI** — Crop to the calibrated rectangle (border pixels excluded).
3. **HSV pre-segmentation** — Build colour masks for the supported colour set.
4. **Contour extraction + size check** — Keep contours whose oriented bounding box has the right aspect ratio (2:1 ± 35 %) and metric size (~100×50 mm ± 30 %).
5. **Stud verification (HoughCircles)** — Inside each candidate, find circular studs and check there are roughly 8 of them at the expected spacing.
6. **Pose + yaw** — Centre from contour moments, yaw from the oriented box, Z from median depth in a small window.
7. **Confidence** — `0.65 max` from colour+size, plus `0.35 bonus` if ≥ 2 verified studs were found (total ≤ 1.0).

### Supported colours

| Colour | HSV ranges |
|---|---|
| Red | hue 0–10 + 170–179 |
| Orange | hue 11–25 |
| Yellow | hue 26–34 |
| Green | hue 35–85 |
| Blue | hue 86–130 |
| Purple | hue 131–169 |
| Black | V < 60, S < 120 |

White is intentionally excluded — the workspace surface is white.

### Per-brick output dictionary (internal)

```python
{
    "center_px":    (320, 240),           # pixel coordinates
    "center_3d":    [0.05, -0.02, 0.35],  # metres, camera frame
    "size_px":      (80, 40),
    "size_m":       (0.098, 0.049),
    "angle":        45.0,                 # degrees, used as yaw
    "colour":       "orange",
    "colour_bgr":   (0, 140, 255),
    "stud_count":   8,
    "brick_config": (4, 2),
    "confidence":   0.92,
}
```

These get repackaged into `vision_msgs/Detection3DArray` when published.

---

## Build-zone analysis

### Geometry

- **Grid**: 12 columns × 14 rows of studs (168 studs total).
- **Stud pitch**: 25 mm centre-to-centre (matches LEGO spec).
- **Brick footprint**: 4×2 studs (the only size currently used).
- **Placement gap rule**: every placed brick needs **1 row + 1 column of free studs around it** before another brick can be placed.

### How occupancy is decided

For each detected brick the node builds an oriented bounding box on the image, then checks every stud's pixel position against that box (using a rotated-coords inside-test). Studs falling inside any brick's footprint are marked occupied; everything else is free.

### How slots are found

A sliding window scans both orientations (4×2 and 2×4) across the grid. A slot is *available* iff:

- All 8 studs inside the window are free.
- Every stud in the surrounding 1-stud ring is either free or outside the grid (the gap-ring check).

The midpoint of each accepted slot, plus its orientation (yaw 0 or π/2), is published in `~/available_slots`.

### `analyze_buildzone()` return value

```python
{
    "studs_px":    np.ndarray,   # (14, 12, 2)  pixel positions
    "studs_3d":    np.ndarray,   # (14, 12, 3)  metres, camera frame
    "occupancy":   np.ndarray,   # (14, 12)     bool
    "free_studs":  [ {"row":r, "col":c, "px":(x,y), "xyz":(x,y,z)}, ... ],
    "slots":       [ {"row":r, "col":c, "long_axis":"x"|"y",
                      "yaw":0.0|π/2, "xyz":(x,y,z)}, ... ],
}
```

---

## Usage

### Build

```bash
cd ~/git/BrickPickNPlace-RS2
colcon build --packages-select brick_vision
source install/setup.bash
```

### Run (on_trigger mode, default)

```bash
ros2 run brick_vision brick_detector
```

First run prompts for workspace + build-zone calibration via the OpenCV window — see keyboard controls below. Calibration is then loaded automatically on every subsequent run.

### Trigger a snapshot

Three ways:

| Method | Command |
|---|---|
| Manual key | Press `t` in the preview window |
| ROS topic | `ros2 topic pub --once /snapshot_trigger std_msgs/msg/Empty "{}"` |
| Programmatic (interaction node) | Publish `std_msgs/Empty` on `/snapshot_trigger` when the arm reaches home |

### Continuous mode (debug)

```bash
ros2 run brick_vision brick_detector --ros-args -p mode:=continuous
```

### Headless

```bash
ros2 run brick_vision brick_detector --ros-args -p show_preview:=false
```

### Standalone (no ROS, OpenCV GUI only)

```bash
python3 src/brick_vision/brick_vision/brick_detector.py
```

### Launch file

```bash
ros2 launch brick_vision brick_detection.launch.py
```

### Keyboard controls (preview window)

| Key | Action |
|---|---|
| `q` | Quit |
| `t` | **Manual trigger** — works in both modes, useful for independent testing |
| `c` | Re-calibrate workspace ROI (drag rectangle) |
| `b` | Re-calibrate build zone (click TL stud, then BR stud) |
| `s` | Save the current annotated frame as a PNG |

---

## Configuration methods (what you can change, and how)

Three layers, from most static to most dynamic:

| Layer | Where | When to use |
|---|---|---|
| **Compile-time constants** | Top of `brick_detector.py` (e.g. `STUD_DIAMETER_M`) | Physical facts about the brick/scene and detector thresholds. Rebuild after change. |
| **ROS 2 parameters** | `--ros-args -p name:=value` | Runtime behaviour: mode, trigger topic, preview, frames-per-snap. No rebuild. |
| **Interactive calibration** | OpenCV window keys `c` and `b` | Geometry that depends on the *physical setup* — where the workspace is in the image and where the build plate sits. Saved to disk. |

### Calibration file

Saved canonically at `~/.brick_vision/workspace_calibration.json`. Contents:

```json
{
  "roi": [x, y, w, h],                  // workspace rectangle (pixels)
  "surface_depth": 0.42,                // metres, table top
  "build_zone_tl": [px, py],            // top-left stud centre (pixels)
  "build_zone_br": [px, py]             // bottom-right stud centre (pixels)
}
```

Override path with `BRICK_VISION_CALIBRATION` env var. A legacy fallback at `src/brick_vision/config/workspace_calibration.json` is read if the canonical file is missing — kept so existing dev setups keep working.

---

## Parameter reference (what each one *means* and *when to tune*)

### Physical brick / stud geometry (rebuild required)

| Constant | Default | What it controls | When to change |
|---|---|---|---|
| `BRICK_LENGTH_M` | 0.100 | 4-stud axis length, used in metric-size check | Different brick size, e.g. swapping to 6×2 |
| `BRICK_WIDTH_M` | 0.050 | 2-stud axis length | Same — must match `BRICK_LENGTH_M` ratio |
| `STUD_DIAMETER_M` | 0.015 | Expected stud radius in HoughCircles | Different stud type (DUPLO ≈ 0.03 m) |
| `STUD_PITCH_M` | 0.025 | Stud centre-to-centre spacing | LEGO is 8 mm pitch — leave it. Only changes for DUPLO. |
| `BRICK_STUDS_LONG` × `_SHORT` | 4 × 2 | Slot-search window size | Match the brick footprint you're picking |

### Detection tolerances (most-tuned parameters)

| Constant | Default | Effect of **increasing** | Effect of **decreasing** | Symptom-driven tuning |
|---|---|---|---|---|
| `ASPECT_TOL` | 0.35 | More candidates accepted (incl. junk) | Stricter — may reject viewed-at-angle bricks | If bricks are missed when seen from oblique angles, raise to ~0.45 |
| `SIZE_TOL` | 0.30 | Tolerates depth-noise size error | Rejects bricks where depth is wrong | Raise if depth is noisy / surface_depth is off |
| `MIN_CONTOUR_AREA` | 500 px² | Rejects more noise blobs | Catches small bricks far from camera | Lower if the camera is mounted far away; raise if false positives appear in clutter |
| `HOUGH_RADIUS_TOL` | 0.50 | More circles found | Stricter — stud verification fails more often | Raise (e.g. 0.6) if you see bricks rejected for "studs missing" but the colour/size is right |
| `MIN_CIRCULARITY` (legacy) | 0.55 | Stricter circle shape | Tolerates elliptical studs (camera at angle) | Lower when the camera looks down at >30° |

### Confidence formula (how scores are built)

```
confidence = base + bonus,  clamped to [0, 1]
  base  = CONF_BASE_MAX − aspect_err − size_err          # 0.65 max
  bonus = (studs_found / 8) × CONF_STUD_BONUS  if studs_found ≥ 2 else 0   # 0.35 max
```

| Constant | Default | Meaning |
|---|---|---|
| `CONF_BASE_MAX` | 0.65 | Cap on shape-only confidence. Forces stud verification to push above 0.65. |
| `CONF_STUD_BONUS` | 0.35 | How much weight stud verification adds. Higher = harder to be confident without studs. |
| `MIN_STUDS_FOR_BONUS` | 2 | Threshold — fewer studs and the brick gets no bonus, even if its shape is perfect. |

### Colour thresholds

```python
COLOUR_RANGES = [
    ("red",    [(0,90,90),(10,255,255)], [(170,90,90),(179,255,255)]),
    ("orange", [(11,90,90),(25,255,255)]),
    ...
]
BLACK_V_MAX = 60     # value (brightness) ceiling for black detection
BLACK_S_MAX = 120    # saturation ceiling for black
```

| Tuning need | Change |
|---|---|
| Red bricks misclassified as orange under warm light | Move the red/orange boundary up: `(11,…)` → `(15,…)` |
| Black bricks missed under bright lights | Raise `BLACK_V_MAX` to 80 |
| Black detected on coloured bricks in shadow | Lower `BLACK_V_MAX` to 40, and/or lower `BLACK_S_MAX` |

### Build-zone constants

| Constant | Default | Description |
|---|---|---|
| `BUILD_GRID_COLS` × `BUILD_GRID_ROWS` | 12 × 14 | Stud-grid size on the plate. Change for a different build plate. |
| `PLACEMENT_GAP_STUDS` | 1 | Required free-stud ring around each placed brick. 0 = bricks can touch. |

### ROS 2 parameters

| Parameter | Default | Tuning rationale |
|---|---|---|
| `mode` | `"on_trigger"` | Use `"continuous"` only for debug — it wastes CPU and produces jittery poses while the arm is mid-motion. |
| `trigger_topic` | `"/snapshot_trigger"` | Rename if another node owns this name — keep `std_msgs/Empty`. |
| `snapshot_frames` | `5` | Raise (8–10) for noisier scenes, lower (3) for faster cycles. More frames = more chance the best one is clean, at the cost of trigger-to-publish latency (~30 ms per frame). |
| `show_preview` | `true` | `false` for headless/SSH runs and when running on the robot PC. |

---

## Troubleshooting

### Detection problems

| Symptom | Likely cause | Fix |
|---|---|---|
| **No bricks detected, ever** | Workspace ROI is wrong or missing | Press `c`, redraw the rectangle. Check `~/.brick_vision/workspace_calibration.json` exists. |
| **No bricks, but ROI is set** | Surface depth wrong → metric size check fails everything | Re-run `c` calibration; make sure the table is visible inside the rectangle. |
| **Bricks detected far from camera but missed up close** | Studs too big for `HOUGH_MIN_DIST_M` | Lower `HOUGH_MIN_DIST_M`, or expect that bricks need to stay within ~30–50 cm. |
| **Bricks detected up close but missed far away** | Studs become sub-pixel | Move the camera closer, lower the resolution requirement (`MIN_CONTOUR_AREA`). |
| **One colour consistently missed** | Colour range needs tuning under the actual lights | Open the OpenCV "Colour Mask" debug window (if enabled) or sample HSV with `cv2.cvtColor` on a saved snapshot. Adjust the corresponding `COLOUR_RANGES` entry. |
| **Random false positives in the background** | ROI too large | Press `c`, draw a tighter rectangle. |
| **Confidence stuck at 0.6–0.65 (never higher)** | Studs aren't being verified | Stud verification is failing — usually `HOUGH_RADIUS_TOL` is too tight or camera angle is too oblique. Raise tolerance or move the camera more vertical. |

### Build-zone problems

| Symptom | Likely cause | Fix |
|---|---|---|
| **No slots published / no `~/free_studs`** | Build zone not calibrated → `analyze_buildzone` returns `None` | Press `b`, click TL then BR stud of the build plate. |
| **Studs/grid drawn but at wrong angle** | The 2-click calibration assumes axis-aligned grid | Either physically align the plate with the camera axes, or upgrade to 4-click perspective calibration |
| **A brick is detected on the plate but its studs show as free** | Brick yaw is far off from what the oriented-box test expects | Check that `det["angle"]` is sane; verify oriented bounding box dims match the brick |
| **Adjacent bricks treated as one slot** | `PLACEMENT_GAP_STUDS = 0` | Set back to 1 |

### ROS-side problems

| Symptom | Likely cause | Fix |
|---|---|---|
| **Trigger published, nothing happens** | Mode is `continuous` (trigger ignored) or topic mismatch | Check `ros2 param get /brick_detector mode`; check `ros2 topic info /snapshot_trigger -v` shows your publisher and the node's subscriber. |
| **Late-starting interaction node sees no messages** | `mode=continuous` (volatile QoS) — switch to `on_trigger` for latched delivery | Or change QoS in `brick_detector.py:1040` for continuous mode too |
| **`detection_image` won't display in rviz** | QoS mismatch — rviz default Image QoS is volatile, the topic is volatile, but reliability may not match | Set rviz's Image display reliability to "Best Effort" or check `ros2 topic info -v` |
| **Snapshot trigger fires but no detections** | Workspace is occluded mid-snapshot (e.g. arm in frame) | Trigger from `home` pose only; raise `snapshot_frames` so best-of-N is more likely clean |

### Camera problems

| Symptom | Likely cause | Fix |
|---|---|---|
| **`get_frames()` returns `None`** | RealSense not connected / claimed by another process | `pkill -f realsense`; `lsusb \| grep Intel`; check `realsense-viewer` works |
| **Depth values are 0 everywhere** | Camera too close (< 0.10 m) or surface is dark/transparent | Move camera to 0.3–0.7 m; ensure the build plate is matte and lit |
| **Big depth jitter on the same stud** | Frame-to-frame noise on D435i | This is why we average inside `_sample_depth` (median of a 21×21 sample window). Increase the sample window if still noisy. |

---

## Reference docs

A full subsystem deep-dive — architecture, every parameter explained with tuning rationale, viva-style Q&A, troubleshooting flowcharts — is available at:

```
docs/brick_vision_subsystem.docx
```

Regenerate with:

```bash
python3 tools/generate_docs.py
```

---

## Files

```
brick_vision/
├── brick_vision/
│   ├── __init__.py
│   ├── brick_detector.py                # main detector + ROS node
│   ├── brick_detector_no_buildzone.py   # backup: state before build-zone work
│   └── brick_detector_dimension_based.py  # legacy contour-only detector
├── config/
│   └── workspace_calibration.json       # legacy fallback (canonical is ~/.brick_vision/)
├── launch/
│   └── brick_detection.launch.py
├── package.xml
└── setup.py
```

---

## How the interaction node consumes this

Brick poses are published in the **camera frame** (`camera_color_optical_frame`). The interaction node is responsible for transforming them to `base_link` via `tf2` — see `brick_interaction/pose_transform_test.py` for a working example.

Typical pick flow:

1. Arm reaches home → interaction node publishes `Empty` to `/snapshot_trigger`.
2. Vision captures 5 frames, picks the best, publishes latched results on `~/detections` and `~/available_slots`.
3. Interaction node reads `~/detections`, picks a brick (e.g. by colour), transforms its pose to `base_link`, sends the goal to MoveIt.
4. After place, arm returns to home → repeat.

The latched QoS guarantees the interaction node sees the most recent snapshot even if it starts up after the vision node.
