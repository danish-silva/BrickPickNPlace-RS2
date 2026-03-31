#!/usr/bin/env python3
"""
Brick Detector - Detects 4x2 LEGO bricks using Intel RealSense + OpenCV.

Brick specifications:
  - 4x2 studs, 100mm x 50mm footprint
  - Stud diameter: 15mm
  - Colour detection: black, red, orange, yellow, green, blue, purple, white

Usage:
  Standalone:  python3 brick_detector.py
  ROS2 node:   ros2 run brick_vision brick_detector
"""

import numpy as np
import cv2
import pyrealsense2 as rs
import time
import json
import os

# ── Stud / peg dimensions (metres) ─────────────────────────────────────────
STUD_DIAMETER_M = 0.015   # 15 mm
STUD_HEIGHT_M   = 0.010   # 10 mm
STUD_SPACING_M  = 0.024   # centre-to-centre spacing between adjacent studs

# ── Stud detection tolerances ─────────────────────────────────────────────
STUD_DIAM_TOL   = 0.50    # ±50 % on stud diameter (generous for camera angle)
STUD_SPACING_TOL = 0.45   # ±45 % on expected spacing when grouping
MIN_CIRCULARITY  = 0.55   # blob circularity threshold (1.0 = perfect circle)

# ── Known brick configurations (studs_long, studs_wide) ──────────────────
# For now only 4x2, but easy to extend later
BRICK_CONFIGS = [
    (4, 2),  # 4x2 brick — 8 studs
]

# ── Colour definitions (HSV ranges) ───────────────────────────────────────
# Each entry: (label, hsv_lower, hsv_upper, display_bgr)
# H: 0-179, S: 0-255, V: 0-255 in OpenCV
COLOUR_RANGES = [
    ("red",    np.array([0,   120, 70]),  np.array([10,  255, 255]), (0, 0, 255)),
    ("red",    np.array([170, 120, 70]),  np.array([179, 255, 255]), (0, 0, 255)),
    ("orange", np.array([11,  120, 70]),  np.array([25,  255, 255]), (0, 140, 255)),
    ("yellow", np.array([26,  80,  70]),  np.array([34,  255, 255]), (0, 255, 255)),
    ("green",  np.array([35,  80,  70]),  np.array([85,  255, 255]), (0, 200, 0)),
    ("blue",   np.array([86,  80,  50]),  np.array([130, 255, 255]), (255, 100, 0)),
    ("purple", np.array([131, 50,  50]),  np.array([169, 255, 255]), (200, 50, 200)),
]

# Black detection via value/saturation
BLACK_V_MAX = 60
BLACK_S_MAX = 120

# ── Calibration file ──────────────────────────────────────────────────────
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "..", "config", "workspace_calibration.json")


class WorkspaceCalibration:
    """Manages workspace ROI calibration for the camera."""

    def __init__(self):
        self.roi = None          # (x, y, w, h) in pixel coords
        self.plane_depth = None  # average depth of the workspace surface (m)
        self.calibrated = False

    def calibrate_interactive(self, color_image, depth_frame):
        """Let the user draw a rectangle over the workspace area."""
        clone = color_image.copy()
        points = []

        def mouse_cb(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                points.append((x, y))
            elif event == cv2.EVENT_MOUSEMOVE and len(points) == 1:
                img = clone.copy()
                cv2.rectangle(img, points[0], (x, y), (0, 255, 0), 2)
                cv2.imshow("Calibration", img)
            elif event == cv2.EVENT_LBUTTONUP:
                points.append((x, y))

        cv2.namedWindow("Calibration")
        cv2.setMouseCallback("Calibration", mouse_cb)

        print("[Calibration] Draw a rectangle around the workspace, then press ENTER.")
        cv2.imshow("Calibration", clone)

        while True:
            key = cv2.waitKey(30) & 0xFF
            if key == 13 and len(points) >= 2:  # ENTER
                break
            if key == 27:  # ESC – cancel
                cv2.destroyWindow("Calibration")
                return False

        x1, y1 = points[0]
        x2, y2 = points[1]
        self.roi = (min(x1, x2), min(y1, y2), abs(x2 - x1), abs(y2 - y1))

        # Sample depth inside ROI to find workspace surface height
        rx, ry, rw, rh = self.roi
        depths = []
        for py in range(ry, ry + rh, 4):
            for px in range(rx, rx + rw, 4):
                d = depth_frame.get_distance(px, py)
                if 0.1 < d < 3.0:
                    depths.append(d)
        if depths:
            self.plane_depth = float(np.median(depths))

        self.calibrated = True
        cv2.destroyWindow("Calibration")
        print(f"[Calibration] ROI={self.roi}, surface depth={self.plane_depth:.3f} m")
        return True

    def save(self, path=None):
        path = path or CALIBRATION_FILE
        os.makedirs(os.path.dirname(path), exist_ok=True)
        data = {"roi": self.roi, "plane_depth": self.plane_depth}
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"[Calibration] Saved to {path}")

    def load(self, path=None):
        path = path or CALIBRATION_FILE
        if not os.path.exists(path):
            return False
        with open(path, "r") as f:
            data = json.load(f)
        self.roi = tuple(data["roi"])
        self.plane_depth = data["plane_depth"]
        self.calibrated = True
        print(f"[Calibration] Loaded from {path}")
        return True


class BrickDetector:
    """Detects 4x2 LEGO bricks from an Intel RealSense stream."""

    def __init__(self):
        # RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = None
        self.intrinsics = None

        # Calibration
        self.calibration = WorkspaceCalibration()

    # ── Camera lifecycle ───────────────────────────────────────────────────
    def start_camera(self, width=640, height=480, fps=30):
        # Verify device is available
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            raise RuntimeError(
                "No RealSense device found. Check USB connection and try:\n"
                "  1. Unplug and replug the camera\n"
                "  2. Run: sudo chmod 666 /dev/video*\n"
                "  3. Check with: rs-enumerate-devices"
            )
        dev = devices[0]
        print(f"[Camera] Found: {dev.get_info(rs.camera_info.name)} "
              f"(FW {dev.get_info(rs.camera_info.firmware_version)})")

        # Hardware reset to release any stale handles
        dev.hardware_reset()
        print("[Camera] Hardware reset issued, waiting for device...")
        time.sleep(3)

        # Re-create pipeline after reset
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipeline.start(self.config)

        # Align depth to colour frame
        self.align = rs.align(rs.stream.color)

        # Get camera intrinsics (needed for pixel → metric conversion)
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_stream.get_intrinsics()

        # Let auto‑exposure settle
        for _ in range(30):
            self.pipeline.wait_for_frames()

        print(f"[Camera] Started {width}x{height}@{fps}fps")

    def stop_camera(self):
        self.pipeline.stop()
        print("[Camera] Stopped")

    def get_frames(self):
        """Return aligned (color_image, depth_frame)."""
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_frame

    # ── Pixel ↔ metric helpers ─────────────────────────────────────────────
    def pixel_size_to_metres(self, pixel_length, depth_m):
        """Convert a length in pixels to metres at a given depth."""
        if depth_m <= 0 or self.intrinsics is None:
            return 0.0
        # Average of fx, fy for a rough conversion
        focal = (self.intrinsics.fx + self.intrinsics.fy) / 2.0
        return pixel_length * depth_m / focal

    def metres_to_pixels(self, metres, depth_m):
        """Convert a metric length to pixels at a given depth."""
        if depth_m <= 0 or self.intrinsics is None:
            return 0
        focal = (self.intrinsics.fx + self.intrinsics.fy) / 2.0
        return int(metres * focal / depth_m)

    def pixel_to_3d(self, px, py, depth_m):
        """Deproject a pixel + depth into a 3D point (camera frame, metres)."""
        if self.intrinsics is None:
            return None
        point = rs.rs2_deproject_pixel_to_point(self.intrinsics, [px, py], depth_m)
        return np.array(point)  # [x, y, z] in metres

    # ── Stud (peg) detection ─────────────────────────────────────────────────
    def _detect_studs(self, work_img, depth_frame, offset_x, offset_y):
        """
        Detect individual circular studs in the working image.

        Returns list of dicts:
          {"center_local": (x,y), "center_full": (x,y), "radius_px": r,
           "diameter_m": float, "depth_m": float}
        """
        gray = cv2.cvtColor(work_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Adaptive threshold to pick up stud circles on any colour brick
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 21, 5)

        # Find contours and filter for circular blobs
        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        studs = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 30:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < MIN_CIRCULARITY:
                continue

            (cx_l, cy_l), radius = cv2.minEnclosingCircle(cnt)
            cx_l, cy_l, radius = float(cx_l), float(cy_l), float(radius)

            if radius < 3:
                continue

            cx_full = int(cx_l + offset_x)
            cy_full = int(cy_l + offset_y)

            # Get depth and validate real-world diameter
            depth_m = depth_frame.get_distance(cx_full, cy_full)
            if depth_m <= 0.05 or depth_m > 2.0:
                depth_m = self._sample_depth(depth_frame, cx_full, cy_full, radius=5)
                if depth_m <= 0.05:
                    continue

            diameter_m = self.pixel_size_to_metres(radius * 2, depth_m)

            # Check if diameter matches expected stud size
            if abs(diameter_m - STUD_DIAMETER_M) > STUD_DIAMETER_M * STUD_DIAM_TOL:
                continue

            studs.append({
                "center_local": (cx_l, cy_l),
                "center_full":  (cx_full, cy_full),
                "radius_px":    radius,
                "diameter_m":   diameter_m,
                "depth_m":      depth_m,
            })

        return studs

    def _group_studs_into_bricks(self, studs):
        """
        Group detected studs into brick patterns by finding clusters
        with the right grid spacing.

        Returns list of groups, each group is a list of stud dicts.
        """
        if len(studs) < 2:
            return []

        # Compute expected pixel spacing from the first stud's depth
        ref_depth = np.median([s["depth_m"] for s in studs])
        expected_spacing_px = self.metres_to_pixels(STUD_SPACING_M, ref_depth)
        if expected_spacing_px < 5:
            return []

        max_dist = expected_spacing_px * (1 + STUD_SPACING_TOL)

        # Build adjacency: two studs are neighbours if ~1 stud-spacing apart
        n = len(studs)
        neighbours = [[] for _ in range(n)]
        for i in range(n):
            xi, yi = studs[i]["center_full"]
            for j in range(i + 1, n):
                xj, yj = studs[j]["center_full"]
                dist = np.hypot(xi - xj, yi - yj)
                min_dist = expected_spacing_px * (1 - STUD_SPACING_TOL)
                if min_dist <= dist <= max_dist:
                    neighbours[i].append(j)
                    neighbours[j].append(i)

        # BFS to find connected components of studs
        visited = [False] * n
        groups = []
        for i in range(n):
            if visited[i]:
                continue
            # Flood-fill from stud i
            queue = [i]
            visited[i] = True
            component = [i]
            while queue:
                current = queue.pop(0)
                for nb in neighbours[current]:
                    if not visited[nb]:
                        visited[nb] = True
                        queue.append(nb)
                        component.append(nb)
            if len(component) >= 2:
                groups.append([studs[idx] for idx in component])

        return groups

    def _classify_brick(self, group):
        """
        Given a group of studs, determine if they match a known brick config.
        Returns (studs_long, studs_wide) or None if no match.
        """
        count = len(group)
        for cols, rows in BRICK_CONFIGS:
            expected = cols * rows
            # Allow ±2 studs tolerance (some may be missed or extra detected)
            if abs(count - expected) <= 2:
                return (cols, rows)
        return None

    def _identify_colour(self, color_image, studs):
        """Sample the colour around the stud centres to determine brick colour."""
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Collect HSV samples from each stud's neighbourhood
        h_samples = []
        s_samples = []
        v_samples = []
        for s in studs:
            cx, cy = s["center_full"]
            r = max(3, int(s["radius_px"]))
            # Sample a small patch around the stud
            y1 = max(0, cy - r)
            y2 = min(hsv.shape[0], cy + r)
            x1 = max(0, cx - r)
            x2 = min(hsv.shape[1], cx + r)
            patch = hsv[y1:y2, x1:x2]
            if patch.size == 0:
                continue
            h_samples.extend(patch[:, :, 0].flatten().tolist())
            s_samples.extend(patch[:, :, 1].flatten().tolist())
            v_samples.extend(patch[:, :, 2].flatten().tolist())

        if not h_samples:
            return "unknown", (128, 128, 128)

        med_h = int(np.median(h_samples))
        med_s = int(np.median(s_samples))
        med_v = int(np.median(v_samples))

        # Check black first (low value)
        if med_v < BLACK_V_MAX:
            return "black", (80, 80, 80)

        # Check chromatic colours
        for label, lower, upper, bgr in COLOUR_RANGES:
            if lower[0] <= med_h <= upper[0] and med_s >= lower[1] and med_v >= lower[2]:
                return label, bgr

        return "unknown", (128, 128, 128)

    # ── Main detection entry point ────────────────────────────────────────
    def detect_bricks(self, color_image, depth_frame):
        """
        Detect bricks by finding stud (peg) patterns.

        Returns a list of dicts:
          {
            "center_px":    (cx, cy),
            "center_3d":    [x, y, z],
            "size_px":      (w, h),
            "size_m":       (w_m, h_m),
            "angle":        float,
            "colour":       str,
            "colour_bgr":   (b, g, r),
            "studs":        list,       # individual stud dicts
            "stud_count":   int,
            "brick_config": (cols, rows),
            "confidence":   float,
          }
        """
        # Crop to ROI if calibrated
        if self.calibration.calibrated and self.calibration.roi:
            rx, ry, rw, rh = self.calibration.roi
            work_img = color_image[ry:ry+rh, rx:rx+rw]
            offset_x, offset_y = rx, ry
        else:
            work_img = color_image
            offset_x, offset_y = 0, 0

        # Step 1: find all studs
        studs = self._detect_studs(work_img, depth_frame, offset_x, offset_y)

        # Step 2: group studs into bricks by spatial proximity
        groups = self._group_studs_into_bricks(studs)

        # Step 3: validate each group and build detections
        detections = []
        for group in groups:
            config = self._classify_brick(group)
            if config is None:
                continue

            cols, rows = config
            expected_count = cols * rows

            # Compute brick centre and bounding rect from stud positions
            pts = np.array([s["center_full"] for s in group], dtype=np.float32)
            rect = cv2.minAreaRect(pts)
            (cx, cy), (w_px, h_px), angle = rect

            # Ensure w >= h
            if w_px < h_px:
                w_px, h_px = h_px, w_px
                angle += 90

            cx, cy = int(cx), int(cy)

            # Depth at centre
            depth_m = depth_frame.get_distance(cx, cy)
            if depth_m <= 0.05 or depth_m > 2.0:
                depth_m = self._sample_depth(depth_frame, cx, cy)
                if depth_m <= 0.05:
                    continue

            w_m = self.pixel_size_to_metres(w_px, depth_m)
            h_m = self.pixel_size_to_metres(h_px, depth_m)

            # Colour from stud samples
            colour_name, colour_bgr = self._identify_colour(color_image, group)

            # Confidence based on how many expected studs we found
            stud_ratio = min(len(group), expected_count) / expected_count
            # Bonus for diameter accuracy
            diam_errors = [abs(s["diameter_m"] - STUD_DIAMETER_M) / STUD_DIAMETER_M
                           for s in group]
            avg_diam_err = np.mean(diam_errors) if diam_errors else 0.5
            confidence = max(0.0, stud_ratio * 0.7 + (1.0 - avg_diam_err) * 0.3)

            center_3d = self.pixel_to_3d(cx, cy, depth_m)

            detections.append({
                "center_px":    (cx, cy),
                "center_3d":    center_3d.tolist() if center_3d is not None else None,
                "size_px":      (w_px, h_px),
                "size_m":       (w_m, h_m),
                "angle":        angle % 180,
                "colour":       colour_name,
                "colour_bgr":   colour_bgr,
                "studs":        group,
                "stud_count":   len(group),
                "brick_config": config,
                "confidence":   round(confidence, 3),
            })

        detections.sort(key=lambda d: d["confidence"], reverse=True)
        return detections

    def _sample_depth(self, depth_frame, cx, cy, radius=10):
        """Sample a small area around (cx, cy) and return median depth."""
        depths = []
        for dy in range(-radius, radius + 1, 2):
            for dx in range(-radius, radius + 1, 2):
                d = depth_frame.get_distance(cx + dx, cy + dy)
                if 0.05 < d < 2.0:
                    depths.append(d)
        return float(np.median(depths)) if depths else 0.0

    # ── Visualisation ──────────────────────────────────────────────────────
    def draw_detections(self, image, detections):
        """Draw stud circles, bounding boxes, and brick info."""
        vis = image.copy()

        # Draw workspace ROI
        if self.calibration.calibrated and self.calibration.roi:
            rx, ry, rw, rh = self.calibration.roi
            cv2.rectangle(vis, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 1)
            cv2.putText(vis, "Workspace", (rx, ry - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        for det in detections:
            cx, cy = det["center_px"]
            angle = det["angle"]
            conf = det["confidence"]
            colour = det["colour"]
            box_colour = det["colour_bgr"]
            cols, rows = det["brick_config"]

            # Draw each detected stud
            for stud in det["studs"]:
                sx, sy = stud["center_full"]
                r = int(stud["radius_px"])
                cv2.circle(vis, (sx, sy), r, (0, 255, 0), 2)
                cv2.circle(vis, (sx, sy), 2, (0, 255, 0), -1)

            # Rotated bounding box around the stud cluster
            rect = ((cx, cy), det["size_px"], angle)
            box = cv2.boxPoints(rect).astype(int)
            cv2.drawContours(vis, [box], 0, box_colour, 2)

            # Centre crosshair
            cv2.drawMarker(vis, (cx, cy), (0, 255, 255),
                           cv2.MARKER_CROSS, 15, 2)

            # Colour tag
            cv2.putText(vis, colour.upper(), (cx - 30, cy - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_colour, 2)

            # Brick info: config, stud count, confidence
            config_label = f"{cols}x{rows} ({det['stud_count']} studs)  conf={conf:.2f}"
            cv2.putText(vis, config_label, (cx - 80, cy - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            label = f"{angle:.0f}deg"
            cv2.putText(vis, label, (cx - 20, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            if det["center_3d"]:
                xyz = det["center_3d"]
                pos_label = f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})m"
                cv2.putText(vis, pos_label, (cx - 80, cy + 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        return vis

    # ── Main loop ──────────────────────────────────────────────────────────
    def run(self):
        """Standalone detection loop with live preview."""
        self.start_camera()

        # Try loading existing calibration, else prompt interactive
        if not self.calibration.load():
            print("[Detector] No calibration found. Starting interactive calibration...")
            color_image, depth_frame = self.get_frames()
            if color_image is not None:
                if self.calibration.calibrate_interactive(color_image, depth_frame):
                    self.calibration.save()

        print("[Detector] Running – press 'q' to quit, 'c' to re-calibrate, 's' to save snapshot")

        try:
            while True:
                color_image, depth_frame = self.get_frames()
                if color_image is None:
                    continue

                detections = self.detect_bricks(color_image, depth_frame)

                vis = self.draw_detections(color_image, detections)

                # HUD
                cv2.putText(vis, f"Bricks: {len(detections)}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.imshow("Brick Detector", vis)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord('c'):
                    if self.calibration.calibrate_interactive(color_image, depth_frame):
                        self.calibration.save()
                elif key == ord('s'):
                    fname = f"snapshot_{int(time.time())}.png"
                    cv2.imwrite(fname, vis)
                    print(f"[Snapshot] Saved {fname}")

        finally:
            cv2.destroyAllWindows()
            self.stop_camera()


def run_standalone():
    """Run detection loop with live OpenCV preview (no ROS 2)."""
    detector = BrickDetector()
    detector.run()


# ── ROS 2 node wrapper ────────────────────────────────────────────────────
def main(args=None):
    """ROS 2 entry point (used by ros2 run / launch)."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped
    from cv_bridge import CvBridge

    class BrickDetectorNode(Node):
        def __init__(self):
            super().__init__("brick_detector")
            self.detector = BrickDetector()
            self.bridge = CvBridge()

            # Publishers
            self.image_pub = self.create_publisher(Image, "~/detection_image", 10)
            self.pose_pub  = self.create_publisher(PoseStamped, "~/brick_pose", 10)

            self.detector.start_camera()
            self.detector.calibration.load()

            self.timer = self.create_timer(1.0 / 15, self.detect_callback)  # 15 Hz
            self.get_logger().info("Brick detector node started")

        def detect_callback(self):
            color_image, depth_frame = self.detector.get_frames()
            if color_image is None:
                return

            detections = self.detector.detect_bricks(color_image, depth_frame)

            # Publish annotated image
            vis = self.detector.draw_detections(color_image, detections)
            img_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            self.image_pub.publish(img_msg)

            # Publish pose of best detection
            if detections:
                best = detections[0]
                if best["center_3d"]:
                    pose = PoseStamped()
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.header.frame_id = "camera_color_optical_frame"
                    pose.pose.position.x = best["center_3d"][0]
                    pose.pose.position.y = best["center_3d"][1]
                    pose.pose.position.z = best["center_3d"][2]
                    # Orientation from brick angle (yaw only, around Z)
                    yaw = np.radians(best["angle"])
                    pose.pose.orientation.z = np.sin(yaw / 2)
                    pose.pose.orientation.w = np.cos(yaw / 2)
                    self.pose_pub.publish(pose)

        def destroy_node(self):
            self.detector.stop_camera()
            super().destroy_node()

    rclpy.init(args=args)
    node = BrickDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    # Running directly → standalone mode with OpenCV GUI
    run_standalone()
