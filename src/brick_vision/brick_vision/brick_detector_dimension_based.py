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

# ── Brick dimensions (metres) ──────────────────────────────────────────────
BRICK_LENGTH_M = 0.100   # 100 mm
BRICK_WIDTH_M  = 0.050   # 50 mm
BRICK_ASPECT   = BRICK_LENGTH_M / BRICK_WIDTH_M  # 2.0
STUD_DIAMETER_M = 0.015  # 15 mm
STUDS_LONG = 4
STUDS_WIDE = 2

# ── Detection tolerances ───────────────────────────────────────────────────
ASPECT_TOL       = 0.35   # ±35 % on aspect ratio
SIZE_TOL         = 0.30   # ±30 % on metric size
MIN_CONTOUR_AREA = 500    # pixels – reject tiny noise

# ── Colour definitions (HSV ranges) ───────────────────────────────────────
# Each entry: (label, hsv_lower, hsv_upper, display_bgr)
# H: 0-179, S: 0-255, V: 0-255 in OpenCV
COLOUR_RANGES = [
    # Red wraps around H=0, so we need two ranges
    ("red",    np.array([0,   120, 70]),  np.array([10,  255, 255]), (0, 0, 255)),
    ("red",    np.array([170, 120, 70]),  np.array([179, 255, 255]), (0, 0, 255)),
    ("orange", np.array([11,  120, 70]),  np.array([25,  255, 255]), (0, 140, 255)),
    ("yellow", np.array([26,  80,  70]),  np.array([34,  255, 255]), (0, 255, 255)),
    ("green",  np.array([35,  80,  70]),  np.array([85,  255, 255]), (0, 200, 0)),
    ("blue",   np.array([86,  80,  50]),  np.array([130, 255, 255]), (255, 100, 0)),
    ("purple", np.array([131, 50,  50]),  np.array([169, 255, 255]), (200, 50, 200)),
]

# Black and white are detected via value/saturation, not hue
BLACK_V_MAX = 60     # V channel ceiling for "black"
BLACK_S_MAX = 120    # allow low saturation

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

    # ── Detection ──────────────────────────────────────────────────────────
    def detect_bricks(self, color_image, depth_frame):
        """
        Detect 4x2 bricks of any supported colour in the image.

        Returns a list of dicts:
          {
            "center_px": (cx, cy),
            "center_3d": [x, y, z],     # metres, camera frame
            "size_px":   (w, h),
            "size_m":    (w_m, h_m),
            "angle":     float,          # degrees
            "colour":    str,            # e.g. "red", "orange", "black"
            "colour_bgr": (b, g, r),    # for drawing
            "contour":   np.ndarray,
            "confidence": float,         # 0‑1
          }
        """
        detections = []

        # Crop to ROI if calibrated
        if self.calibration.calibrated and self.calibration.roi:
            rx, ry, rw, rh = self.calibration.roi
            work_img = color_image[ry:ry+rh, rx:rx+rw]
            offset_x, offset_y = rx, ry
        else:
            work_img = color_image
            offset_x, offset_y = 0, 0

        blurred = cv2.GaussianBlur(work_img, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # Build list of (mask, colour_name, colour_bgr) to scan
        colour_masks = []

        # Chromatic colours from HSV ranges
        for label, lower, upper, bgr in COLOUR_RANGES:
            mask = cv2.inRange(hsv, lower, upper)
            colour_masks.append((mask, label, bgr))

        # Merge the two red masks into one
        merged = []
        red_mask = None
        red_bgr = (0, 0, 255)
        for mask, label, bgr in colour_masks:
            if label == "red":
                red_mask = mask if red_mask is None else cv2.bitwise_or(red_mask, mask)
                red_bgr = bgr
            else:
                merged.append((mask, label, bgr))
        if red_mask is not None:
            merged.insert(0, (red_mask, "red", red_bgr))
        colour_masks = merged

        # Black: low value regardless of hue
        black_mask = cv2.inRange(hsv, np.array([0, 0, 0]),
                                       np.array([179, BLACK_S_MAX, BLACK_V_MAX]))
        colour_masks.append((black_mask, "black", (80, 80, 80)))

        # White detection disabled – workspace surface is white

        # Track centres already detected to avoid duplicates from overlapping masks
        used_centres = []

        for mask, colour_name, colour_bgr in colour_masks:
            # Clean up mask
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_CONTOUR_AREA:
                    continue

                rect = cv2.minAreaRect(cnt)
                (cx_local, cy_local), (w_px, h_px), angle = rect

                if w_px == 0 or h_px == 0:
                    continue

                # Ensure w >= h
                if w_px < h_px:
                    w_px, h_px = h_px, w_px
                    angle += 90

                aspect = w_px / h_px

                if abs(aspect - BRICK_ASPECT) > BRICK_ASPECT * ASPECT_TOL:
                    continue

                cx_full = int(cx_local + offset_x)
                cy_full = int(cy_local + offset_y)

                # Skip if we already detected a brick at roughly this location
                if any(abs(cx_full - px) < 20 and abs(cy_full - py) < 20
                       for px, py in used_centres):
                    continue

                # Depth & metric size check
                depth_m = depth_frame.get_distance(cx_full, cy_full)
                if depth_m <= 0.05 or depth_m > 2.0:
                    depth_m = self._sample_depth(depth_frame, cx_full, cy_full)
                    if depth_m <= 0.05:
                        continue

                w_m = self.pixel_size_to_metres(w_px, depth_m)
                h_m = self.pixel_size_to_metres(h_px, depth_m)

                length_ok = abs(w_m - BRICK_LENGTH_M) < BRICK_LENGTH_M * SIZE_TOL
                width_ok  = abs(h_m - BRICK_WIDTH_M)  < BRICK_WIDTH_M  * SIZE_TOL
                if not (length_ok and width_ok):
                    continue

                # Confidence
                aspect_err = abs(aspect - BRICK_ASPECT) / BRICK_ASPECT
                size_err = (abs(w_m - BRICK_LENGTH_M) / BRICK_LENGTH_M +
                            abs(h_m - BRICK_WIDTH_M) / BRICK_WIDTH_M) / 2.0
                confidence = max(0.0, 1.0 - aspect_err - size_err)

                # 3D position
                center_3d = self.pixel_to_3d(cx_full, cy_full, depth_m)

                # Offset contour to full image coords
                cnt_full = cnt.copy()
                cnt_full[:, :, 0] += offset_x
                cnt_full[:, :, 1] += offset_y

                used_centres.append((cx_full, cy_full))

                detections.append({
                    "center_px": (cx_full, cy_full),
                    "center_3d": center_3d.tolist() if center_3d is not None else None,
                    "size_px":   (w_px, h_px),
                    "size_m":    (w_m, h_m),
                    "angle":     angle % 180,
                    "colour":    colour_name,
                    "colour_bgr": colour_bgr,
                    "contour":   cnt_full,
                    "confidence": round(confidence, 3),
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
        """Draw bounding boxes, centres and info on the image."""
        vis = image.copy()

        # Draw workspace ROI
        if self.calibration.calibrated and self.calibration.roi:
            rx, ry, rw, rh = self.calibration.roi
            cv2.rectangle(vis, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 1)
            cv2.putText(vis, "Workspace", (rx, ry - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        for det in detections:
            cx, cy = det["center_px"]
            w_m, h_m = det["size_m"]
            angle = det["angle"]
            conf = det["confidence"]
            colour = det["colour"]
            box_colour = det["colour_bgr"]

            # Rotated bounding box in the brick's colour
            rect = ((cx, cy), det["size_px"], angle)
            box = cv2.boxPoints(rect).astype(int)
            cv2.drawContours(vis, [box], 0, box_colour, 2)

            # Centre crosshair
            cv2.drawMarker(vis, (cx, cy), (0, 255, 255),
                           cv2.MARKER_CROSS, 15, 2)

            # Colour tag (bold, above the box)
            cv2.putText(vis, colour.upper(), (cx - 30, cy - 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_colour, 2)

            # Info text
            label = f"{w_m*1000:.0f}x{h_m*1000:.0f}mm  {angle:.0f}deg  conf={conf:.2f}"
            cv2.putText(vis, label, (cx - 80, cy - 15),
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
