#!/usr/bin/env python3
"""
Brick Detector - Detects 4x2 LEGO bricks using Intel RealSense + OpenCV.

Brick specifications:
  - 4x2 studs, 100mm x 50mm footprint
  - Stud diameter: 15mm
  - Colour detection: black, red, orange, yellow, green, blue, purple
  - Hybrid approach: colour segmentation (primary) + stud verification (secondary)

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
BRICK_LENGTH_M   = 0.100   # 100 mm (4-stud axis)
BRICK_WIDTH_M    = 0.050   # 50 mm  (2-stud axis)
BRICK_ASPECT     = BRICK_LENGTH_M / BRICK_WIDTH_M  # 2.0
STUD_DIAMETER_M  = 0.015   # 15 mm

# ── Primary detection tolerances (colour + shape) ─────────────────────────
ASPECT_TOL       = 0.35    # ±35 % on aspect ratio
SIZE_TOL         = 0.30    # ±30 % on metric dimensions
MIN_CONTOUR_AREA = 500     # pixels – reject noise

# ── Stud verification (HoughCircles) ─────────────────────────────────────
HOUGH_DP         = 1.2     # inverse ratio of accumulator resolution
HOUGH_MIN_DIST_M = 0.012   # min distance between circle centres (metres)
HOUGH_PARAM1     = 50      # Canny high threshold
HOUGH_PARAM2     = 18      # accumulator threshold (lower = more sensitive)
HOUGH_RADIUS_TOL = 0.50    # ±50 % on expected stud radius in pixels
EXPECTED_STUDS   = 8       # 4x2 brick

# ── Confidence weights ───────────────────────────────────────────────────
CONF_BASE_MAX       = 0.65  # max confidence from colour + size alone
CONF_STUD_BONUS     = 0.35  # bonus when studs are verified (total up to 1.0)
MIN_STUDS_FOR_BONUS = 2     # need at least 2 studs for any bonus

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
# Canonical location is the user's home dir so it survives `colcon build` and
# works from both standalone and `ros2 run`. The legacy in-repo path is kept
# as a read-only fallback for backwards compatibility.
CALIBRATION_FILE = os.environ.get(
    "BRICK_VISION_CALIB",
    os.path.expanduser("~/.brick_vision/workspace_calibration.json"),
)
LEGACY_CALIBRATION_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "config", "workspace_calibration.json"))


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
        # Try the requested/canonical path first, then the in-repo legacy path
        candidates = [path] if path else [CALIBRATION_FILE, LEGACY_CALIBRATION_FILE]
        for p in candidates:
            if not p or not os.path.exists(p):
                continue
            with open(p, "r") as f:
                data = json.load(f)
            self.roi = tuple(data["roi"])
            self.plane_depth = data["plane_depth"]
            self.calibrated = True
            print(f"[Calibration] Loaded from {p}")
            return True
        return False


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

    # ── Primary detection: colour segmentation ───────────────────────────────
    def _find_colour_candidates(self, work_img, depth_frame, offset_x, offset_y):
        """
        Find brick candidates using HSV colour segmentation + shape filtering.

        Returns list of candidate dicts with centre, size, angle, colour, etc.
        """
        blurred = cv2.GaussianBlur(work_img, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # Build colour masks
        colour_masks = []
        red_mask = None
        red_bgr = (0, 0, 255)
        for label, lower, upper, bgr in COLOUR_RANGES:
            mask = cv2.inRange(hsv, lower, upper)
            if label == "red":
                red_mask = mask if red_mask is None else cv2.bitwise_or(red_mask, mask)
                red_bgr = bgr
            else:
                colour_masks.append((mask, label, bgr))
        if red_mask is not None:
            colour_masks.insert(0, (red_mask, "red", red_bgr))

        # Black: low value
        black_mask = cv2.inRange(hsv, np.array([0, 0, 0]),
                                       np.array([179, BLACK_S_MAX, BLACK_V_MAX]))
        colour_masks.append((black_mask, "black", (80, 80, 80)))

        candidates = []
        used_centres = []

        for mask, colour_name, colour_bgr in colour_masks:
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

                if w_px < h_px:
                    w_px, h_px = h_px, w_px
                    angle += 90

                aspect = w_px / h_px

                if abs(aspect - BRICK_ASPECT) > BRICK_ASPECT * ASPECT_TOL:
                    continue

                cx_full = int(cx_local + offset_x)
                cy_full = int(cy_local + offset_y)

                # De-duplicate
                if any(abs(cx_full - px) < 20 and abs(cy_full - py) < 20
                       for px, py in used_centres):
                    continue

                # Depth & metric size
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

                aspect_err = abs(aspect - BRICK_ASPECT) / BRICK_ASPECT
                size_err = (abs(w_m - BRICK_LENGTH_M) / BRICK_LENGTH_M +
                            abs(h_m - BRICK_WIDTH_M) / BRICK_WIDTH_M) / 2.0

                # ROI in work_img coords for stud verification
                bx, by, bw, bh = cv2.boundingRect(cnt)
                pad = 5
                roi_x = max(0, bx - pad)
                roi_y = max(0, by - pad)
                roi_w = min(work_img.shape[1] - roi_x, bw + 2 * pad)
                roi_h = min(work_img.shape[0] - roi_y, bh + 2 * pad)

                # Offset contour to full image coords
                cnt_full = cnt.copy()
                cnt_full[:, :, 0] += offset_x
                cnt_full[:, :, 1] += offset_y

                used_centres.append((cx_full, cy_full))

                candidates.append({
                    "center_px":  (cx_full, cy_full),
                    "size_px":    (w_px, h_px),
                    "angle":      angle % 180,
                    "depth_m":    depth_m,
                    "size_m":     (w_m, h_m),
                    "colour":     colour_name,
                    "colour_bgr": colour_bgr,
                    "contour":    cnt_full,
                    "roi_local":  (roi_x, roi_y, roi_w, roi_h),
                    "aspect_err": aspect_err,
                    "size_err":   size_err,
                })

        return candidates

    # ── Secondary verification: stud detection via HoughCircles ───────────
    def _verify_studs_in_roi(self, work_img, roi_local, depth_m):
        """
        Look for circular studs inside a candidate's bounding region.

        Returns (stud_count, stud_circles) where stud_circles is a list
        of (cx, cy, r) in roi-local coords.
        """
        rx, ry, rw, rh = roi_local
        roi_img = work_img[ry:ry+rh, rx:rx+rw]

        if roi_img.size == 0:
            return 0, []

        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # CLAHE for contrast on dark bricks
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
        gray = clahe.apply(gray)

        # Expected stud radius in pixels
        expected_r_px = self.metres_to_pixels(STUD_DIAMETER_M / 2, depth_m)
        if expected_r_px < 3:
            return 0, []

        min_r = max(3, int(expected_r_px * (1 - HOUGH_RADIUS_TOL)))
        max_r = max(min_r + 1, int(expected_r_px * (1 + HOUGH_RADIUS_TOL)))
        min_dist = max(5, self.metres_to_pixels(HOUGH_MIN_DIST_M, depth_m))

        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT,
            dp=HOUGH_DP,
            minDist=min_dist,
            param1=HOUGH_PARAM1,
            param2=HOUGH_PARAM2,
            minRadius=min_r,
            maxRadius=max_r,
        )

        if circles is None:
            return 0, []

        circles = circles[0]
        # Cap at expected count to avoid over-counting from noise
        count = min(len(circles), EXPECTED_STUDS)
        stud_list = [(int(c[0]), int(c[1]), int(c[2])) for c in circles[:count]]
        return count, stud_list

    # ── Main detection entry point ────────────────────────────────────────
    def detect_bricks(self, color_image, depth_frame):
        """
        Hybrid detection: colour segmentation + stud verification.

        Returns a list of dicts:
          {
            "center_px":    (cx, cy),
            "center_3d":    [x, y, z],
            "size_px":      (w, h),
            "size_m":       (w_m, h_m),
            "angle":        float,
            "colour":       str,
            "colour_bgr":   (b, g, r),
            "stud_count":   int,
            "stud_circles": list,        # [(cx,cy,r), ...] in roi-local coords
            "roi_local":    (x, y, w, h),
            "contour":      np.ndarray,
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

        # Stage 1: colour + shape candidates
        candidates = self._find_colour_candidates(work_img, depth_frame, offset_x, offset_y)

        # Stage 2: verify studs and build final detections
        detections = []
        for cand in candidates:
            stud_count, stud_circles = self._verify_studs_in_roi(
                work_img, cand["roi_local"], cand["depth_m"])

            # Confidence: base from colour+size, bonus from studs
            base_conf = max(0.0, CONF_BASE_MAX - cand["aspect_err"] - cand["size_err"])

            if stud_count >= MIN_STUDS_FOR_BONUS:
                stud_ratio = min(stud_count, EXPECTED_STUDS) / EXPECTED_STUDS
                stud_bonus = stud_ratio * CONF_STUD_BONUS
            else:
                stud_bonus = 0.0

            confidence = round(min(1.0, base_conf + stud_bonus), 3)

            cx, cy = cand["center_px"]
            center_3d = self.pixel_to_3d(cx, cy, cand["depth_m"])

            detections.append({
                "center_px":    (cx, cy),
                "center_3d":    center_3d.tolist() if center_3d is not None else None,
                "size_px":      cand["size_px"],
                "size_m":       cand["size_m"],
                "angle":        cand["angle"],
                "colour":       cand["colour"],
                "colour_bgr":   cand["colour_bgr"],
                "stud_count":   stud_count,
                "stud_circles": stud_circles,
                "roi_local":    cand["roi_local"],
                "contour":      cand["contour"],
                "confidence":   confidence,
            })

        detections.sort(key=lambda d: d["confidence"], reverse=True)
        return detections

    def _sample_depth(self, depth_frame, cx, cy, radius=10):
        """Sample a small area around (cx, cy) and return median depth.

        Clamps every sampled pixel to the frame bounds to avoid librealsense
        raising ``RuntimeError: out of range value for argument "y"`` near
        image borders.
        """
        # Frame dimensions (librealsense exposes these on the depth frame)
        try:
            w = depth_frame.get_width()
            h = depth_frame.get_height()
        except Exception:
            w, h = 640, 480

        depths = []
        for dy in range(-radius, radius + 1, 2):
            for dx in range(-radius, radius + 1, 2):
                px = int(cx + dx)
                py = int(cy + dy)
                if px < 0 or py < 0 or px >= w or py >= h:
                    continue
                d = depth_frame.get_distance(px, py)
                if 0.05 < d < 2.0:
                    depths.append(d)
        return float(np.median(depths)) if depths else 0.0

    # ── Visualisation ──────────────────────────────────────────────────────
    def draw_detections(self, image, detections):
        """Draw bounding boxes, stud markers, and brick info."""
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
            stud_count = det["stud_count"]

            # Draw verified stud circles (offset from roi_local to full image)
            if det["stud_circles"] and self.calibration.calibrated and self.calibration.roi:
                roi_rx, roi_ry, _, _ = det["roi_local"]
                cal_rx, cal_ry = self.calibration.roi[0], self.calibration.roi[1]
                for (scx, scy, sr) in det["stud_circles"]:
                    sx = scx + roi_rx + cal_rx
                    sy = scy + roi_ry + cal_ry
                    cv2.circle(vis, (sx, sy), sr, (0, 255, 0), 1)
                    cv2.circle(vis, (sx, sy), 2, (0, 255, 0), -1)

            # Rotated bounding box in brick colour
            rect = ((cx, cy), det["size_px"], angle)
            box = cv2.boxPoints(rect).astype(int)
            cv2.drawContours(vis, [box], 0, box_colour, 2)

            # Centre crosshair
            cv2.drawMarker(vis, (cx, cy), (0, 255, 255),
                           cv2.MARKER_CROSS, 15, 2)

            # Colour tag
            cv2.putText(vis, colour.upper(), (cx - 30, cy - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_colour, 2)

            # Verification status + info
            verified = "VERIFIED" if stud_count >= MIN_STUDS_FOR_BONUS else "shape-only"
            info = f"studs={stud_count} [{verified}]  conf={conf:.2f}"
            cv2.putText(vis, info, (cx - 80, cy - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            w_m, h_m = det["size_m"]
            label = f"{w_m*1000:.0f}x{h_m*1000:.0f}mm  {angle:.0f}deg"
            cv2.putText(vis, label, (cx - 60, cy + 5),
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
def _rpy_to_quat(roll, pitch, yaw):
    """Roll/pitch/yaw (radians, XYZ intrinsic) → (qx, qy, qz, qw)."""
    cr, cp, cy = np.cos(roll/2), np.cos(pitch/2), np.cos(yaw/2)
    sr, sp, sy = np.sin(roll/2), np.sin(pitch/2), np.sin(yaw/2)
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*sy
    return float(qx), float(qy), float(qz), float(qw)


def main(args=None):
    """ROS 2 entry point (used by ros2 run / launch)."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped
    from vision_msgs.msg import (
        Detection3D, Detection3DArray,
        ObjectHypothesisWithPose, BoundingBox3D,
    )
    from cv_bridge import CvBridge

    class BrickDetectorNode(Node):
        def __init__(self):
            super().__init__("brick_detector")

            # Parameters
            self.declare_parameter("show_preview", True)
            self.show_preview = self.get_parameter("show_preview").value

            self.detector = BrickDetector()
            self.bridge = CvBridge()

            # Publishers
            #   ~/detections        – every brick this frame, each with pose + colour bound
            #   ~/brick_pose        – PoseStamped of the highest-confidence brick (legacy/easy)
            #   ~/detection_image   – annotated camera feed
            self.image_pub      = self.create_publisher(Image,             "~/detection_image", 10)
            self.detections_pub = self.create_publisher(Detection3DArray,  "~/detections",      10)
            self.pose_pub       = self.create_publisher(PoseStamped,       "~/brick_pose",      10)

            self.detector.start_camera()
            self.detector.calibration.load()

            self.timer = self.create_timer(1.0 / 15, self.detect_callback)  # 15 Hz
            self.get_logger().info(
                f"Brick detector node started (preview={'on' if self.show_preview else 'off'})")
            if self.show_preview:
                self.get_logger().info("Preview keys:  q=quit  c=recalibrate  s=snapshot")

        def detect_callback(self):
            color_image, depth_frame = self.detector.get_frames()
            if color_image is None:
                return

            detections = self.detector.detect_bricks(color_image, depth_frame)

            # Publish annotated image
            vis = self.detector.draw_detections(color_image, detections)
            img_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            self.image_pub.publish(img_msg)

            # Publish all bricks bound to their colour in a single message.
            stamp = self.get_clock().now().to_msg()
            frame = "camera_color_optical_frame"

            arr = Detection3DArray()
            arr.header.stamp = stamp
            arr.header.frame_id = frame

            for det in detections:
                if not det.get("center_3d"):
                    continue

                # Brick is flat on the workspace → roll = pitch = 0
                roll, pitch = 0.0, 0.0
                yaw = float(np.radians(det["angle"]))
                qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)

                d3 = Detection3D()
                d3.header.stamp = stamp
                d3.header.frame_id = frame

                bbox = BoundingBox3D()
                bbox.center.position.x = float(det["center_3d"][0])
                bbox.center.position.y = float(det["center_3d"][1])
                bbox.center.position.z = float(det["center_3d"][2])
                bbox.center.orientation.x = qx
                bbox.center.orientation.y = qy
                bbox.center.orientation.z = qz
                bbox.center.orientation.w = qw
                # Brick footprint (length × width) and an estimated height
                bbox.size.x = float(det["size_m"][0]) if det.get("size_m") else BRICK_LENGTH_M
                bbox.size.y = float(det["size_m"][1]) if det.get("size_m") else BRICK_WIDTH_M
                bbox.size.z = STUD_DIAMETER_M  # rough thickness incl. studs
                d3.bbox = bbox

                # Colour goes in the hypothesis class_id; confidence in score
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(det["colour"])
                hyp.hypothesis.score    = float(det["confidence"])
                hyp.pose.pose = bbox.center  # same pose, no covariance
                d3.results.append(hyp)

                arr.detections.append(d3)

            self.detections_pub.publish(arr)

            # Convenience: also publish the best brick's pose on its own topic
            if arr.detections:
                best = arr.detections[0]
                pose = PoseStamped()
                pose.header = best.header
                pose.pose = best.bbox.center
                self.pose_pub.publish(pose)

            # Local OpenCV preview window (mirrors standalone mode)
            if self.show_preview:
                cv2.imshow("Brick Detector (ROS)", vis)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info("Quit requested from preview window")
                    raise KeyboardInterrupt
                elif key == ord('c'):
                    if self.detector.calibration.calibrate_interactive(color_image, depth_frame):
                        self.detector.calibration.save()
                elif key == ord('s'):
                    fname = f"snapshot_{int(time.time())}.png"
                    cv2.imwrite(fname, vis)
                    self.get_logger().info(f"Saved snapshot {fname}")

        def destroy_node(self):
            if self.show_preview:
                cv2.destroyAllWindows()
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
