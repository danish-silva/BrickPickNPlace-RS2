#!/usr/bin/env python3
"""
Brick Detector - Detects 4x2 LEGO bricks AND tracks free build-zone slots.

Brick specifications:
  - 4x2 studs, 100mm x 50mm footprint
  - Stud diameter: 15mm, pitch (centre-to-centre): 25mm
  - Colour detection: black, red, orange, yellow, green, blue, purple
  - Hybrid approach: colour segmentation (primary) + stud verification (secondary)

Build zone:
  - 12 (cols) x 14 (rows) studs at 25mm pitch
  - Calibrated by clicking the centres of the top-left and bottom-right studs
  - Free studs and valid 4x2 placement slots (with a 1-stud gap rule between
    bricks) are computed each frame and published.

Usage:
  Standalone:  python3 brick_detector.py
  ROS2 node:   ros2 run brick_vision brick_detector

Keys (preview window):
  q  quit
  c  re-calibrate workspace ROI
  b  re-calibrate build-zone (click TL stud then BR stud)
  s  save snapshot
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

# ── Build-zone grid ───────────────────────────────────────────────────────
BUILD_GRID_COLS      = 12       # number of stud columns on the build plate
BUILD_GRID_ROWS      = 14       # number of stud rows on the build plate
STUD_PITCH_M         = 0.025    # 25 mm centre-to-centre spacing
PLACEMENT_GAP_STUDS  = 1        # required free stud border between placed bricks
BRICK_STUDS_LONG     = 4        # studs along the brick's long axis (100 mm)
BRICK_STUDS_SHORT    = 2        # studs along the brick's short axis (50 mm)

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
        # Build-zone stud grid: pixel centres of the corner studs.
        # The full 12x14 grid is interpolated linearly between these two points,
        # which assumes the grid is approximately axis-aligned with the image.
        self.build_zone_tl = None  # (px, py) of stud (row=0, col=0)
        self.build_zone_br = None  # (px, py) of stud (row=ROWS-1, col=COLS-1)

    @property
    def has_build_zone(self):
        return self.build_zone_tl is not None and self.build_zone_br is not None

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

    def calibrate_build_zone(self, color_image):
        """Click the centre of the TL stud, then the centre of the BR stud."""
        clone = color_image.copy()
        points = []
        instructions = [
            f"Click the CENTRE of the TOP-LEFT stud (row 0, col 0)",
            f"Click the CENTRE of the BOTTOM-RIGHT stud "
            f"(row {BUILD_GRID_ROWS-1}, col {BUILD_GRID_COLS-1})",
        ]
        win = "Build-Zone Calibration"

        def render():
            img = clone.copy()
            for (px, py) in points:
                cv2.circle(img, (px, py), 6, (0, 255, 255), 2)
                cv2.drawMarker(img, (px, py), (0, 255, 255),
                               cv2.MARKER_CROSS, 12, 1)
            if len(points) == 2:
                cv2.line(img, points[0], points[1], (0, 255, 255), 1)
            msg = instructions[len(points)] if len(points) < 2 \
                  else "ENTER to accept, ESC to cancel"
            cv2.putText(img, msg, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(win, img)

        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
                points.append((x, y))
                render()

        cv2.namedWindow(win)
        cv2.setMouseCallback(win, on_mouse)
        render()
        print("[Build Zone] Click TL stud → click BR stud → press ENTER")

        while True:
            key = cv2.waitKey(30) & 0xFF
            if key == 13 and len(points) == 2:  # ENTER
                break
            if key == 27:  # ESC
                cv2.destroyWindow(win)
                return False

        self.build_zone_tl = tuple(points[0])
        self.build_zone_br = tuple(points[1])
        cv2.destroyWindow(win)
        print(f"[Build Zone] TL={self.build_zone_tl}  BR={self.build_zone_br}")
        return True

    def save(self, path=None):
        path = path or CALIBRATION_FILE
        os.makedirs(os.path.dirname(path), exist_ok=True)
        data = {
            "roi":           self.roi,
            "plane_depth":   self.plane_depth,
            "build_zone_tl": self.build_zone_tl,
            "build_zone_br": self.build_zone_br,
        }
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
            self.roi = tuple(data["roi"]) if data.get("roi") else None
            self.plane_depth = data.get("plane_depth")
            self.calibrated = self.roi is not None
            tl = data.get("build_zone_tl")
            br = data.get("build_zone_br")
            self.build_zone_tl = tuple(tl) if tl else None
            self.build_zone_br = tuple(br) if br else None
            extras = " + build zone" if self.has_build_zone else ""
            print(f"[Calibration] Loaded from {p}{extras}")
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

    # ── Build-zone analysis ──────────────────────────────────────────────
    def _stud_grid_px(self):
        """Return a (ROWS, COLS, 2) array of stud pixel centres, or None.

        Linear interpolation between the two calibrated corner studs. Assumes
        the grid is approximately axis-aligned with the image (true for a
        top-down camera mount).
        """
        if not self.calibration.has_build_zone:
            return None
        tl = np.array(self.calibration.build_zone_tl, dtype=float)
        br = np.array(self.calibration.build_zone_br, dtype=float)
        rows, cols = BUILD_GRID_ROWS, BUILD_GRID_COLS
        grid = np.zeros((rows, cols, 2), dtype=float)
        for r in range(rows):
            tr = r / max(1, rows - 1)
            for c in range(cols):
                tc = c / max(1, cols - 1)
                grid[r, c, 0] = tl[0] + tc * (br[0] - tl[0])
                grid[r, c, 1] = tl[1] + tr * (br[1] - tl[1])
        return grid

    def _stud_grid_3d(self, grid_px, depth_frame):
        """Deproject every stud to 3D camera-frame coords. NaN where depth fails."""
        if grid_px is None or self.intrinsics is None:
            return None
        rows, cols, _ = grid_px.shape
        grid_3d = np.full((rows, cols, 3), np.nan, dtype=float)
        try:
            fw = depth_frame.get_width()
            fh = depth_frame.get_height()
        except Exception:
            fw, fh = 640, 480

        for r in range(rows):
            for c in range(cols):
                px = int(round(grid_px[r, c, 0]))
                py = int(round(grid_px[r, c, 1]))
                if px < 0 or py < 0 or px >= fw or py >= fh:
                    continue
                d = self._sample_depth(depth_frame, px, py, radius=4)
                if d <= 0:
                    continue
                xyz = rs.rs2_deproject_pixel_to_point(self.intrinsics, [px, py], d)
                grid_3d[r, c] = xyz
        return grid_3d

    @staticmethod
    def _stud_inside_brick(stud_xy, brick_center_xy, brick_size_px, brick_angle_deg):
        """Is the stud pixel inside the brick's oriented bounding box?"""
        dx = stud_xy[0] - brick_center_xy[0]
        dy = stud_xy[1] - brick_center_xy[1]
        a = -np.radians(brick_angle_deg)
        cos_a, sin_a = np.cos(a), np.sin(a)
        lx = cos_a * dx - sin_a * dy
        ly = sin_a * dx + cos_a * dy
        return (abs(lx) <= brick_size_px[0] / 2 and
                abs(ly) <= brick_size_px[1] / 2)

    def _stud_occupancy(self, grid_px, detections):
        """Bool array (ROWS, COLS): True if a stud is under any detected brick."""
        rows, cols, _ = grid_px.shape
        occ = np.zeros((rows, cols), dtype=bool)
        for r in range(rows):
            for c in range(cols):
                stud = grid_px[r, c]
                for det in detections:
                    if self._stud_inside_brick(
                        stud, det["center_px"], det["size_px"], det["angle"]):
                        occ[r, c] = True
                        break
        return occ

    def _find_available_slots(self, occupancy):
        """Sliding-window 4x2 (and 2x4) over the grid with a 1-stud gap rule.

        A slot is valid when the brick footprint AND the surrounding 1-stud
        ring (clamped to the grid) contain no occupied studs.
        """
        rows, cols = occupancy.shape
        slots = []
        for orient_label, gh, gw in [
            ("horizontal", BRICK_STUDS_SHORT, BRICK_STUDS_LONG),  # 2 rows x 4 cols
            ("vertical",   BRICK_STUDS_LONG,  BRICK_STUDS_SHORT), # 4 rows x 2 cols
        ]:
            if gh > rows or gw > cols:
                continue
            for r in range(rows - gh + 1):
                for c in range(cols - gw + 1):
                    if occupancy[r:r+gh, c:c+gw].any():
                        continue
                    gap = PLACEMENT_GAP_STUDS
                    r0 = max(0, r - gap)
                    r1 = min(rows, r + gh + gap)
                    c0 = max(0, c - gap)
                    c1 = min(cols, c + gw + gap)
                    if occupancy[r0:r1, c0:c1].any():
                        continue
                    slots.append({
                        "orient":     orient_label,
                        "row":        r,
                        "col":        c,
                        "gh":         gh,
                        "gw":         gw,
                        "center_rc":  (r + (gh - 1) / 2.0, c + (gw - 1) / 2.0),
                    })
        return slots

    @staticmethod
    def _bilinear(grid, fr_rc):
        """Bilinear sample of a grid (..., D) at fractional row/col coord."""
        fr, fc = fr_rc
        rows, cols = grid.shape[0], grid.shape[1]
        r0 = int(np.floor(fr)); r1 = min(rows - 1, r0 + 1)
        c0 = int(np.floor(fc)); c1 = min(cols - 1, c0 + 1)
        wr = fr - r0
        wc = fc - c0
        a = grid[r0, c0]; b = grid[r0, c1]
        c = grid[r1, c0]; d = grid[r1, c1]
        return ((1 - wr) * ((1 - wc) * a + wc * b) +
                wr * ((1 - wc) * c + wc * d))

    def _slot_pose(self, slot, grid_px, grid_3d):
        """Compute pixel + 3D centre of a slot, plus its yaw and footprint."""
        cr, cc = slot["center_rc"]
        px = self._bilinear(grid_px, (cr, cc))
        xyz = self._bilinear(grid_3d, (cr, cc)) if grid_3d is not None else None
        if xyz is not None and np.any(np.isnan(xyz)):
            xyz = None

        # Slot orientation in image axes (assumes axis-aligned grid):
        #   horizontal slot → brick long axis along image x → yaw = 0
        #   vertical slot   → brick long axis along image y → yaw = π/2
        yaw = 0.0 if slot["orient"] == "horizontal" else float(np.pi / 2)

        # Footprint dimensions (long, short)
        if slot["orient"] == "horizontal":
            size_m = (BRICK_LENGTH_M, BRICK_WIDTH_M)
        else:
            size_m = (BRICK_WIDTH_M, BRICK_LENGTH_M)

        return {
            "row":     slot["row"],
            "col":     slot["col"],
            "gh":      slot["gh"],
            "gw":      slot["gw"],
            "orient":  slot["orient"],
            "px":      (int(round(px[0])), int(round(px[1]))),
            "xyz":     tuple(map(float, xyz)) if xyz is not None else None,
            "yaw":     yaw,
            "size_m":  size_m,
        }

    def analyze_buildzone(self, depth_frame, detections):
        """Return dict of build-zone state, or None if not calibrated.

        Result keys:
          studs_px    – (ROWS, COLS, 2) pixel centres
          studs_3d    – (ROWS, COLS, 3) camera-frame coords (NaN where missing)
          occupancy   – (ROWS, COLS) bool, True if covered by a detected brick
          free_studs  – list of {"row","col","px","xyz"} for every free stud
          slots       – list of slot pose dicts (see _slot_pose)
        """
        grid_px = self._stud_grid_px()
        if grid_px is None:
            return None

        grid_3d = self._stud_grid_3d(grid_px, depth_frame)
        occupancy = self._stud_occupancy(grid_px, detections)

        free_studs = []
        rows, cols = occupancy.shape
        for r in range(rows):
            for c in range(cols):
                if occupancy[r, c]:
                    continue
                px = grid_px[r, c]
                xyz = grid_3d[r, c] if grid_3d is not None else None
                if xyz is not None and np.any(np.isnan(xyz)):
                    xyz = None
                free_studs.append({
                    "row": r, "col": c,
                    "px":  (int(round(px[0])), int(round(px[1]))),
                    "xyz": tuple(map(float, xyz)) if xyz is not None else None,
                })

        raw_slots = self._find_available_slots(occupancy)
        slots = [self._slot_pose(s, grid_px, grid_3d) for s in raw_slots]

        return {
            "studs_px":   grid_px,
            "studs_3d":   grid_3d,
            "occupancy":  occupancy,
            "free_studs": free_studs,
            "slots":      slots,
        }

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
    def draw_detections(self, image, detections, build_info=None):
        """Draw bounding boxes, stud markers, brick info, and (optionally)
        the build-zone stud grid + available placement slots."""
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

        # Build-zone overlay
        if build_info is not None:
            self._draw_buildzone(vis, build_info)

        return vis

    def _draw_buildzone(self, vis, build_info):
        """Overlay the stud grid, occupancy, and available placement slots."""
        grid_px = build_info["studs_px"]
        occ = build_info["occupancy"]
        slots = build_info["slots"]
        rows, cols, _ = grid_px.shape

        # Build-zone perimeter (corner studs as bounds)
        tl = tuple(map(int, grid_px[0, 0]))
        tr = tuple(map(int, grid_px[0, cols - 1]))
        bl = tuple(map(int, grid_px[rows - 1, 0]))
        br = tuple(map(int, grid_px[rows - 1, cols - 1]))
        cv2.polylines(vis, [np.array([tl, tr, br, bl], dtype=np.int32)],
                      isClosed=True, color=(255, 200, 0), thickness=1)
        cv2.putText(vis, "Build Zone", (tl[0], tl[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)

        # Studs: green dot if free, red X if occupied
        for r in range(rows):
            for c in range(cols):
                p = (int(round(grid_px[r, c, 0])), int(round(grid_px[r, c, 1])))
                if occ[r, c]:
                    cv2.drawMarker(vis, p, (0, 0, 255),
                                   cv2.MARKER_TILTED_CROSS, 7, 1)
                else:
                    cv2.circle(vis, p, 2, (0, 220, 0), -1)

        # Available slots: cyan polygon outline + filled centre dot
        for slot in slots:
            r, c, gh, gw = slot["row"], slot["col"], slot["gh"], slot["gw"]
            corners = np.array([
                grid_px[r,            c           ],
                grid_px[r,            c + gw - 1  ],
                grid_px[r + gh - 1,   c + gw - 1  ],
                grid_px[r + gh - 1,   c           ],
            ], dtype=float)
            # Expand outwards by half a stud pitch so the rectangle reflects
            # the actual brick footprint instead of the inner stud bounds.
            centroid = corners.mean(axis=0)
            edge_h = np.linalg.norm(corners[1] - corners[0]) / max(1, gw - 1) if gw > 1 else 0
            edge_v = np.linalg.norm(corners[3] - corners[0]) / max(1, gh - 1) if gh > 1 else 0
            pitch_px = max(edge_h, edge_v, 8.0)
            expanded = []
            for p in corners:
                d = p - centroid
                n = np.linalg.norm(d)
                if n > 1e-6:
                    p = p + d / n * (pitch_px * 0.5)
                expanded.append(p)
            poly = np.array(expanded, dtype=np.int32)
            cv2.polylines(vis, [poly], isClosed=True,
                          color=(255, 255, 0), thickness=1)
            cv2.circle(vis, slot["px"], 3, (255, 255, 0), -1)

        # HUD: counts
        n_free = len(build_info["free_studs"])
        n_total = rows * cols
        n_slots = len(slots)
        cv2.putText(vis, f"Studs: {n_free}/{n_total} free   Slots: {n_slots}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 0), 2)

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

        print("[Detector] Running – keys: q=quit  c=workspace  b=build-zone  s=snapshot")

        try:
            while True:
                color_image, depth_frame = self.get_frames()
                if color_image is None:
                    continue

                detections = self.detect_bricks(color_image, depth_frame)
                build_info = self.analyze_buildzone(depth_frame, detections)

                vis = self.draw_detections(color_image, detections, build_info)

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
                elif key == ord('b'):
                    if self.calibration.calibrate_build_zone(color_image):
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
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Empty
    from geometry_msgs.msg import PoseStamped, PoseArray, Pose
    from vision_msgs.msg import (
        Detection3D, Detection3DArray,
        ObjectHypothesisWithPose, BoundingBox3D,
    )
    from cv_bridge import CvBridge

    class BrickDetectorNode(Node):
        def __init__(self):
            super().__init__("brick_detector")

            # ── Parameters ──────────────────────────────────────────────
            self.declare_parameter("show_preview",    True)
            self.declare_parameter("mode",            "on_trigger")  # or "continuous"
            self.declare_parameter("trigger_topic",   "/snapshot_trigger")
            self.declare_parameter("snapshot_frames", 5)
            self.show_preview    = bool(self.get_parameter("show_preview").value)
            self.mode            = str(self.get_parameter("mode").value)
            self.trigger_topic   = str(self.get_parameter("trigger_topic").value)
            self.snapshot_frames = int(self.get_parameter("snapshot_frames").value)
            if self.mode not in ("continuous", "on_trigger"):
                self.get_logger().warn(
                    f"Unknown mode '{self.mode}', falling back to 'on_trigger'")
                self.mode = "on_trigger"

            # ── State ───────────────────────────────────────────────────
            self.detector = BrickDetector()
            self.bridge = CvBridge()
            self._latest_vis = None         # last annotated frame (no status overlay)
            self._last_snapshot_time = None
            self._capturing = False         # re-entrance guard for on_trigger mode

            # ── Publishers ──────────────────────────────────────────────
            # Snapshot results use TRANSIENT_LOCAL durability in on_trigger
            # mode so late subscribers immediately receive the most recent
            # snapshot. The image topic stays on default QoS (image streams
            # shouldn't be latched).
            if self.mode == "on_trigger":
                results_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
            else:
                results_qos = 10  # default

            self.image_pub      = self.create_publisher(Image,            "~/detection_image", 10)
            self.detections_pub = self.create_publisher(Detection3DArray, "~/detections",      results_qos)
            self.pose_pub       = self.create_publisher(PoseStamped,      "~/brick_pose",      results_qos)
            self.free_studs_pub = self.create_publisher(PoseArray,        "~/free_studs",      results_qos)
            self.slots_pub      = self.create_publisher(PoseArray,        "~/available_slots", results_qos)

            # ── Camera ──────────────────────────────────────────────────
            self.detector.start_camera()
            self.detector.calibration.load()

            # ── Mode wiring ─────────────────────────────────────────────
            if self.mode == "continuous":
                self.timer = self.create_timer(1.0 / 15, self._continuous_tick)
            else:
                self.trigger_sub = self.create_subscription(
                    Empty, self.trigger_topic, self._on_trigger, 10)

            # Always-on slow tick keeps the preview window responsive in
            # on_trigger mode (between snapshots) and handles key input.
            if self.show_preview:
                self.preview_timer = self.create_timer(1.0 / 10, self._preview_tick)

            self.get_logger().info(
                f"brick_detector started — mode={self.mode}, preview={self.show_preview}")
            if self.mode == "on_trigger":
                self.get_logger().info(
                    f"Awaiting snapshots on '{self.trigger_topic}' "
                    f"({self.snapshot_frames} frames per trigger)")
            if self.show_preview:
                self.get_logger().info(
                    "Preview keys:  q=quit  t=trigger  c=workspace  b=build-zone  s=snapshot")

        # ── Continuous-mode tick ────────────────────────────────────────
        def _continuous_tick(self):
            color_image, depth_frame = self.detector.get_frames()
            if color_image is None:
                return
            detections = self.detector.detect_bricks(color_image, depth_frame)
            build_info = self.detector.analyze_buildzone(depth_frame, detections)
            self._publish_results(self.get_clock().now().to_msg(),
                                  color_image, detections, build_info)

        # ── On-trigger mode ─────────────────────────────────────────────
        def _on_trigger(self, _msg):
            if self._capturing:
                self.get_logger().warn("Snapshot in progress — ignoring trigger")
                return
            self._capturing = True
            try:
                best = self._capture_and_analyze(self.snapshot_frames)
                if best is None:
                    self.get_logger().warn("Snapshot failed: no valid frames")
                    return
                color_image, _depth_frame, detections, build_info = best
                self._publish_results(self.get_clock().now().to_msg(),
                                      color_image, detections, build_info)
                self._last_snapshot_time = time.time()
                n_slots = len(build_info["slots"]) if build_info else 0
                self.get_logger().info(
                    f"Snapshot taken: {len(detections)} bricks, {n_slots} slots")
            finally:
                self._capturing = False

        def _capture_and_analyze(self, n_frames):
            """Capture N frames, run detection + build-zone analysis on each,
            and return the (color_image, depth_frame, detections, build_info)
            tuple from the highest-scoring frame.

            Score = number of bricks + 0.001 * sum(confidences) so ties break
            by total detection confidence.
            """
            best = None
            best_score = -1.0
            for _ in range(max(1, n_frames)):
                color_image, depth_frame = self.detector.get_frames()
                if color_image is None:
                    continue
                detections = self.detector.detect_bricks(color_image, depth_frame)
                build_info = self.detector.analyze_buildzone(depth_frame, detections)
                score = (len(detections) +
                         0.001 * sum(d.get("confidence", 0.0) for d in detections))
                if score > best_score:
                    best_score = score
                    best = (color_image, depth_frame, detections, build_info)
            return best

        # ── Preview-only tick (display + keys) ──────────────────────────
        def _preview_tick(self):
            if self._latest_vis is not None:
                vis = self._latest_vis.copy()
            else:
                # No snapshot yet → show live feed so the user knows the
                # camera is alive while waiting for the first trigger.
                color_image, _ = self.detector.get_frames()
                if color_image is None:
                    return
                vis = color_image.copy()
                cv2.putText(vis, "(awaiting trigger)", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            self._draw_status_overlay(vis)
            cv2.imshow("Brick Detector (ROS)", vis)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.get_logger().info("Quit requested from preview window")
                raise KeyboardInterrupt
            elif key == ord('t'):
                # Manual override: fire the same path as a /snapshot_trigger
                # message. Works in both modes for independent testing.
                self.get_logger().info("Manual trigger (key 't')")
                self._on_trigger(None)
            elif key in (ord('c'), ord('b'), ord('s')):
                # Calibration needs a fresh frame; snapshot saves the cached vis
                if key == ord('s'):
                    fname = f"snapshot_{int(time.time())}.png"
                    cv2.imwrite(fname, vis)
                    self.get_logger().info(f"Saved snapshot {fname}")
                    return
                color_image, depth_frame = self.detector.get_frames()
                if color_image is None:
                    return
                if key == ord('c'):
                    if self.detector.calibration.calibrate_interactive(color_image, depth_frame):
                        self.detector.calibration.save()
                elif key == ord('b'):
                    if self.detector.calibration.calibrate_build_zone(color_image):
                        self.detector.calibration.save()

        # ── Publish helper (shared between modes) ───────────────────────
        def _publish_results(self, stamp, color_image, detections, build_info):
            frame = "camera_color_optical_frame"

            # Annotated image (cached for preview, also published as-is)
            vis = self.detector.draw_detections(color_image, detections, build_info)
            self._latest_vis = vis
            img_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            self.image_pub.publish(img_msg)

            # Bricks → Detection3DArray
            arr = Detection3DArray()
            arr.header.stamp = stamp
            arr.header.frame_id = frame
            for det in detections:
                if not det.get("center_3d"):
                    continue
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
                bbox.size.x = float(det["size_m"][0]) if det.get("size_m") else BRICK_LENGTH_M
                bbox.size.y = float(det["size_m"][1]) if det.get("size_m") else BRICK_WIDTH_M
                bbox.size.z = STUD_DIAMETER_M
                d3.bbox = bbox

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(det["colour"])
                hyp.hypothesis.score    = float(det["confidence"])
                hyp.pose.pose = bbox.center
                d3.results.append(hyp)
                arr.detections.append(d3)

            self.detections_pub.publish(arr)

            if arr.detections:
                best = arr.detections[0]
                pose = PoseStamped()
                pose.header = best.header
                pose.pose = best.bbox.center
                self.pose_pub.publish(pose)

            # Build-zone outputs (always publish, even empty, so latched
            # subscribers get a consistent snapshot view)
            free_arr = PoseArray()
            free_arr.header.stamp = stamp
            free_arr.header.frame_id = frame
            slots_arr = PoseArray()
            slots_arr.header.stamp = stamp
            slots_arr.header.frame_id = frame

            if build_info is not None:
                for stud in build_info["free_studs"]:
                    if stud["xyz"] is None:
                        continue
                    p = Pose()
                    p.position.x = stud["xyz"][0]
                    p.position.y = stud["xyz"][1]
                    p.position.z = stud["xyz"][2]
                    p.orientation.w = 1.0
                    free_arr.poses.append(p)
                for slot in build_info["slots"]:
                    if slot["xyz"] is None:
                        continue
                    p = Pose()
                    p.position.x = slot["xyz"][0]
                    p.position.y = slot["xyz"][1]
                    p.position.z = slot["xyz"][2]
                    qx, qy, qz, qw = _rpy_to_quat(0.0, 0.0, slot["yaw"])
                    p.orientation.x = qx
                    p.orientation.y = qy
                    p.orientation.z = qz
                    p.orientation.w = qw
                    slots_arr.poses.append(p)

            self.free_studs_pub.publish(free_arr)
            self.slots_pub.publish(slots_arr)

        def _draw_status_overlay(self, vis):
            if self.mode == "on_trigger":
                if self._last_snapshot_time is None:
                    age_str = "no snapshot yet"
                else:
                    age = time.time() - self._last_snapshot_time
                    age_str = f"last snapshot: {age:.1f}s ago"
                txt = f"MODE: on_trigger | {age_str}  [t]=manual"
            else:
                txt = "MODE: continuous  [t]=re-snap"
            cv2.putText(vis, txt, (10, vis.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

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
