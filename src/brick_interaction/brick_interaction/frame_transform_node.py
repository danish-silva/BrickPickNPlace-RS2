#!/usr/bin/env python3
"""
frame_transform_node — bridges brick_vision and interaction_node.

Subscribes to brick_vision's camera-frame topics, transforms every pose
into the robot base frame via tf2, and re-publishes:

    /brick_detector/pickup_detections   →   /pickup_bricks            (Detection3DArray)
    /brick_detector/available_slots     →   /available_slots_base     (PoseArray)

Colour, confidence, and bbox dimensions are preserved for detections;
slot orientation (yaw) is preserved for slots.

    ros2 run brick_interaction frame_transform_node
    ros2 run brick_interaction frame_transform_node --ros-args \\
        -p target_frame:=base_link \\
        -p bricks_in:=/brick_detector/pickup_detections \\
        -p bricks_out:=/pickup_bricks \\
        -p slots_in:=/brick_detector/available_slots \\
        -p slots_out:=/available_slots_base
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import Pose, PoseArray, Quaternion
from vision_msgs.msg import Detection3DArray


def _quat_mul(a: Quaternion, b: Quaternion) -> Quaternion:
    """Hamilton product q = a * b (rotate-by-b then rotate-by-a, or vice versa
    depending on intent; here used to post-multiply a Z-axis correction onto
    an existing orientation: new = current * yaw_offset)."""
    x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    q = Quaternion()
    q.x, q.y, q.z, q.w = x, y, z, w
    return q


def _yaw_quat(angle_rad: float) -> Quaternion:
    """Quaternion representing pure rotation about the Z axis."""
    q = Quaternion()
    q.z = math.sin(angle_rad / 2.0)
    q.w = math.cos(angle_rad / 2.0)
    return q


class FrameTransformNode(Node):
    def __init__(self):
        super().__init__("frame_transform_node")

        self.declare_parameter("target_frame",      "base_link")
        self.declare_parameter("bricks_in",         "/brick_detector/pickup_detections")
        self.declare_parameter("bricks_out",        "/pickup_bricks")
        self.declare_parameter("slots_in",          "/brick_detector/available_slots")
        self.declare_parameter("slots_out",         "/available_slots_base")
        self.declare_parameter("tf_timeout_sec",    1.0)
        # Extra yaw (deg) applied to every brick pose AFTER the tf transform,
        # rotating around the target frame's Z axis. Set to 90 / -90 / 180
        # if the brick spawns in rviz rotated relative to the physical brick.
        # Default 0 — the motion node now respects the published yaw correctly.
        self.declare_parameter("yaw_correction_deg", 0.0)
        # Z lift applied to every placement slot AFTER the tf transform, in
        # the target frame. Compensates for two things at once:
        #   - half the brick collision-box height (~0.02 m) — MoveIt centres
        #     the box on the target, so without lift the box-bottom is
        #     INSIDE the build plate;
        #   - drop-release clearance (~0.01 m) so the brick is released
        #     fractionally above the plate, not pressed into it.
        # Set to 0 to disable.
        self.declare_parameter("slot_z_offset_m", 0.03)
        # Per-axis offsets (in the target frame) applied to every BRICK pose
        # AFTER the tf transform. Use these to nudge the pick target when the
        # gripper consistently lands offset from the brick. Set 0 to disable.
        self.declare_parameter("pickup_offset_x_m", 0.008)
        self.declare_parameter("pickup_offset_y_m", -0.02)
        self.declare_parameter("pickup_offset_z_m", 0.0)

        self.target_frame  = str(self.get_parameter("target_frame").value)
        bricks_in          = str(self.get_parameter("bricks_in").value)
        bricks_out         = str(self.get_parameter("bricks_out").value)
        slots_in           = str(self.get_parameter("slots_in").value)
        slots_out          = str(self.get_parameter("slots_out").value)
        self.tf_timeout    = float(self.get_parameter("tf_timeout_sec").value)
        self.yaw_correction = math.radians(
            float(self.get_parameter("yaw_correction_deg").value))
        self.yaw_offset_q = _yaw_quat(self.yaw_correction)
        self.slot_z_offset = float(self.get_parameter("slot_z_offset_m").value)
        self.pickup_dx = float(self.get_parameter("pickup_offset_x_m").value)
        self.pickup_dy = float(self.get_parameter("pickup_offset_y_m").value)
        self.pickup_dz = float(self.get_parameter("pickup_offset_z_m").value)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.bricks_pub = self.create_publisher(Detection3DArray, bricks_out, qos)
        self.slots_pub  = self.create_publisher(PoseArray,        slots_out,  qos)

        self.create_subscription(Detection3DArray, bricks_in,
                                 self._on_bricks, qos)
        self.create_subscription(PoseArray, slots_in,
                                 self._on_slots, qos)

        self.get_logger().info(
            f"bricks:  {bricks_in} → {self.target_frame} → {bricks_out}  "
            f"(yaw correction = {math.degrees(self.yaw_correction):+.0f} deg, "
            f"pick offset = "
            f"{self.pickup_dx*1000:+.0f}, {self.pickup_dy*1000:+.0f}, "
            f"{self.pickup_dz*1000:+.0f} mm)")
        self.get_logger().info(
            f"slots:   {slots_in} → {self.target_frame} → {slots_out}  "
            f"(z lift = +{self.slot_z_offset*1000:.0f} mm)")

    def _lookup(self, source_frame, stamp):
        """Look up source→target. Returns the TransformStamped or None."""
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"TF lookup failed ({source_frame} → {self.target_frame}): {e}")
            return None

    def _on_bricks(self, msg: Detection3DArray) -> None:
        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame

        if not msg.detections:
            self.bricks_pub.publish(out)
            return

        source_frame = msg.header.frame_id or "camera_color_optical_frame"
        tf = self._lookup(source_frame, msg.header.stamp)
        if tf is None:
            return

        for det in msg.detections:
            transformed_pose = do_transform_pose(det.bbox.center, tf)
            # Post-tf yaw correction (rotate brick about target-frame Z).
            # Disable by setting yaw_correction_deg parameter to 0.
            if self.yaw_correction != 0.0:
                transformed_pose.orientation = _quat_mul(
                    transformed_pose.orientation, self.yaw_offset_q)
            # Per-axis grasp nudge in target frame.
            transformed_pose.position.x += self.pickup_dx
            transformed_pose.position.y += self.pickup_dy
            transformed_pose.position.z += self.pickup_dz
            new_det = det
            new_det.header.frame_id = self.target_frame
            new_det.bbox.center = transformed_pose
            for hyp in new_det.results:
                hyp.pose.pose = transformed_pose
            out.detections.append(new_det)

        self.bricks_pub.publish(out)
        self.get_logger().debug(
            f"Transformed {len(out.detections)} brick(s) "
            f"{source_frame} → {self.target_frame}")

    def _on_slots(self, msg: PoseArray) -> None:
        out = PoseArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame

        if not msg.poses:
            self.slots_pub.publish(out)
            return

        source_frame = msg.header.frame_id or "camera_color_optical_frame"
        tf = self._lookup(source_frame, msg.header.stamp)
        if tf is None:
            return

        for pose in msg.poses:
            tp = do_transform_pose(pose, tf)
            # Lift the placement target by half a brick + clearance so the
            # collision-box bottom sits above the build plate (see param doc).
            if self.slot_z_offset != 0.0:
                tp.position.z += self.slot_z_offset
            out.poses.append(tp)

        self.slots_pub.publish(out)
        self.get_logger().debug(
            f"Transformed {len(out.poses)} slot(s) "
            f"{source_frame} → {self.target_frame}  "
            f"(z offset = +{self.slot_z_offset*1000:.0f} mm)")


def main(args=None):
    rclpy.init(args=args)
    node = FrameTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
