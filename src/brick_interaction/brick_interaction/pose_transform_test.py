#!/usr/bin/env python3
"""
Diagnostic: read brick poses from /brick_detector/detections (camera frame),
transform them into the robot base frame, and print them next to the current
tool0 pose so you can verify hand-eye calibration.

Use case: position the gripper manually over a brick, run this, and check that
the transformed brick pose roughly matches the gripper's TCP pose.

    ros2 run brick_interaction pose_transform_test
    ros2 run brick_interaction pose_transform_test --ros-args \\
        -p target_frame:=base_link -p tcp_frame:=tool0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped

from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray


class PoseTransformTest(Node):
    def __init__(self):
        super().__init__("pose_transform_test")

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tcp_frame",    "tool0")
        self.declare_parameter("topic",        "/brick_detector/detections")

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.tcp_frame    = str(self.get_parameter("tcp_frame").value)
        topic             = str(self.get_parameter("topic").value)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Match brick_detector's latched QoS so we get the most recent
        # snapshot immediately on subscribe (in on_trigger mode).
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Detection3DArray, topic, self._on_detections, qos)

        self.get_logger().info(
            f"Listening on {topic} — target frame: {self.target_frame}, "
            f"TCP frame: {self.tcp_frame}")

    def _on_detections(self, msg: Detection3DArray) -> None:
        if not msg.detections:
            self.get_logger().warn("Empty detection array")
            return

        # Pick the first brick. Swap this for color/index filter later.
        det = msg.detections[0]
        colour = det.results[0].hypothesis.class_id if det.results else "?"

        # 1. Brick pose in camera frame
        ps_cam = PoseStamped()
        ps_cam.header = det.header
        ps_cam.pose   = det.bbox.center

        # 2. Transform to base_link
        try:
            tf_cam_to_base = self.tf_buffer.lookup_transform(
                self.target_frame,
                ps_cam.header.frame_id,
                rclpy.time.Time(),                        # latest available
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"TF lookup failed ({ps_cam.header.frame_id} → "
                f"{self.target_frame}): {e}")
            return

        ps_base = do_transform_pose_stamped(ps_cam, tf_cam_to_base)

        # 3. Where is the gripper right now (in base frame)?
        try:
            tcp_tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.tcp_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            tcp_xyz = (tcp_tf.transform.translation.x,
                       tcp_tf.transform.translation.y,
                       tcp_tf.transform.translation.z)
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"TCP TF lookup failed: {e}")
            tcp_xyz = None

        bx, by, bz = (ps_base.pose.position.x,
                      ps_base.pose.position.y,
                      ps_base.pose.position.z)
        cx, cy, cz = (ps_cam.pose.position.x,
                      ps_cam.pose.position.y,
                      ps_cam.pose.position.z)

        msg_lines = [
            "─" * 60,
            f" Bricks in array: {len(msg.detections)}    (showing #0, colour={colour})",
            f" Camera frame [{ps_cam.header.frame_id}]:",
            f"     x={cx:+.4f}  y={cy:+.4f}  z={cz:+.4f}",
            f" Base frame   [{self.target_frame}]:",
            f"     x={bx:+.4f}  y={by:+.4f}  z={bz:+.4f}",
        ]
        if tcp_xyz is not None:
            tx, ty, tz = tcp_xyz
            msg_lines += [
                f" Current TCP  [{self.tcp_frame} in {self.target_frame}]:",
                f"     x={tx:+.4f}  y={ty:+.4f}  z={tz:+.4f}",
                f" Δ (brick − TCP):",
                f"     dx={bx-tx:+.4f}  dy={by-ty:+.4f}  dz={bz-tz:+.4f}",
            ]
        msg_lines.append("─" * 60)
        self.get_logger().info("\n" + "\n".join(msg_lines))


def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
