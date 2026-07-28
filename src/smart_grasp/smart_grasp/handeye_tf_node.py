#!/usr/bin/env python3
"""Publish the eye-in-hand TF chain without giving an optical frame two parents."""

from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from smart_grasp.depth_geometry import matrix_from_transform


def camera_mount_matrix(calibration):
    tcp_to_optical = matrix_from_transform(
        calibration["tcp_to_color_optical"]["translation"],
        calibration["tcp_to_color_optical"]["quaternion_xyzw"],
    )
    link_to_optical = matrix_from_transform(
        calibration["camera_link_to_color_optical"]["translation"],
        calibration["camera_link_to_color_optical"]["quaternion_xyzw"],
    )
    return tcp_to_optical @ np.linalg.inv(link_to_optical)


class HandeyeTfNode(Node):
    def __init__(self):
        super().__init__("smart_grasp_handeye_tf")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("tcp_pose_topic", "/feedback/tcp_pose")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "tcp_link")
        self.declare_parameter("mount_republish_period", 1.0)
        self.declare_parameter("mount_republish_count", 10)
        calibration_file = Path(
            self.get_parameter("calibration_file").value
        ).expanduser()
        if not calibration_file.is_file():
            raise FileNotFoundError(f"calibration file does not exist: {calibration_file}")
        with calibration_file.open("r", encoding="utf-8") as stream:
            self.calibration = yaml.safe_load(stream)
        self.mount_republish_period = float(self.calibration.get(
            "mount_republish_period",
            self.get_parameter("mount_republish_period").value,
        ))
        self.mount_republish_count = int(self.calibration.get(
            "mount_republish_count",
            self.get_parameter("mount_republish_count").value,
        ))
        if self.mount_republish_period <= 0.0 or self.mount_republish_count < 1:
            raise ValueError("camera mount republish period/count must be positive")
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.camera_mount_transform = self._camera_mount_transform()
        self.mount_publish_count = 0
        self._publish_camera_mount()
        self.mount_timer = self.create_timer(
            self.mount_republish_period,
            self._republish_camera_mount,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("tcp_pose_topic").value,
            self._tcp_callback,
            10,
        )
        validated = bool(self.calibration.get("validated", False))
        self.get_logger().info(
            f"hand-eye TF ready, calibration validated={validated}; "
            "validation is enforced independently by the pick action server"
        )

    def _camera_mount_transform(self):
        tcp_to_link = camera_mount_matrix(self.calibration)
        quaternion = Rotation.from_matrix(tcp_to_link[:3, :3]).as_quat()
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.get_parameter("tcp_frame").value
        transform.child_frame_id = self.calibration.get("camera_link_frame", "camera_link")
        (transform.transform.translation.x,
         transform.transform.translation.y,
         transform.transform.translation.z) = tcp_to_link[:3, 3]
        (transform.transform.rotation.x,
         transform.transform.rotation.y,
         transform.transform.rotation.z,
         transform.transform.rotation.w) = quaternion
        return transform

    def _publish_camera_mount(self):
        self.camera_mount_transform.header.stamp = self.get_clock().now().to_msg()
        self.static_broadcaster.sendTransform(self.camera_mount_transform)
        self.mount_publish_count += 1

    def _republish_camera_mount(self):
        if self.mount_publish_count >= self.mount_republish_count:
            self.mount_timer.cancel()
            self.get_logger().info(
                f"camera mount TF published {self.mount_publish_count} times; "
                "delayed republish complete"
            )
            return
        self._publish_camera_mount()

    def _tcp_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
            transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.get_parameter("base_frame").value
        transform.child_frame_id = self.get_parameter("tcp_frame").value
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation = msg.pose.orientation
        self.dynamic_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
