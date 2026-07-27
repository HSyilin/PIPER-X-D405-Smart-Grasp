#!/usr/bin/env python3
"""Blue-block instance detection and timestamped RGB-D 3-D localization."""

import math
import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from smart_grasp.depth_geometry import (
    depth_to_meters,
    estimate_oriented_box,
    fit_horizontal_plane_z,
    make_grasp_candidates,
    masked_points,
    matrix_from_transform,
    quaternion_from_axes,
    robust_filter,
    size_matches,
    transform_points,
)
from smart_grasp.detection_backends import HsvBackend, YoloSegBackend
from smart_grasp.stability import PoseStabilityWindow
from smart_grasp_interfaces.msg import DetectedObject


class DetectorNode(Node):
    def __init__(self):
        super().__init__("smart_grasp_detector")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.camera_frame = ""
        self.backend = self._create_backend()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tracks = {}
        self.next_track_id = 1

        self.camera_only = bool(self.get_parameter("camera_only").value)
        if self.camera_only:
            self.color_subscription = self.create_subscription(
                Image,
                self.get_parameter("color_topic").value,
                self._color_only_callback,
                qos_profile_sensor_data,
            )
            self.sync = None
        else:
            color = message_filters.Subscriber(
                self, Image, self.get_parameter("color_topic").value,
                qos_profile=qos_profile_sensor_data,
            )
            depth = message_filters.Subscriber(
                self, Image, self.get_parameter("depth_topic").value,
                qos_profile=qos_profile_sensor_data,
            )
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [color, depth], queue_size=8, slop=0.06
            )
            self.sync.registerCallback(self._image_callback)
        self.create_subscription(
            CameraInfo,
            self.get_parameter("info_topic").value,
            self._info_callback,
            qos_profile_sensor_data,
        )

        self.detection_pub = self.create_publisher(
            DetectedObject, "/smart_grasp/detections", 10
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/smart_grasp/object_cloud", 1
        )
        self.object_pose_pub = self.create_publisher(
            PoseStamped, "/smart_grasp/object_pose", 1
        )
        self.candidate_pub = self.create_publisher(
            PoseArray, "/smart_grasp/grasp_candidates", 1
        )
        self.debug_pub = self.create_publisher(Image, "/smart_grasp/debug_image", 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/smart_grasp/debug_markers", 1
        )
        self.get_logger().info(
            f"detector ready: backend={self.get_parameter('detector_backend').value}, "
            f"target={self.get_parameter('target_class').value}, "
            f"camera_only={self.camera_only}"
        )

    def _declare_parameters(self):
        defaults = {
            "camera_only": False,
            "detector_backend": "hsv",
            "target_class": "blue_block",
            "hsv_lower": [90, 80, 50],
            "hsv_upper": [135, 255, 255],
            "min_contour_area": 500.0,
            "min_solidity": 0.85,
            "min_rectangularity": 0.65,
            "yolo_model": "",
            "yolo_class": "blue_block",
            "yolo_confidence": 0.70,
            "min_depth": 0.08,
            "max_depth": 0.60,
            "depth_scale": 0.001,
            "mask_erode_pixels": 3,
            "min_depth_points": 500,
            "min_depth_valid_ratio": 0.60,
            "expected_size": [0.060, 0.040, 0.040],
            "size_tolerance": 0.012,
            "table_height": -999.0,
            "table_ransac_threshold": 0.004,
            "stability_frames": 10,
            "position_outlier_radius": 0.020,
            "max_position_span": 0.015,
            "max_yaw_span_deg": 5.0,
            "ambiguity_score_gap": 0.10,
            "grasp_depth": 0.020,
            "tcp_to_grasp_xyz": [0.0, 0.0, 0.1425],
            "base_frame": "base_link",
            "camera_frame": "camera_color_optical_frame",
            "color_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "info_topic": "/camera/camera/color/camera_info",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _color_only_callback(self, color_msg):
        """Publish 2-D detector diagnostics without depth, robot feedback, or TF."""
        bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        instances = self.backend.detect(bgr)
        debug = bgr.copy()
        if not instances:
            cv2.putText(
                debug, "CAMERA_ONLY: NO_TARGET", (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2,
            )
            self._publish_debug(debug, color_msg)
            return

        overlay = debug.copy()
        for instance in instances:
            overlay[instance.binary_mask > 0] = (255, 100, 0)
        debug = cv2.addWeighted(overlay, 0.25, debug, 0.75, 0.0)

        for instance in instances:
            x, y, w, h = instance.bounding_rect
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 220, 0), 2)
            label = (
                f"{instance.class_name} conf={instance.confidence:.2f} "
                f"angle={instance.angle_deg:.1f} CAMERA_ONLY"
            )
            cv2.putText(
                debug, label, (x, max(18, y - 7)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 0), 1, cv2.LINE_AA,
            )

            msg = DetectedObject()
            msg.header = color_msg.header
            msg.class_name = instance.class_name
            msg.confidence = float(instance.confidence)
            msg.stable = False
            msg.rejection_reason = "CAMERA_ONLY_2D"
            self.detection_pub.publish(msg)

        self._publish_debug(debug, color_msg)

    def _create_backend(self):
        mode = self.get_parameter("detector_backend").value
        if mode == "hsv":
            return HsvBackend(
                self.get_parameter("hsv_lower").value,
                self.get_parameter("hsv_upper").value,
                self.get_parameter("min_contour_area").value,
                self.get_parameter("min_solidity").value,
                self.get_parameter("min_rectangularity").value,
            )
        if mode == "yolo_seg":
            return YoloSegBackend(
                self.get_parameter("yolo_model").value,
                self.get_parameter("yolo_class").value,
                self.get_parameter("yolo_confidence").value,
            )
        raise RuntimeError(f"unsupported detector_backend: {mode}")

    def _info_callback(self, msg):
        self.camera_matrix = np.asarray(msg.k, dtype=float).reshape((3, 3))
        self.camera_frame = msg.header.frame_id

    def _lookup_transform(self, stamp, source_frame):
        transform = self.tf_buffer.lookup_transform(
            self.get_parameter("base_frame").value,
            source_frame,
            rclpy.time.Time.from_msg(stamp),
            timeout=Duration(seconds=0.10),
        ).transform
        return matrix_from_transform(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w],
        )

    def _assign_track(self, center):
        now = time.monotonic()
        expired = [key for key, value in self.tracks.items() if now - value["last"] > 1.0]
        for key in expired:
            del self.tracks[key]
        best_id = None
        best_distance = 0.08
        for track_id, track in self.tracks.items():
            distance = float(np.linalg.norm(center - track["center"]))
            if distance < best_distance:
                best_id, best_distance = track_id, distance
        if best_id is None:
            best_id = self.next_track_id
            self.next_track_id += 1
            self.tracks[best_id] = {
                "window": PoseStabilityWindow(
                    self.get_parameter("stability_frames").value,
                    self.get_parameter("position_outlier_radius").value,
                )
            }
        self.tracks[best_id]["center"] = center
        self.tracks[best_id]["last"] = now
        return best_id, self.tracks[best_id]["window"]

    def _invalid_detection(self, image_msg, instance, reason, valid_ratio=0.0):
        msg = DetectedObject()
        msg.header = image_msg.header
        msg.header.frame_id = self.get_parameter("base_frame").value
        msg.class_name = instance.class_name
        msg.confidence = float(instance.confidence)
        msg.depth_valid_ratio = float(valid_ratio)
        msg.stable = False
        msg.rejection_reason = reason
        self.detection_pub.publish(msg)

    def _image_callback(self, color_msg, depth_msg):
        if self.camera_matrix is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        try:
            depth_m = depth_to_meters(
                raw_depth, depth_msg.encoding, self.get_parameter("depth_scale").value
            )
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)
            return
        instances = self.backend.detect(bgr)
        debug = bgr.copy()
        if not instances:
            cv2.putText(debug, "NO_TARGET", (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)
            self._publish_debug(debug, color_msg)
            return

        source_frame = color_msg.header.frame_id or self.camera_frame or self.get_parameter(
            "camera_frame").value
        try:
            base_from_camera = self._lookup_transform(color_msg.header.stamp, source_frame)
        except TransformException as exc:
            for instance in instances:
                self._invalid_detection(color_msg, instance, "TF_UNAVAILABLE")
            cv2.putText(debug, f"TF_UNAVAILABLE: {str(exc)[:45]}", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            self._publish_debug(debug, color_msg)
            return

        combined_mask = np.zeros(depth_m.shape, dtype=np.uint8)
        for instance in instances:
            combined_mask = cv2.bitwise_or(combined_mask, instance.binary_mask)

        accepted = []
        for instance in instances:
            result = self._process_instance(
                instance, depth_m, combined_mask, base_from_camera, color_msg
            )
            x, y, w, h = instance.bounding_rect
            color = (0, 200, 0) if result is not None and not result.rejection_reason else (0, 0, 255)
            cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
            if result is None:
                continue
            label = result.rejection_reason or (
                f"id={result.track_id} {result.size.x*1000:.0f}x"
                f"{result.size.y*1000:.0f}x{result.size.z*1000:.0f}mm "
                f"d={result.depth_valid_ratio:.2f} stable={result.stable}"
            )
            cv2.putText(debug, label, (x, max(18, y - 7)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)
            if not result.rejection_reason:
                accepted.append(result)

        accepted.sort(key=self._score, reverse=True)
        if len(accepted) >= 2 and self._score(accepted[0]) - self._score(accepted[1]) < self.get_parameter(
            "ambiguity_score_gap").value:
            for msg in accepted[:2]:
                msg.stable = False
                msg.rejection_reason = "AMBIGUOUS_TARGET"
                self.detection_pub.publish(msg)
            cv2.putText(debug, "AMBIGUOUS_TARGET", (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)
        elif accepted:
            self._publish_primary(accepted[0], color_msg)
        self._publish_debug(debug, color_msg)

    def _process_instance(self, instance, depth_m, combined_mask, base_from_camera, image_msg):
        erode = int(self.get_parameter("mask_erode_pixels").value)
        kernel_size = max(1, 2 * erode + 1)
        inner_mask = cv2.erode(
            instance.binary_mask, np.ones((kernel_size, kernel_size), np.uint8)
        )
        points_camera, valid_ratio = masked_points(
            depth_m,
            inner_mask,
            self.camera_matrix,
            self.get_parameter("min_depth").value,
            self.get_parameter("max_depth").value,
        )
        if valid_ratio < self.get_parameter("min_depth_valid_ratio").value or len(
            points_camera) < self.get_parameter("min_depth_points").value:
            self._invalid_detection(image_msg, instance, "INVALID_DEPTH", valid_ratio)
            return None
        points_base = robust_filter(transform_points(points_camera, base_from_camera))
        if len(points_base) < self.get_parameter("min_depth_points").value:
            self._invalid_detection(image_msg, instance, "INVALID_DEPTH", valid_ratio)
            return None

        configured_table = float(self.get_parameter("table_height").value)
        table_z = configured_table if configured_table > -100.0 else None
        if table_z is None:
            x, y, w, h = instance.bounding_rect
            pad = max(12, int(0.25 * max(w, h)))
            roi = np.zeros(depth_m.shape, dtype=np.uint8)
            x0, y0 = max(0, x - pad), max(0, y - pad)
            x1, y1 = min(depth_m.shape[1], x + w + pad), min(depth_m.shape[0], y + h + pad)
            roi[y0:y1, x0:x1] = 255
            background_mask = cv2.bitwise_and(roi, cv2.bitwise_not(combined_mask))
            background_camera, _ = masked_points(
                depth_m, background_mask, self.camera_matrix,
                self.get_parameter("min_depth").value,
                self.get_parameter("max_depth").value,
            )
            background_base = transform_points(background_camera, base_from_camera)
            table_z = fit_horizontal_plane_z(
                background_base,
                self.get_parameter("table_ransac_threshold").value,
            )
        try:
            box = estimate_oriented_box(points_base, table_z)
        except ValueError:
            self._invalid_detection(image_msg, instance, "INVALID_DEPTH", valid_ratio)
            return None

        track_id, window = self._assign_track(box.center)
        window.add(box.center, box.yaw)
        stability = window.result(
            self.get_parameter("max_position_span").value,
            math.radians(self.get_parameter("max_yaw_span_deg").value),
        )
        msg = self._make_detection(image_msg, instance, box, track_id, valid_ratio, stability)
        if table_z is None:
            msg.rejection_reason = "TABLE_NOT_OBSERVED"
            msg.stable = False
        elif not size_matches(
            box.size,
            self.get_parameter("expected_size").value,
            self.get_parameter("size_tolerance").value,
        ):
            msg.rejection_reason = "SIZE_MISMATCH"
            msg.stable = False
        self.detection_pub.publish(msg)
        if not msg.rejection_reason:
            header = Header(stamp=image_msg.header.stamp, frame_id=self.get_parameter("base_frame").value)
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points_base.tolist()))
        return msg

    def _make_detection(self, image_msg, instance, box, track_id, valid_ratio, stability):
        msg = DetectedObject()
        msg.header.stamp = image_msg.header.stamp
        msg.header.frame_id = self.get_parameter("base_frame").value
        msg.track_id = track_id
        msg.class_name = instance.class_name
        msg.confidence = float(instance.confidence)
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = box.center
        quaternion = quaternion_from_axes(box.short_axis, [0.0, 0.0, 1.0])
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = quaternion
        position_variance = max(1e-6, stability.position_span ** 2 if np.isfinite(stability.position_span) else 1e-3)
        yaw_variance = max(1e-6, stability.yaw_span ** 2 if np.isfinite(stability.yaw_span) else 0.1)
        msg.pose.covariance[0] = position_variance
        msg.pose.covariance[7] = position_variance
        msg.pose.covariance[14] = position_variance
        msg.pose.covariance[35] = yaw_variance
        # Object-pose X is the 40 mm grasping edge; keep dimensions in that frame.
        msg.size.x, msg.size.y, msg.size.z = box.size[1], box.size[0], box.size[2]
        msg.depth_valid_ratio = float(valid_ratio)
        msg.stable = bool(stability.stable)
        return msg

    @staticmethod
    def _score(msg):
        return float(msg.confidence) + 0.25 * float(msg.depth_valid_ratio) + (0.20 if msg.stable else 0.0)

    def _publish_primary(self, msg, image_msg):
        object_pose = PoseStamped()
        object_pose.header = msg.header
        object_pose.pose = msg.pose.pose
        self.object_pose_pub.publish(object_pose)

        short_axis = np.array([
            1.0 - 2.0 * (msg.pose.pose.orientation.y ** 2 + msg.pose.pose.orientation.z ** 2),
            2.0 * (msg.pose.pose.orientation.x * msg.pose.pose.orientation.y
                   + msg.pose.pose.orientation.w * msg.pose.pose.orientation.z),
            0.0,
        ])
        box_like = type("Box", (), {})()
        box_like.center = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        box_like.short_axis = short_axis / max(np.linalg.norm(short_axis), 1e-9)
        box_like.top_z = msg.pose.pose.position.z + 0.5 * msg.size.z
        candidates = make_grasp_candidates(
            box_like,
            self.get_parameter("grasp_depth").value,
            self.get_parameter("tcp_to_grasp_xyz").value,
        )
        poses = PoseArray()
        poses.header = msg.header
        for position, quaternion in candidates:
            pose = PoseStamped().pose
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
            poses.poses.append(pose)
        self.candidate_pub.publish(poses)
        self._publish_markers(msg, poses)

    def _publish_markers(self, msg, candidates):
        markers = MarkerArray()
        box = Marker()
        box.header = msg.header
        box.ns = "object_obb"
        box.id = int(msg.track_id)
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose = msg.pose.pose
        box.scale = msg.size
        box.color.r, box.color.g, box.color.b, box.color.a = 0.1, 0.35, 1.0, 0.45
        box.lifetime = Duration(seconds=0.5).to_msg()
        markers.markers.append(box)
        for index, pose in enumerate(candidates.poses):
            marker = Marker()
            marker.header = msg.header
            marker.ns = "grasp_candidates"
            marker.id = 1000 + int(msg.track_id) * 2 + index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x, marker.scale.y, marker.scale.z = 0.10, 0.012, 0.018
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.2, 0.9
            marker.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def _publish_debug(self, bgr, source_msg):
        debug_msg = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        debug_msg.header = source_msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
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
