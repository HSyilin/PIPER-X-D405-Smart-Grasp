#!/usr/bin/env python3
"""Manual multi-pose hand-eye stability recorder."""

import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from smart_grasp_interfaces.msg import DetectedObject
from smart_grasp.stability import sample_spans


class TfValidatorNode(Node):
    def __init__(self):
        super().__init__("smart_grasp_tf_validator")
        self.declare_parameter("target_class", "blue_block")
        self.declare_parameter("minimum_samples", 5)
        self.declare_parameter("maximum_samples", 8)
        self.declare_parameter("max_position_span", 0.020)
        self.declare_parameter("max_orientation_span_deg", 3.0)
        self.declare_parameter("target_max_age", 0.5)
        self.lock = threading.Lock()
        self.latest = None
        self.samples = []
        self.create_subscription(
            DetectedObject, "/smart_grasp/detections", self._detection_callback, 10
        )
        self.create_service(
            Trigger, "/smart_grasp/validation/record", self._record_callback
        )
        self.create_service(
            Trigger, "/smart_grasp/validation/reset", self._reset_callback
        )
        self.get_logger().info(
            "TF validator ready; stop the arm for 0.5 s, then call "
            "/smart_grasp/validation/record once at each observation pose"
        )

    def _detection_callback(self, msg):
        if (msg.class_name == self.get_parameter("target_class").value
                and msg.stable and not msg.rejection_reason):
            with self.lock:
                self.latest = msg

    def _record_callback(self, _request, response):
        with self.lock:
            latest = self.latest
        if latest is None:
            response.success = False
            response.message = "no stable target available"
            return response
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(latest.header.stamp)).nanoseconds / 1e9
        if age < 0.0 or age > self.get_parameter("target_max_age").value:
            response.success = False
            response.message = f"stable target is stale ({age:.3f} s)"
            return response
        pose = latest.pose.pose
        sample = (
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        )
        with self.lock:
            maximum = self.get_parameter("maximum_samples").value
            if len(self.samples) >= maximum:
                response.success = False
                response.message = f"already recorded the maximum {maximum} samples; reset first"
                return response
            self.samples.append(sample)
            samples = list(self.samples)
        position_span, orientation_span = sample_spans(samples)
        minimum = self.get_parameter("minimum_samples").value
        passed = (
            len(samples) >= minimum
            and position_span < self.get_parameter("max_position_span").value
            and orientation_span < self.get_parameter("max_orientation_span_deg").value
        )
        response.success = True
        response.message = (
            f"samples={len(samples)} position_span={position_span*1000:.2f}mm "
            f"orientation_span={orientation_span:.2f}deg "
            f"validation={'PASS' if passed else 'NOT_READY_OR_FAIL'}"
        )
        return response

    def _reset_callback(self, _request, response):
        with self.lock:
            self.samples.clear()
        response.success = True
        response.message = "validation samples cleared"
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(TfValidatorNode())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
