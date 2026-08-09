#!/usr/bin/env python3
"""Mission sequencer for chassis path tracking plus smart grasp."""

from __future__ import annotations

import math
import time
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from smart_grasp_interfaces.action import PickObject
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

try:
    import yaml
except ImportError:  # pragma: no cover - package.xml carries python3-yaml
    yaml = None


@dataclass
class TrajectoryPoint:
    x: float
    y: float
    z: float = 0.0
    yaw: Optional[float] = None


class TuringGraspMissionSequencer(Node):
    def __init__(self) -> None:
        super().__init__("turing_grasp_mission_sequencer")

        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("path_topic", "/trajectory_tracking/path")
        self.declare_parameter("cmd_vel_topic", "/nav_cmd_vel")
        self.declare_parameter("mode_response_topic", "/chassis/mode_response")
        self.declare_parameter("pick_action", "/smart_grasp/pick")
        self.declare_parameter("target_class", "1")
        self.declare_parameter("execute_pick", True)
        self.declare_parameter("pick_index", 2)
        self.declare_parameter("stair_on_index", 10)
        self.declare_parameter("stair_off_index", 11)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("tf_timeout_s", 0.1)
        self.declare_parameter("segment_timeout_s", 0.0)
        self.declare_parameter("service_timeout_s", 20.0)
        self.declare_parameter("action_server_timeout_s", 30.0)
        self.declare_parameter("pick_result_timeout_s", 180.0)
        self.declare_parameter("mode_response_timeout_s", 20.0)
        self.declare_parameter("stop_hold_s", 0.8)
        self.declare_parameter("stabilize_after_sitdown_s", 1.0)
        self.declare_parameter("stabilize_after_stand_s", 2.0)
        self.declare_parameter("stabilize_after_walk_s", 1.0)
        self.declare_parameter("stabilize_after_stair_s", 1.0)
        self.declare_parameter("start_delay_s", 3.0)

        self.trajectory_file = self._str_param("trajectory_file")
        self.frame_id = self._str_param("frame_id")
        self.base_frame = self._str_param("base_frame")
        self.path_topic = self._str_param("path_topic")
        self.cmd_vel_topic = self._str_param("cmd_vel_topic")
        self.pick_action = self._str_param("pick_action")
        self.target_class = self._str_param("target_class")
        self.execute_pick = self._bool_param("execute_pick")
        self.pick_index = self._int_param("pick_index")
        self.stair_on_index = self._int_param("stair_on_index")
        self.stair_off_index = self._int_param("stair_off_index")
        self.goal_tolerance = self._float_param("goal_tolerance")
        self.tf_timeout = Duration(seconds=self._float_param("tf_timeout_s"))
        self.segment_timeout_s = self._float_param("segment_timeout_s")
        self.service_timeout_s = self._float_param("service_timeout_s")
        self.action_server_timeout_s = self._float_param("action_server_timeout_s")
        self.pick_result_timeout_s = self._float_param("pick_result_timeout_s")
        self.mode_response_timeout_s = self._float_param("mode_response_timeout_s")
        self.stop_hold_s = self._float_param("stop_hold_s")
        self.stabilize_after_sitdown_s = self._float_param("stabilize_after_sitdown_s")
        self.stabilize_after_stand_s = self._float_param("stabilize_after_stand_s")
        self.stabilize_after_walk_s = self._float_param("stabilize_after_walk_s")
        self.stabilize_after_stair_s = self._float_param("stabilize_after_stair_s")
        self.start_delay_s = self._float_param("start_delay_s")

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, self.path_topic, path_qos)
        self.zero_cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(
            String,
            self._str_param("mode_response_topic"),
            self._mode_response_callback,
            20,
        )
        self._seen_mode_titles: list[str] = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sitdown_client = self.create_client(Trigger, "/limx/tron2/sitdown")
        self.stand_client = self.create_client(Trigger, "/limx/tron2/stand")
        self.walk_client = self.create_client(Trigger, "/limx/tron2/walk")
        self.stair_client = self.create_client(SetBool, "/limx/tron2/stair_mode")
        self.arm_enable_client = self.create_client(SetBool, "/enable_agx_arm")
        self.move_home_client = self.create_client(Empty, "/move_home")
        self.pick_client = ActionClient(self, PickObject, self.pick_action)

    def run(self) -> bool:
        self._sleep_with_spin(self.start_delay_s)

        try:
            points = self._load_trajectory(self.trajectory_file)
            before_pick, after_pick, stair_segment, after_stair = self._build_segments(points)
        except Exception as exc:
            self.get_logger().error(f"Failed to prepare mission: {exc}")
            self._publish_zero_for(self.stop_hold_s)
            return False

        if not self._wait_for_required_interfaces():
            self._publish_zero_for(self.stop_hold_s)
            return False

        if not self._follow_segment(before_pick, f"point 1 -> point {self.pick_index}"):
            return False

        self._publish_zero_for(self.stop_hold_s)
        if not self._request_trigger(
            self.sitdown_client,
            "/limx/tron2/sitdown",
            {"response_sitdown", "notify_sitdown"},
            "sitdown",
        ):
            return False
        self._sleep_with_spin(self.stabilize_after_sitdown_s)

        if not self._request_trigger(
            self.stand_client,
            "/limx/tron2/stand",
            {"response_stand_mode", "notify_stand_mode"},
            "stand",
        ):
            return False
        self._sleep_with_spin(self.stabilize_after_stand_s)

        if not self._enable_arm():
            return False
        if not self._move_arm_home():
            return False
        if not self._pick_object():
            return False

        if not self._request_trigger(
            self.walk_client,
            "/limx/tron2/walk",
            {"response_walk_mode", "notify_walk_mode"},
            "walk",
        ):
            return False
        self._sleep_with_spin(self.stabilize_after_walk_s)

        if not self._follow_segment(
            after_pick, f"point {self.pick_index} -> point {self.stair_on_index}"
        ):
            return False

        self._publish_zero_for(self.stop_hold_s)
        if not self._set_stair_mode(True):
            return False
        self._sleep_with_spin(self.stabilize_after_stair_s)

        if not self._follow_segment(
            stair_segment, f"point {self.stair_on_index} -> point {self.stair_off_index}"
        ):
            return False

        self._publish_zero_for(self.stop_hold_s)
        if not self._set_stair_mode(False):
            return False
        self._sleep_with_spin(self.stabilize_after_stair_s)

        if len(after_stair) >= 2 and not self._follow_segment(
            after_stair, f"point {self.stair_off_index} -> final point"
        ):
            return False

        self._publish_zero_for(self.stop_hold_s)
        self.get_logger().info("Turing grasp mission complete.")
        return True

    def _build_segments(
        self, points: list[TrajectoryPoint]
    ) -> tuple[
        list[TrajectoryPoint],
        list[TrajectoryPoint],
        list[TrajectoryPoint],
        list[TrajectoryPoint],
    ]:
        if self.pick_index < 2:
            raise ValueError("pick_index must be at least 2")
        if self.pick_index >= self.stair_on_index:
            raise ValueError("pick_index must be less than stair_on_index")
        if self.stair_on_index >= self.stair_off_index:
            raise ValueError("stair_on_index must be less than stair_off_index")
        if self.stair_off_index > len(points):
            raise ValueError(
                f"stair_off_index {self.stair_off_index} exceeds trajectory length {len(points)}"
            )

        pick = self.pick_index - 1
        stair_on = self.stair_on_index - 1
        stair_off = self.stair_off_index - 1
        return (
            points[: pick + 1],
            points[pick : stair_on + 1],
            points[stair_on : stair_off + 1],
            points[stair_off:],
        )

    def _wait_for_required_interfaces(self) -> bool:
        checks = [
            (self.sitdown_client, "/limx/tron2/sitdown"),
            (self.stand_client, "/limx/tron2/stand"),
            (self.walk_client, "/limx/tron2/walk"),
            (self.stair_client, "/limx/tron2/stair_mode"),
            (self.arm_enable_client, "/enable_agx_arm"),
            (self.move_home_client, "/move_home"),
        ]
        for client, name in checks:
            if not self._wait_for_service(client, name, self.service_timeout_s):
                return False

        self.get_logger().info(f"Waiting for action server: {self.pick_action}")
        if not self.pick_client.wait_for_server(timeout_sec=self.action_server_timeout_s):
            self.get_logger().error(
                f"Timed out waiting for action server: {self.pick_action}"
            )
            return False
        return True

    def _follow_segment(self, points: list[TrajectoryPoint], label: str) -> bool:
        if len(points) < 2:
            self.get_logger().error(f"Segment '{label}' has fewer than 2 points.")
            self._publish_zero_for(self.stop_hold_s)
            return False

        path = self._path_from_points(points)
        goal = points[-1]
        self.get_logger().info(
            f"Publishing {label} with {len(points)} poses; "
            f"goal=({goal.x:.3f}, {goal.y:.3f})."
        )
        self.path_pub.publish(path)

        deadline = None
        if self.segment_timeout_s > 0.0:
            deadline = time.monotonic() + self.segment_timeout_s

        while rclpy.ok():
            if deadline is not None and time.monotonic() > deadline:
                self.get_logger().error(f"Timed out while following {label}.")
                self._publish_zero_for(self.stop_hold_s)
                return False

            pose = self._lookup_robot_position()
            if pose is not None:
                distance = math.hypot(goal.x - pose[0], goal.y - pose[1])
                if distance <= self.goal_tolerance:
                    self.get_logger().info(
                        f"Reached {label} goal within {distance:.3f} m."
                    )
                    self._publish_zero_for(self.stop_hold_s)
                    return True
            rclpy.spin_once(self, timeout_sec=0.05)

        self._publish_zero_for(self.stop_hold_s)
        return False

    def _request_trigger(
        self,
        client: Any,
        service_name: str,
        accepted_titles: set[str],
        label: str,
    ) -> bool:
        self.get_logger().info(f"Requesting {label} mode via {service_name}.")
        self._seen_mode_titles.clear()
        future = client.call_async(Trigger.Request())
        if not self._wait_for_future(future, self.service_timeout_s, label):
            self._publish_zero_for(self.stop_hold_s)
            return False
        response = future.result()
        if response is None or not response.success:
            message = "" if response is None else response.message
            self.get_logger().error(f"{label} service failed: {message}")
            self._publish_zero_for(self.stop_hold_s)
            return False
        if not self._wait_for_mode_titles(accepted_titles, label):
            self._publish_zero_for(self.stop_hold_s)
            return False
        return True

    def _set_stair_mode(self, enable: bool) -> bool:
        label = "stair mode ON" if enable else "stair mode OFF"
        self.get_logger().info(f"Requesting {label}.")
        self._seen_mode_titles.clear()
        request = SetBool.Request()
        request.data = enable
        future = self.stair_client.call_async(request)
        if not self._wait_for_future(future, self.service_timeout_s, label):
            self._publish_zero_for(self.stop_hold_s)
            return False
        response = future.result()
        if response is None or not response.success:
            message = "" if response is None else response.message
            self.get_logger().error(f"{label} service failed: {message}")
            self._publish_zero_for(self.stop_hold_s)
            return False
        if not self._wait_for_mode_titles({"response_stair_mode"}, label):
            self._publish_zero_for(self.stop_hold_s)
            return False
        return True

    def _enable_arm(self) -> bool:
        self.get_logger().info("Enabling AGX arm.")
        request = SetBool.Request()
        request.data = True
        future = self.arm_enable_client.call_async(request)
        if not self._wait_for_future(future, self.service_timeout_s, "enable arm"):
            return False
        response = future.result()
        if response is None or not response.success:
            message = "" if response is None else response.message
            self.get_logger().error(f"Failed to enable arm: {message}")
            return False
        return True

    def _move_arm_home(self) -> bool:
        self.get_logger().info("Moving arm to official home via /move_home.")
        future = self.move_home_client.call_async(Empty.Request())
        if not self._wait_for_future(future, self.service_timeout_s, "move home"):
            return False
        response = future.result()
        if response is None:
            self.get_logger().error("/move_home returned no response.")
            return False
        return True

    def _pick_object(self) -> bool:
        self.get_logger().info(
            f"Sending pick goal target_class={self.target_class} execute={self.execute_pick}."
        )
        goal = PickObject.Goal()
        goal.target_class = self.target_class
        goal.execute = self.execute_pick

        send_future = self.pick_client.send_goal_async(goal)
        if not self._wait_for_future(
            send_future, self.action_server_timeout_s, "send pick goal"
        ):
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Pick goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(
            result_future, self.pick_result_timeout_s, "pick result"
        ):
            return False
        wrapped = result_future.result()
        result = wrapped.result
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or not result.success
            or result.error_code != PickObject.Result.OK
        ):
            self.get_logger().error(
                "Pick failed: "
                f"status={wrapped.status} success={result.success} "
                f"error_code={result.error_code} message='{result.message}'"
            )
            return False
        self.get_logger().info(f"Pick completed: {result.message}")
        return True

    def _wait_for_service(self, client: Any, name: str, timeout_s: float) -> bool:
        self.get_logger().info(f"Waiting for service: {name}")
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.2):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().error(f"Timed out waiting for service: {name}")
        return False

    def _wait_for_future(self, future: Any, timeout_s: float, label: str) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                return True
        self.get_logger().error(f"Timed out waiting for {label}.")
        return False

    def _wait_for_mode_titles(self, titles: set[str], label: str) -> bool:
        deadline = time.monotonic() + self.mode_response_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if any(title in titles for title in self._seen_mode_titles):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(
            f"Timed out waiting for {label} response titles: "
            + ", ".join(sorted(titles))
        )
        return False

    def _mode_response_callback(self, msg: String) -> None:
        data = msg.data
        for marker in (
            "response_sitdown",
            "notify_sitdown",
            "response_stand_mode",
            "notify_stand_mode",
            "response_walk_mode",
            "notify_walk_mode",
            "response_stair_mode",
        ):
            if marker in data:
                self._seen_mode_titles.append(marker)
                self.get_logger().info(f"Received chassis mode response: {marker}")
                return

    def _lookup_robot_position(self) -> Optional[tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.base_frame,
                rclpy.time.Time(),
                timeout=self.tf_timeout,
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.frame_id} -> {self.base_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def _publish_zero_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while rclpy.ok() and time.monotonic() < deadline:
            self.zero_cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        self.zero_cmd_pub.publish(Twist())

    def _sleep_with_spin(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _path_from_points(self, points: list[TrajectoryPoint]) -> Path:
        path = Path()
        path.header.frame_id = self.frame_id or "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z
            yaw = point.yaw if point.yaw is not None else self._fallback_yaw(points, index)
            pose.pose.orientation = self._quaternion_from_yaw(yaw)
            path.poses.append(pose)
        return path

    def _load_trajectory(self, file_path: str) -> list[TrajectoryPoint]:
        if yaml is None:
            raise RuntimeError("python3-yaml is required to load YAML trajectories")
        if not file_path:
            raise ValueError("trajectory_file is empty")
        with open(file_path, "r", encoding="utf-8") as stream:
            root = yaml.safe_load(stream)
        if isinstance(root, dict) and root.get("frame_id"):
            self.frame_id = str(root["frame_id"])

        sequence = None
        if isinstance(root, list):
            sequence = root
        elif isinstance(root, dict):
            for key in ("poses", "points", "trajectory"):
                if isinstance(root.get(key), list):
                    sequence = root[key]
                    break
        if not isinstance(sequence, list):
            raise ValueError(
                "YAML must contain poses, points, trajectory, or a root list"
            )
        return [self._point_from_item(item) for item in sequence]

    @staticmethod
    def _point_from_item(item: Any) -> TrajectoryPoint:
        if isinstance(item, Iterable) and not isinstance(item, (dict, str, bytes)):
            fields = list(item)
            if len(fields) < 2:
                raise ValueError("array trajectory points need at least [x, y]")
            yaw = float(fields[2]) if len(fields) > 2 else None
            z = float(fields[3]) if len(fields) > 3 else 0.0
            return TrajectoryPoint(float(fields[0]), float(fields[1]), z, yaw)
        if isinstance(item, dict):
            if "x" not in item or "y" not in item:
                raise ValueError("map trajectory points need x and y")
            yaw_value = item.get("yaw", item.get("theta"))
            yaw = float(yaw_value) if yaw_value is not None else None
            z = float(item.get("z", 0.0))
            return TrajectoryPoint(float(item["x"]), float(item["y"]), z, yaw)
        raise ValueError("trajectory points must be maps or arrays")

    @staticmethod
    def _fallback_yaw(points: list[TrajectoryPoint], index: int) -> float:
        if len(points) < 2:
            return 0.0
        from_index = index
        to_index = index + 1
        if to_index >= len(points):
            from_index = index - 1
            to_index = index
        dx = points[to_index].x - points[from_index].x
        dy = points[to_index].y - points[from_index].y
        if math.hypot(dx, dy) < 1e-6:
            return 0.0
        return math.atan2(dy, dx)

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        quat = Quaternion()
        quat.w = math.cos(yaw * 0.5)
        quat.z = math.sin(yaw * 0.5)
        return quat

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {"1", "true", "yes", "on"}

    def _int_param(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)


def main() -> int:
    rclpy.init()
    node = TuringGraspMissionSequencer()
    try:
        return 0 if node.run() else 1
    finally:
        node._publish_zero_for(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
