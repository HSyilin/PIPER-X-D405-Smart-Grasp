#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
抓取执行节点 (eye-in-hand)
坐标链:  P_base = T_base->tcp (实时 feedback/tcp_pose)
               · T_tcp->cam (手眼标定 JSON)
               · P_cam    (legacy camera-frame target pose)

流程 (std_srvs/Trigger 触发):
  采样目标 -> 张开夹爪 -> 预抓取点(上方) -> 下探 -> 闭合 -> 抬起

接口:
  订阅  /smart_grasp/object_pose   (PoseStamped, base_link)
  订阅  feedback/tcp_pose          (PoseStamped, 基座系)
  发布  control/move_p             (PoseStamped)
  发布  control/joint_states       (JointState, gripper 关节)
  服务  /smart_grasp/legacy_pick   (std_srvs/Trigger)
  服务  /smart_grasp/home          (std_srvs/Trigger, legacy; official homing uses /move_home)
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from agx_arm_msgs.msg import GripperStatus
from rcl_interfaces.msg import (
    SetParametersResult, ParameterDescriptor, FloatingPointRange, IntegerRange)
from scipy.spatial.transform import Rotation as R

from smart_grasp.grasp_geometry import (
    pose_to_matrix,
    tcp_position_for_grasp_center,
)

GRIPPER_JOINT_NAME = "gripper"


def fdesc(desc, lo, hi, step=0.0):
    """带滑条范围的浮点参数描述符 (rqt_reconfigure 显示滑条)"""
    return ParameterDescriptor(
        description=desc,
        floating_point_range=[FloatingPointRange(
            from_value=float(lo), to_value=float(hi), step=step)])


def idesc(desc, lo, hi, step=1):
    return ParameterDescriptor(
        description=desc,
        integer_range=[IntegerRange(from_value=lo, to_value=hi, step=step)])


def pose_to_mat(px, py, pz, qx, qy, qz, qw):
    return pose_to_matrix([px, py, pz], [qx, qy, qz, qw])


class GraspExecutorNode(Node):

    def __init__(self):
        super().__init__("smart_grasp_executor")
        cb = ReentrantCallbackGroup()

        # ---- 参数 (均带 rqt_reconfigure 滑条范围, 运行时可改) ----
        self.declare_parameter(
            "handeye_json",
            "/home/guest/handeye_ws/result/eye_in_hand_d405_px_connected_20260725.json",
            ParameterDescriptor(description="手眼标定JSON (改后自动重载 T_tcp_cam)"))
        self.declare_parameter("sample_num", 10, idesc("目标位姿采样次数", 1, 50))
        self.declare_parameter(
            "sample_timeout", 5.0, fdesc("采样超时(s)", 1.0, 15.0, 0.5))
        self.declare_parameter(
            "target_max_age", 0.5, fdesc("目标最大允许年龄(s)", 0.1, 3.0, 0.1))
        self.declare_parameter(
            "sample_outlier_radius", 0.02,
            fdesc("相对中值的采样离群半径(m)", 0.005, 0.10, 0.005))
        self.declare_parameter(
            "pre_grasp_offset_z", 0.10, fdesc("预抓取点高出目标(m)", -0.05, 0.40, 0.005))
        self.declare_parameter(
            "grasp_depth", 0.02,
            fdesc("夹持中心低于检测表面的深度(m)", 0.0, 0.10, 0.001))
        self.declare_parameter(
            "lift_height", 0.05, fdesc("抬升高度(m)", 0.0, 0.50, 0.01))
        self.declare_parameter(
            "tcp_to_grasp_xyz", [0.0, 0.0, 0.1425],
            ParameterDescriptor(
                description="TCP到夹持中心平移[TCP坐标系,m]"))
        self.declare_parameter(
            "gripper_open", 0.07, fdesc("夹爪张开行程(m)", 0.0, 0.10, 0.001))
        self.declare_parameter(
            "gripper_close", 0.0, fdesc("夹爪闭合行程(m)", 0.0, 0.07, 0.001))
        self.declare_parameter(
            "gripper_effort", 1.0, fdesc("夹爪力矩", 0.0, 5.0, 0.1))
        self.declare_parameter(
            "gripper_feedback_timeout", 1.0,
            fdesc("夹爪反馈超时(s)", 0.2, 5.0, 0.1))
        self.declare_parameter(
            "contact_width_min", 0.005,
            fdesc("闭合后有效接触宽度下限(m)", 0.0, 0.10, 0.001))
        self.declare_parameter(
            "contact_width_max", 0.065,
            fdesc("闭合后有效接触宽度上限(m)", 0.0, 0.10, 0.001))
        self.declare_parameter(
            "execute_enabled", False,
            ParameterDescriptor(
                description="false时只计算并返回抓取点，不发送真机命令"))
        self.declare_parameter(
            "move_timeout", 10.0, fdesc("单步移动超时(s)", 1.0, 30.0, 1.0))
        self.declare_parameter(
            "reach_tolerance", 0.008, fdesc("到位判定阈值(m)", 0.001, 0.05, 0.001))
        self.declare_parameter(
            "settle_time", 0.5, fdesc("到位后稳定等待(s)", 0.0, 3.0, 0.1))
        # 抓取姿态: keep_current=保持当前TCP姿态 | fixed_rpy=使用固定欧拉角
        self.declare_parameter(
            "orientation_mode", "keep_current",
            ParameterDescriptor(description="keep_current | fixed_rpy"))
        self.declare_parameter(
            "base_frame", "base_link",
            ParameterDescriptor(description="抓取目标和控制命令的基座坐标系"))
        self.declare_parameter(
            "grasp_rpy", [math.pi, 0.0, 0.0],
            ParameterDescriptor(description="fixed_rpy 模式下的抓取姿态 [rx,ry,rz](rad)"))
        # 工作空间保护 (基座系, 超范围拒绝执行) —— 数组用 ros2 param set 修改
        self.declare_parameter(
            "ws_x", [0.15, 0.65],
            ParameterDescriptor(description="工作空间X范围[m] (改后即时生效)"))
        self.declare_parameter(
            "ws_y", [-0.40, 0.40],
            ParameterDescriptor(description="工作空间Y范围[m]"))
        self.declare_parameter(
            "ws_z", [-0.05, 0.50],
            ParameterDescriptor(description="工作空间Z范围[m]"))
        self.declare_parameter(
            "home_pose", [0.25, 0.0, 0.30],
            ParameterDescriptor(description="home 位置 [x,y,z](m)"))

        # ---- 手眼矩阵 ----
        self._load_handeye(self.get_parameter("handeye_json").value)

        # 注册参数回调: 修改后即时生效
        self.add_on_set_parameters_callback(self._on_set_params)

        # ---- 状态 ----
        self._lock = threading.Lock()
        self._gripper_condition = threading.Condition(self._lock)
        self._latest_target = None       # (PoseStamped, 收到时的 T_base_tcp)
        self._latest_target_rx = None    # monotonic receive time
        self._latest_tcp = None          # PoseStamped
        self._latest_gripper = None      # GripperStatus
        self._latest_gripper_rx = None   # monotonic receive time
        self._busy = False
        self._warned_empty_tcp_frame = False

        # ---- 通信 ----
        self.create_subscription(
            PoseStamped, "/smart_grasp/object_pose",
            self._target_callback, 1, callback_group=cb)
        self.create_subscription(
            PoseStamped, "feedback/tcp_pose",
            self._tcp_callback, 1, callback_group=cb)
        self.create_subscription(
            GripperStatus, "feedback/gripper_status",
            self._gripper_callback, 1, callback_group=cb)
        self.move_pub = self.create_publisher(PoseStamped, "control/move_p", 1)
        self.joint_pub = self.create_publisher(JointState, "control/joint_states", 1)

        self.create_service(Trigger, "/smart_grasp/legacy_pick",
                            self._pick_service, callback_group=cb)
        self.create_service(Trigger, "/smart_grasp/home",
                            self._home_service, callback_group=cb)

        self.get_logger().info(
            "抓取执行诊断节点就绪, 调用 /smart_grasp/legacy_pick 触发抓取; "
            "官方回零请用 /move_home"
        )

    # ================= 参数/手眼 =================
    def _load_handeye(self, path):
        with open(path, "r") as f:
            he = json.load(f)
        p = he["position"]
        q = he["orientation"]  # [x, y, z, w]
        self.T_tcp_cam = pose_to_mat(p[0], p[1], p[2], q[0], q[1], q[2], q[3])
        self.get_logger().info(
            f"手眼矩阵加载成功: t={np.round(self.T_tcp_cam[:3,3],4).tolist()}")

    def _on_set_params(self, params):
        """rqt_reconfigure / ros2 param set 修改参数后即时生效"""
        for p in params:
            if p.name == "handeye_json" and p.value:
                try:
                    self._load_handeye(str(p.value))
                except Exception as e:
                    self.get_logger().error(f"重载手眼矩阵失败: {e}")
                    return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    # ================= 回调 =================
    def _tcp_callback(self, msg: PoseStamped):
        if not msg.header.frame_id and not self._warned_empty_tcp_frame:
            self.get_logger().warn(
                "feedback/tcp_pose 未设置 frame_id，当前按 base_link 解释")
            self._warned_empty_tcp_frame = True
        with self._lock:
            self._latest_tcp = msg

    def _gripper_callback(self, msg: GripperStatus):
        with self._gripper_condition:
            self._latest_gripper = msg
            self._latest_gripper_rx = time.monotonic()
            self._gripper_condition.notify_all()

    def _target_callback(self, msg: PoseStamped):
        with self._lock:
            base_frame = self.get_parameter("base_frame").value.lstrip("/")
            source_frame = msg.header.frame_id.lstrip("/")
            if source_frame == base_frame:
                self._latest_target = (msg, None)
            elif self._latest_tcp is None:
                return
            else:
                # Legacy camera-frame targets must retain the matching TCP pose.
                self._latest_target = (msg, self._latest_tcp)
            self._latest_target_rx = time.monotonic()

    # ================= 坐标变换 =================
    def _tcp_msg_to_mat(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        return pose_to_mat(p.x, p.y, p.z, o.x, o.y, o.z, o.w)

    def _target_in_base(self):
        """采样多帧目标点, 变换到基座系, 取中值"""
        n = self.get_parameter("sample_num").value
        timeout = self.get_parameter("sample_timeout").value
        pts = []
        deadline = time.time() + timeout
        last_stamp = None
        while len(pts) < n and time.time() < deadline:
            with self._lock:
                item = self._latest_target
                target_rx = self._latest_target_rx
            if item is not None:
                max_age = self.get_parameter("target_max_age").value
                if target_rx is None or time.monotonic() - target_rx > max_age:
                    time.sleep(0.03)
                    continue
                tgt, tcp = item
                stamp = (tgt.header.stamp.sec, tgt.header.stamp.nanosec)
                if stamp != last_stamp:
                    last_stamp = stamp
                    base_frame = self.get_parameter("base_frame").value.lstrip("/")
                    if tgt.header.frame_id.lstrip("/") == base_frame:
                        pts.append(np.array([
                            tgt.pose.position.x,
                            tgt.pose.position.y,
                            tgt.pose.position.z,
                        ]))
                    elif tcp is not None:
                        T_base_tcp = self._tcp_msg_to_mat(tcp)
                        p_cam = np.array([tgt.pose.position.x,
                                          tgt.pose.position.y,
                                          tgt.pose.position.z, 1.0])
                        p_base = T_base_tcp @ self.T_tcp_cam @ p_cam
                        pts.append(p_base[:3])
            time.sleep(0.03)
        if len(pts) < max(3, n // 3):
            return None
        points = np.array(pts)
        median = np.median(points, axis=0)
        radius = self.get_parameter("sample_outlier_radius").value
        inliers = points[np.linalg.norm(points - median, axis=1) <= radius]
        if len(inliers) < max(3, n // 3):
            return None
        return np.median(inliers, axis=0)

    def _check_workspace(self, p):
        wx = self.get_parameter("ws_x").value
        wy = self.get_parameter("ws_y").value
        wz = self.get_parameter("ws_z").value
        return (wx[0] <= p[0] <= wx[1] and
                wy[0] <= p[1] <= wy[1] and
                wz[0] <= p[2] <= wz[1])

    # ================= 运动原语 =================
    def _grasp_quat(self):
        mode = self.get_parameter("orientation_mode").value
        if mode == "fixed_rpy":
            rpy = self.get_parameter("grasp_rpy").value
            q = R.from_euler("xyz", rpy).as_quat()
            return q
        with self._lock:
            o = self._latest_tcp.pose.orientation
        return np.array([o.x, o.y, o.z, o.w])

    def _move_p(self, xyz, quat, wait=True):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("base_frame").value
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self.move_pub.publish(msg)
        if not wait:
            return True
        return self._wait_reach(xyz)

    def _tcp_target_for_grasp_center(self, grasp_center, quat):
        tool_offset = self.get_parameter("tcp_to_grasp_xyz").value
        return tcp_position_for_grasp_center(grasp_center, quat, tool_offset)

    def _gripper_fault_reason(self):
        with self._lock:
            status = self._latest_gripper
            received = self._latest_gripper_rx
        timeout = self.get_parameter("gripper_feedback_timeout").value
        if status is None or received is None:
            return "未收到 feedback/gripper_status"
        if time.monotonic() - received > timeout:
            return "夹爪反馈已过期"
        fault_fields = (
            ("voltage_too_low", "夹爪电压过低"),
            ("motor_overheating", "夹爪电机过热"),
            ("driver_overcurrent", "夹爪驱动过流"),
            ("driver_overheating", "夹爪驱动过热"),
            ("driver_error_status", "夹爪驱动故障"),
        )
        for field, reason in fault_fields:
            if getattr(status, field):
                return reason
        if not status.driver_enable_status:
            return "夹爪驱动未使能"
        return None

    def _wait_reach(self, xyz):
        tol = self.get_parameter("reach_tolerance").value
        timeout = self.get_parameter("move_timeout").value
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                tcp = self._latest_tcp
            if tcp is not None:
                cur = np.array([tcp.pose.position.x,
                                tcp.pose.position.y,
                                tcp.pose.position.z])
                if np.linalg.norm(cur - np.asarray(xyz)) < tol:
                    time.sleep(self.get_parameter("settle_time").value)
                    return True
            time.sleep(0.05)
        self.get_logger().warn(f"移动超时, 目标 {np.round(xyz,3).tolist()}")
        return False

    def _set_gripper(self, position, hold=1.0):
        with self._gripper_condition:
            previous_rx = self._latest_gripper_rx
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [GRIPPER_JOINT_NAME]
        msg.position = [float(position)]
        msg.effort = [float(self.get_parameter("gripper_effort").value)]
        self.joint_pub.publish(msg)
        deadline = time.monotonic() + hold
        received_update = False
        with self._gripper_condition:
            while time.monotonic() < deadline:
                current_rx = self._latest_gripper_rx
                if current_rx is not None and current_rx != previous_rx:
                    received_update = True
                    break
                self._gripper_condition.wait(timeout=max(0.0, deadline - time.monotonic()))
        return received_update and self._gripper_fault_reason() is None

    # ================= 服务 =================
    def _pick_service(self, req, resp):
        with self._lock:
            if self._busy:
                resp.success = False
                resp.message = "上一次抓取尚未完成"
                return resp
            self._busy = True
        try:
            resp.success, resp.message = self._do_pick()
        except Exception as e:
            resp.success = False
            resp.message = f"异常: {e}"
            self.get_logger().error(f"抓取异常: {e}")
        finally:
            with self._lock:
                self._busy = False
        return resp

    def _do_pick(self):
        with self._lock:
            if self._latest_tcp is None:
                return False, "未收到 feedback/tcp_pose, 请先启动机械臂驱动"

        self.get_logger().info("== 1/6 采样目标位置 ==")
        p = self._target_in_base()
        if p is None:
            return False, "采样失败: 视野内未稳定检测到目标"
        self.get_logger().info(f"目标(基座系): {np.round(p, 4).tolist()}")
        if not self._check_workspace(p):
            return False, f"目标 {np.round(p,3).tolist()} 超出工作空间, 拒绝执行"

        quat = self._grasp_quat()
        pre_z = self.get_parameter("pre_grasp_offset_z").value
        grasp_depth = self.get_parameter("grasp_depth").value
        lift = self.get_parameter("lift_height").value
        g_open = self.get_parameter("gripper_open").value
        g_close = self.get_parameter("gripper_close").value

        grasp_center = np.array([p[0], p[1], p[2] - grasp_depth])
        pre_center = grasp_center + np.array([0.0, 0.0, pre_z])
        lift_center = grasp_center + np.array([0.0, 0.0, lift])
        pre_tcp = self._tcp_target_for_grasp_center(pre_center, quat)
        grasp_tcp = self._tcp_target_for_grasp_center(grasp_center, quat)
        lift_tcp = self._tcp_target_for_grasp_center(lift_center, quat)

        for stage, tcp_target in (
            ("预抓取", pre_tcp),
            ("抓取", grasp_tcp),
            ("抬升", lift_tcp),
        ):
            if not self._check_workspace(tcp_target):
                return False, (
                    f"{stage} TCP目标 {np.round(tcp_target, 3).tolist()} "
                    "超出工作空间，拒绝执行")

        contact_min = self.get_parameter("contact_width_min").value
        contact_max = self.get_parameter("contact_width_max").value
        if contact_min > contact_max:
            return False, "contact_width_min 不能大于 contact_width_max"

        plan_text = (
            f"pre_tcp={np.round(pre_tcp, 4).tolist()}, "
            f"grasp_tcp={np.round(grasp_tcp, 4).tolist()}, "
            f"lift_tcp={np.round(lift_tcp, 4).tolist()}"
        )
        if not self.get_parameter("execute_enabled").value:
            self.get_logger().warn(f"仅计算模式，不发送控制命令: {plan_text}")
            return True, f"抓取点计算完成（未执行）: {plan_text}"

        fault = self._gripper_fault_reason()
        if fault is not None:
            return False, fault

        self.get_logger().info("== 2/6 张开夹爪 ==")
        if not self._set_gripper(g_open):
            return False, self._gripper_fault_reason() or "夹爪张开失败"

        self.get_logger().info("== 3/6 移动到预抓取点 ==")
        if not self._move_p(pre_tcp, quat):
            return False, "预抓取点移动失败"

        self.get_logger().info("== 4/6 下探到抓取点 ==")
        if not self._move_p(grasp_tcp, quat):
            self._move_p(pre_tcp, quat)
            return False, "下探失败, 已退回"

        self.get_logger().info("== 5/6 闭合夹爪 ==")
        if not self._set_gripper(g_close, hold=1.5):
            self._move_p(pre_tcp, quat)
            return False, self._gripper_fault_reason() or "夹爪闭合失败"
        with self._lock:
            contact_width = self._latest_gripper.width
        if not contact_min <= contact_width <= contact_max:
            self._move_p(pre_tcp, quat)
            return False, (
                f"闭合宽度 {contact_width:.4f} m 不在预期接触区间 "
                f"[{contact_min:.4f}, {contact_max:.4f}] m，已退回")

        self.get_logger().info("== 6/6 抬起 ==")
        if not self._move_p(lift_tcp, quat):
            return False, "夹持后抬升失败"

        return True, (
            f"抓取完成, 目标表面 {np.round(p,3).tolist()}, "
            f"接触宽度 {contact_width:.4f} m")

    def _home_service(self, req, resp):
        with self._lock:
            if self._latest_tcp is None:
                resp.success = False
                resp.message = "未收到 tcp_pose"
                return resp
        home = self.get_parameter("home_pose").value
        quat = self._grasp_quat()
        ok = self._move_p(home, quat)
        resp.success = ok
        resp.message = "已回 home" if ok else "回 home 超时"
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
