#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
交互式参数调节器 —— 无需 GUI, 适合 SSH 远程直接输入调整参数。

用法:
  python3 ~/grasp_ws/src/smart_grasp/smart_grasp/param_tuner.py <node_name>
  # 或 (安装后) ros2 run smart_grasp param_tuner <node_name>

交互命令:
  name=value   设置参数, 自动识别 int/float/bool/list/str
  list         列出当前全部参数及取值
  quit / q     退出

示例:
  smart_grasp_executor> pre_grasp_offset_z=0.15
  smart_grasp_executor> gripper_open=0.08
  smart_grasp_executor> ws_x=[0.1,0.6]
  smart_grasp_detector> detect_mode=aruco
  smart_grasp_detector> aruco_dict=DICT_5X5_100
"""
import ast
import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def parse_value(text):
    """尽量把输入字符串解析成合适的 python 类型"""
    t = text.strip()
    low = t.lower()
    if low in ("true", "false"):
        return low == "true"
    try:
        return int(t)
    except ValueError:
        pass
    try:
        return float(t)
    except ValueError:
        pass
    if t.startswith("[") or t.startswith("("):
        try:
            return ast.literal_eval(t)
        except Exception:
            pass
    return t  # 字符串


class Tuner(Node):
    def __init__(self, target):
        super().__init__("param_tuner")
        self.target = target
        self.set_cli = self.create_client(
            SetParameters, f"/{target}/set_parameters")
        self.get_cli = self.create_client(
            GetParameters, f"/{target}/get_parameters")
        for cli in (self.set_cli, self.get_cli):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"等待节点 {target} ...")

    def get_all(self):
        req = GetParameters.Request()
        req.names = []
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().values

    def get_one(self, name):
        req = GetParameters.Request()
        req.names = [name]
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        vals = fut.result().values
        return vals[0] if vals else None

    def set_one(self, name, raw):
        cur = self.get_one(name)
        if cur is None:
            print(f"  参数 '{name}' 不存在")
            return
        ptype = cur.type
        value = parse_value(raw)
        pv = ParameterValue()
        pv.type = ptype
        try:
            if ptype == ParameterType.PARAMETER_BOOL:
                pv.bool_value = value if isinstance(value, bool) \
                    else str(value).lower() in ("true", "1", "yes")
            elif ptype == ParameterType.PARAMETER_INTEGER:
                pv.integer_value = int(round(float(value)))
            elif ptype == ParameterType.PARAMETER_DOUBLE:
                pv.double_value = float(value)
            elif ptype == ParameterType.PARAMETER_STRING:
                pv.string_value = str(value)
            elif ptype in (ParameterType.PARAMETER_DOUBLE_ARRAY,
                           ParameterType.PARAMETER_INTEGER_ARRAY,
                           ParameterType.PARAMETER_BYTE_ARRAY):
                arr = value if isinstance(value, (list, tuple)) else ast.literal_eval(str(value))
                pv.double_array_value = [float(x) for x in arr]
            elif ptype == ParameterType.PARAMETER_STRING_ARRAY:
                arr = value if isinstance(value, (list, tuple)) else ast.literal_eval(str(value))
                pv.string_array_value = [str(x) for x in arr]
            elif ptype == ParameterType.PARAMETER_BOOL_ARRAY:
                arr = value if isinstance(value, (list, tuple)) else ast.literal_eval(str(value))
                pv.bool_array_value = [bool(x) for x in arr]
            else:
                print("  不支持的参数类型")
                return
        except Exception as e:
            print(f"  值解析失败: {e}")
            return

        p = Parameter()
        p.name = name
        p.value = pv
        req = SetParameters.Request()
        req.parameters = [p]
        fut = self.set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res.results and res.results[0].successful:
            print(f"  OK  {name} = {value}")
        else:
            reason = res.results[0].reason if res.results else "unknown"
            print(f"  FAIL {name}: {reason}")

    def list_params(self):
        vals = self.get_all()
        print(f"=== {self.target} 参数 ===")
        for v in vals:
            print(f"  {v.name} = {v.value}")


def main(args=None):
    target = sys.argv[1] if len(sys.argv) > 1 else None
    if target is None:
        # 未指定节点名时, 自动列出当前在线节点让用户选
        rclpy.init(args=args)
        import rclpy.node
        tmp = rclpy.node.Node("_tmp_lister")
        names = tmp.get_node_names()
        rclpy.shutdown()
        cands = [n for n in names if "smart_grasp" in n] or names
        print("用法: param_tuner.py <node_name>")
        print("当前在线节点:")
        for n in cands:
            print(f"  {n}")
        if cands:
            print(f"\n示例: param_tuner.py {cands[0]}")
        return
    rclpy.init(args=args)
    rclpy.init(args=args)
    tuner = Tuner(target)
    prompt = f"{target}> "
    try:
        while True:
            try:
                line = input(prompt).strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                continue
            if line in ("quit", "q", "exit"):
                break
            if line == "list":
                tuner.list_params()
                continue
            if "=" not in line:
                print("  格式: name=value  (或 list / quit)")
                continue
            name, _, raw = line.partition("=")
            tuner.set_one(name.strip(), raw.strip())
    finally:
        tuner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
