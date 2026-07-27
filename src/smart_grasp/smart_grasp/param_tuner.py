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
  smart_grasp_pick_server> pregrasp_distance=0.15
  smart_grasp_pick_server> gripper_open=0.055
  smart_grasp_detector> hsv_lower=[90,80,50]
"""
import ast
import json
import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
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
            try:
                return json.loads(t)
            except Exception:
                pass
    return t  # 字符串


def _parse_bool(value):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"无法解析布尔值: {value}")


def make_parameter_value(parameter_type, raw):
    """Create a correctly typed ROS ParameterValue from interactive input."""
    value = parse_value(raw)
    result = ParameterValue(type=parameter_type)
    if parameter_type == ParameterType.PARAMETER_BOOL:
        result.bool_value = _parse_bool(value)
    elif parameter_type == ParameterType.PARAMETER_INTEGER:
        result.integer_value = int(round(float(value)))
    elif parameter_type == ParameterType.PARAMETER_DOUBLE:
        result.double_value = float(value)
    elif parameter_type == ParameterType.PARAMETER_STRING:
        result.string_value = str(value)
    elif parameter_type == ParameterType.PARAMETER_BYTE_ARRAY:
        numbers = [int(item) for item in value]
        if any(item < 0 or item > 255 for item in numbers):
            raise ValueError("字节数组元素必须在 0..255 范围内")
        result.byte_array_value = [bytes((item,)) for item in numbers]
    elif parameter_type == ParameterType.PARAMETER_BOOL_ARRAY:
        result.bool_array_value = [_parse_bool(item) for item in value]
    elif parameter_type == ParameterType.PARAMETER_INTEGER_ARRAY:
        result.integer_array_value = [int(item) for item in value]
    elif parameter_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        result.double_array_value = [float(item) for item in value]
    elif parameter_type == ParameterType.PARAMETER_STRING_ARRAY:
        result.string_array_value = [str(item) for item in value]
    else:
        raise ValueError(f"不支持的参数类型: {parameter_type}")
    return result


def parameter_value_to_python(value):
    """Return the active value field from a ROS ParameterValue."""
    fields = {
        ParameterType.PARAMETER_BOOL: "bool_value",
        ParameterType.PARAMETER_INTEGER: "integer_value",
        ParameterType.PARAMETER_DOUBLE: "double_value",
        ParameterType.PARAMETER_STRING: "string_value",
        ParameterType.PARAMETER_BYTE_ARRAY: "byte_array_value",
        ParameterType.PARAMETER_BOOL_ARRAY: "bool_array_value",
        ParameterType.PARAMETER_INTEGER_ARRAY: "integer_array_value",
        ParameterType.PARAMETER_DOUBLE_ARRAY: "double_array_value",
        ParameterType.PARAMETER_STRING_ARRAY: "string_array_value",
    }
    field = fields.get(value.type)
    if field is None:
        return None
    result = getattr(value, field)
    if value.type == ParameterType.PARAMETER_BYTE_ARRAY:
        return [item[0] for item in result]
    return list(result) if value.type >= ParameterType.PARAMETER_BOOL_ARRAY else result


class Tuner(Node):
    def __init__(self, target):
        super().__init__("param_tuner")
        self.target = target
        self.set_cli = self.create_client(
            SetParameters, f"/{target}/set_parameters")
        self.get_cli = self.create_client(
            GetParameters, f"/{target}/get_parameters")
        self.list_cli = self.create_client(
            ListParameters, f"/{target}/list_parameters")
        for cli in (self.set_cli, self.get_cli, self.list_cli):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"等待节点 {target} ...")

    def get_all(self):
        list_request = ListParameters.Request()
        list_request.depth = ListParameters.Request.DEPTH_RECURSIVE
        list_future = self.list_cli.call_async(list_request)
        rclpy.spin_until_future_complete(self, list_future)
        list_response = list_future.result()
        if list_response is None:
            raise RuntimeError("列出参数失败")
        names = sorted(list_response.result.names)
        if not names:
            return []
        get_request = GetParameters.Request(names=names)
        get_future = self.get_cli.call_async(get_request)
        rclpy.spin_until_future_complete(self, get_future)
        get_response = get_future.result()
        if get_response is None:
            raise RuntimeError("读取参数失败")
        return list(zip(names, get_response.values))

    def get_one(self, name):
        req = GetParameters.Request()
        req.names = [name]
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        response = fut.result()
        vals = response.values if response is not None else []
        return vals[0] if vals else None

    def set_one(self, name, raw):
        cur = self.get_one(name)
        if cur is None or cur.type == ParameterType.PARAMETER_NOT_SET:
            print(f"  参数 '{name}' 不存在")
            return
        try:
            value = parse_value(raw)
            pv = make_parameter_value(cur.type, raw)
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
        print(f"=== {self.target} 参数 ===")
        try:
            parameters = self.get_all()
        except RuntimeError as exc:
            print(f"  FAIL: {exc}")
            return
        for name, value in parameters:
            print(f"  {name} = {parameter_value_to_python(value)!r}")


def main(args=None):
    target = sys.argv[1] if len(sys.argv) > 1 else None
    if target is None:
        # 未指定节点名时, 自动列出当前在线节点让用户选
        rclpy.init(args=args)
        tmp = Node("_tmp_lister")
        try:
            names = tmp.get_node_names()
        finally:
            tmp.destroy_node()
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
