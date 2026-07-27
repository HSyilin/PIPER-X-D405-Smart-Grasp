"""
一键启动智能夹抓系统:
  1. realsense2_camera (D405, 深度对齐彩色)
  2. 视觉检测节点
    3. 仅启动感知；MoveIt抓取请使用 smart_grasp_bringup

机械臂驱动 (agx_arm_ctrl_single_node) 请在另一个终端按原有方式启动。

用法:
  ros2 launch smart_grasp smart_grasp.launch.py
  ros2 launch smart_grasp smart_grasp.launch.py use_camera:=false   # 相机已单独启动

  # 看图像 + 调参数 (本机有显示器/DISPLAY 时):
  ros2 launch smart_grasp smart_grasp.launch.py open_gui:=true

  # 看图像 (SSH 远程无 DISPLAY, 浏览器查看; 另用 param_tuner 调参):
  ros2 launch smart_grasp smart_grasp.launch.py open_web:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("smart_grasp")
    params = os.path.join(pkg_share, "config", "grasp_params.yaml")

    use_camera = LaunchConfiguration("use_camera")
    open_gui = LaunchConfiguration("open_gui")   # rqt 图像 + 参数 (需本机 DISPLAY)
    open_web = LaunchConfiguration("open_web")   # web_video_server 浏览器看图 (SSH 友好)

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch", "rs_launch.py")),
        launch_arguments={
            "camera_namespace": "camera",
            "camera_name": "camera",
            "align_depth.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
        }.items(),
        condition=IfCondition(use_camera),
    )

    # 本机有显示器时: 同时打开图像查看器 与 参数调节面板
    image_view_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="smart_grasp_image_view",
        arguments=["/smart_grasp/debug_image"],
        condition=IfCondition(open_gui),
    )
    reconfigure_node = Node(
        package="rqt_reconfigure",
        executable="rqt_reconfigure",
        name="rqt_reconfigure",
        arguments=["smart_grasp_detector", "smart_grasp_executor"],
        condition=IfCondition(open_gui),
    )
    # SSH 远程无 DISPLAY: 用浏览器看图像 (另用 ros2 run smart_grasp param_tuner 调参)
    web_video_node = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        condition=IfCondition(open_web),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("open_gui", default_value="false"),
        DeclareLaunchArgument("open_web", default_value="false"),
        realsense_launch,
        Node(
            package="smart_grasp",
            executable="detector_node",
            name="smart_grasp_detector",
            parameters=[params],
            output="screen",
        ),
        image_view_node,
        reconfigure_node,
        web_video_node,
    ])
