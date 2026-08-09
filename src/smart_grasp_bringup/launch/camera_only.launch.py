"""Start D405 color and 2-D detection without an attached robot or hand-eye TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("smart_grasp_bringup")
    default_perception_config = os.path.join(share, "config", "perception.yaml")

    use_camera = LaunchConfiguration("use_camera")
    detector_backend = LaunchConfiguration("detector_backend")
    yolo_model = LaunchConfiguration("yolo_model")
    open_gui = LaunchConfiguration("open_gui")
    perception_config = LaunchConfiguration("perception_config")
    color_profile = LaunchConfiguration("color_profile")
    camera_serial_no = LaunchConfiguration("camera_serial_no")

    def quote_serial_no(context):
        serial = camera_serial_no.perform(context)
        if serial.startswith("'") and serial.endswith("'"):
            return serial
        return f"'{serial}'"

    def launch_camera(context, *args, **kwargs):
        return [
            GroupAction(
                scoped=True,
                forwarding=False,
                launch_configurations={"color_profile": color_profile},
                condition=IfCondition(use_camera),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(
                            get_package_share_directory("realsense2_camera"),
                            "launch", "rs_launch.py")),
                        launch_arguments={
                            "camera_namespace": "camera",
                            "camera_name": "camera",
                            "serial_no": quote_serial_no(context),
                            "align_depth.enable": "false",
                            "enable_color": "true",
                            "enable_depth": "false",
                            "pointcloud.enable": "false",
                            "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                            "depth_module.color_profile": LaunchConfiguration("color_profile"),
                        }.items(),
                    )
                ],
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument(
            "detector_backend", default_value="hsv", choices=["hsv", "yolo_seg"]),
        DeclareLaunchArgument("yolo_model", default_value=""),
        DeclareLaunchArgument(
            "perception_config", default_value=default_perception_config),
        DeclareLaunchArgument(
            "color_profile",
            default_value="640x480x30",
            description="D405 color stream profile: WIDTHxHEIGHTxFPS",
        ),
        DeclareLaunchArgument(
            "camera_serial_no",
            default_value="260322272696",
            description="D405 serial number used to pin the camera node.",
        ),
        DeclareLaunchArgument("open_gui", default_value="false"),
        OpaqueFunction(function=launch_camera),
        Node(
            package="smart_grasp",
            executable="detector_node",
            name="smart_grasp_detector",
            parameters=[
                perception_config,
                {
                    "camera_only": True,
                    "detector_backend": ParameterValue(detector_backend, value_type=str),
                    "yolo_model": ParameterValue(yolo_model, value_type=str),
                },
            ],
            output="screen",
        ),
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="smart_grasp_camera_only_view",
            arguments=["/smart_grasp/debug_image"],
            condition=IfCondition(open_gui),
        ),
    ])
