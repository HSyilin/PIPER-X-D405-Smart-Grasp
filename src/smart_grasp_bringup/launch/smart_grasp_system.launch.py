"""Start D405 perception, eye-in-hand TF, MoveIt, and guarded pick action."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("smart_grasp_bringup")
    default_perception_config = os.path.join(share, "config", "perception.yaml")
    default_grasp_config = os.path.join(share, "config", "grasp.yaml")
    calibration = os.path.join(share, "config", "handeye_20260725.yaml")
    moveit_share = get_package_share_directory("agx_arm_moveit")
    with open(os.path.join(moveit_share, "config", "kinematics.yaml"),
              "r", encoding="utf-8") as stream:
        kinematics = yaml.safe_load(stream)

    use_driver = LaunchConfiguration("use_driver")
    use_moveit = LaunchConfiguration("use_moveit")
    use_camera = LaunchConfiguration("use_camera")
    use_pick_server = LaunchConfiguration("use_pick_server")
    use_handeye_tf = LaunchConfiguration("use_handeye_tf")
    use_tf_validator = LaunchConfiguration("use_tf_validator")
    use_rviz = LaunchConfiguration("use_rviz")
    execute = LaunchConfiguration("execute")
    calibration_validated = LaunchConfiguration("calibration_validated")
    detector_backend = LaunchConfiguration("detector_backend")
    yolo_model = LaunchConfiguration("yolo_model")
    perception_config = LaunchConfiguration("perception_config")
    grasp_config = LaunchConfiguration("grasp_config")

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("agx_arm_ctrl"), "launch",
            "start_single_agx_arm.launch.py")),
        launch_arguments={
            "can_port": LaunchConfiguration("can_port"),
            "arm_type": "piper_x",
            "effector_type": "agx_gripper",
            "auto_enable": "false",
            "speed_percent": "10",
            "gripper_default_effort": "0.5",
            "control_enabled": "false",
        }.items(),
        condition=IfCondition(use_driver),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("agx_arm_moveit"), "launch", "demo.launch.py")),
        launch_arguments={
            "arm_type": "piper_x",
            "effector_type": "agx_gripper",
            "follow": IfElseSubstitution(use_driver, if_value="true", else_value="false"),
            "feedback_topic": "feedback/joint_states",
            "auto_control_gate": IfElseSubstitution(
                use_driver, if_value="true", else_value="false"),
            "control_gate_service": "control_enable",
            "use_rviz": use_rviz,
        }.items(),
        condition=IfCondition(use_moveit),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")),
        launch_arguments={
            "camera_namespace": "camera",
            "camera_name": "camera",
            "align_depth.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "pointcloud.enable": "false",
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
            "depth_module.color_profile": "640x480x30",
        }.items(),
        condition=IfCondition(use_camera),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_driver", default_value="true"),
        DeclareLaunchArgument("use_moveit", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_pick_server", default_value="true"),
        DeclareLaunchArgument("use_handeye_tf", default_value="true"),
        DeclareLaunchArgument("use_tf_validator", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("detector_backend", default_value="hsv",
                              choices=["hsv", "yolo_seg"]),
        DeclareLaunchArgument("yolo_model", default_value=""),
        DeclareLaunchArgument(
            "perception_config", default_value=default_perception_config),
        DeclareLaunchArgument("grasp_config", default_value=default_grasp_config),
        DeclareLaunchArgument("execute", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("calibration_validated", default_value="false",
                              choices=["true", "false"]),
        driver_launch,
        moveit_launch,
        camera_launch,
        Node(
            package="smart_grasp",
            executable="handeye_tf_node",
            name="smart_grasp_handeye_tf",
            parameters=[{"calibration_file": calibration}],
            output="screen",
            condition=IfCondition(use_handeye_tf),
        ),
        Node(
            package="smart_grasp",
            executable="detector_node",
            name="smart_grasp_detector",
            parameters=[
                perception_config,
                {
                    "detector_backend": ParameterValue(detector_backend, value_type=str),
                    "yolo_model": ParameterValue(yolo_model, value_type=str),
                },
            ],
            output="screen",
        ),
        Node(
            package="smart_grasp",
            executable="tf_validator_node",
            name="smart_grasp_tf_validator",
            output="screen",
            condition=IfCondition(use_tf_validator),
        ),
        Node(
            package="smart_grasp_moveit",
            executable="pick_server",
            name="smart_grasp_pick_server",
            parameters=[
                grasp_config,
                {
                    "execution_allowed": ParameterValue(execute, value_type=bool),
                    "calibration_validated": ParameterValue(
                        calibration_validated, value_type=bool),
                    "robot_description_kinematics": kinematics,
                    "joint_state_topic": ParameterValue(
                        IfElseSubstitution(
                            use_driver,
                            if_value="/feedback/joint_states",
                            else_value="/control/joint_states"),
                        value_type=str),
                },
            ],
            output="screen",
            condition=IfCondition(use_pick_server),
        ),
    ])
