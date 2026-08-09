"""Start D405 perception, eye-in-hand TF, MoveIt, and guarded pick action."""

import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.join(
    get_package_share_directory("agx_arm_moveit"), "launch"))
from _moveit_config_builder import build_moveit_config as build_agx_moveit_config


def generate_launch_description():
    share = get_package_share_directory("smart_grasp_bringup")
    default_perception_config = os.path.join(share, "config", "perception.yaml")
    default_grasp_config = os.path.join(
        share, "config", "grasp_test_box_60x40x40.yaml")
    default_calibration = os.path.join(share, "config", "handeye_20260725.yaml")

    use_driver = LaunchConfiguration("use_driver")
    use_moveit = LaunchConfiguration("use_moveit")
    use_camera = LaunchConfiguration("use_camera")
    use_detector = LaunchConfiguration("use_detector")
    use_pick_server = LaunchConfiguration("use_pick_server")
    use_handeye_tf = LaunchConfiguration("use_handeye_tf")
    use_tf_validator = LaunchConfiguration("use_tf_validator")
    use_rviz = LaunchConfiguration("use_rviz")
    use_live_feedback = LaunchConfiguration("use_live_feedback")
    auto_enable = LaunchConfiguration("auto_enable")
    firmware_override = LaunchConfiguration("firmware_override")
    execute = LaunchConfiguration("execute")
    calibration_validated = LaunchConfiguration("calibration_validated")
    home_joint_positions = LaunchConfiguration("home_joint_positions")
    post_pick_joint_positions = LaunchConfiguration("post_pick_joint_positions")
    camera_serial_no = LaunchConfiguration("camera_serial_no")
    detector_backend = LaunchConfiguration("detector_backend")
    yolo_model = LaunchConfiguration("yolo_model")
    yolo_class = LaunchConfiguration("yolo_class")
    yolo_confidence = LaunchConfiguration("yolo_confidence")
    perception_config = LaunchConfiguration("perception_config")
    grasp_config = LaunchConfiguration("grasp_config")
    calibration_file = LaunchConfiguration("calibration_file")
    handeye_base_to_tcp_source = LaunchConfiguration("handeye_base_to_tcp_source")
    offline_observation_joint_positions = LaunchConfiguration(
        "offline_observation_joint_positions")

    def reconcile_calibration(context):
        """Keep the runtime execution gate consistent with the hand-eye record.

        The pick server only trusts ``calibration_validated``. If the operator
        passes ``calibration_validated:=true`` but the on-disk hand-eye file still
        reports ``validated: false``, the gate must NOT open. We force it closed
        and warn instead of silently allowing real execution on unverified data.
        """
        requested = calibration_validated.perform(context).lower() == "true"
        resolved_calibration = calibration_file.perform(context)
        try:
            with open(resolved_calibration, "r", encoding="utf-8") as stream:
                eye = yaml.safe_load(stream) or {}
            on_disk_valid = bool(eye.get("validated", False))
        except Exception as exc:  # pragma: no cover - defensive
            on_disk_valid = False
            print(
                f"[smart_grasp_bringup] could not read "
                f"{resolved_calibration}: {exc}")
        if requested and not on_disk_valid:
            print(
                "[smart_grasp_bringup] WARNING: calibration_validated:=true requested, "
                "but hand-eye file reports validated: false. Forcing execution gate "
                "CLOSED to prevent unverified execution.")
            return [SetLaunchConfiguration("calibration_validated", "false")]
        return []

    def normalize_home_joint_positions(context):
        """Keep AGX home-joint launch args in the string form the node expects.

        launch_ros rejects an empty YAML list here, while the AGX node itself
        interprets an empty string as "use the default home pose".
        """
        home = home_joint_positions.perform(context).strip()
        if home == "[]":
            return [SetLaunchConfiguration("home_joint_positions", "")]
        return []

    def parsed_post_pick_joint_positions(context):
        post_pick = post_pick_joint_positions.perform(context).strip()
        if not post_pick:
            return []
        try:
            values = yaml.safe_load(post_pick)
        except yaml.YAMLError as exc:
            raise RuntimeError(
                f"post_pick_joint_positions must be a YAML numeric list: {exc}")
        if not isinstance(values, list) or len(values) != 6:
            raise RuntimeError(
                "post_pick_joint_positions must contain exactly 6 numeric values")
        try:
            return [float(value) for value in values]
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                "post_pick_joint_positions must contain only numeric values") from exc

    def quote_serial_no(context):
        serial = camera_serial_no.perform(context)
        if serial.startswith("'") and serial.endswith("'"):
            return serial
        return f"'{serial}'"

    def launch_pick_server(context, *args, **kwargs):
        moveit_config = build_agx_moveit_config(context)
        return [
            Node(
                package="smart_grasp_moveit",
                executable="pick_server",
                name="smart_grasp_pick_server",
                parameters=[
                    grasp_config,
                    moveit_config.to_dict(),
                    {
                        "execution_allowed": ParameterValue(execute, value_type=bool),
                        "calibration_validated": ParameterValue(
                            calibration_validated, value_type=bool),
                        "simulation_mode": ParameterValue(
                            IfElseSubstitution(
                                use_live_feedback,
                                if_value="false",
                                else_value="true"),
                            value_type=bool),
                        "joint_state_topic": ParameterValue(
                            IfElseSubstitution(
                                use_live_feedback,
                                if_value="/feedback/joint_states",
                                else_value="/control/joint_states"),
                            value_type=str),
                        "post_pick_joint_positions": parsed_post_pick_joint_positions(context),
                    },
                ],
                remappings=[
                    (
                        "joint_states",
                        IfElseSubstitution(
                            use_live_feedback,
                            if_value="/feedback/joint_states",
                            else_value="/control/joint_states"),
                    ),
                ],
                output="screen",
                condition=IfCondition(use_pick_server),
            )
        ]

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("agx_arm_ctrl"), "launch",
            "start_single_agx_arm.launch.py")),
        launch_arguments={
            "can_port": LaunchConfiguration("can_port"),
            "arm_type": "piper_x",
            "effector_type": "agx_gripper",
            "auto_enable": auto_enable,
            "speed_percent": "10",
            "gripper_default_effort": "0.5",
            "control_enabled": IfElseSubstitution(
                use_live_feedback,
                if_value="false",
                else_value="true",
            ),
            "allow_remote_disable": "false",
            "home_joint_positions": home_joint_positions,
            "firmware_override": firmware_override,
        }.items(),
        condition=IfCondition(use_driver),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("agx_arm_moveit"), "launch", "demo.launch.py")),
        launch_arguments={
            "arm_type": "piper_x",
            "effector_type": "agx_gripper",
            "follow": use_live_feedback,
            "feedback_topic": "feedback/joint_states",
            "auto_control_gate": use_live_feedback,
            "control_gate_service": "control_enable",
            "use_rviz": use_rviz,
        }.items(),
        condition=IfCondition(use_moveit),
    )

    def launch_camera(context, *args, **kwargs):
        # Keep the serial quoted so rs_launch.py preserves it as a string.
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory("realsense2_camera"),
                    "launch", "rs_launch.py")),
                launch_arguments={
                    "camera_namespace": "camera",
                    "camera_name": "camera",
                    "serial_no": quote_serial_no(context),
                    "align_depth.enable": "true",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "enable_infra": "false",
                    "pointcloud.enable": "false",
                    "rgb_camera.color_profile": "848x480x30",
                    "depth_module.depth_profile": "848x480x30",
                }.items(),
                condition=IfCondition(use_camera),
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument("use_driver", default_value="true"),
        DeclareLaunchArgument("use_moveit", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_detector", default_value="true"),
        DeclareLaunchArgument("use_pick_server", default_value="true"),
        DeclareLaunchArgument("use_handeye_tf", default_value="true"),
        DeclareLaunchArgument("use_tf_validator", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "auto_enable",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Auto-enable the AGX arm driver on startup."
            ),
        ),
        DeclareLaunchArgument(
            "firmware_override",
            default_value="",
            description=(
                "Optional AGX arm firmware version used only if the driver "
                "cannot query firmware from CAN during startup."
            ),
        ),
        DeclareLaunchArgument(
            "camera_serial_no",
            default_value="260322272696",
            description="D405 serial number used to pin the camera node.",
        ),
        DeclareLaunchArgument(
            "use_live_feedback", default_value="true",
            choices=["true", "false"],
            description=(
                "Use live /feedback/joint_states and /control_enable even when "
                "use_driver:=false, allowing an already-running arm driver."
            )),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument(
            "home_joint_positions",
            default_value="",
            description=(
                "Arm joint positions used by official /move_home. Empty uses the "
                "AGX driver default home."
            )),
        DeclareLaunchArgument(
            "post_pick_joint_positions",
            default_value=(
                "[-1.583554684, 0.186139365, -0.379190233, "
                "0.550424486, -0.055798176, 0.0]"
            ),
            description=(
                "Intermediate joint pose reached after a successful pick. This "
                "does not change official /move_home."
            )),
        DeclareLaunchArgument("detector_backend", default_value="hsv",
                              choices=["hsv", "yolo_seg"]),
        DeclareLaunchArgument("yolo_model", default_value=""),
        DeclareLaunchArgument(
            "yolo_class", default_value="blue_block",
            description=(
                "Class name the YOLO-Seg model must output to be treated as the "
                "grasp target. MUST match model.names (e.g. best.pt uses '1')."
            )),
        DeclareLaunchArgument(
            "yolo_confidence", default_value="0.5",
            description="Minimum confidence for the YOLO-Seg detector.",
        ),
        DeclareLaunchArgument(
            "perception_config", default_value=default_perception_config),
        DeclareLaunchArgument("grasp_config", default_value=default_grasp_config),
        DeclareLaunchArgument(
            "calibration_file", default_value=default_calibration),
        DeclareLaunchArgument(
            "handeye_base_to_tcp_source",
            default_value="tcp_pose",
            choices=["tcp_pose", "offline_observation"],
            description=(
                "Source for base_link->tcp_link in hand-eye TF. Use "
                "offline_observation only for camera/perception checks when the "
                "arm is unpowered and fixed at the configured observation pose."
            )),
        DeclareLaunchArgument(
            "offline_observation_joint_positions",
            default_value="[-1.570796327, 0.195633956, -0.481920313, 0.945113243, 0.0, 0.0]",
            description=(
                "Joint1..joint6 used to synthesize base_link->tcp_link when "
                "handeye_base_to_tcp_source:=offline_observation."
            )),
        DeclareLaunchArgument("execute", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("calibration_validated", default_value="false",
                              choices=["true", "false"]),
        OpaqueFunction(function=reconcile_calibration),
        OpaqueFunction(function=normalize_home_joint_positions),
        driver_launch,
        moveit_launch,
        OpaqueFunction(function=launch_camera),
        Node(
            package="smart_grasp",
            executable="handeye_tf_node",
            name="smart_grasp_handeye_tf",
            parameters=[{
                "calibration_file": ParameterValue(
                    calibration_file, value_type=str),
                "base_to_tcp_source": ParameterValue(
                    handeye_base_to_tcp_source, value_type=str),
                "offline_observation_joint_positions": ParameterValue(
                    offline_observation_joint_positions, value_type=str),
            }],
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
                    "yolo_class": ParameterValue(yolo_class, value_type=str),
                    "yolo_confidence": ParameterValue(yolo_confidence, value_type=float),
                },
            ],
            output="screen",
            condition=IfCondition(use_detector),
        ),
        Node(
            package="smart_grasp",
            executable="tf_validator_node",
            name="smart_grasp_tf_validator",
            parameters=[{
                "target_class": ParameterValue(yolo_class, value_type=str),
            }],
            output="screen",
            condition=IfCondition(use_tf_validator),
        ),
        OpaqueFunction(function=launch_pick_server),
    ])
