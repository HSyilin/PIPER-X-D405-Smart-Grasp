"""Start navigation, smart grasp, and the Turing grasp mission sequencer."""

import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("smart_grasp_bringup")
    default_trajectory_file = os.path.join(
        "/home", "guest", "funny_lidar_slam", "data", "trajectories", "example_path.yaml"
    )
    default_map = os.path.join("/home", "guest", "funny_lidar_slam", "data", "map.yaml")
    default_grasp_config = os.path.join(
        share, "config", "grasp_test_box_60x40x40.yaml"
    )
    default_perception_config = os.path.join(
        share, "config", "perception_test_box_60x40x40.yaml"
    )
    default_calibration_file = os.path.join(share, "config", "handeye_20260725.yaml")

    use_navigation = LaunchConfiguration("use_navigation")
    use_lidar = LaunchConfiguration("use_lidar")
    use_localization = LaunchConfiguration("use_localization")
    use_chassis = LaunchConfiguration("use_chassis")
    use_map_server = LaunchConfiguration("use_map_server")
    use_path_rviz = LaunchConfiguration("use_path_rviz")
    use_grasp_system = LaunchConfiguration("use_grasp_system")
    use_arm_driver = LaunchConfiguration("use_arm_driver")
    use_camera = LaunchConfiguration("use_camera")
    use_grasp_rviz = LaunchConfiguration("use_grasp_rviz")
    use_mission_sequencer = LaunchConfiguration("use_mission_sequencer")
    run_can_bind = LaunchConfiguration("run_can_bind")

    trajectory_file = LaunchConfiguration("trajectory_file")
    map_yaml = LaunchConfiguration("map")
    robot_ip = LaunchConfiguration("robot_ip")
    accid = LaunchConfiguration("accid")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    path_topic = LaunchConfiguration("path_topic")
    cancel_topic = LaunchConfiguration("cancel_topic")
    base_frame = LaunchConfiguration("base_frame")
    lookahead_distance = LaunchConfiguration("lookahead_distance")
    target_speed = LaunchConfiguration("target_speed")
    pick_index = LaunchConfiguration("pick_index")
    stair_on_index = LaunchConfiguration("stair_on_index")
    stair_off_index = LaunchConfiguration("stair_off_index")
    target_class = LaunchConfiguration("target_class")
    detector_backend = LaunchConfiguration("detector_backend")
    yolo_model = LaunchConfiguration("yolo_model")
    yolo_class = LaunchConfiguration("yolo_class")
    yolo_confidence = LaunchConfiguration("yolo_confidence")
    grasp_config = LaunchConfiguration("grasp_config")
    perception_config = LaunchConfiguration("perception_config")
    calibration_file = LaunchConfiguration("calibration_file")
    camera_serial_no = LaunchConfiguration("camera_serial_no")
    can_port = LaunchConfiguration("can_port")
    can_bind_script = LaunchConfiguration("can_bind_script")
    grasp_start_delay = LaunchConfiguration("grasp_start_delay")
    mission_start_delay = LaunchConfiguration("mission_start_delay")

    can_bind = ExecuteProcess(
        cmd=["bash", can_bind_script],
        additional_env={"CAN_IFACE": can_port},
        output="screen",
        condition=IfCondition(run_can_bind),
    )

    smart_grasp_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "smart_grasp_system.launch.py")
        ),
        launch_arguments={
            "use_driver": use_arm_driver,
            "use_camera": use_camera,
            "use_moveit": "true",
            "use_detector": "true",
            "use_pick_server": "true",
            "use_handeye_tf": "true",
            "use_tf_validator": "true",
            "use_rviz": use_grasp_rviz,
            "use_live_feedback": "true",
            "auto_enable": "false",
            "can_port": can_port,
            "execute": "true",
            "calibration_validated": "true",
            "camera_serial_no": camera_serial_no,
            "detector_backend": detector_backend,
            "yolo_model": yolo_model,
            "yolo_class": yolo_class,
            "yolo_confidence": yolo_confidence,
            "grasp_config": grasp_config,
            "perception_config": perception_config,
            "calibration_file": calibration_file,
        }.items(),
        condition=IfCondition(use_grasp_system),
    )

    def launch_navigation(context, *args, **kwargs):
        if use_navigation.perform(context).lower() not in {"1", "true", "yes", "on"}:
            return []

        launch_args = {
            "use_rviz": use_path_rviz.perform(context),
            "use_lidar": use_lidar.perform(context),
            "use_localization": use_localization.perform(context),
            "use_chassis": use_chassis.perform(context),
            "use_map_server": use_map_server.perform(context),
            "robot_ip": robot_ip.perform(context),
            "accid": accid.perform(context),
            "cmd_vel_topic": cmd_vel_topic.perform(context),
            "path_topic": path_topic.perform(context),
            "cancel_topic": cancel_topic.perform(context),
            "auto_execute_trajectory": "false",
            "trajectory_file": trajectory_file.perform(context),
            "stair_on_index": stair_on_index.perform(context),
            "stair_off_index": stair_off_index.perform(context),
            "map": map_yaml.perform(context),
            "base_frame": base_frame.perform(context),
            "lookahead_distance": lookahead_distance.perform(context),
            "target_speed": target_speed.perform(context),
        }
        rendered_args = " ".join(
            f"{name}:={shlex.quote(value)}" for name, value in launch_args.items()
        )
        command = (
            "source /opt/ros/humble/setup.bash && "
            "source /home/guest/base_node/install/setup.bash && "
            "source /home/guest/lidar_ws/install/setup.bash && "
            "source /home/guest/funny_lidar_slam/install/setup.bash && "
            f"ros2 launch funny_lidar_slam trajectory_tracking_turing.launch.py {rendered_args}"
        )
        return [
            ExecuteProcess(
                cmd=["bash", "-lc", command],
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", "/tmp"),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("use_navigation", default_value="true"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("use_localization", default_value="true"),
            DeclareLaunchArgument("use_chassis", default_value="true"),
            DeclareLaunchArgument("use_map_server", default_value="true"),
            DeclareLaunchArgument("use_path_rviz", default_value="true"),
            DeclareLaunchArgument("use_grasp_system", default_value="true"),
            DeclareLaunchArgument("use_arm_driver", default_value="true"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("use_grasp_rviz", default_value="false"),
            DeclareLaunchArgument("use_mission_sequencer", default_value="true"),
            DeclareLaunchArgument("run_can_bind", default_value="true"),
            DeclareLaunchArgument("can_bind_script", default_value="/home/guest/can_bind.sh"),
            DeclareLaunchArgument("can_port", default_value="can0"),
            DeclareLaunchArgument("robot_ip", default_value="10.192.1.2"),
            DeclareLaunchArgument("accid", default_value="SF_TRON2A_199"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/nav_cmd_vel"),
            DeclareLaunchArgument("path_topic", default_value="/trajectory_tracking/path"),
            DeclareLaunchArgument("cancel_topic", default_value="/trajectory_tracking/cancel"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("lookahead_distance", default_value="0.45"),
            DeclareLaunchArgument("target_speed", default_value="0.25"),
            DeclareLaunchArgument("trajectory_file", default_value=default_trajectory_file),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("pick_index", default_value="2"),
            DeclareLaunchArgument("stair_on_index", default_value="10"),
            DeclareLaunchArgument("stair_off_index", default_value="11"),
            DeclareLaunchArgument("target_class", default_value="1"),
            DeclareLaunchArgument("detector_backend", default_value="yolo_seg"),
            DeclareLaunchArgument("yolo_model", default_value="/home/guest/best.pt"),
            DeclareLaunchArgument("yolo_class", default_value="1"),
            DeclareLaunchArgument("yolo_confidence", default_value="0.7"),
            DeclareLaunchArgument("grasp_config", default_value=default_grasp_config),
            DeclareLaunchArgument("perception_config", default_value=default_perception_config),
            DeclareLaunchArgument("calibration_file", default_value=default_calibration_file),
            DeclareLaunchArgument("camera_serial_no", default_value="260322272696"),
            DeclareLaunchArgument("grasp_start_delay", default_value="3.0"),
            DeclareLaunchArgument("mission_start_delay", default_value="10.0"),
            can_bind,
            OpaqueFunction(function=launch_navigation),
            TimerAction(period=grasp_start_delay, actions=[smart_grasp_system]),
            TimerAction(
                period=mission_start_delay,
                actions=[
                    Node(
                        package="smart_grasp",
                        executable="mission_sequencer",
                        name="turing_grasp_mission_sequencer",
                        output="screen",
                        condition=IfCondition(use_mission_sequencer),
                        parameters=[
                            {
                                "trajectory_file": ParameterValue(
                                    trajectory_file, value_type=str
                                ),
                                "frame_id": "map",
                                "base_frame": ParameterValue(base_frame, value_type=str),
                                "path_topic": ParameterValue(path_topic, value_type=str),
                                "cmd_vel_topic": ParameterValue(
                                    cmd_vel_topic, value_type=str
                                ),
                                "target_class": ParameterValue(
                                    target_class, value_type=str
                                ),
                                "execute_pick": True,
                                "pick_index": ParameterValue(
                                    pick_index, value_type=int
                                ),
                                "stair_on_index": ParameterValue(
                                    stair_on_index, value_type=int
                                ),
                                "stair_off_index": ParameterValue(
                                    stair_off_index, value_type=int
                                ),
                                "start_delay_s": 0.0,
                            }
                        ],
                    )
                ],
            ),
        ]
    )
