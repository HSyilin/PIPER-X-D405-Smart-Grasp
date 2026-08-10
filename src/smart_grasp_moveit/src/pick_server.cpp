#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <agx_arm_msgs/msg/gripper_status.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <smart_grasp_interfaces/action/pick_object.hpp>
#include <smart_grasp_interfaces/msg/detected_object.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "smart_grasp_moveit/pick_safety.hpp"

using namespace std::chrono_literals;

class PickServer : public rclcpp::Node
{
public:
  using PickObject = smart_grasp_interfaces::action::PickObject;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PickObject>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  PickServer()
  : Node("smart_grasp_pick_server"), busy_(false)
  {
    declare_parameter("planning_group", "arm");
    declare_parameter("base_frame", "base_link");
    declare_parameter("end_effector_link", "tcp_link");
    declare_parameter("planner_id", "RRTConnectkConfigDefault");
    declare_parameter("planning_time", 5.0);
    declare_parameter("planning_attempts", 5);
    declare_parameter("velocity_scaling", 0.10);
    declare_parameter("acceleration_scaling", 0.10);
    declare_parameter("execution_allowed", false);
    declare_parameter("simulation_mode", false);
    declare_parameter("calibration_validated", false);
    declare_parameter("target_max_age", 0.5);
    declare_parameter("detection_wait_timeout", 1.0);
    declare_parameter("stable_detection_wait_timeout", 3.0);
    declare_parameter("ambiguity_score_gap", 0.10);
    declare_parameter("pregrasp_distance", 0.100);
    declare_parameter("grasp_depth", 0.020);
    declare_parameter("lift_height", 0.050);
    declare_parameter("tcp_to_grasp_xyz", std::vector<double>{0.0, 0.0, 0.1425});
    declare_parameter("cartesian_step", 0.005);
    declare_parameter("cartesian_min_fraction", 0.95);
    declare_parameter("time_parameterize_cartesian", true);
    declare_parameter("cartesian_velocity_scaling", -1.0);
    declare_parameter("cartesian_acceleration_scaling", -1.0);
    declare_parameter("validate_all_candidate_approaches", false);
    declare_parameter("pregrasp_reobserve_mode", "update");
    declare_parameter("reobserve_max_xy_shift", 0.005);
    declare_parameter("reobserve_max_z_shift", 0.005);
    declare_parameter("reobserve_max_axis_yaw_deg", 5.0);
    // When true, a missing/unstable reobservation is not fatal: keep the
    // observation-pose locked target and proceed to grasp. Used for static
    // objects / YOLO-based picking where the pregrasp top-down view may not
    // reacquire the same track.
    declare_parameter("allow_reobserve_fallback", true);
    declare_parameter("start_joint_tolerance", 0.05);
    declare_parameter("execution_joint_tolerance", 0.03);
    declare_parameter("execution_settle_timeout", 20.0);
    declare_parameter("wrist_jump_threshold", 0.5);
    declare_parameter("observation_joint_positions", std::vector<double>{});
    declare_parameter("observation_joint_tolerance", 0.02);
    declare_parameter("post_pick_joint_positions", std::vector<double>{});
    declare_parameter("post_pick_joint_tolerance", 0.05);
    declare_parameter("post_pick_final_joint_positions", std::vector<double>{});
    declare_parameter("post_pick_final_joint_tolerance", 0.05);
    declare_parameter("gripper_open", 0.090);
    declare_parameter("gripper_close", 0.050);
    declare_parameter("gripper_force", 0.5);
    declare_parameter("gripper_motion_time", 1.0);
    declare_parameter("contact_width_min", 0.052);
    declare_parameter("contact_width_max", 0.068);
    declare_parameter("release_on_contact_not_detected", false);
    declare_parameter("simulation_contact_width", 0.060);
    declare_parameter("gripper_timeout", 3.0);
    declare_parameter("arm_action", "/arm_controller/follow_joint_trajectory");
    declare_parameter("gripper_action", "/gripper_controller/follow_joint_trajectory");
    declare_parameter("joint_state_topic", "/feedback/joint_states");
    declare_parameter("target_class", "blue_block");
    declare_parameter("table_height", -999.0);
    declare_parameter("table_size", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("table_center_xy", std::vector<double>{0.0, 0.0});
    declare_parameter("target_table_clearance", 0.001);
    // Static obstacles fixed relative to base_frame (radar mast, main controller
    // box, cable trays, etc.). One obstacle per line, CSV without spaces:
    //   id,size_x,size_y,size_z,pos_x,pos_y,pos_z
    // Empty string means "no extra obstacles". Coordinates are in base_frame.
    declare_parameter("static_obstacles", "");
    // Planning workspace box [min_x,min_y,min_z,max_x,max_y,max_z] in base_frame.
    // Acts as a hard bound so the planner never routes the arm outside the safe
    // volume around the radar / controller. Empty disables the bound.
    declare_parameter("workspace", std::vector<double>{});

    detection_sub_ = create_subscription<smart_grasp_interfaces::msg::DetectedObject>(
      "/smart_grasp/detections", 20,
      std::bind(&PickServer::detectionCallback, this, std::placeholders::_1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), 20,
      std::bind(&PickServer::jointCallback, this, std::placeholders::_1));
    gripper_sub_ = create_subscription<agx_arm_msgs::msg::GripperStatus>(
      "/feedback/gripper_status", 20,
      std::bind(&PickServer::gripperCallback, this, std::placeholders::_1));
    planning_scene_client_ = create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");
    arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, get_parameter("arm_action").as_string());
    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, get_parameter("gripper_action").as_string());
  }

  ~PickServer() override
  {
    std::lock_guard<std::mutex> lock(execution_thread_mutex_);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

  void initialize()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), get_parameter("planning_group").as_string());
    move_group_->setPoseReferenceFrame(get_parameter("base_frame").as_string());
    move_group_->setEndEffectorLink(get_parameter("end_effector_link").as_string());
    move_group_->setPlannerId(get_parameter("planner_id").as_string());
    move_group_->setPlanningTime(get_parameter("planning_time").as_double());
    move_group_->setNumPlanningAttempts(get_parameter("planning_attempts").as_int());
    move_group_->setMaxVelocityScalingFactor(get_parameter("velocity_scaling").as_double());
    move_group_->setMaxAccelerationScalingFactor(get_parameter("acceleration_scaling").as_double());
    const auto ws = get_parameter("workspace").as_double_array();
    if (ws.size() == 6) {
      move_group_->setWorkspace(
        ws[0], ws[1], ws[2], ws[3], ws[4], ws[5]);
      RCLCPP_INFO(get_logger(),
        "planning workspace set to [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
        ws[0], ws[1], ws[2], ws[3], ws[4], ws[5]);
    }
    action_server_ = rclcpp_action::create_server<PickObject>(
      this, "/smart_grasp/pick",
      std::bind(&PickServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickServer::handleCancel, this, std::placeholders::_1),
      std::bind(&PickServer::handleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "pick server ready; execution_allowed=%s",
      get_parameter("execution_allowed").as_bool() ? "true" : "false");
  }

private:
  struct Candidate
  {
    smart_grasp_interfaces::msg::DetectedObject object;
    geometry_msgs::msg::Pose grasp;
    geometry_msgs::msg::Pose pregrasp;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::msg::RobotTrajectory validated_approach;
    double approach_fraction{0.0};
    double score{0.0};
    bool reverse{false};
  };

  void detectionCallback(const smart_grasp_interfaces::msg::DetectedObject::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      pruneDetectionsLocked(get_clock()->now());
      detections_[msg->track_id] = *msg;
    }
    detection_condition_.notify_all();
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_joints_ = *msg;
    }
    joint_condition_.notify_all();
  }

  void gripperCallback(const agx_arm_msgs::msg::GripperStatus::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_gripper_ = *msg;
      have_gripper_ = true;
      ++gripper_feedback_sequence_;
    }
    gripper_condition_.notify_all();
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PickObject::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    if (move_group_) {
      move_group_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (busy_.exchange(true)) {
      abort(goal_handle, PickObject::Result::BUSY, "pick server is busy");
      return;
    }
    std::lock_guard<std::mutex> lock(execution_thread_mutex_);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread([this, goal_handle]() {
      try {
        execute(goal_handle);
      } catch (const std::exception & error) {
        abort(goal_handle, PickObject::Result::INTERNAL_ERROR,
          std::string("unhandled pick exception: ") + error.what());
      } catch (...) {
        abort(goal_handle, PickObject::Result::INTERNAL_ERROR, "unhandled pick exception");
      }
      busy_ = false;
    });
  }

  void feedback(
    const std::shared_ptr<GoalHandle> & handle, const std::string & stage,
    const std::string & detail, uint32_t count)
  {
    auto msg = std::make_shared<PickObject::Feedback>();
    msg->stage = stage;
    msg->detail = detail;
    msg->candidate_count = count;
    handle->publish_feedback(msg);
  }

  bool canceled(const std::shared_ptr<GoalHandle> & handle)
  {
    if (!handle->is_canceling()) {
      return false;
    }
    auto result = std::make_shared<PickObject::Result>();
    result->success = false;
    result->error_code = PickObject::Result::CANCELED;
    result->message = "pick canceled";
    handle->canceled(result);
    return true;
  }

  void abort(
    const std::shared_ptr<GoalHandle> & handle, uint16_t code,
    const std::string & message,
    const smart_grasp_interfaces::msg::DetectedObject * object = nullptr)
  {
    auto result = std::make_shared<PickObject::Result>();
    result->success = false;
    result->error_code = code;
    result->message = message;
    if (object) {
      result->object_pose.header = object->header;
      result->object_pose.pose = object->pose.pose;
      result->size = object->size;
    }
    handle->abort(result);
  }

  class PickTiming
  {
  public:
    explicit PickTiming(const rclcpp::Logger & logger)
    : logger_(logger),
      started_(std::chrono::steady_clock::now()),
      last_(started_)
    {
    }

    void mark(const std::string & stage)
    {
      const auto now = std::chrono::steady_clock::now();
      const double stage_seconds = std::chrono::duration<double>(now - last_).count();
      const double total_seconds = std::chrono::duration<double>(now - started_).count();
      last_ = now;
      entries_.push_back({stage, stage_seconds, total_seconds});
      RCLCPP_INFO(
        logger_, "pick timing %-24s stage=%.3fs total=%.3fs",
        stage.c_str(), stage_seconds, total_seconds);
    }

    std::string summary() const
    {
      std::ostringstream out;
      out << std::fixed << std::setprecision(2);
      for (size_t i = 0; i < entries_.size(); ++i) {
        if (i > 0) {
          out << ", ";
        }
        out << entries_[i].stage << "=" << entries_[i].stage_seconds << "s";
      }
      if (!entries_.empty()) {
        out << ", total=" << entries_.back().total_seconds << "s";
      }
      return out.str();
    }

  private:
    struct Entry
    {
      std::string stage;
      double stage_seconds;
      double total_seconds;
    };

    rclcpp::Logger logger_;
    std::chrono::steady_clock::time_point started_;
    std::chrono::steady_clock::time_point last_;
    std::vector<Entry> entries_;
  };

  static double detectionScore(const smart_grasp_interfaces::msg::DetectedObject & object)
  {
    return object.confidence + 0.25 * object.depth_valid_ratio + (object.stable ? 0.20 : 0.0);
  }

  void pruneDetectionsLocked(const rclcpp::Time & now)
  {
    const double max_age = get_parameter("target_max_age").as_double();
    for (auto it = detections_.begin(); it != detections_.end();) {
      const double age = (now - rclcpp::Time(it->second.header.stamp)).seconds();
      if (!std::isfinite(age) || age < -max_age || age > max_age) {
        it = detections_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::vector<smart_grasp_interfaces::msg::DetectedObject> currentDetectionsLocked(
    const std::string & target_class,
    const std::optional<uint32_t> target_track_id = std::nullopt,
    const std::optional<rclcpp::Time> minimum_stamp = std::nullopt)
  {
    std::vector<smart_grasp_interfaces::msg::DetectedObject> current;
    const auto now = get_clock()->now();
    const double max_age = get_parameter("target_max_age").as_double();
    pruneDetectionsLocked(now);
    for (const auto & entry : detections_) {
      const auto & object = entry.second;
      const double age = (now - rclcpp::Time(object.header.stamp)).seconds();
      if (object.class_name == target_class &&
        (!target_track_id || object.track_id == *target_track_id) &&
        (!minimum_stamp || rclcpp::Time(object.header.stamp) > *minimum_stamp) &&
        age >= 0.0 && age <= max_age)
      {
        current.push_back(object);
      }
    }
    std::sort(current.begin(), current.end(), [](const auto & left, const auto & right) {
      return detectionScore(left) > detectionScore(right);
    });
    return current;
  }

  std::vector<smart_grasp_interfaces::msg::DetectedObject> currentDetections(
    const std::string & target_class,
    const std::optional<uint32_t> target_track_id = std::nullopt,
    const std::optional<rclcpp::Time> minimum_stamp = std::nullopt)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return currentDetectionsLocked(target_class, target_track_id, minimum_stamp);
  }

  std::vector<smart_grasp_interfaces::msg::DetectedObject> waitForCurrentDetections(
    const std::string & target_class,
    const std::optional<uint32_t> target_track_id = std::nullopt,
    bool require_stable = false,
    const std::optional<rclcpp::Time> minimum_stamp = std::nullopt)
  {
    const auto timeout = std::chrono::duration<double>(
      get_parameter(
        require_stable ? "stable_detection_wait_timeout" : "detection_wait_timeout").as_double());
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::vector<smart_grasp_interfaces::msg::DetectedObject> current;
    const auto ready = [&]() {
      current = currentDetectionsLocked(target_class, target_track_id, minimum_stamp);
      return !current.empty() &&
        (!require_stable || std::any_of(current.begin(), current.end(), [](const auto & object) {
          return object.stable && object.rejection_reason.empty();
        }));
    };
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (ready()) {
      return current;
    }
    detection_condition_.wait_until(lock, deadline, ready);
    return current;
  }

  static geometry_msgs::msg::Pose makeGraspPose(
    const smart_grasp_interfaces::msg::DetectedObject & object, bool reverse,
    double grasp_depth, const std::vector<double> & tcp_offset)
  {
    tf2::Quaternion object_q;
    tf2::fromMsg(object.pose.pose.orientation, object_q);
    tf2::Matrix3x3 object_rotation(object_q);
    // The detector's object X axis is the chosen horizontal axis on the top
    // face. In the Piper-X URDF the finger prismatic axes resolve to TCP X
    // after the gripper's fixed yaw, so aligning TCP X with this axis keeps the
    // jaws centered on the block face.
    tf2::Vector3 x_axis = object_rotation.getColumn(0);
    x_axis.setZ(0.0);
    x_axis.normalize();
    if (reverse) {
      x_axis = -x_axis;
    }
    const tf2::Vector3 z_axis(0.0, 0.0, -1.0);
    tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
    x_axis = y_axis.cross(z_axis).normalized();
    tf2::Matrix3x3 rotation(
      x_axis.x(), y_axis.x(), z_axis.x(),
      x_axis.y(), y_axis.y(), z_axis.y(),
      x_axis.z(), y_axis.z(), z_axis.z());
    tf2::Quaternion grasp_q;
    rotation.getRotation(grasp_q);
    tf2::Vector3 center(
      object.pose.pose.position.x,
      object.pose.pose.position.y,
      object.pose.pose.position.z + 0.5 * object.size.z - grasp_depth);
    const tf2::Vector3 offset(tcp_offset[0], tcp_offset[1], tcp_offset[2]);
    const tf2::Vector3 tcp_position = center - tf2::quatRotate(grasp_q, offset);
    geometry_msgs::msg::Pose pose;
    pose.position.x = tcp_position.x();
    pose.position.y = tcp_position.y();
    pose.position.z = tcp_position.z();
    pose.orientation = tf2::toMsg(grasp_q);
    return pose;
  }

  static double trajectoryLength(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    const auto & points = trajectory.joint_trajectory.points;
    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
      const size_t count = std::min(points[i - 1].positions.size(), points[i].positions.size());
      for (size_t j = 0; j < count; ++j) {
        const double difference = points[i].positions[j] - points[i - 1].positions[j];
        length += difference * difference;
      }
    }
    return std::sqrt(length);
  }

  bool hasWristJump(const moveit_msgs::msg::RobotTrajectory & trajectory) const
  {
    const auto & names = trajectory.joint_trajectory.joint_names;
    const auto found = std::find(names.begin(), names.end(), "joint6");
    if (found == names.end()) {
      return true;
    }
    const size_t index = static_cast<size_t>(std::distance(names.begin(), found));
    const double threshold = get_parameter("wrist_jump_threshold").as_double();
    const auto & points = trajectory.joint_trajectory.points;
    for (size_t i = 1; i < points.size(); ++i) {
      if (index >= points[i - 1].positions.size() || index >= points[i].positions.size() ||
        std::abs(points[i].positions[index] - points[i - 1].positions[index]) > threshold)
      {
        return true;
      }
    }
    return false;
  }

  bool startStateMatches(const moveit_msgs::msg::RobotTrajectory & trajectory) const
  {
    if (trajectory.joint_trajectory.points.empty()) {
      return false;
    }
    sensor_msgs::msg::JointState joints;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      joints = latest_joints_;
    }
    std::map<std::string, double> live;
    for (size_t i = 0; i < joints.name.size() && i < joints.position.size(); ++i) {
      live[joints.name[i]] = joints.position[i];
    }
    const auto & trajectory_names = trajectory.joint_trajectory.joint_names;
    const auto & first = trajectory.joint_trajectory.points.front().positions;
    const double tolerance = get_parameter("start_joint_tolerance").as_double();
    for (size_t i = 0; i < trajectory_names.size() && i < first.size(); ++i) {
      const auto found = live.find(trajectory_names[i]);
      if (found == live.end() || std::abs(found->second - first[i]) > tolerance) {
        return false;
      }
    }
    return true;
  }

  bool waitForTrajectoryEnd(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    if (trajectory.joint_trajectory.points.empty()) {
      return false;
    }
    const auto & names = trajectory.joint_trajectory.joint_names;
    const auto & target = trajectory.joint_trajectory.points.back().positions;
    if (names.empty() || names.size() != target.size()) {
      return false;
    }
    const double tolerance = get_parameter("execution_joint_tolerance").as_double();
    const auto reached = [this, &names, &target, tolerance]() {
      std::map<std::string, double> actual;
      for (size_t i = 0;
        i < latest_joints_.name.size() && i < latest_joints_.position.size(); ++i)
      {
        actual[latest_joints_.name[i]] = latest_joints_.position[i];
      }
      for (size_t i = 0; i < names.size() && i < target.size(); ++i) {
        const auto found = actual.find(names[i]);
        if (found == actual.end() || std::abs(found->second - target[i]) > tolerance) {
          return false;
        }
      }
      return true;
    };
    std::unique_lock<std::mutex> lock(data_mutex_);
    return joint_condition_.wait_for(
      lock,
      std::chrono::duration<double>(get_parameter("execution_settle_timeout").as_double()),
      reached);
  }

  bool observationConfigured() const
  {
    const auto positions = get_parameter("observation_joint_positions").as_double_array();
    const auto names = move_group_->getJointNames();
    return positions.size() == names.size() &&
           std::all_of(positions.begin(), positions.end(), [](double value) {
             return std::isfinite(value);
           });
  }

  bool atObservationPose() const
  {
    if (!observationConfigured()) {
      return false;
    }
    sensor_msgs::msg::JointState joints;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      joints = latest_joints_;
    }
    std::map<std::string, double> actual;
    for (size_t i = 0; i < joints.name.size() && i < joints.position.size(); ++i) {
      actual[joints.name[i]] = joints.position[i];
    }
    const auto names = move_group_->getJointNames();
    const auto target = get_parameter("observation_joint_positions").as_double_array();
    const double tolerance = get_parameter("observation_joint_tolerance").as_double();
    for (size_t i = 0; i < names.size(); ++i) {
      const auto found = actual.find(names[i]);
      if (found == actual.end() || std::abs(found->second - target[i]) > tolerance) {
        return false;
      }
    }
    return true;
  }

  bool tableConfigured() const
  {
    const auto size = get_parameter("table_size").as_double_array();
    return get_parameter("table_height").as_double() > -100.0 && size.size() == 3 &&
           size[0] > 0.0 && size[1] > 0.0 && size[2] > 0.0;
  }

  static size_t ensureCollisionEntry(
    moveit_msgs::msg::AllowedCollisionMatrix & matrix, const std::string & name)
  {
    const size_t current_size = matrix.entry_names.size();
    matrix.entry_values.resize(current_size);
    for (auto & entry : matrix.entry_values) {
      entry.enabled.resize(current_size, false);
    }
    const auto found = std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name);
    if (found != matrix.entry_names.end()) {
      return static_cast<size_t>(std::distance(matrix.entry_names.begin(), found));
    }
    const size_t old_size = matrix.entry_names.size();
    matrix.entry_names.push_back(name);
    for (auto & entry : matrix.entry_values) {
      entry.enabled.resize(old_size + 1, false);
    }
    moveit_msgs::msg::AllowedCollisionEntry entry;
    entry.enabled.resize(old_size + 1, false);
    matrix.entry_values.push_back(std::move(entry));
    return old_size;
  }

  bool allowTargetFingerCollisions()
  {
    if (!planning_scene_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "get_planning_scene service is unavailable");
      return false;
    }
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = planning_scene_client_->async_send_request(request);
    if (future.wait_for(2s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "timed out while reading the allowed collision matrix");
      return false;
    }

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.allowed_collision_matrix = future.get()->scene.allowed_collision_matrix;
    auto & matrix = scene.allowed_collision_matrix;
    const size_t target = ensureCollisionEntry(matrix, "smart_grasp_target");
    const size_t finger1 = ensureCollisionEntry(matrix, "gripper_link1");
    const size_t finger2 = ensureCollisionEntry(matrix, "gripper_link2");
    matrix.entry_values[target].enabled[finger1] = true;
    matrix.entry_values[finger1].enabled[target] = true;
    matrix.entry_values[target].enabled[finger2] = true;
    matrix.entry_values[finger2].enabled[target] = true;
    return planning_scene_interface_.applyPlanningScene(scene);
  }

  bool applyStaticObstacles(std::vector<moveit_msgs::msg::CollisionObject> & objects)
  {
    const std::string raw = get_parameter("static_obstacles").as_string();
    if (raw.empty()) {
      return true;
    }
    const std::string base = get_parameter("base_frame").as_string();
    std::istringstream stream(raw);
    std::string line;
    size_t parsed = 0;
    while (std::getline(stream, line)) {
      // Trim and skip blank / comment lines.
      line.erase(0, line.find_first_not_of(" \t\r\n"));
      line.erase(line.find_last_not_of(" \t\r\n") + 1);
      if (line.empty() || line.front() == '#') {
        continue;
      }
      std::istringstream tokens(line);
      std::string field;
      std::string obstacle_id;
      if (!std::getline(tokens, obstacle_id, ',')) {
        RCLCPP_WARN(get_logger(),
          "skipping malformed static_obstacles line: '%s'", line.c_str());
        continue;
      }
      obstacle_id.erase(0, obstacle_id.find_first_not_of(" \t\r\n"));
      obstacle_id.erase(obstacle_id.find_last_not_of(" \t\r\n") + 1);
      std::vector<double> fields;
      while (std::getline(tokens, field, ',')) {
        try {
          fields.push_back(std::stod(field));
        } catch (const std::exception &) {
          RCLCPP_WARN(get_logger(),
            "skipping malformed static_obstacles line: '%s'", line.c_str());
          fields.clear();
          break;
        }
      }
      if (obstacle_id.empty() || fields.size() != 6) {
        RCLCPP_WARN(get_logger(),
          "static_obstacles line needs 7 fields (id,sx,sy,sz,px,py,pz), got %zu: '%s'",
          fields.size() + (obstacle_id.empty() ? 0U : 1U), line.c_str());
        continue;
      }
      if (!std::all_of(fields.begin(), fields.end(), [](double value) {
          return std::isfinite(value);
        }) || fields[0] <= 0.0 || fields[1] <= 0.0 || fields[2] <= 0.0)
      {
        RCLCPP_WARN(get_logger(),
          "static_obstacles dimensions must be positive and all values finite: '%s'",
          line.c_str());
        continue;
      }
      moveit_msgs::msg::CollisionObject box;
      box.header.frame_id = base;
      box.id = std::to_string(parsed) + "_" + obstacle_id;
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions = {fields[0], fields[1], fields[2]};
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = fields[3];
      pose.position.y = fields[4];
      pose.position.z = fields[5];
      box.primitives.push_back(primitive);
      box.primitive_poses.push_back(pose);
      box.operation = box.ADD;
      objects.push_back(box);
      ++parsed;
    }
    if (parsed > 0) {
      RCLCPP_INFO(get_logger(),
        "loaded %zu static obstacle(s) into the planning scene", parsed);
    }
    return true;
  }

  bool appendEnvironmentObjects(
    std::vector<moveit_msgs::msg::CollisionObject> & objects)
  {
    if (tableConfigured()) {
      const auto size = get_parameter("table_size").as_double_array();
      const auto center = get_parameter("table_center_xy").as_double_array();
      moveit_msgs::msg::CollisionObject table;
      table.header.frame_id = get_parameter("base_frame").as_string();
      table.id = "smart_grasp_table";
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions = {size[0], size[1], size[2]};
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = center.at(0);
      pose.position.y = center.at(1);
      pose.position.z = get_parameter("table_height").as_double() - 0.5 * size[2];
      table.primitives.push_back(primitive);
      table.primitive_poses.push_back(pose);
      table.operation = table.ADD;
      objects.push_back(table);
    }
    return applyStaticObstacles(objects);
  }

  bool applyEnvironmentScene()
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    if (!appendEnvironmentObjects(objects)) {
      return false;
    }
    if (!planning_scene_interface_.getObjects({"smart_grasp_target"}).empty()) {
      planning_scene_interface_.removeCollisionObjects({"smart_grasp_target"});
    }
    return planning_scene_interface_.applyCollisionObjects(objects);
  }

  bool applyScene(const smart_grasp_interfaces::msg::DetectedObject & object)
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    if (!appendEnvironmentObjects(objects)) {
      return false;
    }
    moveit_msgs::msg::CollisionObject target;
    target.header = object.header;
    target.id = "smart_grasp_target";
    shape_msgs::msg::SolidPrimitive target_shape;
    target_shape.type = target_shape.BOX;
    target_shape.dimensions = {object.size.x, object.size.y, object.size.z};
    target.primitives.push_back(target_shape);
    auto target_pose = object.pose.pose;
    if (tableConfigured() && object.size.z > 0.0) {
      const double clearance = std::max(0.0, get_parameter("target_table_clearance").as_double());
      const double min_center_z =
        get_parameter("table_height").as_double() + 0.5 * object.size.z + clearance;
      if (target_pose.position.z < min_center_z) {
        RCLCPP_WARN(get_logger(),
          "raising target collision box from z=%.4f to %.4f to keep it above the table",
          target_pose.position.z, min_center_z);
        target_pose.position.z = min_center_z;
      }
    }
    target.primitive_poses.push_back(target_pose);
    target.operation = target.ADD;
    objects.push_back(target);
    if (!planning_scene_interface_.applyCollisionObjects(objects)) {
      RCLCPP_ERROR(get_logger(), "failed to apply smart grasp collision objects");
      return false;
    }
    return allowTargetFingerCollisions();
  }

  bool commandGripper(double width)
  {
    const double timeout = get_parameter("gripper_timeout").as_double();
    if (!gripper_client_->wait_for_action_server(std::chrono::duration<double>(timeout))) {
      return false;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {"gripper"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {width};
    const double motion_time = std::max(0.1, get_parameter("gripper_motion_time").as_double());
    point.time_from_start = rclcpp::Duration::from_seconds(motion_time);
    goal.trajectory.points.push_back(point);
    auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      return false;
    }
    auto handle = goal_future.get();
    if (!handle) {
      return false;
    }
    auto result_future = gripper_client_->async_get_result(handle);
    if (result_future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      gripper_client_->async_cancel_goal(handle);
      return false;
    }
    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      return false;
    }
    if (get_parameter("simulation_mode").as_bool()) {
      return true;
    }
    uint64_t feedback_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      feedback_sequence = gripper_feedback_sequence_;
    }
    std::unique_lock<std::mutex> lock(data_mutex_);
    return gripper_condition_.wait_for(
      lock, std::chrono::duration<double>(get_parameter("gripper_timeout").as_double()),
      [this, feedback_sequence]() {
        return gripper_feedback_sequence_ > feedback_sequence;
      });
  }

  bool executeArmTrajectory(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    if (trajectory.joint_trajectory.points.empty()) {
      return false;
    }
    const double timeout = get_parameter("execution_settle_timeout").as_double();
    if (!arm_client_->wait_for_action_server(std::chrono::duration<double>(timeout))) {
      RCLCPP_ERROR(get_logger(), "arm trajectory action server is unavailable");
      return false;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory = trajectory.joint_trajectory;
    auto goal_future = arm_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "timed out while sending arm trajectory goal");
      return false;
    }
    auto handle = goal_future.get();
    if (!handle) {
      RCLCPP_ERROR(get_logger(), "arm trajectory goal was rejected");
      return false;
    }
    auto result_future = arm_client_->async_get_result(handle);
    if (result_future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      arm_client_->async_cancel_goal(handle);
      RCLCPP_ERROR(get_logger(), "timed out while waiting for arm trajectory result");
      return false;
    }
    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "arm trajectory action did not succeed");
      return false;
    }
    return true;
  }

  bool waitForJointTarget(
    const std::vector<std::string> & names, const std::vector<double> & target,
    double tolerance, double timeout_seconds)
  {
    if (names.size() != target.size() || names.empty()) {
      return false;
    }
    const auto reached = [this, &names, &target, tolerance]() {
      std::map<std::string, double> actual;
      for (size_t i = 0; i < latest_joints_.name.size() &&
        i < latest_joints_.position.size(); ++i)
      {
        actual[latest_joints_.name[i]] = latest_joints_.position[i];
      }
      for (size_t i = 0; i < names.size(); ++i) {
        const auto found = actual.find(names[i]);
        if (found == actual.end() || std::abs(found->second - target[i]) > tolerance) {
          return false;
        }
      }
      return true;
    };
    std::unique_lock<std::mutex> lock(data_mutex_);
    return joint_condition_.wait_for(
      lock, std::chrono::duration<double>(timeout_seconds), reached);
  }

  bool moveToObservationWithMoveIt()
  {
    if (!applyEnvironmentScene()) {
      RCLCPP_ERROR(get_logger(),
        "failed to update the environment before observation motion");
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan observation_plan;
    move_group_->setStartStateToCurrentState();
    const auto observation = get_parameter("observation_joint_positions").as_double_array();
    if (!move_group_->setJointValueTarget(std::vector<double>(
        observation.begin(), observation.end())) ||
      move_group_->plan(observation_plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "no collision-free plan to the observation pose");
      return false;
    }
    if (hasWristJump(observation_plan.trajectory_)) {
      RCLCPP_ERROR(get_logger(), "observation trajectory contains a wrist jump");
      return false;
    }
    if (!startStateMatches(observation_plan.trajectory_)) {
      RCLCPP_ERROR(get_logger(),
        "observation plan start differs from real joint feedback");
      return false;
    }
    if (!executeArmTrajectory(observation_plan.trajectory_) ||
      !waitForTrajectoryEnd(observation_plan.trajectory_))
    {
      RCLCPP_ERROR(get_logger(), "planned arm trajectory did not reach the observation pose");
      return false;
    }
    return true;
  }

  bool jointTargetConfigured(const std::string & parameter_name) const
  {
    const auto positions = get_parameter(parameter_name).as_double_array();
    const auto names = move_group_->getJointNames();
    return positions.size() == names.size() &&
           std::all_of(positions.begin(), positions.end(), [](double value) {
             return std::isfinite(value);
           });
  }

  bool moveToConfiguredJointTargetWithMoveIt(
    const std::string & positions_parameter,
    const std::string & tolerance_parameter,
    const std::string & target_label)
  {
    const auto names = move_group_->getJointNames();
    const auto configured_target = get_parameter(positions_parameter).as_double_array();
    auto target = std::vector<double>(configured_target.begin(), configured_target.end());
    if (target.size() != names.size()) {
      RCLCPP_ERROR(get_logger(),
        "%s must match the planning group joint count", positions_parameter.c_str());
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setJointValueTarget(target) ||
      move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "no collision-free plan to the %s pose", target_label.c_str());
      return false;
    }
    if (hasWristJump(plan.trajectory_)) {
      RCLCPP_ERROR(get_logger(), "%s trajectory contains a wrist jump", target_label.c_str());
      return false;
    }
    if (!startStateMatches(plan.trajectory_)) {
      RCLCPP_ERROR(get_logger(), "%s plan start differs from real joint feedback",
        target_label.c_str());
      return false;
    }
    if (!executeArmTrajectory(plan.trajectory_) ||
      !waitForJointTarget(
        names, target, get_parameter(tolerance_parameter).as_double(),
        get_parameter("execution_settle_timeout").as_double()))
    {
      RCLCPP_ERROR(get_logger(), "real joints did not reach the %s pose", target_label.c_str());
      return false;
    }
    return true;
  }

  bool moveToPostPickWithMoveIt()
  {
    return moveToConfiguredJointTargetWithMoveIt(
      "post_pick_joint_positions", "post_pick_joint_tolerance", "post-pick");
  }

  bool moveToPostPickFinalWithMoveIt()
  {
    return moveToConfiguredJointTargetWithMoveIt(
      "post_pick_final_joint_positions", "post_pick_final_joint_tolerance", "post-pick-final");
  }

  bool moveToPostPickSequence(
    const std::shared_ptr<GoalHandle> & handle, uint32_t count,
    const std::string & reason)
  {
    feedback(handle, "MOVE_TO_POST_PICK", reason + "; moving to configured post-pick pose", count);
    if (!moveToPostPickWithMoveIt()) {
      return false;
    }
    if (jointTargetConfigured("post_pick_final_joint_positions")) {
      feedback(handle, "MOVE_TO_POST_PICK_FINAL",
        reason + "; moving from post-pick pose to configured final pose", count);
      if (!moveToPostPickFinalWithMoveIt()) {
        return false;
      }
    }
    move_group_->setStartStateToCurrentState();
    return true;
  }

  void abortAfterGraspMotion(
    const std::shared_ptr<GoalHandle> & handle, uint16_t code,
    const std::string & message,
    const smart_grasp_interfaces::msg::DetectedObject & object,
    uint32_t count)
  {
    clearTargetFromPlanningScene();
    std::string final_message = message;
    if (moveToPostPickSequence(handle, count, "grasp failed")) {
      final_message += "; moved to configured post-pick sequence";
    } else {
      final_message += "; failed to move to configured post-pick sequence";
    }
    abort(handle, code, final_message, &object);
  }

  void clearTargetFromPlanningScene()
  {
    if (move_group_) {
      move_group_->detachObject("smart_grasp_target");
    }
    planning_scene_interface_.removeCollisionObjects({"smart_grasp_target"});
  }

  bool gripperHealthy(double * width = nullptr) const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (get_parameter("simulation_mode").as_bool()) {
      if (width) {
        *width = get_parameter("simulation_contact_width").as_double();
      }
      return true;
    }
    if (!have_gripper_) {
      return false;
    }
    if (width) {
      *width = latest_gripper_.width;
    }
    return !latest_gripper_.voltage_too_low && !latest_gripper_.motor_overheating &&
           !latest_gripper_.driver_overcurrent && !latest_gripper_.driver_overheating &&
           !latest_gripper_.sensor_status && !latest_gripper_.driver_error_status;
  }

  bool planCartesian(
    const geometry_msgs::msg::Pose & target, moveit_msgs::msg::RobotTrajectory & trajectory,
    double * fraction)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints{target};
    *fraction = move_group_->computeCartesianPath(
      waypoints, get_parameter("cartesian_step").as_double(), 0.0, trajectory, true);
    if (!smart_grasp_moveit::cartesianCandidateAccepted(
        *fraction, get_parameter("cartesian_min_fraction").as_double(),
        hasWristJump(trajectory)))
    {
      return false;
    }
    return retimeCartesianTrajectory(trajectory);
  }

  double scalingParameter(const std::string & parameter_name, const std::string & fallback_name) const
  {
    double value = get_parameter(parameter_name).as_double();
    if (!std::isfinite(value) || value <= 0.0) {
      value = get_parameter(fallback_name).as_double();
    }
    if (!std::isfinite(value) || value <= 0.0) {
      return 0.1;
    }
    return std::clamp(value, 0.01, 1.0);
  }

  bool retimeCartesianTrajectory(moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    if (!get_parameter("time_parameterize_cartesian").as_bool() ||
      trajectory.joint_trajectory.points.size() < 2)
    {
      return true;
    }
    auto reference_state = move_group_->getCurrentState(2.0);
    if (!reference_state) {
      RCLCPP_ERROR(get_logger(), "failed to read current robot state for Cartesian retiming");
      return false;
    }
    robot_trajectory::RobotTrajectory robot_trajectory(
      move_group_->getRobotModel(), get_parameter("planning_group").as_string());
    robot_trajectory.setRobotTrajectoryMsg(*reference_state, trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    const double velocity_scaling = scalingParameter(
      "cartesian_velocity_scaling", "velocity_scaling");
    const double acceleration_scaling = scalingParameter(
      "cartesian_acceleration_scaling", "acceleration_scaling");
    if (!time_parameterization.computeTimeStamps(
        robot_trajectory, velocity_scaling, acceleration_scaling))
    {
      RCLCPP_ERROR(get_logger(),
        "failed to time-parameterize Cartesian trajectory with velocity=%.3f acceleration=%.3f",
        velocity_scaling, acceleration_scaling);
      return false;
    }
    robot_trajectory.getRobotTrajectoryMsg(trajectory);
    return true;
  }

  void setStartFromTrajectory(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    auto state = move_group_->getCurrentState(2.0);
    if (!state || trajectory.joint_trajectory.points.empty()) {
      return;
    }
    state->setVariablePositions(
      trajectory.joint_trajectory.joint_names,
      trajectory.joint_trajectory.points.back().positions);
    state->update();
    move_group_->setStartState(*state);
  }

  void execute(const std::shared_ptr<GoalHandle> handle)
  {
    const auto goal = handle->get_goal();
    const bool do_execute = goal->execute;
    PickTiming timing(get_logger());
    if (!observationConfigured()) {
      abort(handle, PickObject::Result::INTERNAL_ERROR,
        "observation_joint_positions must match the planning group joint count");
      return;
    }
    if (do_execute && !get_parameter("execution_allowed").as_bool()) {
      abort(handle, PickObject::Result::EXECUTION_DISABLED,
        "real execution is not armed (execution_allowed=false)");
      return;
    }
    if (do_execute && !tableConfigured()) {
      abort(handle, PickObject::Result::TABLE_UNCONFIGURED,
        "table size and height must be measured before observation motion");
      return;
    }
    const std::string reobserve_mode = get_parameter("pregrasp_reobserve_mode").as_string();
    if (reobserve_mode != "update" && reobserve_mode != "validate_only") {
      abort(handle, PickObject::Result::INTERNAL_ERROR,
        "pregrasp_reobserve_mode must be update or validate_only");
      return;
    }
    const double reobserve_max_xy = get_parameter("reobserve_max_xy_shift").as_double();
    const double reobserve_max_z = get_parameter("reobserve_max_z_shift").as_double();
    const double reobserve_max_yaw =
      get_parameter("reobserve_max_axis_yaw_deg").as_double() *
      smart_grasp_moveit::kPi / 180.0;
    if (reobserve_mode == "validate_only" &&
      (!std::isfinite(reobserve_max_xy) || reobserve_max_xy < 0.0 ||
      !std::isfinite(reobserve_max_z) || reobserve_max_z < 0.0 ||
      !std::isfinite(reobserve_max_yaw) || reobserve_max_yaw < 0.0))
    {
      abort(handle, PickObject::Result::INTERNAL_ERROR,
        "reobserve validation limits must be finite and non-negative");
      return;
    }
    if (!atObservationPose()) {
      if (!do_execute) {
        abort(handle, PickObject::Result::START_STATE_MISMATCH,
          "plan-only requires the real arm at the configured observation pose");
        return;
      }
      if (!get_parameter("simulation_mode").as_bool()) {
        feedback(handle, "MOVE_TO_OBSERVE", "moving to observation pose with planned arm trajectory", 0);
        if (!moveToObservationWithMoveIt()) {
          abort(handle, PickObject::Result::START_STATE_MISMATCH,
            "planned arm trajectory did not reach the configured observation pose");
          return;
        }
        std::this_thread::sleep_for(500ms);
        move_group_->setStartStateToCurrentState();
      } else {
        feedback(handle, "MOVE_TO_OBSERVE", "planning configured observation pose", 0);
        if (!applyEnvironmentScene()) {
          abort(handle, PickObject::Result::PLANNING_FAILED,
            "failed to update the environment before observation motion");
          return;
        }
        moveit::planning_interface::MoveGroupInterface::Plan observation_plan;
        move_group_->setStartStateToCurrentState();
        const auto observation = get_parameter("observation_joint_positions").as_double_array();
        if (!move_group_->setJointValueTarget(std::vector<double>(
            observation.begin(), observation.end())) ||
          move_group_->plan(observation_plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
          abort(handle, PickObject::Result::PLANNING_FAILED,
            "no collision-free plan to the observation pose");
          return;
        }
        if (hasWristJump(observation_plan.trajectory_)) {
          abort(handle, PickObject::Result::WRIST_JUMP,
            "observation trajectory contains a wrist jump");
          return;
        }
        if (!startStateMatches(observation_plan.trajectory_)) {
          abort(handle, PickObject::Result::START_STATE_MISMATCH,
            "observation plan start differs from real joint feedback");
          return;
        }
        feedback(handle, "EXEC_OBSERVE", "executing configured observation pose", 0);
        if (!executeArmTrajectory(observation_plan.trajectory_) ||
          !waitForTrajectoryEnd(observation_plan.trajectory_))
        {
          abort(handle, PickObject::Result::START_STATE_MISMATCH,
            "real joints did not reach the observation pose");
          return;
        }
        std::this_thread::sleep_for(500ms);
        move_group_->setStartStateToCurrentState();
      }
    } else {
      feedback(handle, "MOVE_TO_OBSERVE", "real arm is already at the observation pose", 0);
    }
    timing.mark("observe");
    if (canceled(handle)) {return;}
    feedback(handle, "DETECT", "waiting for a fresh stable target", 0);
    const std::string default_target_class = get_parameter("target_class").as_string();
    const std::string target_class =
      goal->target_class.empty() ? default_target_class : goal->target_class;
    auto detections = waitForCurrentDetections(
      target_class, std::nullopt, true);
    if (detections.empty()) {
      abort(handle, PickObject::Result::NO_TARGET, "no fresh target detection");
      return;
    }
    timing.mark("detect");
    feedback(handle, "VALIDATE_DEPTH_AND_POSE", "checking rejection and stability", detections.size());
    auto valid = std::vector<smart_grasp_interfaces::msg::DetectedObject>();
    for (const auto & detection : detections) {
      if (detection.rejection_reason.empty() && detection.stable) {
        valid.push_back(detection);
      }
    }
    if (valid.empty()) {
      const auto & rejected = detections.front();
      uint16_t code = PickObject::Result::UNSTABLE_TARGET;
      if (rejected.rejection_reason == "INVALID_DEPTH" ||
        rejected.rejection_reason == "TABLE_NOT_OBSERVED") {code = PickObject::Result::INVALID_DEPTH;}
      if (rejected.rejection_reason == "TF_UNAVAILABLE") {code = PickObject::Result::TF_UNAVAILABLE;}
      if (rejected.rejection_reason == "AMBIGUOUS_TARGET") {code = PickObject::Result::AMBIGUOUS_TARGET;}
      abort(handle, code, rejected.rejection_reason.empty() ? "target is not stable" : rejected.rejection_reason,
        &rejected);
      return;
    }
    if (valid.size() > 1 &&
      detectionScore(valid[0]) - detectionScore(valid[1]) < get_parameter("ambiguity_score_gap").as_double())
    {
      abort(handle, PickObject::Result::AMBIGUOUS_TARGET, "top target scores are ambiguous", &valid[0]);
      return;
    }
    const auto object = valid.front();
    if (do_execute && !get_parameter("calibration_validated").as_bool()) {
      abort(handle, PickObject::Result::CALIBRATION_UNVALIDATED,
        "hand-eye calibration is not validated", &object);
      return;
    }
    if (!applyScene(object)) {
      abort(handle, PickObject::Result::PLANNING_FAILED,
        "failed to update the MoveIt planning scene", &object);
      return;
    }
    timing.mark("scene");
    feedback(handle, "GENERATE_CANDIDATES", "generating two 180-degree top grasps", 2);
    const auto offset = get_parameter("tcp_to_grasp_xyz").as_double_array();
    std::vector<Candidate> candidates;
    bool wrist_jump_rejected = false;
    bool cartesian_rejected = false;
    std::vector<double> rejected_approach_fractions;
    const bool validate_candidate_approaches =
      get_parameter("validate_all_candidate_approaches").as_bool();
    for (bool reverse : {false, true}) {
      Candidate candidate;
      candidate.reverse = reverse;
      candidate.object = object;
      candidate.grasp = makeGraspPose(
        object, reverse, get_parameter("grasp_depth").as_double(), offset);
      candidate.pregrasp = candidate.grasp;
      candidate.pregrasp.position.z += get_parameter("pregrasp_distance").as_double();
      move_group_->setStartStateToCurrentState();
      move_group_->setPoseTarget(candidate.pregrasp);
      feedback(handle, "PLAN_PREGRASP", "planning RRTConnect candidate", 2);
      const auto planned = move_group_->plan(candidate.plan);
      move_group_->clearPoseTargets();
      if (planned == moveit::core::MoveItErrorCode::SUCCESS) {
        if (hasWristJump(candidate.plan.trajectory_)) {
          wrist_jump_rejected = true;
        } else {
          if (validate_candidate_approaches) {
            setStartFromTrajectory(candidate.plan.trajectory_);
            feedback(handle, "PLAN_APPROACH_CANDIDATE",
              "validating complete Cartesian approach candidate", 2);
            const bool approach_valid = planCartesian(
              candidate.grasp, candidate.validated_approach,
              &candidate.approach_fraction);
            move_group_->setStartStateToCurrentState();
            if (!approach_valid) {
              cartesian_rejected = true;
              rejected_approach_fractions.push_back(candidate.approach_fraction);
              continue;
            }
          }
          candidate.score = detectionScore(object) -
            0.05 * trajectoryLength(candidate.plan.trajectory_) -
            0.02 * trajectoryLength(candidate.validated_approach);
          candidates.push_back(std::move(candidate));
        }
      }
    }
    move_group_->setStartStateToCurrentState();
    if (candidates.empty()) {
      if (cartesian_rejected) {
        std::ostringstream message;
        message << "no wrist candidate has a complete Cartesian approach; fractions=";
        for (size_t index = 0; index < rejected_approach_fractions.size(); ++index) {
          if (index > 0) {
            message << ',';
          }
          message << std::fixed << std::setprecision(6) << rejected_approach_fractions[index];
        }
        abort(handle, PickObject::Result::CARTESIAN_INCOMPLETE, message.str(), &object);
        return;
      }
      abort(handle,
        wrist_jump_rejected ? PickObject::Result::WRIST_JUMP : PickObject::Result::PLANNING_FAILED,
        wrist_jump_rejected ? "all pregrasp plans contain a wrist jump" :
        "no collision-free pregrasp plan", &object);
      return;
    }
    timing.mark("plan_pregrasp");
    std::sort(candidates.begin(), candidates.end(), [](const auto & left, const auto & right) {
      return left.score > right.score;
    });
    Candidate selected = candidates.front();
    if (!startStateMatches(selected.plan.trajectory_)) {
      abort(handle, PickObject::Result::START_STATE_MISMATCH,
        "planned first point differs from /feedback/joint_states", &object);
      return;
    }
    if (canceled(handle)) {return;}
    feedback(handle, "EXEC_PREGRASP", do_execute ? "executing pregrasp" : "plan-only: pregrasp accepted",
      candidates.size());
    std::optional<rclcpp::Time> reobserve_after;
    if (do_execute) {
      if (!commandGripper(get_parameter("gripper_open").as_double())) {
        abort(handle, PickObject::Result::GRIPPER_FAULT,
          "failed to open gripper before pregrasp", &object);
        return;
      }
      timing.mark("open_gripper");
      if (!executeArmTrajectory(selected.plan.trajectory_)) {
        abort(handle, PickObject::Result::PLANNING_FAILED,
          "pregrasp trajectory execution failed", &object);
        return;
      }
      feedback(handle, "WAIT_PREGRASP_SETTLE",
        "waiting for real joint feedback at pregrasp", candidates.size());
      if (!waitForTrajectoryEnd(selected.plan.trajectory_)) {
        abort(handle, PickObject::Result::START_STATE_MISMATCH,
          "real joints did not reach the pregrasp trajectory endpoint", &object);
        return;
      }
      std::this_thread::sleep_for(500ms);
      move_group_->setStartStateToCurrentState();
      reobserve_after = get_clock()->now();
      timing.mark("exec_pregrasp");
    } else {
      setStartFromTrajectory(selected.plan.trajectory_);
    }
    feedback(handle, "REOBSERVE",
      reobserve_mode == "validate_only" ?
      "validating a fresh pregrasp observation without changing the grasp pose" :
      "using latest stable object pose at pregrasp",
      candidates.size());
    if (canceled(handle)) {return;}
    if (do_execute) {
      auto refreshed = waitForCurrentDetections(
        object.class_name, object.track_id, true,
        reobserve_mode == "validate_only" ? reobserve_after : std::nullopt);
      auto same_track = std::find_if(refreshed.begin(), refreshed.end(), [&object](const auto & item) {
        return item.track_id == object.track_id && item.stable && item.rejection_reason.empty();
      });
      if (same_track == refreshed.end()) {
        if (get_parameter("allow_reobserve_fallback").as_bool()) {
          // YOLO-based picking: trust the observation-pose locked target and
          // proceed to the grasp even if the pregrasp top-down view does not
          // reacquire the same track.
          feedback(handle, "REOBSERVE_FALLBACK",
            "reobservation not reacquired; using locked observation pose", candidates.size());
        } else {
          abort(handle, PickObject::Result::STALE_TARGET,
            "target was not stable after reaching pregrasp", &object);
          return;
        }
      }
      // When allow_reobserve_fallback fired above, same_track == refreshed.end()
      // and we must NOT dereference it. Keep the observation-pose-locked grasp
      // (selected already holds object + its grasp) and skip validate/update.
      if (same_track == refreshed.end()) {
        // Fallback already logged REOBSERVE_FALLBACK above; keep the
        // observation-pose-locked grasp (selected holds object + its grasp)
        // and skip validate/update to avoid dereferencing the end iterator.
      } else if (reobserve_mode == "validate_only") {
        const auto delta = smart_grasp_moveit::poseDelta(
          object.pose.pose, same_track->pose.pose);
        if (!smart_grasp_moveit::poseDeltaAccepted(
            delta, reobserve_max_xy, reobserve_max_z, reobserve_max_yaw))
        {
          std::ostringstream message;
          message << std::fixed << std::setprecision(4)
                  << "pregrasp observation differs from locked pose: xy=" << delta.xy
                  << " z=" << delta.z << " axis_yaw_deg="
                  << delta.axis_yaw * 180.0 / smart_grasp_moveit::kPi;
          abort(handle, PickObject::Result::STALE_TARGET, message.str(), &object);
          return;
        }
        feedback(handle, "REOBSERVE_VALIDATED",
          "fresh pregrasp observation accepted; keeping locked observation pose",
          candidates.size());
      } else {
        selected.object = *same_track;
        selected.grasp = makeGraspPose(
          selected.object, selected.reverse, get_parameter("grasp_depth").as_double(), offset);
        if (!applyScene(selected.object)) {
          abort(handle, PickObject::Result::PLANNING_FAILED,
            "failed to update the MoveIt planning scene", &object);
          return;
        }
      }
    }
    timing.mark("reobserve");
    moveit_msgs::msg::RobotTrajectory approach;
    double approach_fraction = 0.0;
    feedback(handle, "CARTESIAN_APPROACH", "planning 5 mm Cartesian approach", candidates.size());
    if (!planCartesian(selected.grasp, approach, &approach_fraction)) {
      abort(handle, PickObject::Result::CARTESIAN_INCOMPLETE,
        "Cartesian approach fraction=" + std::to_string(approach_fraction), &object);
      return;
    }
    timing.mark("plan_approach");
    if (do_execute && !executeArmTrajectory(approach)) {
      abort(handle, PickObject::Result::PLANNING_FAILED, "Cartesian approach execution failed", &object);
      return;
    }
    if (do_execute) {
      feedback(handle, "WAIT_APPROACH_SETTLE",
        "waiting for real joint feedback after approach", candidates.size());
      if (!waitForTrajectoryEnd(approach)) {
        abort(handle, PickObject::Result::START_STATE_MISMATCH,
          "real joints did not reach the Cartesian approach endpoint", &object);
        return;
      }
      timing.mark("exec_approach");
    }
    if (!do_execute) {
      setStartFromTrajectory(approach);
    }
    double contact_width = 0.0;
    bool contact_width_ignored = false;
    std::string contact_width_message;
    if (do_execute) {
      feedback(handle, "CLOSE_GRIPPER", "closing AgileX gripper", candidates.size());
      const double close_width = get_parameter("simulation_mode").as_bool() ?
        std::min(object.size.x, object.size.y) : get_parameter("gripper_close").as_double();
      if (!commandGripper(close_width)) {
        abortAfterGraspMotion(
          handle, PickObject::Result::GRIPPER_FAULT,
          "gripper command failed", object, candidates.size());
        return;
      }
      timing.mark("close_gripper");
      feedback(handle, "VERIFY_CONTACT", "checking gripper feedback width and faults", candidates.size());
      if (!gripperHealthy(&contact_width)) {
        abortAfterGraspMotion(
          handle, PickObject::Result::GRIPPER_FAULT,
          "gripper feedback reports a fault", object, candidates.size());
        return;
      }
      const double contact_width_min = get_parameter("contact_width_min").as_double();
      const double contact_width_max = get_parameter("contact_width_max").as_double();
      if (contact_width < contact_width_min || contact_width > contact_width_max)
      {
        contact_width_message =
          "gripper width " + std::to_string(contact_width) +
          " is outside the contact interval [" + std::to_string(contact_width_min) +
          ", " + std::to_string(contact_width_max) + "]";
        if (!get_parameter("release_on_contact_not_detected").as_bool()) {
          abortAfterGraspMotion(
            handle, PickObject::Result::CONTACT_NOT_DETECTED,
            contact_width_message, object, candidates.size());
          return;
        }
        contact_width_ignored = true;
        RCLCPP_WARN(get_logger(),
          "%s; release_on_contact_not_detected=true, continuing to lift and release",
          contact_width_message.c_str());
        feedback(handle, "VERIFY_CONTACT_SKIPPED",
          contact_width_message + "; continuing to lift and release",
          candidates.size());
      }
      feedback(handle, "ATTACH_OBJECT", "attaching target to gripper", candidates.size());
      move_group_->attachObject(
        "smart_grasp_target", "gripper_base", {"gripper_link1", "gripper_link2"});
      move_group_->setStartStateToCurrentState();
      timing.mark("verify_attach");
    }
    geometry_msgs::msg::Pose lift = selected.grasp;
    lift.position.z += get_parameter("lift_height").as_double();
    moveit_msgs::msg::RobotTrajectory lift_trajectory;
    double lift_fraction = 0.0;
    feedback(handle, "CARTESIAN_LIFT", "planning 50 mm vertical lift", candidates.size());
    if (!planCartesian(lift, lift_trajectory, &lift_fraction)) {
      if (do_execute) {
        abortAfterGraspMotion(
          handle, PickObject::Result::CARTESIAN_INCOMPLETE,
          "Cartesian lift fraction=" + std::to_string(lift_fraction), object, candidates.size());
      } else {
        abort(handle, PickObject::Result::CARTESIAN_INCOMPLETE,
          "Cartesian lift fraction=" + std::to_string(lift_fraction), &object);
      }
      return;
    }
    timing.mark("plan_lift");
    if (do_execute && !executeArmTrajectory(lift_trajectory)) {
      abortAfterGraspMotion(
        handle, PickObject::Result::PLANNING_FAILED,
        "lift execution failed", object, candidates.size());
      return;
    }
    if (do_execute) {
      feedback(handle, "WAIT_LIFT_SETTLE",
        "waiting for real joint feedback after lift", candidates.size());
      if (!waitForTrajectoryEnd(lift_trajectory)) {
        abortAfterGraspMotion(
          handle, PickObject::Result::START_STATE_MISMATCH,
          "real joints did not reach the lift trajectory endpoint", object, candidates.size());
        return;
      }
      timing.mark("exec_lift");
      clearTargetFromPlanningScene();
      if (!moveToPostPickSequence(handle, candidates.size(), "object lifted")) {
        abort(handle, PickObject::Result::PLANNING_FAILED,
          "planned arm trajectory did not reach the configured post-pick sequence", &object);
        return;
      }
      timing.mark("move_post_pick");
      feedback(handle, "OPEN_GRIPPER_AT_PLACE",
        "opening gripper after reaching the configured post-pick sequence", candidates.size());
      if (!commandGripper(get_parameter("gripper_open").as_double())) {
        abort(handle, PickObject::Result::GRIPPER_FAULT,
          "failed to open gripper after reaching the configured post-pick sequence", &object);
        return;
      }
      timing.mark("release_gripper");
    }
    const auto timing_summary = timing.summary();
    RCLCPP_INFO(get_logger(), "pick timing summary: %s", timing_summary.c_str());
    feedback(handle, "DONE",
      do_execute ?
      (contact_width_ignored ?
      "object released at configured post-pick sequence; contact width check skipped" :
      "object released at configured post-pick sequence") :
      "all paths planned without execution",
      candidates.size());
    auto result = std::make_shared<PickObject::Result>();
    result->success = true;
    result->error_code = PickObject::Result::OK;
    if (do_execute) {
      result->message = contact_width_ignored ?
        "pick completed and released at configured post-pick sequence; " +
        contact_width_message + " was ignored; timing: " + timing_summary :
        "pick completed and released at configured post-pick sequence; timing: " + timing_summary;
    } else {
      result->message = "plan-only completed; timing: " + timing_summary;
    }
    result->object_pose.header = object.header;
    result->object_pose.pose = object.pose.pose;
    result->grasp_pose.header = object.header;
    result->grasp_pose.pose = selected.grasp;
    result->size = object.size;
    result->contact_width = contact_width;
    handle->succeed(result);
  }

  std::atomic_bool busy_;
  mutable std::mutex data_mutex_;
  std::condition_variable detection_condition_;
  std::condition_variable joint_condition_;
  std::condition_variable gripper_condition_;
  std::mutex execution_thread_mutex_;
  std::thread execution_thread_;
  std::map<uint32_t, smart_grasp_interfaces::msg::DetectedObject> detections_;
  sensor_msgs::msg::JointState latest_joints_;
  agx_arm_msgs::msg::GripperStatus latest_gripper_;
  bool have_gripper_{false};
  uint64_t gripper_feedback_sequence_{0};
  rclcpp::Subscription<smart_grasp_interfaces::msg::DetectedObject>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<agx_arm_msgs::msg::GripperStatus>::SharedPtr gripper_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr planning_scene_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp_action::Server<PickObject>::SharedPtr action_server_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickServer>();
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
