#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <agx_arm_msgs/msg/gripper_status.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <smart_grasp_interfaces/action/pick_object.hpp>
#include <smart_grasp_interfaces/msg/detected_object.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

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
    declare_parameter("execution_allowed", false);
    declare_parameter("calibration_validated", false);
    declare_parameter("target_max_age", 0.5);
    declare_parameter("ambiguity_score_gap", 0.10);
    declare_parameter("pregrasp_distance", 0.100);
    declare_parameter("grasp_depth", 0.020);
    declare_parameter("lift_height", 0.050);
    declare_parameter("tcp_to_grasp_xyz", std::vector<double>{0.0, 0.0, 0.1425});
    declare_parameter("cartesian_step", 0.005);
    declare_parameter("cartesian_min_fraction", 0.95);
    declare_parameter("start_joint_tolerance", 0.05);
    declare_parameter("wrist_jump_threshold", 0.5);
    declare_parameter("gripper_open", 0.055);
    declare_parameter("gripper_close", 0.0);
    declare_parameter("gripper_force", 0.5);
    declare_parameter("contact_width_min", 0.032);
    declare_parameter("contact_width_max", 0.048);
    declare_parameter("gripper_timeout", 3.0);
    declare_parameter("gripper_action", "/gripper_controller/follow_joint_trajectory");
    declare_parameter("joint_state_topic", "/feedback/joint_states");
    declare_parameter("table_height", -999.0);
    declare_parameter("table_size", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("table_center_xy", std::vector<double>{0.0, 0.0});

    detection_sub_ = create_subscription<smart_grasp_interfaces::msg::DetectedObject>(
      "/smart_grasp/detections", 20,
      std::bind(&PickServer::detectionCallback, this, std::placeholders::_1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), 20,
      std::bind(&PickServer::jointCallback, this, std::placeholders::_1));
    gripper_sub_ = create_subscription<agx_arm_msgs::msg::GripperStatus>(
      "/feedback/gripper_status", 20,
      std::bind(&PickServer::gripperCallback, this, std::placeholders::_1));
    planning_scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", 10);
    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, get_parameter("gripper_action").as_string());
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
    double score{0.0};
    bool reverse{false};
  };

  void detectionCallback(const smart_grasp_interfaces::msg::DetectedObject::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    detections_[msg->track_id] = *msg;
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_joints_ = *msg;
  }

  void gripperCallback(const agx_arm_msgs::msg::GripperStatus::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_gripper_ = *msg;
      have_gripper_ = true;
    }
    gripper_condition_.notify_all();
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PickObject::Goal>)
  {
    if (busy_.exchange(true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
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
    std::thread([this, goal_handle]() {
      execute(goal_handle);
      busy_ = false;
    }).detach();
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

  static double detectionScore(const smart_grasp_interfaces::msg::DetectedObject & object)
  {
    return object.confidence + 0.25 * object.depth_valid_ratio + (object.stable ? 0.20 : 0.0);
  }

  std::vector<smart_grasp_interfaces::msg::DetectedObject> currentDetections(
    const std::string & target_class)
  {
    std::vector<smart_grasp_interfaces::msg::DetectedObject> current;
    const auto now = get_clock()->now();
    const double max_age = get_parameter("target_max_age").as_double();
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto & entry : detections_) {
      const auto & object = entry.second;
      const double age = (now - rclcpp::Time(object.header.stamp)).seconds();
      if (object.class_name == target_class && age >= 0.0 && age <= max_age) {
        current.push_back(object);
      }
    }
    std::sort(current.begin(), current.end(), [](const auto & left, const auto & right) {
      return detectionScore(left) > detectionScore(right);
    });
    return current;
  }

  static geometry_msgs::msg::Pose makeGraspPose(
    const smart_grasp_interfaces::msg::DetectedObject & object, bool reverse,
    double grasp_depth, const std::vector<double> & tcp_offset)
  {
    tf2::Quaternion object_q;
    tf2::fromMsg(object.pose.pose.orientation, object_q);
    tf2::Matrix3x3 object_rotation(object_q);
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

  bool tableConfigured() const
  {
    const auto size = get_parameter("table_size").as_double_array();
    return get_parameter("table_height").as_double() > -100.0 && size.size() == 3 &&
           size[0] > 0.0 && size[1] > 0.0 && size[2] > 0.0;
  }

  void applyScene(const smart_grasp_interfaces::msg::DetectedObject & object)
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
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
    moveit_msgs::msg::CollisionObject target;
    target.header = object.header;
    target.id = "smart_grasp_target";
    shape_msgs::msg::SolidPrimitive target_shape;
    target_shape.type = target_shape.BOX;
    target_shape.dimensions = {object.size.x, object.size.y, object.size.z};
    target.primitives.push_back(target_shape);
    target.primitive_poses.push_back(object.pose.pose);
    target.operation = target.ADD;
    objects.push_back(target);
    planning_scene_interface_.applyCollisionObjects(objects);

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    auto & matrix = scene.allowed_collision_matrix;
    matrix.entry_names = {"smart_grasp_target", "gripper_link1", "gripper_link2"};
    for (size_t row = 0; row < 3; ++row) {
      moveit_msgs::msg::AllowedCollisionEntry entry;
      entry.enabled = {false, false, false};
      entry.enabled[row] = true;
      if (row == 0) {
        entry.enabled[1] = true;
        entry.enabled[2] = true;
      } else {
        entry.enabled[0] = true;
      }
      matrix.entry_values.push_back(entry);
    }
    planning_scene_pub_->publish(scene);
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
    point.effort = {get_parameter("gripper_force").as_double()};
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);
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
    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  bool gripperHealthy(double * width = nullptr) const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
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
    return *fraction >= get_parameter("cartesian_min_fraction").as_double() &&
           !hasWristJump(trajectory);
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
    feedback(handle, "MOVE_TO_OBSERVE", "using the current stopped observation pose", 0);
    if (canceled(handle)) {return;}
    feedback(handle, "DETECT", "selecting a fresh target", 0);
    auto detections = currentDetections(goal->target_class.empty() ? "blue_block" : goal->target_class);
    if (detections.empty()) {
      abort(handle, PickObject::Result::NO_TARGET, "no fresh target detection");
      return;
    }
    feedback(handle, "VALIDATE_DEPTH_AND_SIZE", "checking rejection and stability", detections.size());
    auto valid = std::vector<smart_grasp_interfaces::msg::DetectedObject>();
    for (const auto & detection : detections) {
      if (detection.rejection_reason.empty() && detection.stable) {
        valid.push_back(detection);
      }
    }
    if (valid.empty()) {
      const auto & rejected = detections.front();
      uint16_t code = PickObject::Result::UNSTABLE_TARGET;
      if (rejected.rejection_reason == "SIZE_MISMATCH") {code = PickObject::Result::SIZE_MISMATCH;}
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
    if (do_execute && !get_parameter("execution_allowed").as_bool()) {
      abort(handle, PickObject::Result::EXECUTION_DISABLED,
        "real execution is not armed (execution_allowed=false)", &object);
      return;
    }
    if (do_execute && !get_parameter("calibration_validated").as_bool()) {
      abort(handle, PickObject::Result::CALIBRATION_UNVALIDATED,
        "hand-eye calibration is not validated", &object);
      return;
    }
    if (do_execute && !tableConfigured()) {
      abort(handle, PickObject::Result::TABLE_UNCONFIGURED,
        "table size and height must be measured before execution", &object);
      return;
    }
    applyScene(object);
    feedback(handle, "GENERATE_CANDIDATES", "generating two 180-degree top grasps", 2);
    const auto offset = get_parameter("tcp_to_grasp_xyz").as_double_array();
    std::vector<Candidate> candidates;
    bool wrist_jump_rejected = false;
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
          candidate.score = detectionScore(object) - 0.05 * trajectoryLength(candidate.plan.trajectory_);
          candidates.push_back(std::move(candidate));
        }
      }
    }
    if (candidates.empty()) {
      abort(handle,
        wrist_jump_rejected ? PickObject::Result::WRIST_JUMP : PickObject::Result::PLANNING_FAILED,
        wrist_jump_rejected ? "all pregrasp plans contain a wrist jump" :
        "no collision-free pregrasp plan", &object);
      return;
    }
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
    if (do_execute) {
      if (!commandGripper(get_parameter("gripper_open").as_double()) ||
        move_group_->execute(selected.plan) != moveit::core::MoveItErrorCode::SUCCESS)
      {
        abort(handle, PickObject::Result::PLANNING_FAILED, "pregrasp execution failed", &object);
        return;
      }
      std::this_thread::sleep_for(500ms);
      move_group_->setStartStateToCurrentState();
    } else {
      setStartFromTrajectory(selected.plan.trajectory_);
    }
    feedback(handle, "REOBSERVE", "using latest stable object pose at pregrasp", candidates.size());
    if (canceled(handle)) {return;}
    if (do_execute) {
      auto refreshed = currentDetections(object.class_name);
      auto same_track = std::find_if(refreshed.begin(), refreshed.end(), [&object](const auto & item) {
        return item.track_id == object.track_id && item.stable && item.rejection_reason.empty();
      });
      if (same_track == refreshed.end()) {
        abort(handle, PickObject::Result::STALE_TARGET,
          "target was not stable after reaching pregrasp", &object);
        return;
      }
      selected.object = *same_track;
      selected.grasp = makeGraspPose(
        selected.object, selected.reverse, get_parameter("grasp_depth").as_double(), offset);
      applyScene(selected.object);
    }
    moveit_msgs::msg::RobotTrajectory approach;
    double approach_fraction = 0.0;
    feedback(handle, "CARTESIAN_APPROACH", "planning 5 mm Cartesian approach", candidates.size());
    if (!planCartesian(selected.grasp, approach, &approach_fraction)) {
      abort(handle, PickObject::Result::CARTESIAN_INCOMPLETE,
        "Cartesian approach fraction=" + std::to_string(approach_fraction), &object);
      return;
    }
    if (do_execute && move_group_->execute(approach) != moveit::core::MoveItErrorCode::SUCCESS) {
      abort(handle, PickObject::Result::PLANNING_FAILED, "Cartesian approach execution failed", &object);
      return;
    }
    if (!do_execute) {
      setStartFromTrajectory(approach);
    }
    double contact_width = 0.0;
    if (do_execute) {
      feedback(handle, "CLOSE_GRIPPER", "closing AgileX gripper", candidates.size());
      if (!commandGripper(get_parameter("gripper_close").as_double())) {
        abort(handle, PickObject::Result::GRIPPER_FAULT, "gripper command failed", &object);
        return;
      }
      feedback(handle, "VERIFY_CONTACT", "checking gripper feedback width and faults", candidates.size());
      if (!gripperHealthy(&contact_width)) {
        abort(handle, PickObject::Result::GRIPPER_FAULT, "gripper feedback reports a fault", &object);
        return;
      }
      if (contact_width < get_parameter("contact_width_min").as_double() ||
        contact_width > get_parameter("contact_width_max").as_double())
      {
        abort(handle, PickObject::Result::CONTACT_NOT_DETECTED,
          "gripper width is outside the contact interval", &object);
        return;
      }
      feedback(handle, "ATTACH_OBJECT", "attaching target to gripper", candidates.size());
      move_group_->attachObject(
        "smart_grasp_target", "gripper_base", {"gripper_link1", "gripper_link2"});
      move_group_->setStartStateToCurrentState();
    }
    geometry_msgs::msg::Pose lift = selected.grasp;
    lift.position.z += get_parameter("lift_height").as_double();
    moveit_msgs::msg::RobotTrajectory lift_trajectory;
    double lift_fraction = 0.0;
    feedback(handle, "CARTESIAN_LIFT", "planning 50 mm vertical lift", candidates.size());
    if (!planCartesian(lift, lift_trajectory, &lift_fraction)) {
      abort(handle, PickObject::Result::CARTESIAN_INCOMPLETE,
        "Cartesian lift fraction=" + std::to_string(lift_fraction), &object);
      return;
    }
    if (do_execute && move_group_->execute(lift_trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
      abort(handle, PickObject::Result::PLANNING_FAILED, "lift execution failed", &object);
      return;
    }
    feedback(handle, "DONE", do_execute ? "object lifted" : "all paths planned without execution",
      candidates.size());
    auto result = std::make_shared<PickObject::Result>();
    result->success = true;
    result->error_code = PickObject::Result::OK;
    result->message = do_execute ? "pick completed" : "plan-only completed";
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
  std::condition_variable gripper_condition_;
  std::map<uint32_t, smart_grasp_interfaces::msg::DetectedObject> detections_;
  sensor_msgs::msg::JointState latest_joints_;
  agx_arm_msgs::msg::GripperStatus latest_gripper_;
  bool have_gripper_{false};
  rclcpp::Subscription<smart_grasp_interfaces::msg::DetectedObject>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<agx_arm_msgs::msg::GripperStatus>::SharedPtr gripper_sub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
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
