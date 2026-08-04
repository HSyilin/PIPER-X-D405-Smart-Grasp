#pragma once

#include <cmath>

#include <geometry_msgs/msg/pose.hpp>

namespace smart_grasp_moveit
{

inline constexpr double kPi = 3.14159265358979323846;

struct PoseDelta
{
  double xy{0.0};
  double z{0.0};
  double axis_yaw{0.0};
};

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 * (
    quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

inline double axisYawDifference(double left, double right)
{
  const double difference = right - left;
  return std::abs(
    0.5 * std::atan2(
      std::sin(2.0 * difference), std::cos(2.0 * difference)));
}

inline PoseDelta poseDelta(
  const geometry_msgs::msg::Pose & reference,
  const geometry_msgs::msg::Pose & observed)
{
  const double dx = observed.position.x - reference.position.x;
  const double dy = observed.position.y - reference.position.y;
  return PoseDelta{
    std::hypot(dx, dy),
    std::abs(observed.position.z - reference.position.z),
    axisYawDifference(
      yawFromQuaternion(reference.orientation),
      yawFromQuaternion(observed.orientation))};
}

inline bool poseDeltaAccepted(
  const PoseDelta & delta, double max_xy, double max_z, double max_axis_yaw)
{
  return std::isfinite(delta.xy) && std::isfinite(delta.z) &&
         std::isfinite(delta.axis_yaw) && delta.xy <= max_xy &&
         delta.z <= max_z && delta.axis_yaw <= max_axis_yaw;
}

inline bool cartesianCandidateAccepted(
  double fraction, double required_fraction, bool wrist_jump)
{
  return std::isfinite(fraction) && fraction >= required_fraction && !wrist_jump;
}

}  // namespace smart_grasp_moveit
