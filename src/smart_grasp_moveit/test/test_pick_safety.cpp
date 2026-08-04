#include <cmath>

#include <gtest/gtest.h>

#include "smart_grasp_moveit/pick_safety.hpp"

namespace
{

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

TEST(PickSafety, AcceptsPoseInsideEveryLimit)
{
  geometry_msgs::msg::Pose reference;
  reference.orientation = quaternionFromYaw(0.0);
  geometry_msgs::msg::Pose observed = reference;
  observed.position.x = 0.003;
  observed.position.y = 0.004;
  observed.position.z = 0.005;
  observed.orientation = quaternionFromYaw(4.0 * smart_grasp_moveit::kPi / 180.0);

  const auto delta = smart_grasp_moveit::poseDelta(reference, observed);
  EXPECT_NEAR(delta.xy, 0.005, 1e-9);
  EXPECT_TRUE(
    smart_grasp_moveit::poseDeltaAccepted(
      delta, 0.005, 0.005, 5.0 * smart_grasp_moveit::kPi / 180.0));
}

TEST(PickSafety, RejectsEachCrossViewLimitIndependently)
{
  const double five_degrees = 5.0 * smart_grasp_moveit::kPi / 180.0;
  EXPECT_FALSE(
    smart_grasp_moveit::poseDeltaAccepted(
      {0.0051, 0.0, 0.0}, 0.005, 0.005, five_degrees));
  EXPECT_FALSE(
    smart_grasp_moveit::poseDeltaAccepted(
      {0.0, 0.0051, 0.0}, 0.005, 0.005, five_degrees));
  EXPECT_FALSE(
    smart_grasp_moveit::poseDeltaAccepted(
      {0.0, 0.0, 5.1 * smart_grasp_moveit::kPi / 180.0},
      0.005, 0.005, five_degrees));
}

TEST(PickSafety, TreatsYawAsAnAxisWithHalfTurnSymmetry)
{
  geometry_msgs::msg::Pose reference;
  reference.orientation = quaternionFromYaw(
    179.0 * smart_grasp_moveit::kPi / 180.0);
  geometry_msgs::msg::Pose observed;
  observed.orientation = quaternionFromYaw(
    1.0 * smart_grasp_moveit::kPi / 180.0);

  const auto delta = smart_grasp_moveit::poseDelta(reference, observed);
  EXPECT_NEAR(
    delta.axis_yaw, 2.0 * smart_grasp_moveit::kPi / 180.0, 1e-9);
}

TEST(PickSafety, RequiresTheConfiguredCartesianFractionAndNoWristJump)
{
  EXPECT_TRUE(smart_grasp_moveit::cartesianCandidateAccepted(1.0, 1.0, false));
  EXPECT_FALSE(smart_grasp_moveit::cartesianCandidateAccepted(0.999, 1.0, false));
  EXPECT_FALSE(smart_grasp_moveit::cartesianCandidateAccepted(1.0, 1.0, true));
}

}  // namespace
