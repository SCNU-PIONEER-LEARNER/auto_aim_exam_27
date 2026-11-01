#include <Eigen/Dense>

#include <cmath>

#include <gtest/gtest.h>

#include "rm_tracker/target/robot_target.hpp"

namespace
{
rm_auto_aim::tracker::NoiseParams_s makeNoise()
{
  rm_auto_aim::tracker::NoiseParams_s noise;
  noise.s2qxyz = 1.0;
  noise.s2qyaw_min = 1.0;
  noise.s2qyaw_max = 1.0;
  noise.r_py = 1.0;
  noise.r_d = 1.0;
  noise.r_yaw = 1.0;
  return noise;
}

rm_auto_aim::tracker::StateMachineParams_s makeStateParams()
{
  rm_auto_aim::tracker::StateMachineParams_s params;
  params.tracking_thres = 1;
  params.change_thres = 1;
  params.lost_time_thres = 0.3;
  return params;
}
}  // namespace

TEST(RobotTargetGeometry, ArmorPositionUsesProvidedState)
{
  rm_auto_aim::tracker::RobotTarget target(makeNoise(), makeStateParams());
  target.armors_num_ = rm_auto_aim::ArmorsNum_i::NORMAL_4;
  target.initTarget(Eigen::Vector4d(1.0, 0.0, 0.2, 0.0));

  auto state = target.X_;
  state[rm_auto_aim::tracker::kf::x] = 2.0;
  state[rm_auto_aim::tracker::kf::y] = 3.0;
  state[rm_auto_aim::tracker::kf::z] = 0.7;
  state[rm_auto_aim::tracker::kf::yaw] = 0.3;
  state[rm_auto_aim::tracker::kf::r] = 0.4;
  state[rm_auto_aim::tracker::kf::dr] = 0.1;
  state[rm_auto_aim::tracker::kf::dz] = 0.2;

  const auto armor = target.getArmorPositionFromState(state, 1);
  const double armor_yaw = state[rm_auto_aim::tracker::kf::yaw] + M_PI_2;
  const double radius = state[rm_auto_aim::tracker::kf::r] + state[rm_auto_aim::tracker::kf::dr];

  EXPECT_NEAR(armor.x(), state[rm_auto_aim::tracker::kf::x] - radius * std::cos(armor_yaw), 1e-12);
  EXPECT_NEAR(armor.y(), state[rm_auto_aim::tracker::kf::y] - radius * std::sin(armor_yaw), 1e-12);
  EXPECT_NEAR(
    armor.z(), state[rm_auto_aim::tracker::kf::z] + state[rm_auto_aim::tracker::kf::dz], 1e-12);
}

TEST(RobotTargetGeometry, ArmorStateUsesPoseOrientationYaw)
{
  rm_auto_aim::tracker::RobotTarget target(makeNoise(), makeStateParams());
  rm_auto_aim::tracker::Armor armor;
  constexpr double pose_yaw = 0.4;
  armor.pose.position.x = 1.0;
  armor.pose.position.y = 0.0;
  armor.pose.position.z = 0.2;
  armor.pose.orientation.z = std::sin(pose_yaw * 0.5);
  armor.pose.orientation.w = std::cos(pose_yaw * 0.5);
  armor.yaw = 90.0;

  const auto state = target.getStateFromArmor(armor);

  EXPECT_NEAR(state[3], pose_yaw, 1e-12);
}

TEST(RobotTargetStateMachine, EarlyTrackingAllowsInitialLargeYawVelocity)
{
  rm_auto_aim::tracker::RobotTarget target(makeNoise(), makeStateParams());
  target.armors_num_ = rm_auto_aim::ArmorsNum_i::NORMAL_4;
  target.initTarget(Eigen::Vector4d(1.0, 0.0, 0.2, 0.0));
  target.state_ = rm_auto_aim::tracker::State_e::TRACKING;
  target.X_[rm_auto_aim::tracker::kf::v_yaw] = 20.0;

  target.updateStateMachine(true, rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_EQ(target.state_, rm_auto_aim::tracker::State_e::TRACKING);
  EXPECT_EQ(target.cnt_.track_count, 1);
}

TEST(RobotTargetStateMachine, SettledTrackingDropsDivergedFilter)
{
  rm_auto_aim::tracker::RobotTarget target(makeNoise(), makeStateParams());
  target.armors_num_ = rm_auto_aim::ArmorsNum_i::NORMAL_4;
  target.initTarget(Eigen::Vector4d(1.0, 0.0, 0.2, 0.0));
  target.state_ = rm_auto_aim::tracker::State_e::TRACKING;
  target.cnt_.track_count = 4;
  target.X_[rm_auto_aim::tracker::kf::v_yaw] = 20.0;

  target.updateStateMachine(true, rclcpp::Time(1, 0, RCL_ROS_TIME));

  EXPECT_EQ(target.state_, rm_auto_aim::tracker::State_e::LOST);
  EXPECT_EQ(target.cnt_.track_count, 0);
}
