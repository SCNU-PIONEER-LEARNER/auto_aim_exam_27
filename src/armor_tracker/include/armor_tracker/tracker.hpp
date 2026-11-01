// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <cmath>
#include <memory>
#include <string>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "rm_tools/extended_kalman_filter.hpp"
#include "rm_tools/math.hpp"

namespace rm_auto_aim
{

enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };
constexpr double OUTPOST_D_H = 102.0/1000.0;

class Tracker
{
public:
  Tracker(double max_match_angle_error);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armors::SharedPtr & armors_msg);

  Eigen::VectorXd predict(const Armor & a, double dt);
  void update(const Armors::SharedPtr & armors_msg);

  // void adaptAngularVelocity(const double & duration);

  rm_tools::ExtendedKalmanFilter ekf;
  // Target tracked_target;

  bool init_in_priority;
  int tracking_thres;
  int lost_thres;
  int change_thres;

  double dt;
  double dz;
  double dr;

  double outpost_h_dowm;
  double outpost_h_middle;
  double outpost_h_up;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
    CHANGE_TARGET,
  } tracker_state;

  double s2qxyz_max_, s2qxyz_min_, s2qyaw_max_, s2qyaw_min_;
  double r_py, r_d, r_yaw;

  std::string tracked_id;
  std::string last_tracked_id;
  Armor first_tracked_armor;
  ArmorsNum tracked_armors_num;
  int armorNumAsInt;
  int visiable_armor_num;
  
  double info_position_diff;
  double info_yaw_diff;
  double info_angle_diff;
  Eigen::VectorXd measurement;
  Eigen::VectorXd target_state;
  double debug_yaw = 0;

  int yaw_threshold;

private:
  // void initTarget(const Armor & a);
  void initEKF(const Armor & a);

  void initChange(const Armor & armor_msg);

  void updateArmorsNum(const Armor & a);

  void handleArmorJump(const Armor & a);

  void outpostCompare(const Armor & a);

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  Eigen::Vector3d Pos2Polar(const Armor & armor) const;

  void update_ypda(const Armor & armor, int id);

  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

  bool diverged() const;
  bool converged(const Armor & armor);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x, int id = 0) const;
  std::vector<Eigen::Vector4d> getArmorsPositionFromState(const Eigen::VectorXd & x) const;

  double max_match_distance_;

  double max_match_yaw_diff_;
  double max_angle_error_;

  int detect_count_;
  int lost_count_;
  int change_count_;
  int diff_count_;
  int update_count_;

  //int id;
  int last_id;
  int unmatched_count;

  bool jumped;

  double last_yaw_;
  bool is_converged_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
