// Copyright 2022 Chen Jun

#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>

namespace rm_auto_aim
{
Tracker::Tracker(double max_angle_error)
: tracker_state(LOST),
  tracked_id(std::string("")),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(11)),
  max_angle_error_(max_angle_error)
{
}

void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    return;
  }
  last_id = 0;

  // choose in priority
  if (init_in_priority) {
    double last_priority = DBL_MAX;
    first_tracked_armor = armors_msg->armors[0];
    for (const auto & armor : armors_msg->armors) {
      if (armor.priority < last_priority) {
        last_priority = armor.priority;
        first_tracked_armor = armor;
      }
    }
  }

  // Simply choose the armor that is closest to image center
  else {
    double min_distance = DBL_MAX;
    first_tracked_armor = armors_msg->armors[0];
    for (const auto & armor : armors_msg->armors) {
      if (armor.distance_to_image_center < min_distance) {
        min_distance = armor.distance_to_image_center;
        first_tracked_armor = armor;
      }
    }
  }
  initEKF(first_tracked_armor);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  tracked_id = first_tracked_armor.number;
  tracker_state = DETECTING;

  change_count_ = 0;
  change_thres = 20;
  diff_count_ = 0;
  diff_count_ = 0;
  unmatched_count = 0;

  updateArmorsNum(first_tracked_armor);
}

void Tracker::initChange(const Armor & armor_msg)
{
  initEKF(armor_msg);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Target change and Init EKF!");

  tracked_id = armor_msg.number;
  tracker_state = DETECTING;

  change_count_ = 0;
  change_thres = 20;

  updateArmorsNum(armor_msg);
}

void Tracker::update(const Armors::SharedPtr & armors_msg)
{
  int id;
  double min_angle_error = DBL_MAX;
  // KF predict
  Eigen::VectorXd ekfprediction = predict(first_tracked_armor, dt);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF predict");
  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  target_state = ekf.x;

  if (!armors_msg->armors.empty()) {
    int same_id_armors_count = 0;
    std::vector<Armor> same_id_armors;
    std::vector<const Armor *>
      diff_id_armors;  //! 涉及此容器及其内容的改动时，请检查该函数内的所有有关该容器的代码段，确保生命周期正确，避免野指针风险
    // Find the closest armor with the same id
    auto target_armors_position = getArmorsPositionFromState(ekf.x);
    std::vector<std::pair<Eigen::Vector4d, int>> target_armors_position_i;
    for (int i = 0; i < static_cast<int>(tracked_armors_num); i++) {
      target_armors_position_i.push_back({target_armors_position[i], i});
    }

    std::sort(
      target_armors_position_i.begin(), target_armors_position_i.end(),
      [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
        Eigen::Vector3d ypd1 = rm_tools::xyz2ypd(a.first.head(3));
        Eigen::Vector3d ypd2 = rm_tools::xyz2ypd(b.first.head(3));
        return ypd1[2] < ypd2[2];
      });

    for (const auto & armor : armors_msg->armors) {
      if (armor.number == tracked_id) {
        same_id_armors_count++;
        same_id_armors.emplace_back(armor);
      } else {
        diff_id_armors.push_back(&armor);
      }
    }
    visiable_armor_num = same_id_armors_count;
    std::sort(same_id_armors.begin(), same_id_armors.end(), [](const auto & a, const auto & b) {
      return std::abs(a.yaw) < std::abs(b.yaw);
    });

    //滤除yaw很大的装甲板（噪声大）
    same_id_armors.erase(
      std::remove_if(
        same_id_armors.begin(), same_id_armors.end(),
        [this](const Armor & armor) { return std::abs(armor.yaw) > yaw_threshold; }),
      same_id_armors.end());
      
   for (auto & armor : same_id_armors) {
      auto update_armor = armor;
      auto yaw = orientationToYaw(update_armor.pose.orientation);  //观测板
      debug_yaw = yaw;
      // 取前3个distance最小的装甲板
      for (int i = 0; i < 3; i++) {
        const auto & xyza = target_armors_position_i[i].first;
        auto armor_ypd = Pos2Polar(update_armor);
        Eigen::Vector3d ypd = rm_tools::xyz2ypd(xyza.head(3));
        //观测-ekf
        auto angle_error =
          std::abs(rm_tools::limit_rad(yaw - xyza[3])) +  //观测的装甲板朝向 - ekf各装甲板朝向
          std::abs(
            rm_tools::limit_rad(armor_ypd[0] - ypd[0]));  //观测装甲板质点的yaw - ekf装甲板质点的yaw

        if (std::abs(angle_error) < std::abs(min_angle_error)) {
          id = target_armors_position_i[i].second;
          min_angle_error = angle_error;
          first_tracked_armor = update_armor;
        }
      }
      RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "current id :%d!", id);
      if (id != 0) {
        jumped = true;
      }
      if (id != last_id) {
        RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");
        last_id = id;
      }
      auto xyz = target_armors_position_i[id].first;
      auto measured_yaw = orientationToYaw(first_tracked_armor.pose.orientation);
      measurement = Eigen::Vector4d(xyz[0], xyz[1], xyz[2], measured_yaw);
      RCLCPP_DEBUG(
        rclcpp::get_logger("armor_tracker"), "current min_angle_error :%f!", min_angle_error);
      if (min_angle_error < max_angle_error_) {
        matched = true;
        update_ypda(update_armor, id);
        RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update");
      }
    }
    if (tracker_state == CHANGE_TARGET) {
      Armor diff_first_tracked_armor;
      double diff_min_position_diff = DBL_MAX;
      for (const auto armor_ptr :
           diff_id_armors) {  // 这个改目标的逻辑是找到和当前跟踪最近的不同id的装甲板
        // Count diff_armor
        diff_count_ += 1;
        // Calculate the difference between the predicted position and the current armor position
        const auto & p = armor_ptr->pose.position;  // ! 注意生命周期问题
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (target_armors_position_i[0].first.head(3) - position_vec).norm();
        if (position_diff < diff_min_position_diff) {
          // Find the closest armor
          diff_min_position_diff = position_diff;
          diff_first_tracked_armor = *armor_ptr;
        }
      }
      // todo: 现在这样把这个if拉进CHANGE_TARGET的作用域好像挺好
      if (diff_count_ != 0) {
        initChange(diff_first_tracked_armor);
        return;
      }
    }  // if CHANGE_TARGET
  }
  // Prevent radius from spreading
  if (target_state(8) < 0.12) {
    target_state(8) = 0.12;
    ekf.x(8) = 0.12;
  } else if (target_state(8) > 0.4) {
    target_state(8) = 0.4;
    ekf.x(8) = 0.4;
  }

  if (tracker_state == TRACKING || tracker_state == TEMP_LOST) {
    if (this->diverged()) {
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "EKF diverged!");
      tracker_state = LOST;
    }
  }
  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  } else if (tracker_state == CHANGE_TARGET) {
    if (change_count_ > change_thres) {
      tracker_state = TRACKING;
      change_count_ = 0;
    } else {
      change_count_++;
    }
  }
}

void Tracker::initEKF(const Armor & a)
{
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(11);
  Eigen::VectorXd P0_dig(11);
  if (a.number == "6")  //outpost
  {
    P0_dig << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
    //P0_dig << 0.01, 4, 0.01, 4, 0.01, 0.01, 0.16, 16, 1e-6, 0, 0;
  } else if (a.number == "7" || a.number == "8")  //base
  {
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
    //P0_dig << 0.01, 4, 0.01, 4, 0.01, 0.01, 0.16, 16, 1e-6, 0, 0;
  } else {
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
    //P0_dig << 0.01, 4, 0.01, 4, 0.01, 0.01, 0.16, 36, 0.01, 0.01, 0.0025;
  }
  double r = 0.2;
  if (a.number == "6") r = 0.2765;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  dz = 0, dr = 0;
  target_state << xc, 0, yc, 0, za, 0, yaw, 0, r, dr, dz;

  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = rm_tools::limit_rad(c[6]);
    return c;
  };
  ekf =
    rm_tools::ExtendedKalmanFilter(target_state, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Tracker::update_ypda(const Armor & armor, int id)
{
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf.x, id);
  //Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  auto &p = armor.pose.position;
  Eigen::Vector3d vec(p.x, p.y, p.z);
  double distance = vec.norm();
  auto center_yaw = std::atan2(p.y, p.x);
  double yaw = orientationToYaw(armor.pose.orientation);
  //弧度制
  auto delta_angle = rm_tools::limit_rad(yaw - center_yaw);
  //不严谨数据，仅定性
  double r_distance_gain = 1.5 / r_d;
  double r_yaw_gain = 10 / r_yaw;
  Eigen::VectorXd R_dig{
    {r_py, r_py, log(std::abs(delta_angle) + 1) / r_distance_gain + r_d,
     log(std::sqrt(distance) + 1) / r_yaw_gain + r_yaw}};

  //测量过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = getArmorPositionFromState(x, id);
    Eigen::VectorXd ypd = rm_tools::xyz2ypd(xyz);
    auto angle = rm_tools::limit_rad(x[6] + id * 2 * CV_PI / armorNumAsInt);
    //auto angle = x[6] + id * 2 * CV_PI / armorNumAsInt;
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = rm_tools::limit_rad(c[0]);
    c[1] = rm_tools::limit_rad(c[1]);
    c[3] = rm_tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = Pos2Polar(armor);
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], yaw}};  //获得观测量

  ekf.update(z, H, R, h, z_subtract);
  target_state = ekf.x;
}

Eigen::MatrixXd Tracker::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = rm_tools::limit_rad(x[6] + id * 2 * CV_PI / armorNumAsInt);
  auto use_l_h = (armorNumAsInt == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = getArmorPositionFromState(x, id);
  Eigen::MatrixXd H_armor_ypd = rm_tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

void Tracker::updateArmorsNum(const Armor & armor)
{
  //已经过时了（没平衡步兵了）
  if (armor.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
    tracked_armors_num = ArmorsNum::BALANCE_2;
  } else if (tracked_id == "6") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
  armorNumAsInt = static_cast<int>(tracked_armors_num);
}

bool Tracker::diverged() const
{
  auto r_ok = ekf.x[8] > 0.05 && ekf.x[8] < 0.5;
  auto l_ok = ekf.x[8] + ekf.x[9] > 0.05 && ekf.x[8] + ekf.x[9] < 0.5;
  auto v_yaw_ok = abs(ekf.x[7]) < 15;

  return !(r_ok && l_ok && v_yaw_ok);
}

bool Tracker::converged(const Armor & armor)
{
  if (armor.number != "6" && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  } else

  //前哨站特殊判断
  if (armor.number == "6" && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x, int id) const
{
  // Calculate predicted position of the current armor
  double xc = x[0], yc = x[2];
  double yaw = rm_tools::limit_rad(x[6] + id * 2 * CV_PI / armorNumAsInt);
  auto use_l_h = (armorNumAsInt == 4) && (id == 1 || id == 3);
  double r = (use_l_h) ? x[8] + x[9] : x[8];
  double xa = xc - r * std::cos(yaw);
  double ya = yc - r * std::sin(yaw);
  double za = (use_l_h) ? x[4] + x[10] : x[4];
  return Eigen::Vector3d(xa, ya, za);
}

std::vector<Eigen::Vector4d> Tracker::getArmorsPositionFromState(const Eigen::VectorXd & x) const
{
  std::vector<Eigen::Vector4d> armors_p;
  for (int i = 0; i < armorNumAsInt; i++) {
    auto yaw = rm_tools::limit_rad(x(6) + i * 2 * CV_PI / armorNumAsInt);
    Eigen::Vector3d xyz = getArmorPositionFromState(x, i);
    armors_p.emplace_back(xyz[0], xyz[1], xyz[2], yaw);
  }
  return armors_p;
}

Eigen::VectorXd Tracker::predict(const Armor & armor, double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
  /* x  vx   y   vy  z  vz yaw v_yaw r   dr  dz*/
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0}, //x
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0}, //vx
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0}, //y
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0}, //vy
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0}, //z
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0}, //vz
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0}, //yaw
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0}, //v_yaw
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0}, //r
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0}, //dr
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}  //dz
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double x, y;
  double vx = ekf.x[1];
  double vy = ekf.x[3];
  double v_yaw = ekf.x[7];
  double dx = pow(pow(vx, 2) + pow(vy, 2), 0.5);
  double dy = abs(v_yaw);
  x = exp(-dy) * (s2qxyz_max_ - s2qxyz_min_) + s2qxyz_min_;  //加速度方差
  y = exp(-dx) * (s2qyaw_max_ - s2qyaw_min_) + s2qyaw_min_;  //角加速度方差
  //一般认为目标几乎不会往z轴运动，或z轴加速度小
  double xz = 0.01 * x;
  if (armor.number == "6") {
    x *= 0.1;   // 前哨站加速度方差
    y *= 0.05;  // 前哨站角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    { a * x,   b * x,     0,      0,      0,      0,      0,       0, 0, 0, 0},
    { b * x,   c * x,     0,      0,      0,      0,      0,       0, 0, 0, 0},
    {     0,      0,  a * x,  b * x,      0,      0,      0,       0, 0, 0, 0},
    {     0,      0,  b * x,  c * x,      0,      0,      0,       0, 0, 0, 0},
    {     0,      0,      0,      0,  a * xz,  b * xz,    0,       0, 0, 0, 0},
    {     0,      0,      0,      0,  b * xz,  c * xz,    0,       0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,  a * y,   b * y, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,  b * y,   c * y, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,       0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,       0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,       0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = rm_tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (converged(armor) && armor.number == "6" && std::abs(this->ekf.x[7]) > 2) {
    this->ekf.x[7] = this->ekf.x[7] > 0 ? 2.51 : -2.51;
    target_state[7] = ekf.x[7];
  }

  Eigen::VectorXd ekf_prediction = ekf.predict(F, Q, f);
  return ekf_prediction;
}

Eigen::Vector3d Tracker::Pos2Polar(const Armor & armor) const
{
  const auto & position = armor.pose.position;
  Eigen::Vector3d armor_position(position.x, position.y, position.z);
  return rm_tools::xyz2ypd(armor_position);
}

void Tracker::outpostCompare(const Armor & armor)
{
  if (armor.number != "6") return;
  double z = armor.pose.position.z;
  double cur_z = ekf.x[5];
  double z_error_min = 10e6;
  double z_error;
  double z_fix;
  for (int i = -2; i < 2; i++) {
    double temp_z = cur_z + i * OUTPOST_D_H;
    z_error = abs(temp_z - z);
    if (z_error < z_error_min) {
      z_error_min = z_error;
      z_fix = temp_z;
    }
  }
  ekf.x[5] = z_fix;
}
}  // namespace rm_auto_aim
