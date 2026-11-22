// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_detector/armor_pose_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logger.hpp>

#include "armor_detector/armor.hpp"
#include "rm_tools/math.hpp"

namespace rm_auto_aim
{
static constexpr double DETECTOR_ERROR_PIXEL_BY_SLOPE = 2.0;
static constexpr double STATIC_OUTPOST_PITCH_DEG = 75.0;
static constexpr double STATIC_OUTPOST_PITCH_TOLERANCE_DEG = 30.0;

static double square(double value) { return value * value; }

double calculateUnifiedYawEdgeCost(double pixel_distance, double angle_distance, double append_yaw)
{
  const double pixel_term = pixel_distance * std::sin(append_yaw);
  const double angle_term = angle_distance * std::cos(append_yaw);
  return std::sqrt(square(pixel_term) + square(angle_term) * DETECTOR_ERROR_PIXEL_BY_SLOPE);
}

double calculateAdjacentArmorYaw(double anchor_world_yaw, double angle_between, int sign)
{
  return rm_tools::limit_rad(anchor_world_yaw + static_cast<double>(sign) * angle_between);
}

int ArmorPoseEstimator::getArmorNum(const Armor & armor)
{
  return armor.name == ArmorName::outpost ? 3 : 4;
}

double ArmorPoseEstimator::getAngleBetweenArmors(const Armor & armor)
{
  return 2.0 * M_PI / static_cast<double>(getArmorNum(armor));
}

double ArmorPoseEstimator::limitRelativeYawDegrees(double yaw_degrees)
{
  if (!std::isfinite(yaw_degrees)) {
    return yaw_degrees;
  }

  while (yaw_degrees > 180.0) yaw_degrees -= 360.0;
  while (yaw_degrees <= -180.0) yaw_degrees += 360.0;
  return yaw_degrees;
}

ArmorPoseEstimator::ArmorPoseEstimator(sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  // Setup pnp solver
  pnp_solver_ = std::make_unique<rm_tools::PnPSolver>(camera_info->k, camera_info->d);
  pnp_solver_->setObjectPoints(
    "small", Armor::buildObjectPoints<cv::Point3f>(SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT));
  pnp_solver_->setObjectPoints(
    "large", Armor::buildObjectPoints<cv::Point3f>(LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT));

  //相机坐标系转换方向
  R_gimbal_camera_ideal = Eigen::Matrix3d::Identity();
  R_gimbal_camera_ideal << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  img_cost = cv::Mat(IMG_H, IMG_W, CV_8UC3, cv::Scalar(0, 0, 0));
}

std::vector<rm_interfaces::msg::Armor> ArmorPoseEstimator::extractArmorPoses(
  std::vector<Armor> & armors, Eigen::Matrix3d R_imu_camera)
{
  std::vector<PoseCandidate> candidates;
  for (auto & armor : armors) {
    std::vector<cv::Mat> rvecs, tvecs;
    std::string coord_frame_name = (armor.type == ArmorType::small ? "small" : "large");
    // Use PnP to get the initial pose information
    if (pnp_solver_->solvePnPGeneric(armor.points, rvecs, tvecs, coord_frame_name)) {
      sortPnPResult(armor, rvecs, tvecs);

      auto get_yaw_from_rvec = [this](const cv::Mat & rvec) {
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d R = rm_tools::cvToEigen(rmat);
        return rotationMatrixToRPY(R_gimbal_camera_ideal * R)[2];
      };

      if (is_use_plus_pnp) {
        constexpr int YAW_CORRECTION_ITERATIONS = 2;
        double correction_yaw = get_yaw_from_rvec(rvecs[0]);
        for (int i = 0; i < YAW_CORRECTION_ITERATIONS; ++i) {
          std::vector<cv::Mat> corrected_rvecs, corrected_tvecs;
          if (!pnp_solver_->solvePnPGenericWithYawCorrection(
                armor.points, corrected_rvecs, corrected_tvecs, coord_frame_name, correction_yaw)) {
            RCLCPP_WARN(
              rclcpp::get_logger("armor_detector"),
              "Yaw-corrected PnP failed; using previous PnP result.");
            break;
          }
          sortPnPResult(armor, corrected_rvecs, corrected_tvecs);
          rvecs = std::move(corrected_rvecs);
          tvecs = std::move(corrected_tvecs);
          correction_yaw = get_yaw_from_rvec(rvecs[0]);
        }
      }

      cv::Mat rmat;
      cv::Rodrigues(rvecs[0], rmat);

      Eigen::Matrix3d R = rm_tools::cvToEigen(rmat);
      Eigen::Vector3d t = rm_tools::cvToEigen(tvecs[0]);

      const double raw_world_pitch_deg = rotationMatrixToRPY(R_imu_camera * R)[1] * 180 / M_PI;
      if (
        armor.name == ArmorName::outpost && std::isfinite(raw_world_pitch_deg) &&
        std::abs(raw_world_pitch_deg - STATIC_OUTPOST_PITCH_DEG) <=
          STATIC_OUTPOST_PITCH_TOLERANCE_DEG) {
        continue;
      }

      Eigen::Vector3d armor_rpy_in_gimbal =
        rotationMatrixToRPY(R_gimbal_camera_ideal * R) * 180 / M_PI;
      double armor_roll_in_gimbal = armor_rpy_in_gimbal[0];

      if (is_optimize_yaw && armor_roll_in_gimbal < 15) {
        if (is_use_ternary)
        //三分法
        {
          R = ternary_optimize_yaw(armor, t, R_imu_camera);
          if (is_show_cost) displayYawPnPCost(armor, t, R_imu_camera);
        } else
          //暴力搜索法
          R = optimize_yaw(armor, t, R, R_imu_camera);
      }

      const double world_yaw = rotationMatrixToRPY(R_imu_camera * R)[2];
      const double append_yaw = rm_tools::limit_rad(world_yaw - getGimbalYaw(R_imu_camera));
      candidates.push_back(PoseCandidate{&armor, t, R, world_yaw, append_yaw});
    } else {
      RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "PnP Failed!");
    }
  }

  if (is_optimize_yaw) {
    refinePairedYawCandidates(candidates, R_imu_camera);
  }

  std::vector<rm_interfaces::msg::Armor> armors_msg;
  armors_msg.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    armors_msg.emplace_back(buildArmorMsg(candidate, R_imu_camera));
  }

  return armors_msg;
}

rm_interfaces::msg::Armor ArmorPoseEstimator::buildArmorMsg(
  const PoseCandidate & candidate, const Eigen::Matrix3d & R_imu_camera) const
{
  const Armor & armor = *candidate.armor;
  Eigen::Quaterniond q(candidate.R);
  Eigen::Vector3d rpy = rotationMatrixToRPY(R_imu_camera * candidate.R) * 180 / M_PI;
  Eigen::Matrix3d R_gimbal_odom = R_imu_camera * R_gimbal_camera_ideal.inverse();
  Eigen::Vector3d imu_rpy = rotationMatrixToRPY(R_gimbal_odom) * 180 / M_PI;

  rm_interfaces::msg::Armor armor_msg;
  armor_msg.type = ARMOR_TYPES[static_cast<int>(armor.type)];
  armor_msg.number = ARMOR_NAMES[static_cast<int>(armor.name)];
  armor_msg.color = COLORS[static_cast<int>(armor.color)];
  armor_msg.prob = armor.confidence;
  armor_msg.yaw = limitRelativeYawDegrees(rpy(2) - imu_rpy(2));
  armor_msg.pitch = rpy(1);
  armor_msg.roll = rpy(0);
  // armor_msg.side_ratio = armor.side_ratio;

  armor_msg.pose.position.x = candidate.t(0);
  armor_msg.pose.position.y = candidate.t(1);
  armor_msg.pose.position.z = candidate.t(2);
  armor_msg.pose.orientation.x = q.x();
  armor_msg.pose.orientation.y = q.y();
  armor_msg.pose.orientation.z = q.z();
  armor_msg.pose.orientation.w = q.w();

  armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

  armor_msg.kpts.clear();
  for (const auto & pt : armor.points) {
    geometry_msgs::msg::Point point;
    point.x = pt.x;
    point.y = pt.y;
    armor_msg.kpts.emplace_back(point);
  }

  return armor_msg;
}

void ArmorPoseEstimator::refinePairedYawCandidates(
  std::vector<PoseCandidate> & candidates, const Eigen::Matrix3d & R_imu_camera) const
{
  std::vector<bool> visited(candidates.size(), false);
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (visited[i]) {
      continue;
    }

    std::vector<std::size_t> group_indices{i};
    visited[i] = true;
    for (std::size_t j = i + 1; j < candidates.size(); ++j) {
      if (canRefineYawPair(candidates[i], candidates[j])) {
        group_indices.emplace_back(j);
        visited[j] = true;
      }
    }

    if (group_indices.size() != 2) {
      continue;
    }

    PoseCandidate * anchor = &candidates[group_indices[0]];
    PoseCandidate * other = &candidates[group_indices[1]];
    if (std::abs(anchor->append_yaw) > std::abs(other->append_yaw)) {
      std::swap(anchor, other);
    }
    refineYawPair(*anchor, *other, R_imu_camera);
  }
}

bool ArmorPoseEstimator::canRefineYawPair(
  const PoseCandidate & lhs, const PoseCandidate & rhs) const
{
  if (lhs.armor == nullptr || rhs.armor == nullptr) {
    return false;
  }
  return lhs.armor->color == rhs.armor->color && lhs.armor->name == rhs.armor->name &&
         lhs.armor->type == rhs.armor->type && getArmorNum(*lhs.armor) == getArmorNum(*rhs.armor);
}

void ArmorPoseEstimator::refineYawPair(
  PoseCandidate & anchor, PoseCandidate & other, const Eigen::Matrix3d & R_imu_camera) const
{
  const double angle_between = getAngleBetweenArmors(*anchor.armor);

  double best_cost = std::numeric_limits<double>::infinity();
  double best_anchor_yaw = anchor.world_yaw;
  const int sign = rm_tools::limit_rad(other.world_yaw - anchor.world_yaw) >= 0.0 ? 1 : -1;

  double left = anchor.world_yaw - PAIR_YAW_SEARCH_HALF_RANGE;
  double right = anchor.world_yaw + PAIR_YAW_SEARCH_HALF_RANGE;
  const double phi = (std::sqrt(5.0) - 1.0) / 2.0;

  double mid_left = left + (right - left) * (1.0 - phi);
  double mid_right = left + (right - left) * phi;
  double mid_left_cost = getPairedYawCost(anchor, other, mid_left, sign, R_imu_camera);
  double mid_right_cost = getPairedYawCost(anchor, other, mid_right, sign, R_imu_camera);

  for (int i = 0; i < PAIR_YAW_REFINE_ITERATIONS; ++i) {
    if (mid_left_cost < mid_right_cost) {
      right = mid_right;
      mid_right = mid_left;
      mid_right_cost = mid_left_cost;
      mid_left = left + (right - left) * (1.0 - phi);
      mid_left_cost = getPairedYawCost(anchor, other, mid_left, sign, R_imu_camera);
    } else {
      left = mid_left;
      mid_left = mid_right;
      mid_left_cost = mid_right_cost;
      mid_right = left + (right - left) * phi;
      mid_right_cost = getPairedYawCost(anchor, other, mid_right, sign, R_imu_camera);
    }
  }

  const double refined_anchor_yaw = (left + right) / 2.0;
  const double refined_cost =
    getPairedYawCost(anchor, other, refined_anchor_yaw, sign, R_imu_camera);
  if (std::isfinite(refined_cost) && refined_cost < best_cost) {
    best_cost = refined_cost;
    best_anchor_yaw = rm_tools::limit_rad(refined_anchor_yaw);
  }

  if (!std::isfinite(best_cost)) {
    return;
  }

  const double gimbal_yaw_in_world = getGimbalYaw(R_imu_camera);
  const double best_other_yaw = calculateAdjacentArmorYaw(best_anchor_yaw, angle_between, sign);
  anchor.world_yaw = best_anchor_yaw;
  anchor.append_yaw = rm_tools::limit_rad(best_anchor_yaw - gimbal_yaw_in_world);
  anchor.R = buildCameraArmorRotationFromWorldYaw(*anchor.armor, best_anchor_yaw, R_imu_camera);

  other.world_yaw = best_other_yaw;
  other.append_yaw = rm_tools::limit_rad(best_other_yaw - gimbal_yaw_in_world);
  other.R = buildCameraArmorRotationFromWorldYaw(*other.armor, best_other_yaw, R_imu_camera);
}

double ArmorPoseEstimator::getPairedYawCost(
  const PoseCandidate & anchor, const PoseCandidate & other, double anchor_world_yaw, int sign,
  const Eigen::Matrix3d & R_imu_camera) const
{
  const double angle_between = getAngleBetweenArmors(*anchor.armor);
  const double other_world_yaw = calculateAdjacentArmorYaw(anchor_world_yaw, angle_between, sign);
  const double anchor_cost = getYawCostAtWorldYaw(anchor, anchor_world_yaw, R_imu_camera);
  const double other_cost = getYawCostAtWorldYaw(other, other_world_yaw, R_imu_camera);
  if (!std::isfinite(anchor_cost) || !std::isfinite(other_cost)) {
    return std::numeric_limits<double>::infinity();
  }
  return anchor_cost + other_cost;
}

double ArmorPoseEstimator::getYawCostAtWorldYaw(
  const PoseCandidate & candidate, double world_yaw, const Eigen::Matrix3d & R_imu_camera) const
{
  gimbal_yaw = getGimbalYaw(R_imu_camera);
  const double append_yaw = rm_tools::limit_rad(world_yaw - gimbal_yaw);
  return getUnifiedYawCost(append_yaw, *candidate.armor, candidate.t, R_imu_camera);
}

Eigen::Matrix3d ArmorPoseEstimator::buildCameraArmorRotationFromWorldYaw(
  const Armor & armor, double world_yaw, const Eigen::Matrix3d & R_imu_camera) const
{
  const double pitch =
    (armor.name == ArmorName::outpost) ? -FIFTTEN_DEGREE_RAD : FIFTTEN_DEGREE_RAD;
  const double sin_yaw = std::sin(world_yaw);
  const double cos_yaw = std::cos(world_yaw);
  const double sin_pitch = std::sin(pitch);
  const double cos_pitch = std::cos(pitch);

  const Eigen::Matrix3d R_world_armor{
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch},
    {-sin_pitch, 0, cos_pitch}};

  return R_imu_camera.transpose() * R_world_armor;
}

double ArmorPoseEstimator::getGimbalYaw(const Eigen::Matrix3d & R_imu_camera) const
{
  Eigen::Matrix3d R_gimbal_odom = R_imu_camera * R_gimbal_camera_ideal.inverse();
  return rotationMatrixToRPY(R_gimbal_odom)[2];
}

Eigen::Vector3d ArmorPoseEstimator::rotationMatrixToRPY(const Eigen::Matrix3d & R)
{
  // Transform to camera frame
  Eigen::Quaterniond q(R);
  // Get armor yaw
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  Eigen::Vector3d rpy;
  tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
  return rpy;
}

void ArmorPoseEstimator::sortPnPResult(
  const Armor & armor, std::vector<cv::Mat> & rvecs, std::vector<cv::Mat> & tvecs) const
{
  constexpr float PROJECT_ERR_THRES = 3.0;
  if (rvecs.size() < 2 || tvecs.size() < 2) {
    return;
  }

  // 获取这两个解
  cv::Mat & rvec1 = rvecs.at(0);
  cv::Mat & tvec1 = tvecs.at(0);
  cv::Mat & rvec2 = rvecs.at(1);
  cv::Mat & tvec2 = tvecs.at(1);

  // 将旋转向量转换为旋转矩阵
  cv::Mat R1_cv, R2_cv;
  cv::Rodrigues(rvec1, R1_cv);
  cv::Rodrigues(rvec2, R2_cv);

  // 转换为Eigen矩阵
  Eigen::Matrix3d R1 = rm_tools::cvToEigen(R1_cv);
  Eigen::Matrix3d R2 = rm_tools::cvToEigen(R2_cv);

  // 计算云台系下装甲板的RPY角
  auto rpy1 = rotationMatrixToRPY(R_gimbal_camera_ideal * R1);
  auto rpy2 = rotationMatrixToRPY(R_gimbal_camera_ideal * R2);

  std::string coord_frame_name = (armor.type == ArmorType::small ? "small" : "large");
  double error1 = is_use_plus_pnp ? pnp_solver_->calculateYawCorrectedReprojectionError(
                                      armor.points, rvec1, tvec1, coord_frame_name, rpy1[2])
                                  : pnp_solver_->calculateReprojectionError(
                                      armor.points, rvec1, tvec1, coord_frame_name);
  double error2 = is_use_plus_pnp ? pnp_solver_->calculateYawCorrectedReprojectionError(
                                      armor.points, rvec2, tvec2, coord_frame_name, rpy2[2])
                                  : pnp_solver_->calculateReprojectionError(
                                      armor.points, rvec2, tvec2, coord_frame_name);
  // 两个解的重投影误差差距较大或者roll角度较大时，不做选择
  if (
    (error2 / error1 > PROJECT_ERR_THRES) || (rpy1[0] > 10 * 180 / M_PI) ||
    (rpy2[0] > 10 * 180 / M_PI)) {
    //RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "PnP derror too large or roll is too large!");
    return;
  }

  // 计算灯条在图像中的倾斜角度
  double l_angle =
    std::atan2((armor.points[3] - armor.points[0]).y, (armor.points[3] - armor.points[0]).x) * 180 /
    M_PI;
  double r_angle =
    std::atan2((armor.points[2] - armor.points[1]).y, (armor.points[2] - armor.points[1]).x) * 180 /
    M_PI;
  double angle = (l_angle + r_angle) / 2;
  angle += 90.0;

  if (armor.name == ArmorName::outpost) angle = -angle;

  // 根据倾斜角度选择解
  // 如果装甲板左倾（angle > 0），选择Yaw为负的解
  // 如果装甲板右倾（angle < 0），选择Yaw为正的解
  if ((angle > 0 && rpy1[2] > 0 && rpy2[2] < 0) || (angle < 0 && rpy1[2] < 0 && rpy2[2] > 0)) {
    std::swap(rvec1, rvec2);
    std::swap(tvec1, tvec2);
    RCLCPP_DEBUG(rclcpp::get_logger("armor_detector"), "PnP Solution 2 Selected");
  }
}

Eigen::Matrix3d ArmorPoseEstimator::optimize_yaw(
  const Armor & armor, const Eigen::Vector3d & t_camera_armor,
  const Eigen::Matrix3d & R_camera_armor, Eigen::Matrix3d R_imu_camera) const
{
  std::string coord_frame_name = (armor.type == ArmorType::small ? "small" : "large");

  constexpr double SEARCH_RANGE = 140;  // degree

  // 从当前姿态估计初始yaw
  Eigen::Matrix3d R_world_armor = R_imu_camera * R_camera_armor;
  Eigen::Matrix3d R_gimbal_odom = R_imu_camera * R_gimbal_camera_ideal.inverse();
  Eigen::Vector3d imu_rpy = rotationMatrixToRPY(R_gimbal_odom);
  Eigen::Vector3d armor_rpy_in_world = rotationMatrixToRPY(R_world_armor);

  auto yaw0 = rm_tools::limit_rad(imu_rpy[2] - SEARCH_RANGE / 2 * CV_PI / 180.0);
  auto pitch = (armor.name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;

  auto min_error = 1e10;
  double best_yaw = armor_rpy_in_world[2];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = rm_tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // 构建世界系下的装甲板姿态(ZYX欧拉角)
    const Eigen::Matrix3d R_world_armor_new{
      {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
      {sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch},
      {-sin_pitch, 0, cos_pitch}};

    // 转换到相机系
    Eigen::Matrix3d R_camera_armor_new = R_imu_camera.transpose() * R_world_armor_new;

    // 转换为OpenCV格式
    cv::Mat R_camera_armor_cv;
    cv::eigen2cv(R_camera_armor_new, R_camera_armor_cv);

    cv::Vec3d rvec;
    cv::Rodrigues(R_camera_armor_cv, rvec);
    cv::Vec3d tvec(t_camera_armor[0], t_camera_armor[1], t_camera_armor[2]);

    // 计算重投影误差
    const double correction_yaw =
      rotationMatrixToRPY(R_gimbal_camera_ideal * R_camera_armor_new)[2];
    auto error = is_use_plus_pnp ? pnp_solver_->calculateYawCorrectedReprojectionError(
                                     armor.points, rvec, tvec, coord_frame_name, correction_yaw)
                                 : pnp_solver_->calculateReprojectionError(
                                     armor.points, rvec, tvec, coord_frame_name);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }
  // 返回优化后的旋转矩阵(世界系)
  auto sin_yaw = std::sin(best_yaw);
  auto cos_yaw = std::cos(best_yaw);
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  const Eigen::Matrix3d R_world_armor_best{
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch},
    {-sin_pitch, 0, cos_pitch}};

  return R_imu_camera.transpose() * R_world_armor_best;
}

Eigen::Matrix3d ArmorPoseEstimator::ternary_optimize_yaw(
  const Armor & armor, const Eigen::Vector3d & t_camera_armor, Eigen::Matrix3d R_imu_camera) const
{
  //get gimbal_yaw
  Eigen::Matrix3d R_gimbal_odom = R_imu_camera * R_gimbal_camera_ideal.inverse();
  Eigen::Vector3d imu_rpy = rotationMatrixToRPY(R_gimbal_odom);
  gimbal_yaw = imu_rpy[2];

  double append_yaw = getYawByUnifiedCost(
    -(M_PI / 2), (M_PI / 2), UNIFIED_YAW_COARSE_STEP, UNIFIED_YAW_REFINE_EPS, armor, t_camera_armor,
    R_imu_camera);
  if (!std::isfinite(append_yaw)) {
    append_yaw = 0.0;
  }
  double optimized_yaw = gimbal_yaw + append_yaw;

  auto pitch = (armor.name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;

  // 返回优化后的旋转矩阵(世界系)
  auto sin_yaw = std::sin(optimized_yaw);
  auto cos_yaw = std::cos(optimized_yaw);
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  const Eigen::Matrix3d R_world_armor_best{
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch},
    {-sin_pitch, 0, cos_pitch}};

  return R_imu_camera.transpose() * R_world_armor_best;
}

double ArmorPoseEstimator::getYawByUnifiedCost(
  double left, double right, double coarse_step, double epsilon, const Armor & armor,
  const Eigen::Vector3d & t_camera_armor, const Eigen::Matrix3d & R_imu_camera) const
{
  double best_yaw = std::numeric_limits<double>::quiet_NaN();
  double best_cost = std::numeric_limits<double>::infinity();

  for (double yaw = left; yaw <= right + 1e-9; yaw += coarse_step) {
    const double cost = getUnifiedYawCost(yaw, armor, t_camera_armor, R_imu_camera);
    if (std::isfinite(cost) && cost < best_cost) {
      best_cost = cost;
      best_yaw = yaw;
    }
  }

  if (!std::isfinite(best_yaw)) {
    return best_yaw;
  }

  double refine_left = std::max(left, best_yaw - coarse_step);
  double refine_right = std::min(right, best_yaw + coarse_step);
  while (refine_right - refine_left > epsilon) {
    double mid1 = refine_left + (refine_right - refine_left) / 3.0;
    double mid2 = refine_right - (refine_right - refine_left) / 3.0;

    double f1 = getUnifiedYawCost(mid1, armor, t_camera_armor, R_imu_camera);
    double f2 = getUnifiedYawCost(mid2, armor, t_camera_armor, R_imu_camera);
    if (!std::isfinite(f1)) f1 = std::numeric_limits<double>::infinity();
    if (!std::isfinite(f2)) f2 = std::numeric_limits<double>::infinity();

    if (f1 < f2) {
      refine_right = mid2;
    } else {
      refine_left = mid1;
    }
  }
  return (refine_left + refine_right) / 2.0;
}

double ArmorPoseEstimator::getUnifiedYawCost(
  double append_yaw, const Armor & armor, const Eigen::Vector3d & t_camera_armor,
  const Eigen::Matrix3d & R_imu_camera) const
{
  double yaw = rm_tools::limit_rad(gimbal_yaw + append_yaw);
  auto pitch = (armor.name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  const Eigen::Matrix3d R_world_armor_new{
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch},
    {-sin_pitch, 0, cos_pitch}};

  Eigen::Matrix3d R_camera_armor_new = R_imu_camera.transpose() * R_world_armor_new;

  cv::Mat R_camera_armor_cv;
  cv::eigen2cv(R_camera_armor_new, R_camera_armor_cv);

  cv::Vec3d rvec;
  cv::Rodrigues(R_camera_armor_cv, rvec);
  cv::Vec3d tvec(t_camera_armor[0], t_camera_armor[1], t_camera_armor[2]);
  std::string coord_frame_name = (armor.type == ArmorType::small ? "small" : "large");
  const double correction_yaw = rotationMatrixToRPY(R_gimbal_camera_ideal * R_camera_armor_new)[2];
  auto reprojected_points = convertPointsToEigen(
    is_use_plus_pnp
      ? pnp_solver_->getYawCorrectedReprojectionPoint(rvec, tvec, coord_frame_name, correction_yaw)
      : pnp_solver_->getReprojectionPoint(rvec, tvec, coord_frame_name));
  auto armor_image_points = convertPointsToEigen(armor.points);

  if (reprojected_points.size() != armor_image_points.size() || armor_image_points.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }

  double cost = 0.0;
  for (size_t i = 0; i < armor_image_points.size(); i++) {
    size_t index_next = (i + 1) % armor_image_points.size();
    Eigen::Vector2d pixel_line = armor_image_points[index_next] - armor_image_points[i];
    Eigen::Vector2d project_line = reprojected_points[index_next] - reprojected_points[i];

    const double pixel_line_norm = pixel_line.norm();
    const double project_line_norm = project_line.norm();
    if (pixel_line_norm <= 1e-9 || project_line_norm <= 1e-9) {
      return std::numeric_limits<double>::infinity();
    }

    double this_dist = (armor_image_points[i] - reprojected_points[i]).norm();
    double next_dist = (armor_image_points[index_next] - reprojected_points[index_next]).norm();
    double line_dist = std::abs(pixel_line_norm - project_line_norm);

    double pixel_dist = (0.5 * (this_dist + next_dist) + line_dist) / pixel_line_norm;
    double cos_angle = pixel_line.dot(project_line) / (pixel_line_norm * project_line_norm);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    double angle_dist = std::abs(std::acos(cos_angle));

    cost += calculateUnifiedYawEdgeCost(pixel_dist, angle_dist, append_yaw);
  }
  return cost;
}

std::vector<Eigen::Vector2d> ArmorPoseEstimator::convertPointsToEigen(
  const std::vector<cv::Point2f> & image_point) const
{
  std::vector<Eigen::Vector2d> eigen_points;
  for (const auto & point : image_point) {
    eigen_points.push_back(Eigen::Vector2d(point.x, point.y));  // 转换为 Eigen::Vector2d
  }
  return eigen_points;
}

void ArmorPoseEstimator::displayYawPnPCost(
  const Armor & armor, const Eigen::Vector3d & t_camera_armor,
  const Eigen::Matrix3d & R_imu_camera) const
{
  img_cost = cv::Mat(IMG_H, IMG_W, CV_8UC3, cv::Scalar(0, 0, 0));

  double step = M_PI / N_SAMPLES;  // 从 -PI/2 到 +PI/2
  double max_val = -1.0, min_val = 1e10;

  std::vector<double> unified_cost_list(N_SAMPLES);

  // ========== 第一遍：采样所有代价值，找 min/max ==========
  for (int i = 0; i < N_SAMPLES; i++) {
    double app_yaw = i * step - M_PI / 2;

    unified_cost_list[i] = getUnifiedYawCost(app_yaw, armor, t_camera_armor, R_imu_camera);
    if (std::isfinite(unified_cost_list[i])) {
      max_val = std::max(max_val, unified_cost_list[i]);
      min_val = std::min(min_val, unified_cost_list[i]);
    }
  }

  // 防止 max == min 导致除零
  if (std::abs(max_val - min_val) < 1e-12) {
    max_val = min_val + 1.0;
  }

  // ========== 第二遍：归一化并绘制代价曲线 ==========
  for (int i = 0; i < N_SAMPLES; i++) {
    int unified_y =
      std::isfinite(unified_cost_list[i])
        ? static_cast<int>((unified_cost_list[i] - min_val) / (max_val - min_val) * (IMG_H - 1))
        : 0;

    int x = i * 2;

    // 紫色 = unified cost
    if (std::isfinite(unified_cost_list[i])) {
      cv::circle(img_cost, cv::Point(x, IMG_H - 1 - unified_y), 1, cv::Scalar(255, 0, 255), 2);
    }
  }

  // ========== 计算 unified cost 最优 yaw ==========
  double unified_yaw = getYawByUnifiedCost(
    -(M_PI / 2), (M_PI / 2), UNIFIED_YAW_COARSE_STEP, UNIFIED_YAW_REFINE_EPS, armor, t_camera_armor,
    R_imu_camera);
  double append_yaw = std::isfinite(unified_yaw) ? unified_yaw : 0.0;

  // ========== 绘制最优 yaw 竖线 ==========
  // append_yaw ∈ [-PI/2, PI/2] -> 映射到 [0, 500]
  auto yawToX = [&](double yaw) -> int {
    return static_cast<int>((yaw + M_PI / 2) / M_PI * IMG_W);
  };

  // 白色竖线 = 最终 yaw
  int final_x = yawToX(append_yaw);
  cv::line(
    img_cost, cv::Point(final_x, 0), cv::Point(final_x, IMG_H), cv::Scalar(255, 255, 255), 2);

  // 紫色虚线 = unified cost 最优
  if (std::isfinite(unified_yaw)) {
    int unified_x = yawToX(unified_yaw);
    for (int y = 0; y < IMG_H; y += 8) {
      cv::line(
        img_cost, cv::Point(unified_x, y), cv::Point(unified_x, std::min(y + 4, IMG_H)),
        cv::Scalar(255, 0, 255), 1);
    }
  }

  // ========== 绘制刻度线 (每 0.2 rad 一个刻度) ==========
  int center_x = IMG_W / 2;  // 对应 append_yaw = 0
  for (int i = 1; i <= 7; i++) {
    int offset = static_cast<int>(N_SAMPLES * 0.2 * i / M_PI);
    int left_x = center_x - offset;
    int right_x = center_x + offset;
    cv::line(
      img_cost, cv::Point(left_x, IMG_H - 14), cv::Point(left_x, IMG_H - 1),
      cv::Scalar(255, 255, 255), 1);
    cv::line(
      img_cost, cv::Point(right_x, IMG_H - 14), cv::Point(right_x, IMG_H - 1),
      cv::Scalar(255, 255, 255), 1);
  }
  // 中心刻度线（加粗）
  cv::line(
    img_cost, cv::Point(center_x, IMG_H - 17), cv::Point(center_x, IMG_H - 1),
    cv::Scalar(255, 255, 255), 2);

  // ========== 显示文字信息 ==========
  // 装甲板类型
  std::string armor_type_str = (armor.type == ArmorType::small) ? "Small" : "Large";
  cv::putText(
    img_cost, armor_type_str, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
    cv::Scalar(255, 150, 0), 1);

  // 装甲板名称
  std::string armor_name_str;
  switch (armor.name) {
    case ArmorName::outpost:
      armor_name_str = "Outpost(DW15)";
      break;
    default:
      armor_name_str = "Normal(UP15)";
      break;
  }
  cv::putText(
    img_cost, armor_name_str, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
    cv::Scalar(255, 150, 0), 1);

  // 显示数值
  char buf[128];

  snprintf(
    buf, sizeof(buf), "gimbal_yaw: %.3f rad (%.1f deg)", gimbal_yaw, gimbal_yaw * 180.0 / M_PI);
  cv::putText(
    img_cost, buf, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);

  snprintf(buf, sizeof(buf), "unified_yaw: %.4f rad", unified_yaw);
  cv::putText(
    img_cost, buf, cv::Point(20, 115), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 0, 255), 1);

  const double final_yaw = rm_tools::limit_rad(gimbal_yaw + append_yaw);
  snprintf(
    buf, sizeof(buf), "final_yaw:  %.4f rad (%.1f deg)", final_yaw, final_yaw * 180.0 / M_PI);
  cv::putText(
    img_cost, buf, cv::Point(20, 140), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);

  snprintf(buf, sizeof(buf), "yaw_correction: %s", is_use_plus_pnp ? "on" : "off");
  cv::putText(
    img_cost, buf, cv::Point(20, 165), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200),
    1);

  // 图例
  cv::putText(
    img_cost, "--- Unified Cost", cv::Point(320, 70), cv::FONT_HERSHEY_SIMPLEX, 0.45,
    cv::Scalar(255, 0, 255), 1);
  cv::putText(
    img_cost, "| Final Yaw", cv::Point(320, 110), cv::FONT_HERSHEY_SIMPLEX, 0.45,
    cv::Scalar(255, 255, 255), 1);
}
}  // namespace rm_auto_aim
