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

#ifndef ARMOR_DETECTOR_ARMOR_POSE_ESTIMATOR_HPP_
#define ARMOR_DETECTOR_ARMOR_POSE_ESTIMATOR_HPP_

// std
#include <array>
#include <memory>
#include <vector>
// OpenCV
#include <opencv2/opencv.hpp>
// Eigen
#include <Eigen/Dense>
// ros2
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
#include "armor.hpp"
#include "rm_interfaces/msg/armor.hpp"
#include "rm_tools/pnp_solver.hpp"

namespace rm_auto_aim
{
double calculateUnifiedYawEdgeCost(double pixel_distance, double angle_distance, double append_yaw);
double calculateAdjacentArmorYaw(double anchor_world_yaw, double angle_between, int sign);

class ArmorPoseEstimator
{
public:
  explicit ArmorPoseEstimator(sensor_msgs::msg::CameraInfo::SharedPtr camera_info);
  std::vector<rm_interfaces::msg::Armor> extractArmorPoses(
    std::vector<Armor> & armors, Eigen::Matrix3d R_imu_camera);

  static int getArmorNum(const Armor & armor);
  static double getAngleBetweenArmors(const Armor & armor);
  static double limitRelativeYawDegrees(double yaw_degrees);

  bool is_optimize_yaw;
  bool is_use_plus_pnp;
  bool is_use_ternary;
  bool is_show_cost;
  mutable double gimbal_yaw;
  static constexpr int IMG_W = 500;
  static constexpr int IMG_H = 500;
  static constexpr int N_SAMPLES = 250;
  static constexpr double PAIR_YAW_SEARCH_HALF_RANGE = 20.0 * CV_PI / 180.0;
  static constexpr int PAIR_YAW_REFINE_ITERATIONS = 12;

  mutable cv::Mat img_cost;

private:
  struct PoseCandidate
  {
    Armor * armor = nullptr;
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    double world_yaw = 0.0;
    double append_yaw = 0.0;
  };

  // Select the best PnP solution according to the armor's direction in image, only available for SOLVEPNP_IPPE
  void sortPnPResult(
    const Armor & armor, std::vector<cv::Mat> & rvecs, std::vector<cv::Mat> & tvecs) const;
  Eigen::Matrix3d optimize_yaw(
    const Armor & armor, const Eigen::Vector3d & t_camera_armor,
    const Eigen::Matrix3d & R_camera_armor, Eigen::Matrix3d R_imu_camera) const;

  // Convert a rotation matrix to RPY
  static Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d & R);

  rm_interfaces::msg::Armor buildArmorMsg(
    const PoseCandidate & candidate, const Eigen::Matrix3d & R_imu_camera) const;

  void refinePairedYawCandidates(
    std::vector<PoseCandidate> & candidates, const Eigen::Matrix3d & R_imu_camera) const;
  bool canRefineYawPair(const PoseCandidate & lhs, const PoseCandidate & rhs) const;
  void refineYawPair(
    PoseCandidate & anchor, PoseCandidate & other, const Eigen::Matrix3d & R_imu_camera) const;
  double getPairedYawCost(
    const PoseCandidate & anchor, const PoseCandidate & other, double anchor_world_yaw, int sign,
    const Eigen::Matrix3d & R_imu_camera) const;
  double getYawCostAtWorldYaw(
    const PoseCandidate & candidate, double world_yaw, const Eigen::Matrix3d & R_imu_camera) const;
  Eigen::Matrix3d buildCameraArmorRotationFromWorldYaw(
    const Armor & armor, double world_yaw, const Eigen::Matrix3d & R_imu_camera) const;
  double getGimbalYaw(const Eigen::Matrix3d & R_imu_camera) const;

  Eigen::Matrix3d ternary_optimize_yaw(
    const Armor & armor, const Eigen::Vector3d & t_camera_armor,
    Eigen::Matrix3d R_imu_camera) const;

  std::vector<cv::Point2f> get_reproject_points(const Armor & armor);

  double getUnifiedYawCost(
    double append_yaw, const Armor & armor, const Eigen::Vector3d & t_camera_armor,
    const Eigen::Matrix3d & R_imu_camera) const;
  double getYawByUnifiedCost(
    double left, double right, double coarse_step, double epsilon, const Armor & armor,
    const Eigen::Vector3d & t_camera_armor, const Eigen::Matrix3d & R_imu_camera) const;

  void displayYawPnPCost(
    const Armor & armor, const Eigen::Vector3d & t_camera_armor,
    const Eigen::Matrix3d & R_imu_camera) const;

  std::vector<Eigen::Vector2d> convertPointsToEigen(
    const std::vector<cv::Point2f> & image_point) const;
  static constexpr double UNIFIED_YAW_COARSE_STEP = 2.0 * CV_PI / 180.0;
  static constexpr double UNIFIED_YAW_REFINE_EPS = 0.003;
  Eigen::Matrix3d R_gimbal_camera_ideal;
  Eigen::Matrix4d tf_T;  // camera_optical_frame -> odom
  std::unique_ptr<rm_tools::PnPSolver> pnp_solver_;
};
}  // namespace rm_auto_aim
#endif  // ARMOR_POSE_ESTIMATOR_HPP_
