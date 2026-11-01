// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef RM_TOOLS__PNP_SOLVER_HPP_
#define RM_TOOLS__PNP_SOLVER_HPP_

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <vector>
// STD
#include <array>
#include <vector>

#include "iostream"

namespace rm_auto_aim
{
class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients,
    cv::SolvePnPMethod method = cv::SOLVEPNP_IPPE);

  // Set an object coord system
  void setObjectPoints(
    const std::string & coord_frame_name, const std::vector<cv::Point3f> & object_points) noexcept;

  // Get 3d position of the object coord system using PnP algorithm
  template <class InputArray>
  bool solvePnPGeneric(
    const InputArray & image_points, std::vector<cv::Mat> & rvecs, std::vector<cv::Mat> & tvecs,
    const std::string & coord_frame_name, double side_ratio = 1.0)
  {
    rvecs.clear();
    tvecs.clear();
    if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
      const auto & object_points = object_points_map_[coord_frame_name];
      std::vector<cv::Point3f> modified_object_points = object_points;
      get_better_object_points(modified_object_points, side_ratio);
      int solutions = cv::solvePnPGeneric(
        modified_object_points, image_points, camera_matrix_, distortion_coefficients_, rvecs,
        tvecs, false, method_);
      return solutions > 0;
    } else {
      return false;
    }
  }

  void get_better_object_points(std::vector<cv::Point3f> & world_points, double side_ratio)
  {
    double deviation = abs(side_ratio - 1);
    double gain = 0.5;  //测量极值得出
    double ratio = 1 - gain * deviation;
    double y_gain = 0.1;
    if (side_ratio < 1)  //left is shorter
    {
      ratio = std::max(ratio, 0.95);
      double deviation_ = 1 - ratio;
      world_points[0].y += world_points[0].y * deviation_ * y_gain;
      world_points[3].y += world_points[3].y * deviation_ * y_gain;
      // world_points[0].y *= (1.5 - ratio / 2);
      // world_points[3].y *= (1.5 - ratio / 2);
      world_points[0].z *= ratio;
      world_points[3].z *= ratio;
    } else {
      ratio = std::max(ratio, 0.95);
      double deviation_ = 1 - ratio;
      // world_points[1].y *= (1.5 - ratio / 2);
      // world_points[2].y *= (1.5 - ratio / 2);
      world_points[1].y += world_points[1].y * deviation_ * y_gain;
      world_points[2].y += world_points[2].y * deviation_ * y_gain;
      world_points[1].z *= ratio;
      world_points[2].z *= ratio;
    }
  }
  bool get_world_points(
    const std::string & coord_frame_name, std::vector<cv::Point3f> & world_points,
    double side_ratio = 1.0)
  {
    if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
      const auto & object_points = object_points_map_[coord_frame_name];
      world_points = object_points;
      get_better_object_points(world_points, side_ratio);
      return true;
    } else {
      return false;
    }
  }
  // Get 3d position of the object coord system using PnP algorithm
  template <class InputArray>
  bool solvePnP(
    const InputArray & image_points, cv::Mat & rvec, cv::Mat & tvec,
    const std::string & coord_frame_name)
  {
    if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
      const auto & object_points = object_points_map_[coord_frame_name];
      return cv::solvePnP(
        object_points, image_points, camera_matrix_, distortion_coefficients_, rvec, tvec, false,
        method_);
    } else {
      return false;
    }
  }

  // Calculate the distance between armor center and image center
  float calculateDistanceToCenter(const cv::Point2f & image_point) const noexcept;

  double calculateReprojectionError(
    const std::vector<cv::Point2f> & image_points, const cv::Mat & rvec, const cv::Mat & tvec,
    const std::string & coord_frame_name, double side_ratio = 1.0);

  double calculateReprojectionError(
    const std::vector<cv::Point2f> & image_points, const cv::Vec3d & rvec, const cv::Vec3d & tvec,
    const std::string & coord_frame_name, double side_ratio = 1.0);

  std::vector<cv::Point2f> getReprojectionPoint(
    const cv::Vec3d & rvec, const cv::Vec3d & tvec, const std::string & coord_frame_name,
    double side_ratio = 1.0);
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;

private:
  std::unordered_map<std::string, std::vector<cv::Point3f>> object_points_map_;
  cv::SolvePnPMethod method_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
