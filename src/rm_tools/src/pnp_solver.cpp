// Copyright 2022 Chen Jun

#include "rm_tools/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_auto_aim
{
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & distortion_coefficients,
  cv::SolvePnPMethod method)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  distortion_coefficients_(
    cv::Mat(1, 5, CV_64F, const_cast<double *>(distortion_coefficients.data())).clone()),
  method_(method)
{
}

void PnPSolver::setObjectPoints(
  const std::string & coord_frame_name, const std::vector<cv::Point3f> & object_points) noexcept
{
  object_points_map_[coord_frame_name] = object_points;
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point) const noexcept
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

double PnPSolver::calculateReprojectionError(
  const std::vector<cv::Point2f> & image_points, const cv::Mat & rvec, const cv::Mat & tvec,
  const std::string & coord_frame_name, double side_ratio)
{
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto & object_points = object_points_map_.at(coord_frame_name);
    std::vector<cv::Point3f> modified_object_points = object_points;
    get_better_object_points(modified_object_points, side_ratio);
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      object_points, rvec, tvec, camera_matrix_, distortion_coefficients_, reprojected_points);
    double error = 0;
    for (size_t i = 0; i < image_points.size(); ++i) {
      error += cv::norm(image_points[i] - reprojected_points[i]);
    }
    return error;
  } else {
    return 0;
  }
}

double PnPSolver::calculateReprojectionError(
  const std::vector<cv::Point2f> & image_points, const cv::Vec3d & rvec, const cv::Vec3d & tvec,
  const std::string & coord_frame_name, double side_ratio)
{
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto & object_points = object_points_map_.at(coord_frame_name);
    std::vector<cv::Point3f> modified_object_points = object_points;
    get_better_object_points(modified_object_points, side_ratio);
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      object_points, rvec, tvec, camera_matrix_, distortion_coefficients_, reprojected_points);
    double error = 0;
    for (size_t i = 0; i < image_points.size(); ++i) {
      error += cv::norm(image_points[i] - reprojected_points[i]);
    }
    return error;
  } else {
    return 0;
  }
}
std::vector<cv::Point2f> PnPSolver::getReprojectionPoint(const cv::Vec3d & rvec, const cv::Vec3d & tvec, const std::string & coord_frame_name, double side_ratio)
{
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto & object_points = object_points_map_.at(coord_frame_name);
    std::vector<cv::Point3f> modified_object_points = object_points;
    get_better_object_points(modified_object_points, side_ratio);
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      object_points, rvec, tvec, camera_matrix_, distortion_coefficients_, reprojected_points);
      return reprojected_points;
}
}
}  // namespace rm_auto_aim
