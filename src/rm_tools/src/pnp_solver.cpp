// Copyright 2022 Chen Jun

#include "rm_tools/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace rm_tools
{
double PnPSolver::interpolateYawShape(
  double abs_yaw_deg, const YawShapeSamples & yaw_samples,
  const YawShapeSamples & shape_samples) noexcept
{
  if (abs_yaw_deg <= yaw_samples.front()) {
    return shape_samples.front();
  }
  if (abs_yaw_deg >= yaw_samples.back()) {
    return shape_samples.back();
  }

  for (std::size_t i = 1; i < yaw_samples.size(); ++i) {
    if (abs_yaw_deg <= yaw_samples[i]) {
      const double left_yaw = yaw_samples[i - 1];
      const double right_yaw = yaw_samples[i];
      const double ratio = (abs_yaw_deg - left_yaw) / (right_yaw - left_yaw);
      return shape_samples[i - 1] + ratio * (shape_samples[i] - shape_samples[i - 1]);
    }
  }
  return shape_samples.back();
}

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

PnPSolver::YawObjectPointCorrection PnPSolver::calculate_yaw_object_point_correction(
  double armor_yaw_rad) noexcept
{
  const double abs_yaw_deg = std::abs(armor_yaw_rad) * 180.0 / CV_PI;
  const double raw_width_shape =
    interpolateYawShape(abs_yaw_deg, YAW_DEG_SAMPLES, RAW_WIDTH_SHAPE_MM);
  const double raw_height_shape =
    interpolateYawShape(abs_yaw_deg, YAW_DEG_SAMPLES, RAW_HEIGHT_SHAPE_MM);

  return {
    std::clamp(raw_width_shape / RAW_WIDTH_SHAPE_MAX_MM * MAX_WIDTH_GAIN_M, 0.0, MAX_WIDTH_GAIN_M),
    std::clamp(
      raw_height_shape / RAW_HEIGHT_SHAPE_MAX_MM * MAX_HEIGHT_LOSS_M, 0.0, MAX_HEIGHT_LOSS_M)};
}

void PnPSolver::get_yaw_corrected_object_points(
  std::vector<cv::Point3f> & world_points, double armor_yaw_rad) noexcept
{
  if (world_points.size() < 4 || std::abs(armor_yaw_rad) < 1e-9) {
    return;
  }

  const auto correction = calculate_yaw_object_point_correction(armor_yaw_rad);
  const float width_gain = static_cast<float>(correction.width_gain_m);
  const float half_height_loss = static_cast<float>(correction.height_loss_m * 0.5);

  if (armor_yaw_rad < 0.0) {
    world_points[0].y += width_gain;
    world_points[3].y += width_gain;
    world_points[0].z -= half_height_loss;
    world_points[3].z += half_height_loss;
  } else {
    world_points[1].y -= width_gain;
    world_points[2].y -= width_gain;
    world_points[1].z -= half_height_loss;
    world_points[2].z += half_height_loss;
  }
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point) const noexcept
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

double PnPSolver::calculateReprojectionError(
  const std::vector<cv::Point2f> & image_points, const cv::Mat & rvec, const cv::Mat & tvec,
  const std::string & coord_frame_name)
{
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto & object_points = object_points_map_.at(coord_frame_name);
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

double PnPSolver::calculateYawCorrectedReprojectionError(
  const std::vector<cv::Point2f> & image_points, const cv::Mat & rvec, const cv::Mat & tvec,
  const std::string & coord_frame_name, double armor_yaw_rad)
{
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto & object_points = object_points_map_.at(coord_frame_name);
    std::vector<cv::Point3f> modified_object_points = object_points;
    get_yaw_corrected_object_points(modified_object_points, armor_yaw_rad);
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      modified_object_points, rvec, tvec, camera_matrix_, distortion_coefficients_,
      reprojected_points);
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
  const std::string & coord_frame_name)
{
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto & object_points = object_points_map_.at(coord_frame_name);
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

double PnPSolver::calculateYawCorrectedReprojectionError(
  const std::vector<cv::Point2f> & image_points, const cv::Vec3d & rvec, const cv::Vec3d & tvec,
  const std::string & coord_frame_name, double armor_yaw_rad)
{
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto & object_points = object_points_map_.at(coord_frame_name);
    std::vector<cv::Point3f> modified_object_points = object_points;
    get_yaw_corrected_object_points(modified_object_points, armor_yaw_rad);
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      modified_object_points, rvec, tvec, camera_matrix_, distortion_coefficients_,
      reprojected_points);
    double error = 0;
    for (size_t i = 0; i < image_points.size(); ++i) {
      error += cv::norm(image_points[i] - reprojected_points[i]);
    }
    return error;
  } else {
    return 0;
  }
}

std::vector<cv::Point2f> PnPSolver::getReprojectionPoint(
  const cv::Vec3d & rvec, const cv::Vec3d & tvec, const std::string & coord_frame_name)
{
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto & object_points = object_points_map_.at(coord_frame_name);
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      object_points, rvec, tvec, camera_matrix_, distortion_coefficients_, reprojected_points);
    return reprojected_points;
  }
  return std::vector<cv::Point2f>();
}

std::vector<cv::Point2f> PnPSolver::getYawCorrectedReprojectionPoint(
  const cv::Vec3d & rvec, const cv::Vec3d & tvec, const std::string & coord_frame_name,
  double armor_yaw_rad)
{
  if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
    const auto & object_points = object_points_map_.at(coord_frame_name);
    std::vector<cv::Point3f> modified_object_points = object_points;
    get_yaw_corrected_object_points(modified_object_points, armor_yaw_rad);
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      modified_object_points, rvec, tvec, camera_matrix_, distortion_coefficients_,
      reprojected_points);
    return reprojected_points;
  }
  return std::vector<cv::Point2f>();
}
}  // namespace rm_tools
