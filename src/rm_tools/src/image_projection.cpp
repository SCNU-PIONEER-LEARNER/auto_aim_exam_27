#include "rm_tools/image_projection.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace rm_tools
{

CameraModel makeCameraModel(const sensor_msgs::msg::CameraInfo & camera_info)
{
  CameraModel model;
  model.camera_matrix = (cv::Mat_<double>(3, 3) << camera_info.k[0], camera_info.k[1],
                         camera_info.k[2], camera_info.k[3], camera_info.k[4], camera_info.k[5],
                         camera_info.k[6], camera_info.k[7], camera_info.k[8]);
  if (camera_info.d.empty()) {
    model.distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
  } else {
    model.distortion_coefficients = cv::Mat(camera_info.d, true).reshape(1, 1);
  }
  model.image_size =
    cv::Size(static_cast<int>(camera_info.width), static_cast<int>(camera_info.height));
  return model;
}

std::optional<cv::Point2f> projectCameraPoint(
  const cv::Point3d & camera_point, const CameraModel & camera_model)
{
  if (
    !std::isfinite(camera_point.x) || !std::isfinite(camera_point.y) ||
    !std::isfinite(camera_point.z) || camera_point.z <= camera_model.min_depth) {
    return std::nullopt;
  }

  std::vector<cv::Point3d> object_points{camera_point};
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(
    object_points, cv::Vec3d::zeros(), cv::Vec3d::zeros(), camera_model.camera_matrix,
    camera_model.distortion_coefficients, image_points);

  if (
    image_points.empty() || !std::isfinite(image_points.front().x) ||
    !std::isfinite(image_points.front().y)) {
    return std::nullopt;
  }

  return cv::Point2f(
    static_cast<float>(image_points.front().x), static_cast<float>(image_points.front().y));
}

std::vector<cv::Point3d> buildRectangleFaceCorners(
  const geometry_msgs::msg::Pose & pose, double width, double height)
{
  Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (orientation.norm() <= 1e-12) {
    orientation = Eigen::Quaterniond::Identity();
  } else {
    orientation.normalize();
  }

  const Eigen::Vector3d center(pose.position.x, pose.position.y, pose.position.z);
  const double half_width = width / 2.0;
  const double half_height = height / 2.0;
  const std::array<Eigen::Vector3d, 4> local_corners{
    Eigen::Vector3d{0.0, half_width, half_height},
    Eigen::Vector3d{0.0, -half_width, half_height},
    Eigen::Vector3d{0.0, -half_width, -half_height},
    Eigen::Vector3d{0.0, half_width, -half_height},
  };

  std::vector<cv::Point3d> corners;
  corners.reserve(local_corners.size());
  for (const auto & local_corner : local_corners) {
    const Eigen::Vector3d corner = center + orientation * local_corner;
    corners.emplace_back(corner.x(), corner.y(), corner.z());
  }
  return corners;
}

bool drawProjectedPolygon(
  cv::Mat & image, const std::vector<cv::Point3d> & camera_corners,
  const CameraModel & camera_model, const cv::Scalar & color, int thickness)
{
  if (image.empty() || camera_corners.size() < 4) {
    return false;
  }

  std::vector<cv::Point> image_corners;
  image_corners.reserve(camera_corners.size());
  for (const auto & camera_corner : camera_corners) {
    const auto projected = projectCameraPoint(camera_corner, camera_model);
    if (!projected.has_value()) {
      return false;
    }
    image_corners.emplace_back(cvRound(projected->x), cvRound(projected->y));
  }

  cv::polylines(image, image_corners, true, color, std::max(1, thickness), cv::LINE_AA);
  return true;
}

bool drawProjectedPath(
  cv::Mat & image, const std::vector<cv::Point3d> & camera_points,
  const CameraModel & camera_model, const cv::Scalar & color, int thickness)
{
  if (image.empty() || camera_points.empty()) {
    return false;
  }

  bool drew = false;
  std::optional<cv::Point> previous_point;
  const int safe_thickness = std::max(1, thickness);
  for (const auto & camera_point : camera_points) {
    const auto projected = projectCameraPoint(camera_point, camera_model);
    if (!projected.has_value()) {
      previous_point.reset();
      continue;
    }

    const cv::Point image_point(cvRound(projected->x), cvRound(projected->y));
    cv::circle(image, image_point, safe_thickness, color, -1, cv::LINE_AA);
    if (previous_point.has_value()) {
      cv::line(image, *previous_point, image_point, color, safe_thickness, cv::LINE_AA);
    }
    previous_point = image_point;
    drew = true;
  }

  return drew;
}

bool drawProjectedPoint(
  cv::Mat & image, const cv::Point3d & camera_point, const CameraModel & camera_model,
  const cv::Scalar & color, int radius)
{
  if (image.empty()) {
    return false;
  }

  const auto projected = projectCameraPoint(camera_point, camera_model);
  if (!projected.has_value()) {
    return false;
  }

  cv::circle(
    image, cv::Point(cvRound(projected->x), cvRound(projected->y)), std::max(1, radius), color, -1,
    cv::LINE_AA);
  return true;
}

}  // namespace rm_tools
