#ifndef RM_TOOLS__IMAGE_PROJECTION_HPP_
#define RM_TOOLS__IMAGE_PROJECTION_HPP_

#include <optional>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace rm_tools
{

struct CameraModel
{
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;
  cv::Size image_size;
  double min_depth = 1e-6;
};

CameraModel makeCameraModel(const sensor_msgs::msg::CameraInfo & camera_info);

std::optional<cv::Point2f> projectCameraPoint(
  const cv::Point3d & camera_point, const CameraModel & camera_model);

std::vector<cv::Point3d> buildRectangleFaceCorners(
  const geometry_msgs::msg::Pose & pose, double width, double height);

bool drawProjectedPolygon(
  cv::Mat & image, const std::vector<cv::Point3d> & camera_corners,
  const CameraModel & camera_model, const cv::Scalar & color, int thickness);

bool drawProjectedPath(
  cv::Mat & image, const std::vector<cv::Point3d> & camera_points,
  const CameraModel & camera_model, const cv::Scalar & color, int thickness);

bool drawProjectedPoint(
  cv::Mat & image, const cv::Point3d & camera_point, const CameraModel & camera_model,
  const cv::Scalar & color, int radius);

}  // namespace rm_tools

#endif  // RM_TOOLS__IMAGE_PROJECTION_HPP_
