#include "rm_tools/image_projection.hpp"

#include <gtest/gtest.h>

#include <limits>

#include <opencv2/core.hpp>

namespace
{
sensor_msgs::msg::CameraInfo makeCameraInfo()
{
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.width = 640;
  camera_info.height = 480;
  camera_info.k = {100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0};
  camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  return camera_info;
}
}  // namespace

TEST(ImageProjection, BuildsCameraModelFromCameraInfo)
{
  const auto model = rm_tools::makeCameraModel(makeCameraInfo());

  EXPECT_EQ(model.image_size, cv::Size(640, 480));
  EXPECT_NEAR(model.camera_matrix.at<double>(0, 0), 100.0, 1e-12);
  EXPECT_NEAR(model.camera_matrix.at<double>(0, 2), 320.0, 1e-12);
  EXPECT_EQ(model.distortion_coefficients.cols, 5);
}

TEST(ImageProjection, BuildsCameraModelWithEmptyDistortion)
{
  auto camera_info = makeCameraInfo();
  camera_info.d.clear();

  const auto model = rm_tools::makeCameraModel(camera_info);

  EXPECT_EQ(model.distortion_coefficients.cols, 5);
  EXPECT_EQ(cv::countNonZero(model.distortion_coefficients), 0);
}

TEST(ImageProjection, ProjectsForwardCameraPoint)
{
  const auto model = rm_tools::makeCameraModel(makeCameraInfo());

  const auto projected = rm_tools::projectCameraPoint(cv::Point3d{1.0, 2.0, 10.0}, model);

  ASSERT_TRUE(projected.has_value());
  EXPECT_NEAR(projected->x, 330.0, 1e-6);
  EXPECT_NEAR(projected->y, 260.0, 1e-6);
}

TEST(ImageProjection, RejectsInvalidOrBehindCameraPoint)
{
  const auto model = rm_tools::makeCameraModel(makeCameraInfo());

  EXPECT_FALSE(rm_tools::projectCameraPoint(cv::Point3d{1.0, 2.0, -0.1}, model).has_value());
  EXPECT_FALSE(
    rm_tools::projectCameraPoint(cv::Point3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0},
                                 model)
      .has_value());
}

TEST(ImageProjection, BuildsRectangleFaceCornersAroundPose)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 10.0;
  pose.orientation.w = 1.0;

  const auto corners = rm_tools::buildRectangleFaceCorners(pose, 0.20, 0.10);

  ASSERT_EQ(corners.size(), 4U);
  EXPECT_NEAR(corners[0].x, 1.0, 1e-12);
  EXPECT_NEAR(corners[0].y, 2.10, 1e-12);
  EXPECT_NEAR(corners[0].z, 10.05, 1e-12);
  EXPECT_NEAR(corners[2].x, 1.0, 1e-12);
  EXPECT_NEAR(corners[2].y, 1.90, 1e-12);
  EXPECT_NEAR(corners[2].z, 9.95, 1e-12);
}

TEST(ImageProjection, TreatsZeroQuaternionAsIdentity)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 10.0;

  const auto corners = rm_tools::buildRectangleFaceCorners(pose, 0.20, 0.10);

  ASSERT_EQ(corners.size(), 4U);
  EXPECT_NEAR(corners[0].x, 1.0, 1e-12);
  EXPECT_NEAR(corners[0].y, 2.10, 1e-12);
  EXPECT_NEAR(corners[0].z, 10.05, 1e-12);
}

TEST(ImageProjection, DrawsProjectedPathAndPoint)
{
  const auto model = rm_tools::makeCameraModel(makeCameraInfo());
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  const bool drew_path = rm_tools::drawProjectedPath(
    image, {{0.0, 0.0, 10.0}, {1.0, 0.0, 10.0}, {2.0, 0.0, 10.0}}, model,
    cv::Scalar(0, 255, 0), 2);
  const bool drew_point = rm_tools::drawProjectedPoint(
    image, cv::Point3d{0.0, 0.0, 10.0}, model, cv::Scalar(0, 0, 255), 4);

  EXPECT_TRUE(drew_path);
  EXPECT_TRUE(drew_point);
  EXPECT_GT(cv::countNonZero(image.reshape(1)), 0);
  EXPECT_EQ(image.at<cv::Vec3b>(240, 320), cv::Vec3b(0, 0, 255));
}

TEST(ImageProjection, RejectsEmptyDrawingInputs)
{
  const auto model = rm_tools::makeCameraModel(makeCameraInfo());
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::Mat empty_image;

  EXPECT_FALSE(rm_tools::drawProjectedPoint(
    empty_image, cv::Point3d{0.0, 0.0, 10.0}, model, cv::Scalar(0, 0, 255), 4));
  EXPECT_FALSE(rm_tools::drawProjectedPath(image, {}, model, cv::Scalar(0, 255, 0), 2));
  EXPECT_FALSE(rm_tools::drawProjectedPolygon(
    image, {{0.0, 0.0, 10.0}, {1.0, 0.0, 10.0}, {2.0, 0.0, 10.0}}, model,
    cv::Scalar(255, 0, 0), 2));
}

TEST(ImageProjection, BreaksPathAcrossInvalidProjectedPoints)
{
  const auto model = rm_tools::makeCameraModel(makeCameraInfo());
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);

  const bool drew = rm_tools::drawProjectedPath(
    image, {{0.0, 0.0, 10.0}, {0.0, 0.0, -1.0}, {2.0, 0.0, 10.0}}, model,
    cv::Scalar(0, 255, 0), 1);

  EXPECT_TRUE(drew);
  EXPECT_EQ(image.at<cv::Vec3b>(240, 330), cv::Vec3b(0, 0, 0));
}
