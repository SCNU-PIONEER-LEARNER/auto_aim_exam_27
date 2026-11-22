#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

#include "armor_detector/armor_pose_estimator.hpp"

#include <gtest/gtest.h>
#include <opencv2/core/eigen.hpp>

namespace
{
using rm_auto_aim::Armor;
using rm_auto_aim::ArmorName;
using rm_auto_aim::ArmorPoseEstimator;

Armor makeArmor(ArmorName name)
{
  std::vector<cv::Point2f> points{
    {0.0F, 0.0F}, {1.0F, 0.0F}, {1.0F, 1.0F}, {0.0F, 1.0F}};
  Armor armor(0, 0, 1.0F, cv::Rect{}, points);
  armor.name = name;
  return armor;
}

sensor_msgs::msg::CameraInfo::SharedPtr makeCameraInfo()
{
  auto camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->k = {1000.0, 0.0, 320.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 1.0};
  camera_info->d = {0.0, 0.0, 0.0, 0.0, 0.0};
  return camera_info;
}

Eigen::Matrix3d makeWorldArmorRotation(double yaw, double pitch)
{
  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);
  const double sin_pitch = std::sin(pitch);
  const double cos_pitch = std::cos(pitch);

  Eigen::Matrix3d rotation;
  rotation << cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch, sin_yaw * cos_pitch, cos_yaw,
    sin_yaw * sin_pitch, -sin_pitch, 0.0, cos_pitch;
  return rotation;
}

Armor makeProjectedArmor(ArmorName name, const Eigen::Matrix3d & rotation)
{
  cv::Mat rotation_cv;
  cv::eigen2cv(rotation, rotation_cv);

  cv::Vec3d rvec;
  cv::Rodrigues(rotation_cv, rvec);
  const cv::Vec3d tvec(0.0, 0.0, 3.0);

  const auto object_points = Armor::buildObjectPoints<cv::Point3f>(
    rm_auto_aim::SMALL_ARMOR_WIDTH, rm_auto_aim::SMALL_ARMOR_HEIGHT);

  std::vector<cv::Point2f> image_points;
  const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) << 1000.0, 0.0, 320.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 1.0);
  const cv::Mat distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
  cv::projectPoints(
    object_points, rvec, tvec, camera_matrix, distortion_coefficients, image_points);

  Armor armor(0, 0, 1.0F, cv::Rect{}, image_points);
  armor.name = name;
  armor.type = rm_auto_aim::ArmorType::small;
  armor.center = std::accumulate(
                   image_points.begin(), image_points.end(), cv::Point2f{},
                   [](const cv::Point2f & sum, const cv::Point2f & point) {
                     return sum + point;
                   }) *
                 (1.0F / static_cast<float>(image_points.size()));
  return armor;
}
}  // namespace

TEST(ArmorPoseEstimatorYawPair, UsesArmorCountToCalculateAdjacentYawSpacing)
{
  const Armor normal = makeArmor(ArmorName::one);
  EXPECT_EQ(ArmorPoseEstimator::getArmorNum(normal), 4);
  EXPECT_NEAR(ArmorPoseEstimator::getAngleBetweenArmors(normal), M_PI / 2.0, 1e-12);

  const Armor outpost = makeArmor(ArmorName::outpost);
  EXPECT_EQ(ArmorPoseEstimator::getArmorNum(outpost), 3);
  EXPECT_NEAR(ArmorPoseEstimator::getAngleBetweenArmors(outpost), 2.0 * M_PI / 3.0, 1e-12);
}

TEST(ArmorPoseEstimatorYawPair, CalculatesAdjacentArmorYawWithSignAndWrapping)
{
  EXPECT_NEAR(rm_auto_aim::calculateAdjacentArmorYaw(0.2, M_PI / 2.0, 1), 0.2 + M_PI / 2.0, 1e-12);
  EXPECT_NEAR(rm_auto_aim::calculateAdjacentArmorYaw(0.2, M_PI / 2.0, -1), 0.2 - M_PI / 2.0, 1e-12);
  EXPECT_NEAR(
    rm_auto_aim::calculateAdjacentArmorYaw(M_PI - 0.1, M_PI / 2.0, 1),
    -M_PI / 2.0 - 0.1, 1e-12);
}

TEST(ArmorPoseEstimatorYaw, LimitsRelativeYawDegreesBeforePublishing)
{
  EXPECT_NEAR(ArmorPoseEstimator::limitRelativeYawDegrees(358.0), -2.0, 1e-12);
  EXPECT_NEAR(ArmorPoseEstimator::limitRelativeYawDegrees(-358.0), 2.0, 1e-12);
  EXPECT_NEAR(ArmorPoseEstimator::limitRelativeYawDegrees(181.0), -179.0, 1e-12);
  EXPECT_NEAR(ArmorPoseEstimator::limitRelativeYawDegrees(-180.0), 180.0, 1e-12);
}

TEST(ArmorPoseEstimatorYawCost, UnifiedEdgeCostShiftsFromAngleToPixelByYaw)
{
  constexpr double pixel_distance = 0.4;
  constexpr double angle_distance = 0.2;

  EXPECT_NEAR(
    rm_auto_aim::calculateUnifiedYawEdgeCost(pixel_distance, angle_distance, 0.0),
    angle_distance * std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(
    rm_auto_aim::calculateUnifiedYawEdgeCost(pixel_distance, angle_distance, M_PI / 2.0),
    pixel_distance, 1e-12);

  const double mid_cost =
    rm_auto_aim::calculateUnifiedYawEdgeCost(pixel_distance, angle_distance, M_PI / 4.0);
  EXPECT_GT(mid_cost, angle_distance);
  EXPECT_LT(mid_cost, pixel_distance);
}

TEST(ArmorPoseEstimatorOutpostFilter, DropsStaticOutpostBeforeFixedPitchYawOptimization)
{
  ArmorPoseEstimator estimator(makeCameraInfo());
  estimator.is_optimize_yaw = true;
  estimator.is_use_plus_pnp = false;
  estimator.is_use_ternary = true;
  estimator.is_show_cost = false;

  std::vector<Armor> armors{
    makeProjectedArmor(ArmorName::outpost, makeWorldArmorRotation(0.0, 75.0 * M_PI / 180.0))};

  const auto armor_messages = estimator.extractArmorPoses(armors, Eigen::Matrix3d::Identity());

  EXPECT_TRUE(armor_messages.empty());
}

TEST(ArmorPoseEstimatorOutpostFilter, KeepsRotatingOutpostPitch)
{
  ArmorPoseEstimator estimator(makeCameraInfo());
  estimator.is_optimize_yaw = true;
  estimator.is_use_plus_pnp = false;
  estimator.is_use_ternary = true;
  estimator.is_show_cost = false;

  std::vector<Armor> armors{
    makeProjectedArmor(ArmorName::outpost, makeWorldArmorRotation(0.0, -15.0 * M_PI / 180.0))};

  const auto armor_messages = estimator.extractArmorPoses(armors, Eigen::Matrix3d::Identity());

  EXPECT_EQ(armor_messages.size(), 1u);
}

TEST(ArmorPoseEstimatorOutpostFilter, DoesNotFilterNormalArmorByStaticOutpostPitch)
{
  ArmorPoseEstimator estimator(makeCameraInfo());
  estimator.is_optimize_yaw = true;
  estimator.is_use_plus_pnp = false;
  estimator.is_use_ternary = true;
  estimator.is_show_cost = false;

  std::vector<Armor> armors{
    makeProjectedArmor(ArmorName::one, makeWorldArmorRotation(0.0, 75.0 * M_PI / 180.0))};

  const auto armor_messages = estimator.extractArmorPoses(armors, Eigen::Matrix3d::Identity());

  EXPECT_EQ(armor_messages.size(), 1u);
}
