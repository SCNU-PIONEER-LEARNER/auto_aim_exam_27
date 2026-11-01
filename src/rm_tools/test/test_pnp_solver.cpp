#include "rm_tools/pnp_solver.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

static constexpr double kTolerance = 1e-7;

static std::vector<cv::Point3f> buildArmorPoints(const double width, const double height)
{
  return {
    cv::Point3f(0.0F, width / 2.0F, height / 2.0F),
    cv::Point3f(0.0F, -width / 2.0F, height / 2.0F),
    cv::Point3f(0.0F, -width / 2.0F, -height / 2.0F),
    cv::Point3f(0.0F, width / 2.0F, -height / 2.0F)};
}

static rm_tools::PnPSolver buildTestSolver()
{
  const std::array<double, 9> camera_matrix = {
    1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1.0};
  const std::vector<double> distortion = {0.0, 0.0, 0.0, 0.0, 0.0};
  return rm_tools::PnPSolver(camera_matrix, distortion);
}

TEST(PnPSolverYawCorrection, NegativeYawExtendsPositiveYSideAndShortensVisibleHeight)
{
  auto points = buildArmorPoints(0.135, 0.055);

  rm_tools::PnPSolver::get_yaw_corrected_object_points(points, -80.0 * CV_PI / 180.0);

  EXPECT_NEAR(points[0].y, 0.0675 + 0.00156, kTolerance);
  EXPECT_NEAR(points[3].y, 0.0675 + 0.00156, kTolerance);
  EXPECT_NEAR(points[0].z, 0.0275 - 0.00362, kTolerance);
  EXPECT_NEAR(points[3].z, -0.0275 + 0.00362, kTolerance);

  EXPECT_NEAR(points[1].y, -0.0675, kTolerance);
  EXPECT_NEAR(points[2].y, -0.0675, kTolerance);
  EXPECT_NEAR(points[1].z, 0.0275, kTolerance);
  EXPECT_NEAR(points[2].z, -0.0275, kTolerance);
}

TEST(PnPSolverYawCorrection, PositiveYawExtendsNegativeYSideAndClampsAtMaximumSizeDelta)
{
  auto points = buildArmorPoints(0.135, 0.055);

  rm_tools::PnPSolver::get_yaw_corrected_object_points(points, 100.0 * CV_PI / 180.0);

  EXPECT_NEAR(points[1].y, -0.0675 - 0.00156, kTolerance);
  EXPECT_NEAR(points[2].y, -0.0675 - 0.00156, kTolerance);
  EXPECT_NEAR(points[1].z, 0.0275 - 0.00362, kTolerance);
  EXPECT_NEAR(points[2].z, -0.0275 + 0.00362, kTolerance);

  EXPECT_NEAR(points[0].y, 0.0675, kTolerance);
  EXPECT_NEAR(points[3].y, 0.0675, kTolerance);
  EXPECT_NEAR(points[0].z, 0.0275, kTolerance);
  EXPECT_NEAR(points[3].z, -0.0275, kTolerance);
}

TEST(PnPSolverYawCorrection, InterpolatesCorrectionShapeBeforeApplyingDimensionLimits)
{
  auto correction = rm_tools::PnPSolver::calculate_yaw_object_point_correction(
    40.0 * CV_PI / 180.0);

  EXPECT_NEAR(correction.width_gain_m, 0.00156 * 4.702 / 9.902, kTolerance);
  EXPECT_NEAR(correction.height_loss_m, 0.00724 * 3.958 / 6.065, kTolerance);
}

TEST(PnPSolverReprojection, UsesStoredObjectPointsForReprojection)
{
  auto solver = buildTestSolver();
  const auto object_points = buildArmorPoints(0.1, 0.04);
  solver.setObjectPoints("armor", object_points);

  const cv::Vec3d rvec(0.0, 0.0, 0.0);
  const cv::Vec3d tvec(0.0, 0.0, 1.0);
  const auto reprojected = solver.getReprojectionPoint(rvec, tvec, "armor");

  ASSERT_EQ(reprojected.size(), 4U);
  std::vector<cv::Point2f> expected;
  cv::projectPoints(
    object_points, rvec, tvec, solver.camera_matrix_, solver.distortion_coefficients_, expected);

  EXPECT_NEAR(reprojected[0].x, expected[0].x, 1e-6);
  EXPECT_NEAR(reprojected[0].y, expected[0].y, 1e-6);
}

TEST(PnPSolverReprojection, YawCorrectedReprojectionUsesYawModifiedObjectPoints)
{
  auto solver = buildTestSolver();
  const auto object_points = buildArmorPoints(0.135, 0.055);
  solver.setObjectPoints("armor", object_points);

  const cv::Vec3d rvec(0.0, 0.0, 0.0);
  const cv::Vec3d tvec(0.0, 0.0, 1.0);
  const double yaw = 80.0 * CV_PI / 180.0;

  const auto reprojected = solver.getYawCorrectedReprojectionPoint(rvec, tvec, "armor", yaw);

  ASSERT_EQ(reprojected.size(), 4U);
  auto modified_object_points = object_points;
  solver.get_yaw_corrected_object_points(modified_object_points, yaw);

  std::vector<cv::Point2f> expected;
  cv::projectPoints(
    modified_object_points, rvec, tvec, solver.camera_matrix_, solver.distortion_coefficients_,
    expected);

  EXPECT_NEAR(reprojected[1].x, expected[1].x, 1e-6);
  EXPECT_NEAR(reprojected[1].y, expected[1].y, 1e-6);
}

TEST(PnPSolverReprojection, YawCorrectedErrorIsZeroForYawCorrectedProjection)
{
  auto solver = buildTestSolver();
  const auto object_points = buildArmorPoints(0.135, 0.055);
  solver.setObjectPoints("armor", object_points);

  const cv::Vec3d rvec(0.0, 0.0, 0.0);
  const cv::Vec3d tvec(0.0, 0.0, 1.0);
  const double yaw = -80.0 * CV_PI / 180.0;

  auto modified_object_points = object_points;
  solver.get_yaw_corrected_object_points(modified_object_points, yaw);
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(
    modified_object_points, rvec, tvec, solver.camera_matrix_, solver.distortion_coefficients_,
    image_points);

  const double corrected_error =
    solver.calculateYawCorrectedReprojectionError(image_points, rvec, tvec, "armor", yaw);
  const double uncorrected_error =
    solver.calculateReprojectionError(image_points, rvec, tvec, "armor");

  EXPECT_NEAR(corrected_error, 0.0, 1e-6);
  EXPECT_GT(uncorrected_error, 1.0);
}
