#include "rm_tools/rune_geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
double distanceToCenter(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & center)
{
  const double dx = point.x - center.x;
  const double dy = point.y - center.y;
  const double dz = point.z - center.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

TEST(RuneGeometry, CalculatesBladePointFromCenterRadiusAndTheta)
{
  geometry_msgs::msg::Point center;
  center.x = 3.0;
  center.y = 0.0;
  center.z = 1.0;

  const auto blade = rm_tools::calculateRuneBladePoint(center, 0.7, 0.0);

  EXPECT_NEAR(blade.x, 3.0, 1e-12);
  EXPECT_NEAR(blade.y, 0.0, 1e-12);
  EXPECT_NEAR(blade.z, 1.7, 1e-12);
}

TEST(RuneGeometry, BuildsFiveBladePointsAtFixedRadius)
{
  geometry_msgs::msg::Point center;
  center.x = 3.0;
  center.y = 0.0;
  center.z = 1.0;

  const auto blades = rm_tools::buildRuneBladePoints(center, 0.7, 0.2);

  ASSERT_EQ(blades.size(), 5U);
  for (const auto & blade : blades) {
    EXPECT_NEAR(distanceToCenter(blade, center), 0.7, 1e-9);
  }
}

TEST(RuneGeometry, RotateBladePointMatchesThetaStep)
{
  geometry_msgs::msg::Point center;
  center.x = 3.0;
  center.y = 0.0;
  center.z = 1.0;
  constexpr double radius = 0.7;
  constexpr double base_theta = 0.3;

  const auto blade_zero = rm_tools::calculateRuneBladePoint(center, radius, base_theta);
  const auto rotated = rm_tools::rotateRuneBladePoint(blade_zero, center, -2);
  const auto expected = rm_tools::calculateRuneBladePoint(
    center, radius, base_theta - 2.0 * rm_tools::kRuneBladeAngleStep);

  EXPECT_NEAR(rotated.x, expected.x, 1e-9);
  EXPECT_NEAR(rotated.y, expected.y, 1e-9);
  EXPECT_NEAR(rotated.z, expected.z, 1e-9);
}

TEST(RuneGeometry, HandlesDegenerateCenterAxis)
{
  geometry_msgs::msg::Point center;
  center.z = 1.0;

  const auto blade = rm_tools::calculateRuneBladePoint(center, 0.7, 0.0);
  const auto rotated = rm_tools::rotateRuneBladePoint(blade, center, 1);

  EXPECT_TRUE(std::isfinite(blade.x));
  EXPECT_TRUE(std::isfinite(blade.y));
  EXPECT_TRUE(std::isfinite(blade.z));
  EXPECT_NEAR(rotated.x, blade.x, 1e-12);
  EXPECT_NEAR(rotated.y, blade.y, 1e-12);
  EXPECT_NEAR(rotated.z, blade.z, 1e-12);
}
