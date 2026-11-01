#include "rm_tracker/test/armor_overlay.hpp"

#include <cmath>

#include <gtest/gtest.h>

#include "rm_tools/math.hpp"
#include "rm_tools/rotation_bridge.hpp"

namespace
{
rm_interfaces::msg::Target makeTrackingTarget()
{
  rm_interfaces::msg::Target target;
  target.header.frame_id = "odom";
  target.tracking = true;
  target.id = "one";
  target.armors_num = 4;
  target.position.x = 1.0;
  target.position.y = 2.0;
  target.position.z = 0.5;
  target.yaw = 0.25;
  target.radius = 0.2;
  target.dr1 = 0.03;
  target.dr2 = -0.01;
  target.dr3 = 0.02;
  target.dz1 = 0.04;
  target.dz2 = -0.02;
  target.dz3 = 0.01;
  return target;
}
}  // namespace

TEST(ArmorOverlay, BuildsArmorFacesFromTrackingTargetState)
{
  const auto target = makeTrackingTarget();

  const auto overlay = rm_auto_aim::buildArmorOverlay(target);

  ASSERT_TRUE(overlay.tracking);
  EXPECT_EQ(overlay.header.frame_id, target.header.frame_id);
  ASSERT_EQ(overlay.armor_poses.size(), 4U);
  ASSERT_EQ(overlay.armor_widths.size(), 4U);
  ASSERT_EQ(overlay.armor_heights.size(), 4U);

  const double yaw_step = 2.0 * M_PI / target.armors_num;
  const double expected_yaw = target.yaw + yaw_step;
  const double expected_radius = target.radius + target.dr1;

  EXPECT_NEAR(
    overlay.armor_poses[1].position.x,
    target.position.x - expected_radius * std::cos(expected_yaw), 1e-12);
  EXPECT_NEAR(
    overlay.armor_poses[1].position.y,
    target.position.y - expected_radius * std::sin(expected_yaw), 1e-12);
  EXPECT_NEAR(overlay.armor_poses[1].position.z, target.position.z + target.dz1, 1e-12);
  EXPECT_NEAR(
    rm_tools::orientationToYaw(overlay.armor_poses[1].orientation),
    rm_tools::limit_rad(expected_yaw), 1e-12);
  EXPECT_FLOAT_EQ(overlay.armor_widths[0], 0.23F);
  EXPECT_FLOAT_EQ(overlay.armor_heights[0], 0.055F);
}

TEST(ArmorOverlay, ReturnsEmptyOverlayWhenTargetIsNotTracking)
{
  auto target = makeTrackingTarget();
  target.tracking = false;

  const auto overlay = rm_auto_aim::buildArmorOverlay(target);

  EXPECT_FALSE(overlay.tracking);
  EXPECT_EQ(overlay.header.frame_id, target.header.frame_id);
  EXPECT_TRUE(overlay.armor_poses.empty());
  EXPECT_TRUE(overlay.armor_widths.empty());
  EXPECT_TRUE(overlay.armor_heights.empty());
}
