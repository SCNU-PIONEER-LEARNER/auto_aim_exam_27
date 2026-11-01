#include "rm_tracker/test/armor_overlay.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <cstddef>
#include <string>

namespace rm_auto_aim
{
namespace
{
constexpr float kSmallArmorWidth = 0.135F;
constexpr float kLargeArmorWidth = 0.23F;
constexpr float kArmorHeight = 0.055F;
constexpr double kRegularArmorPitch = 0.2618;
constexpr double kOutpostArmorPitch = -0.2618;

bool isLargeArmorId(const std::string & id)
{
  return id == "1" || id == "one" || id == "hero";
}

bool isOutpostId(const std::string & id)
{
  return id == "6" || id == "outpost";
}

float armorWidthForTargetId(const std::string & id)
{
  return isLargeArmorId(id) ? kLargeArmorWidth : kSmallArmorWidth;
}

double armorRadiusForIndex(
  const rm_interfaces::msg::Target & target_msg, std::size_t index)
{
  if (target_msg.armors_num == 4) {
    if (index == 1) {
      return target_msg.radius + target_msg.dr1;
    }
    if (index == 2) {
      return target_msg.radius + target_msg.dr2;
    }
    if (index == 3) {
      return target_msg.radius + target_msg.dr3;
    }
  }
  return target_msg.radius;
}

double armorZForIndex(
  const rm_interfaces::msg::Target & target_msg, std::size_t index)
{
  if (target_msg.armors_num == 4) {
    if (index == 1) {
      return target_msg.position.z + target_msg.dz1;
    }
    if (index == 2) {
      return target_msg.position.z + target_msg.dz2;
    }
    if (index == 3) {
      return target_msg.position.z + target_msg.dz3;
    }
  } else if (target_msg.armors_num == 3) {
    if (index == 1) {
      return target_msg.position.z + target_msg.dz1;
    }
    if (index == 2) {
      return target_msg.position.z + target_msg.dz2;
    }
  }
  return target_msg.position.z;
}
}  // namespace

rm_interfaces::msg::ArmorOverlay buildArmorOverlay(
  const rm_interfaces::msg::Target & target_msg)
{
  rm_interfaces::msg::ArmorOverlay overlay_msg;
  overlay_msg.header = target_msg.header;
  overlay_msg.tracking = target_msg.tracking;

  if (!target_msg.tracking || target_msg.armors_num <= 0) {
    overlay_msg.tracking = false;
    return overlay_msg;
  }

  const auto armor_count = static_cast<std::size_t>(target_msg.armors_num);
  const double yaw_step = 2.0 * M_PI / static_cast<double>(armor_count);
  const float armor_width = armorWidthForTargetId(target_msg.id);
  const double armor_pitch =
    isOutpostId(target_msg.id) ? kOutpostArmorPitch : kRegularArmorPitch;

  overlay_msg.armor_poses.reserve(armor_count);
  overlay_msg.armor_widths.reserve(armor_count);
  overlay_msg.armor_heights.reserve(armor_count);

  for (std::size_t i = 0; i < armor_count; ++i) {
    const double armor_yaw = target_msg.yaw + static_cast<double>(i) * yaw_step;
    const double armor_radius = armorRadiusForIndex(target_msg, i);

    geometry_msgs::msg::Pose armor_pose;
    armor_pose.position.x =
      target_msg.position.x - armor_radius * std::cos(armor_yaw);
    armor_pose.position.y =
      target_msg.position.y - armor_radius * std::sin(armor_yaw);
    armor_pose.position.z = armorZForIndex(target_msg, i);

    tf2::Quaternion q;
    q.setRPY(0.0, armor_pitch, armor_yaw);
    armor_pose.orientation = tf2::toMsg(q);

    overlay_msg.armor_poses.emplace_back(armor_pose);
    overlay_msg.armor_widths.emplace_back(armor_width);
    overlay_msg.armor_heights.emplace_back(kArmorHeight);
  }

  return overlay_msg;
}

}  // namespace rm_auto_aim
