#ifndef RM_TOOLS__RUNE_GEOMETRY_HPP_
#define RM_TOOLS__RUNE_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

namespace rm_tools
{

inline constexpr int kRuneBladeCount = 5;
inline constexpr double kRunePi = 3.14159265358979323846;
inline constexpr double kRuneBladeAngleStep = 2.0 * kRunePi / kRuneBladeCount;

inline double runeBladeAngleStep(int blade_count = kRuneBladeCount) noexcept
{
  return blade_count > 0 ? 2.0 * kRunePi / static_cast<double>(blade_count) : 0.0;
}

inline geometry_msgs::msg::Point calculateRuneBladePoint(
  const geometry_msgs::msg::Point & center, double radius, double theta) noexcept
{
  geometry_msgs::msg::Point blade;
  const double dn = std::max(1e-6, std::hypot(center.x, center.y));
  blade.x = center.x + radius * std::sin(theta) * center.y / dn;
  blade.y = center.y - radius * std::sin(theta) * center.x / dn;
  blade.z = center.z + radius * std::cos(theta);
  return blade;
}

inline geometry_msgs::msg::Point rotateRuneBladePoint(
  const geometry_msgs::msg::Point & blade, const geometry_msgs::msg::Point & center,
  int blade_id_delta, int blade_count = kRuneBladeCount) noexcept
{
  if (blade_count <= 0) {
    return blade;
  }

  const double dn = std::hypot(center.x, center.y);
  if (dn <= 1e-6) {
    return blade;
  }

  const double angle = static_cast<double>(blade_id_delta) * runeBladeAngleStep(blade_count);
  const double ux = center.x / dn;
  const double uy = center.y / dn;
  const double cos_a = std::cos(angle);
  const double sin_a = std::sin(angle);

  const double vx = blade.x - center.x;
  const double vy = blade.y - center.y;
  const double vz = blade.z - center.z;
  const double dot = ux * vx + uy * vy;
  const double cross_x = uy * vz;
  const double cross_y = -ux * vz;
  const double cross_z = ux * vy - uy * vx;

  geometry_msgs::msg::Point rotated;
  rotated.x = center.x + vx * cos_a + cross_x * sin_a + ux * dot * (1.0 - cos_a);
  rotated.y = center.y + vy * cos_a + cross_y * sin_a + uy * dot * (1.0 - cos_a);
  rotated.z = center.z + vz * cos_a + cross_z * sin_a;
  return rotated;
}

inline std::vector<geometry_msgs::msg::Point> buildRuneBladePoints(
  const geometry_msgs::msg::Point & center, double radius, double base_theta,
  int blade_count = kRuneBladeCount)
{
  std::vector<geometry_msgs::msg::Point> blades;
  if (blade_count <= 0) {
    return blades;
  }

  blades.reserve(static_cast<std::size_t>(blade_count));
  const double step = runeBladeAngleStep(blade_count);
  for (int blade_id = 0; blade_id < blade_count; ++blade_id) {
    blades.emplace_back(calculateRuneBladePoint(center, radius, base_theta - blade_id * step));
  }
  return blades;
}

}  // namespace rm_tools

#endif  // RM_TOOLS__RUNE_GEOMETRY_HPP_
