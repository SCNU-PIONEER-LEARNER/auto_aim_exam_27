#ifndef ARMOR_TRACKER__ARMOR_OVERLAY_HPP_
#define ARMOR_TRACKER__ARMOR_OVERLAY_HPP_

#include <string>

#include "rm_interfaces/msg/armor_overlay.hpp"
#include "rm_interfaces/msg/target.hpp"

namespace rm_auto_aim
{

rm_interfaces::msg::ArmorOverlay buildArmorOverlay(const rm_interfaces::msg::Target & target_msg);

} // namespace rm_auto_aim

#endif // ARMOR_TRACKER__ARMOR_OVERLAY_HPP_
