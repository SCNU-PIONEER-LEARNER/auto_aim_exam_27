#pragma once

#include <vector>

#include "rm_interfaces/msg/armor.hpp"
#include "rm_interfaces/msg/target.hpp"
namespace rm_auto_aim
{

std::vector<geometry_msgs::msg::Pose> Target2Armors(const rm_interfaces::msg::Target & _target);

}