#pragma once

namespace rm_tools
{

constexpr bool visualization_enabled(const bool is_play) noexcept
{
  return !is_play;
}

constexpr bool debug_enabled(const bool configured_debug, const bool is_play) noexcept
{
  return configured_debug && visualization_enabled(is_play);
}

}  // namespace rm_tools
