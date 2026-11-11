// Copyright (c) 2026 tomoon
// Licensed under the Apache-2.0 License.

#ifndef VIRTUAL_LOW_BOARD__PSEUDO_RANDOM_HPP_
#define VIRTUAL_LOW_BOARD__PSEUDO_RANDOM_HPP_

#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string>

#include "virtual_low_board/packet.hpp"

namespace virtual_low_board
{
/// How the pseudo-random stream moves from frame to frame.
enum class DataMode
{
  kRandom,  ///< every field re-drawn per frame; wide coverage, no continuity
  kSmooth,  ///< angles and speed follow a continuous trajectory, events are rare
};

/// Parse the `data_mode` node parameter.
inline DataMode parseDataMode(const std::string & text)
{
  if (text == "random") {
    return DataMode::kRandom;
  }
  if (text == "smooth") {
    return DataMode::kSmooth;
  }
  throw std::invalid_argument{"data_mode must be 'random' or 'smooth', got: " + text};
}

/// Name of a mode, for logging.
inline const char * dataModeName(DataMode mode)
{
  return mode == DataMode::kSmooth ? "smooth" : "random";
}

/**
  * @brief Shape of the pseudo-random uplink stream.
  * @details Simulator and verifier compile the same header, so the same profile
  *          and the same frame index always produce the same frame.
  */
struct FrameProfile
{
  uint32_t seed = 90;
  DataMode mode = DataMode::kRandom;
  double roll_limit = 0.05;        ///< rad
  double pitch_limit = 0.30;       ///< rad
  double bullet_speed_min = 15.0;  ///< m/s
  double bullet_speed_max = 30.0;  ///< m/s
  int edge_percent = 1;            ///< percent chance for reset/change edges (random mode)
};

/// SplitMix64 step, used as a cheap deterministic hash for the smooth trajectory.
inline uint64_t splitmix64(uint64_t value)
{
  value += 0x9E3779B97F4A7C15ULL;
  value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ULL;
  value = (value ^ (value >> 27)) * 0x94D049BB133111EBULL;
  return value ^ (value >> 31);
}

/// Map a 64 bit hash onto [0, 1).
inline double unitUniform(uint64_t value)
{
  return static_cast<double>(splitmix64(value) >> 11) * 0x1.0p-53;
}

/// Independent random frame: no relation to its neighbours, maximal value coverage.
inline rm_serial_driver::ReceivePacket makeRandomFrame(const FrameProfile & profile, uint64_t index)
{
  // Seed per index so frames are independent and can be rebuilt out of order.
  std::seed_seq seed{
    profile.seed, static_cast<uint32_t>(index >> 32), static_cast<uint32_t>(index)};
  std::mt19937 rng(seed);

  auto integer = [&rng](int low, int high) {
    return std::uniform_int_distribution<int>(low, high)(rng);
  };
  auto real = [&rng](double low, double high) {
    return std::uniform_real_distribution<double>(low, high)(rng);
  };

  constexpr double kPi = 3.14159265358979323846;

  rm_serial_driver::ReceivePacket packet;
  packet.detect_color = static_cast<uint8_t>(integer(0, 1));
  packet.task_mode = static_cast<uint8_t>(integer(0, 2));
  packet.reset_tracker = integer(1, 100) <= profile.edge_percent;
  packet.is_play = static_cast<uint8_t>(integer(0, 1));
  packet.change_target = integer(1, 100) <= profile.edge_percent;
  packet.reserved = 0;
  packet.roll = static_cast<float>(real(-profile.roll_limit, profile.roll_limit));
  packet.pitch = static_cast<float>(real(-profile.pitch_limit, profile.pitch_limit));
  packet.yaw = static_cast<float>(real(-kPi, kPi));
  packet.bullet_speed = static_cast<float>(real(profile.bullet_speed_min, profile.bullet_speed_max));
  packet.checksum = 0;
  return packet;
}

/**
  * @brief Smooth frame: every field is a deterministic function of the index.
  * @details Angles are sums of two sinusoids, so they stay continuous from frame
  *          to frame. Discrete fields hold for thousands of frames; command bits
  *          pulse for a single frame at fixed indices. Rates are tuned for the
  *          default 100 Hz.
  */
inline rm_serial_driver::ReceivePacket makeSmoothFrame(const FrameProfile & profile, uint64_t index)
{
  constexpr double kPi = 3.14159265358979323846;
  const double i = static_cast<double>(index);

  // Independent phase per channel, derived from the seed.
  const auto phase = [&profile](uint64_t salt) {
    return 2.0 * kPi * unitUniform(splitmix64(profile.seed) + salt);
  };
  // Constant per block of frames; changes only when the block index changes.
  const auto block = [&profile, index](uint64_t span, uint64_t salt) {
    return splitmix64(profile.seed + (index / span) * 0x9E3779B97F4A7C15ULL + salt);
  };

  rm_serial_driver::ReceivePacket packet;
  packet.detect_color = static_cast<uint8_t>(block(1200, 1) & 1U);
  packet.task_mode = static_cast<uint8_t>(block(800, 2) % 3U);
  packet.reset_tracker = (index % 500) == 137;
  packet.is_play = static_cast<uint8_t>(block(900, 3) & 1U);
  packet.change_target = (index % 700) == 331;
  packet.reserved = 0;
  packet.roll = static_cast<float>(
    profile.roll_limit *
    (0.6 * std::sin(0.006 * i + phase(4)) + 0.4 * std::sin(0.017 * i + phase(5))));
  packet.pitch = static_cast<float>(
    profile.pitch_limit *
    (0.7 * std::sin(0.008 * i + phase(6)) + 0.3 * std::sin(0.021 * i + phase(7))));
  packet.yaw = static_cast<float>(
    kPi * (0.65 * std::sin(0.006 * i + phase(8)) + 0.35 * std::sin(0.015 * i + phase(9))));

  const double middle = 0.5 * (profile.bullet_speed_min + profile.bullet_speed_max);
  const double half = 0.5 * (profile.bullet_speed_max - profile.bullet_speed_min);
  packet.bullet_speed = static_cast<float>(middle + 0.1 * half * std::sin(0.003 * i + phase(10)));
  packet.checksum = 0;
  return packet;
}

/// Rebuild frame `index` from `profile`; every call with the same arguments returns the same frame.
inline rm_serial_driver::ReceivePacket makeFrame(const FrameProfile & profile, uint64_t index)
{
  return profile.mode == DataMode::kSmooth ? makeSmoothFrame(profile, index)
                                           : makeRandomFrame(profile, index);
}

}  // namespace virtual_low_board

#endif  // VIRTUAL_LOW_BOARD__PSEUDO_RANDOM_HPP_
