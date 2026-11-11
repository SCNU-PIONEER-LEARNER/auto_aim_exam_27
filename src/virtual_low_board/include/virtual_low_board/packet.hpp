// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef VIRTUAL_LOW_BOARD__PACKET_HPP_
#define VIRTUAL_LOW_BOARD__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  uint8_t task_mode : 2;     // 0-armor 1-small_buff 2-large-buff
  bool reset_tracker : 1;
  uint8_t is_play : 1;
  bool change_target : 1;
  uint8_t reserved : 2;
  float roll;
  float pitch;
  float yaw;
  float bullet_speed;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t control;
  uint8_t fire;  // 0-false 1-true
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // VIRTUAL_LOW_BOARD__PACKET_HPP_
