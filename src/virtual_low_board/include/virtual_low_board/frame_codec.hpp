// Copyright (c) 2026 tomoon
// Licensed under the Apache-2.0 License.

#ifndef VIRTUAL_LOW_BOARD__FRAME_CODEC_HPP_
#define VIRTUAL_LOW_BOARD__FRAME_CODEC_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

#include "virtual_low_board/crc.hpp"
#include "virtual_low_board/packet.hpp"

namespace virtual_low_board
{
static_assert(
  sizeof(rm_serial_driver::ReceivePacket) == 20,
  "ReceivePacket must stay packed and 20 bytes wide");

/**
  * @brief Serialize a ReceivePacket into the on-wire frame.
  * @details The 20 byte frame is copied from the packed struct and its CRC16 is
  *          appended over the first 18 bytes (header included).
  */
inline std::vector<uint8_t> encodeFrame(const rm_serial_driver::ReceivePacket & packet)
{
  std::vector<uint8_t> frame(sizeof(packet));
  std::copy(
    reinterpret_cast<const uint8_t *>(&packet),
    reinterpret_cast<const uint8_t *>(&packet) + sizeof(packet), frame.begin());
  crc16::Append_CRC16_Check_Sum(frame.data(), static_cast<uint32_t>(frame.size()));
  return frame;
}

/**
  * @brief Parse a raw frame into a ReceivePacket.
  * @param[in] frame : exactly sizeof(ReceivePacket) bytes.
  * @param[out] packet : parsed fields, only valid on true.
  * @return false on wrong size, wrong header or bad CRC.
  */
inline bool decodeFrame(const std::vector<uint8_t> & frame, rm_serial_driver::ReceivePacket & packet)
{
  if (frame.size() != sizeof(packet) || frame.front() != 0x5A) {
    return false;
  }
  std::copy(frame.begin(), frame.end(), reinterpret_cast<uint8_t *>(&packet));
  return crc16::Verify_CRC16_Check_Sum(frame.data(), static_cast<uint32_t>(frame.size()));
}

}  // namespace virtual_low_board

#endif  // VIRTUAL_LOW_BOARD__FRAME_CODEC_HPP_
