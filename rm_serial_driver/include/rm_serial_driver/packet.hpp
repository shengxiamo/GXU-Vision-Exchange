// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
  uint32_t timestamp;  // (ms) board time
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacketArm
{
  uint8_t header = 0xA5;
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacketArm & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacketArm));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacketArm), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
