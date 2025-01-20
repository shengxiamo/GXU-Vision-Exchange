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
  uint8_t result;
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
  uint16_t checksum = 0;
} __attribute__((packed));
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
