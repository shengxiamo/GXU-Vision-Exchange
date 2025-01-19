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
  uint8_t detect_color;  // 0-red 1-blue
  uint8_t task_mode ;     // 0-auto 1-aim 2-buff
  bool reset_tracker ;
  uint8_t is_play ;
  bool change_target ;
  uint8_t reserved ;
  float roll;
  float pitch;
  float yaw;
  float relative_angle; // relative yaw angle of radar and camera
  uint16_t game_time;  // (s) game time [0, 450]
  uint32_t timestamp;  // (ms) board time
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacketArmor
{
  uint8_t header = 0xA5;
  uint8_t state;       // 0-untracking 1-tracking-aim 2-tracking-buff
  uint8_t id;          // aim: 0-outpost 6-guard 7-base
  uint8_t armors_num;  // 2-balance 3-outpost 4-normal
  uint8_t isfire;
  uint8_t back = 0;
  float x;                 // aim: robot-center || buff: rune-center
  float y;                 // aim: robot-center || buff: rune-center
  float z;                 // aim: robot-center || buff: rune-center
  float yaw;               // aim: robot-yaw || buff: rune-theta
  // spd = a*sin(w*t)+b || spd > 0 ==> clockwise
  float vx;  // aim: robot-vx || buff: rune spin speed param - a
  float vy;  // aim: robot-vy || buff: rune spin speed param - b
  float vz;  // aim: robot-vz || buff: rune spin speed param - w
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint32_t cap_timestamp;  // (ms) frame capture time
  uint16_t t_offset;       // (ms) speed t offset
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacketNav
{
  uint8_t header = 0xB5;
  char mode;
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
  uint16_t checksum = 0;
} __attribute__((packed));


inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVectorArmor(const SendPacketArmor & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacketArmor));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacketArmor), packet.begin());
  return packet;
}

inline std::vector<uint8_t> toVectorNav(const SendPacketNav & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacketNav));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacketNav), packet.begin());
  return packet;
}  

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
