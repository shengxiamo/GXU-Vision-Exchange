// Copyright (C) 2026 Yang Shengjun
// Licensed under the Apache-2.0 License.

#include "rm_arm_hardware/crc.hpp"
#include "rm_arm_hardware/packet.hpp"
#include "rm_arm_hardware/rm_arm_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>


namespace rm_arm_hardware
{
hardware_interface::CallbackReturn RMArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{

    info_ = info;

    // 1. 初始化存储状态和命令的向量大小
    hw_states_position_.resize(info_.joints.size(), 0.0);
    hw_states_velocity_.resize(info_.joints.size(), 0.0); // 初始化速度向量
    hw_commands_.resize(info_.joints.size(), 0.0);

    owned_ctx_ = std::make_unique<IoContext>(2);
    serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_);
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    baud_rate = 115200;
    device_name_ = "/dev/ttyUSB0";

    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;
    device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// 2. 实现导出状态接口
std::vector<hardware_interface::StateInterface>
RMArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // 将 info_.joints[i] 的 position 状态绑定到 hw_states_position_[i] 变量的地址
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
      
    // 必须导出 velocity 接口，否则 ros2_control 会报错，因为 urdf 中定义了 velocity state interface
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }
  return state_interfaces;
}

// 3. 实现导出命令接口
std::vector<hardware_interface::CommandInterface>
RMArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // 将 info_.joints[i] 的 position 命令绑定到 hw_commands_[i] 变量的地址
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RMArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    
    // 为测试暂时注释掉串口打开代码
    // try {
    //     serial_driver_->init_port(device_name_, *device_config_);
    //     if (!serial_driver_->port()->is_open()) {
    //       serial_driver_->port()->open();
    //       receive_thread_ = std::thread(&rm_arm_hardware::RMArmHardwareInterface::receiveData, this);
    //     }
    //   } catch (const std::exception & ex) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rm_arm_hardware_interface"), "Failed to open serial port: %s", ex.what());
    //     return hardware_interface::CallbackReturn::ERROR;
    //   }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RMArmHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    // 4. 在 read 中把接收到的 packet 数据更新到 ros2_control 的状态向量中
    if (hw_states_position_.size() >= 6) {
        hw_states_position_[0] = received_packet_.joint1;
        hw_states_position_[1] = received_packet_.joint2;
        hw_states_position_[2] = received_packet_.joint3;
        hw_states_position_[3] = received_packet_.joint4;
        hw_states_position_[4] = received_packet_.joint5;
        hw_states_position_[5] = received_packet_.joint6;

        // 以下代码仅为测试，使用时删除
        std::vector<double> pre_states_potion = hw_states_position_;

        hw_states_position_[0] = hw_commands_[0];
        hw_states_position_[1] = hw_commands_[1];
        hw_states_position_[2] = hw_commands_[2];
        hw_states_position_[3] = hw_commands_[3];
        hw_states_position_[4] = hw_commands_[4];
        hw_states_position_[5] = hw_commands_[5];

        hw_states_velocity_[0] = (hw_commands_[0] - pre_states_potion[0]) / period.seconds();
        hw_states_velocity_[1] = (hw_commands_[1] - pre_states_potion[1]) / period.seconds();
        hw_states_velocity_[2] = (hw_commands_[2] - pre_states_potion[2]) / period.seconds();
        hw_states_velocity_[3] = (hw_commands_[3] - pre_states_potion[3]) / period.seconds();
        hw_states_velocity_[4] = (hw_commands_[4] - pre_states_potion[4]) / period.seconds();
        hw_states_velocity_[5] = (hw_commands_[5] - pre_states_potion[5]) / period.seconds();
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RMArmHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    //为测试暂时注释掉发送数据的代码
    // send_packet_.joint1 = hw_commands_[0];
    // send_packet_.joint2 = hw_commands_[1];
    // send_packet_.joint3 = hw_commands_[2];
    // send_packet_.joint4 = hw_commands_[3];
    // send_packet_.joint5 = hw_commands_[4];
    // send_packet_.joint6 = hw_commands_[5];

    
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&send_packet_), sizeof(send_packet_));

    // 序列化并发送
    std::vector<uint8_t> data = rm_serial_driver::toVector(send_packet_);
    // 为测试暂时注释掉发送数据的代码
    // try {
    //     serial_driver_->port()->send(data);
    // } catch (const std::exception & ex) {
    //     RCLCPP_ERROR(
    //         rclcpp::get_logger("rm_arm_hardware_interface"), "Error while sending data: %s", ex.what());
    //     reopenPort();
    // }
    return hardware_interface::return_type::OK;
}

void RMArmHardwareInterface::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(received_packet_));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        data.resize(sizeof(received_packet_) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        received_packet_ = rm_serial_driver::fromVector(data);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&received_packet_), sizeof(received_packet_));
        if (!crc_ok) {
            RCLCPP_ERROR(rclcpp::get_logger("rm_arm_hardware_interface"), "CRC error!");
        }
      } else {
        RCLCPP_WARN(rclcpp::get_logger("rm_arm_hardware_interface"), "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rm_arm_hardware_interface"), "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMArmHardwareInterface::reopenPort()
{
  RCLCPP_WARN(rclcpp::get_logger("rm_arm_hardware"), "Attempting to reopen port...");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(rclcpp::get_logger("rm_arm_hardware"), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rm_arm_hardware"), "Failed to reopen port: %s", ex.what());
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

}  // namespace rm_arm_hardware
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rm_arm_hardware::RMArmHardwareInterface,
    hardware_interface::SystemInterface)