// Copyright (C) 2026 Yang Shengjun
// Licensed under the Apache-2.0 License.

#ifndef RM_ARM_HARDWARE__RM_ARM_HARDWARE_INTERFACE_HPP_
#define RM_ARM_HARDWARE__RM_ARM_HARDWARE_INTERFACE_HPP_

#include <serial_driver/serial_driver.hpp>
#include <hardware_interface/system_interface.hpp>
#include <thread>

#include "rm_arm_hardware/packet.hpp"

namespace rm_arm_hardware
{
class RMArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Lifecycle node interface overrides
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
 
    // System interface overrides
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
    
    // 添加这两个导出接口函数的声明
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
private:
    // Serial port
    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    uint32_t baud_rate;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    std::thread receive_thread_;
    void receiveData();
    
    void reopenPort(); 
    
    rm_serial_driver::ReceivePacket received_packet_;
    rm_serial_driver::SendPacketArm send_packet_;

    // 添加用于存储关节状态和命令的成员变量
    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_; // 新增: 速度状态容器
    std::vector<double> hw_commands_;
};

}  // namespace rm_arm_hardware


#endif  // RM_ARM_HARDWARE__RM_ARM_HARDWARE_INTERFACE_HPP_

