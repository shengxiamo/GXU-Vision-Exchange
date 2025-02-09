// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/jointstate.hpp>
// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>


namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  void sendArmData(sensor_msgs::msg::JointState msg);

  void reopenPort();

  void setParam(const rclcpp::Parameter & param);


  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;

  // JointState Subscriber
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;

  // JointState Publisher
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  uint8_t back_result;

  typedef message_filters::sync_policies::ApproximateTime<
    auto_aim_interfaces::msg::Target, auto_aim_interfaces::msg::TimeInfo>
    aim_syncpolicy;
  typedef message_filters::Synchronizer<aim_syncpolicy> AimSync;
  std::shared_ptr<AimSync> aim_sync_;





  // For debug usage
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::thread receive_thread_;

  // Task message
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;

  // Time message
  rclcpp::Publisher<auto_aim_interfaces::msg::TimeInfo>::SharedPtr aim_time_info_pub_;


  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr record_controller_pub_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
