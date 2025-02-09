#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "srv_test_trigger",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("srv_test_trigger");
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr init_moveit_client_ = node->create_client<std_srvs::srv::Trigger>("/trigger_service");
  // Next step goes here
  if (!init_moveit_client_->service_is_ready()) {
    RCLCPP_WARN(logger, "Service not ready");
  }
  else {             
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    init_moveit_client_->async_send_request(request);
    RCLCPP_INFO(logger, "Trigger now!");
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

