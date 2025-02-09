#ifndef ARM_CONTROL_NODE_HPP_
#define ARM_CONTROL_NODE_HPP_

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

namespace rm_arm_control
{

class ArmControlNode : public rclcpp::Node
{
public:
    explicit ArmControlNode(const rclcpp::NodeOptions & options);

    moveit_msgs::msg::CollisionObject boxFaceCreate(geometry_msgs::msg::Pose box_pose, std::string frame_id, std::string object_id);
    geometry_msgs::msg::Pose fullPoseGet(tf2::Vector3 position, tf2::Quaternion rotate);
    void exchangeBoxCreate(const geometry_msgs::msg::Pose & box_front_face_pose,
                            const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_interface_);
    void initMoveit(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                    std_srvs::srv::Trigger::Response::SharedPtr response);
private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_moveit_srv_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

};

} //namespace rm_arm_control

#endif //ARM_CONTROL_NODE_HPP_