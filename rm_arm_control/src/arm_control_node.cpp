#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>


moveit_msgs::msg::CollisionObject boxFaceCreate(geometry_msgs::msg::Pose box_pose, std::string frame_id, std::string object_id)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = object_id;

  // 定义盒状障碍物的大小
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.24;  // 长
  primitive.dimensions[primitive.BOX_Y] = 0.24;  // 宽
  primitive.dimensions[primitive.BOX_Z] = 0.002;  // 高
  
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose); 
  collision_object.operation = collision_object.ADD;  // 添加操作

  return collision_object;
}

geometry_msgs::msg::Pose fullPoseGet(tf2::Vector3 position, tf2::Quaternion rotate)
{
  geometry_msgs::msg::Pose full_pose;
  geometry_msgs::msg::Quaternion quaternion_msg;
  geometry_msgs::msg::Point position_msg;
  position_msg.x = position.x();
  position_msg.y = position.y();
  position_msg.z = position.z();
  tf2::convert(rotate, quaternion_msg);
  full_pose.orientation = quaternion_msg;
  full_pose.position = position_msg;
  return full_pose;
}

void exchangeBoxCreate(const geometry_msgs::msg::Pose & box_front_face_pose, 
                      const moveit::planning_interface::MoveGroupInterface & move_interface)
{
  //box_face
  //0      1      2      3      4     5 
  //front  back   left   right  top   bottom
  auto box_faces = std::vector<moveit_msgs::msg::CollisionObject>();

  //通过front面确定中心点
  //将msg转换为tf2形式，便于计算
  tf2::Quaternion front_face_pose_quaternion;
  tf2::convert(box_front_face_pose.orientation, front_face_pose_quaternion);

  tf2::Vector3 front_face_pose_position = [box_front_face_pose]{
    tf2::Vector3 vector3_point;
    tf2::fromMsg(box_front_face_pose.position, vector3_point);
    return vector3_point;
  }();

  tf2::Matrix3x3 front_face_pose_matrix(front_face_pose_quaternion);    //转换为旋转矩阵以获得三个旋转轴
  tf2::Vector3 front_face_right_axis = front_face_pose_matrix.getColumn(0);     //右轴 
  tf2::Vector3 front_face_upper_axis = front_face_pose_matrix.getColumn(1);     //上轴
  tf2::Vector3 front_face_normal_vector = front_face_pose_matrix.getColumn(2); // 法向量
  
  tf2::Vector3 center_point;
  center_point = front_face_pose_position + front_face_normal_vector * 0.12;

  //left face and right face
  //parallel face have same rotate
  tf2::Vector3 left_face_pose_position = center_point - front_face_right_axis * 0.12;
  tf2::Vector3 right_face_pose_position = center_point + front_face_right_axis * 0.12;
  tf2::Quaternion left_face_pose_quaternion;
  left_face_pose_quaternion.setRotation(front_face_upper_axis, M_PI/2.0);

  //top face and bottom face
  tf2::Vector3 top_face_pose_position = center_point + front_face_upper_axis * 0.12;
  tf2::Vector3 bottom_face_pose_position = center_point - front_face_upper_axis * 0.12;
  tf2::Quaternion top_face_pose_quaternion;
  top_face_pose_quaternion.setRotation(front_face_right_axis, M_PI/2.0);

  //vector3 trans to point
  //tf2 trans to msg
  geometry_msgs::msg::Pose left_face_pose = fullPoseGet(left_face_pose_position, left_face_pose_quaternion);
  geometry_msgs::msg::Pose right_face_pose = fullPoseGet(right_face_pose_position, left_face_pose_quaternion);
  geometry_msgs::msg::Pose top_face_pose = fullPoseGet(top_face_pose_position, top_face_pose_quaternion);
  geometry_msgs::msg::Pose bottom_face_pose = fullPoseGet(bottom_face_pose_position, top_face_pose_quaternion);

  auto const collision_box_front = boxFaceCreate(box_front_face_pose, move_interface.getPlanningFrame(), "box_front_face");
  auto const collision_box_left = boxFaceCreate(left_face_pose, move_interface.getPlanningFrame(), "box_left_face");
  auto const collision_box_right = boxFaceCreate(right_face_pose, move_interface.getPlanningFrame(), "box_right_face");
  auto const collision_box_top = boxFaceCreate(top_face_pose, move_interface.getPlanningFrame(), "box_top_face");
  auto const collision_box_bottom = boxFaceCreate(bottom_face_pose, move_interface.getPlanningFrame(), "box_bottom_face");

  //创建plan接口，添加障碍物
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_box_front);
  planning_scene_interface.applyCollisionObject(collision_box_left);
  planning_scene_interface.applyCollisionObject(collision_box_right);
  planning_scene_interface.applyCollisionObject(collision_box_top);
  planning_scene_interface.applyCollisionObject(collision_box_bottom);
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
             
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // 定义盒状障碍物的位置test
  geometry_msgs::msg::Pose box_test_pose;
  box_test_pose.orientation.w = 1.0;  // 朝向（单位四元数）
  // [&box_test_pose]{
  //   tf2::Quaternion middle_rotate;
  //   middle_rotate.setRPY(1.0, 1.0, 1.0);
  //   tf2::convert(middle_rotate, box_test_pose.orientation);
  // }();
  box_test_pose.position.x = 0;     // x 坐标
  box_test_pose.position.y = 0.3;     // y 坐标
  box_test_pose.position.z = 0.25;    // z 坐标

  //create box
  exchangeBoxCreate(box_test_pose, move_group_interface);

  // Set a target Pose
  auto const target_rpy_pose = []{
    double roll = 0.0;  // 绕x轴旋转
    double pitch = 3.1; // 绕y轴旋转
    double yaw = 0.0;   // 绕z轴旋转
    // 创建tf2::Quaternion对象
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion msg_q = tf2::toMsg(q);
    return msg_q;
  }();
  auto const target_pose = [target_rpy_pose]{
    geometry_msgs::msg::Pose msg;
    //msg.orientation.w = 1.0;
    msg.orientation = target_rpy_pose;
    msg.position.x = 0;
    msg.position.y = 0.3;
    msg.position.z = 0.2;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Next step goes here
  // Create a ROS logger
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}