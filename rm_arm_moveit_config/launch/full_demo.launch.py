from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    pacakge_path = get_package_share_directory("rm_arm_moveit_config")
    print(pacakge_path)

    # Load the robot configuration
    moveit_config = MoveItConfigsBuilder("rm_arm_2025_last", package_name="rm_arm_moveit_config").to_moveit_configs()

    # 发布机械臂状态
    robot_desc_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # 发布虚拟关节坐标系
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["--frame_id", "world", "--child_frame_id", "base_link", "0", "0", "0", "0", "0", "0"],
    )

    # 启动Rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", pacakge_path + "/config/moveit.rviz"],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # 启动ros2_control的Controller Manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="ros2_control_node",
        parameters=[pacakge_path + "/config/ros2_controllers.yaml"],
        output="screen",
    )

    # 启动机械臂的ros2_control控制器
    controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster","arm_controller",
        ],
    )

    # MoveIt2控制节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            robot_desc_node,
            static_tf_node,
            rviz_node,
            ros2_control_node,
            controller_spawner_node,
            move_group_node,
        ]
    )



