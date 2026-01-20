from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rm_arm_2025_last", package_name="rm_arm_moveit_config") \
        .robot_description(file_path="config/arm_real.urdf.xacro") \
        .to_moveit_configs()

    # 1. 启动 ros2_control Node (硬件接口管理器)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            # 移除 robot_description 参数，改用话题订阅
            # moveit_config.robot_description,
            moveit_config.robot_description_semantic, 
            get_package_share_directory("rm_arm_moveit_config") + "/config/ros2_controllers.yaml"
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # 2. 发布机器人状态 (TF)
    robot_desc_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # 3. 启动控制器 (Joint State Broadcaster)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 4. 启动控制器 (Arm Controller - Joint Trajectory Controller)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 5. MoveIt! Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # 6. RViz2 可视化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", get_package_share_directory("rm_arm_moveit_config") + "/config/moveit.rviz"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        output="screen",
    )
    
    # 7. 静态 TF 发布 (World -> Base Link)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    return LaunchDescription([
        ros2_control_node,
        robot_desc_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
        static_tf_node
    ])