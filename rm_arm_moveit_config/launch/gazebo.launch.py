from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable
import os

def generate_launch_description():
    pacakge_path = get_package_share_directory("rm_arm_moveit_config")
    
    # 获取机械臂描述包的路径，用于设置 Gazebo 资源路径
    rm_arm_pkg_path = get_package_share_directory("rm_arm_2025_last")
    # 获取 share 目录 (即 rm_arm_pkg_path 的上一级)
    rm_arm_share_path = os.path.dirname(rm_arm_pkg_path)

    # 设置环境变量，让 Ignition Gazebo 能找到 meshes
    # 将 install/share 目录添加到资源路径
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=rm_arm_share_path
    )

    # Load the robot configuration
    moveit_config = MoveItConfigsBuilder("rm_arm_2025_last", package_name="rm_arm_moveit_config") \
        .robot_description('config/arm.gazebo.urdf.xacro') \
        .robot_description_semantic('config/rm_arm_2025_last.srdf') \
        .to_moveit_configs()

    # 启动Gazebo ignition
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ros_gz_sim") + "/launch/gz_sim.launch.py"]),
            launch_arguments=[('gz_args', 'empty.sdf -r')]
    )

    # 将机械臂添加到gazebo
    robot_to_gazebo_node =   Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'diff_drive_robot',
                   '-z', '0.1'], # 稍微抬高一点避免卡在地里
    )

    # Clock Bridge
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    
    # 发布机械臂状态
    robot_desc_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[moveit_config.robot_description,
                    {'use_sim_time': True},
                    {'publish_frequency': 30.0}],
        output="screen",
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
            {'use_sim_time': True}
        ],
    )

    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # MoveIt2控制节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}], 
    )

    return LaunchDescription(
        [
            set_ign_resource_path, # 添加环境变量设置
            gazebo_node,
            robot_to_gazebo_node,
            clock_bridge_node,
            robot_desc_node,
            rviz_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            move_group_node,
        ]
    )



