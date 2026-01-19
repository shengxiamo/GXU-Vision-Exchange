# 项目介绍
基于MoveIT和ros2 control的机械臂控制和视觉兑矿（只有前者），包括仿真环境和真臂部署。

## 依赖安装
在使用realsense-ros包之前需要安装realsense2的SDK，具体方法可在这里查看“https://github.com/IntelRealSense/librealsense/releases

以下是总结的命令：
```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

sudo apt-get install librealsense2-dev librealsense2-dkms librealsense2-utils
```
其余依赖均可使用apt安装。

## 环境
Ubuntu 22.04LTS

ROS 2 Humble

Gazebo Fortress(Ignition Gazebo 6)

## 编译
```
colcon build
```
## 启动
Gazebo仿真:
```bash
ros2 launch launch rm_arm_moveit_config gazebo.launch.py
```
真臂控制：
```bash
ros2 launch rm_arm_moveit_config real_controll.launch.py
```

## 关键讲解：
ros2 control将控制器(controller)与硬件接口(hardware_interface)解耦（但不是通过话题进行通信），机械臂控制器可通用，在本项目中为joint_trajectory_controller/JointTrajectoryController。

因此不同场景只需替换相应的硬件接口，体现在rm_arm_moveit_config/config中各种.xacro文件仅有`<hardware>`标签中的`<plugin>`不同。

rm_arm_2025_last.ros2_control.xacro为moveit自动导出，使用的硬件接口mock_components/GenericSystem为ros2_control自带的虚拟硬件接口，用于模拟任何硬件。
```xml
<ros2_control name="${name}" type="system">
            <hardware>
                <!-- By default, set up controllers for simulation. This won't work on real hardware -->
                <plugin>mock_components/GenericSystem</plugin>
            </hardware>
```unch_params_o_pqhftc --params-file /tmp/launch_params_rfb1kuz2'].
<ros2_control name="${name}" type="system">
            <hardware>
                <plugin>ign_ros2_control/IgnitionSystem</plugin>
            </hardware>
```

arm_real.ros2_control.xacro将硬件接口替换成自定义的RMArmHardwareInterface类，实现串口与controler的通信
<ros2_control name="${name}" type="system">
            <hardware>
                <plugin>rm_arm_hardware/RMArmHardwareInterface</plugin>
            </hardware>

RMArmHardwareInterface的定义在rm_arm_hardware中，为了测试正确性注释了部分代码（主要是串口收发）并添加了额外的逻辑，可根据rm_arm_hardware_interface.cpp中的引导修改。

## 小结
这是上赛季遗留的工作，原本因ros2_control文档不全、资料太少，以及个人能力问题早早弃坑，看到交龙的同学的分享才决定重新拾起，但已不适配本赛季需求，所以后续大概率不会再更新视觉伺服、仿真环境搭建相关的代码。

但是其中 Gazebo、MoveIt 与 ros2_control三者之间的联动，也是我摸索了一整年才弄懂的，或能为后来做工程的同学提供一点参考，帮助大家走出最难的第一步。当然也可作为一个较为通用机械臂的驱动框架（仅需做简单简单修改），专注于上层算法的开发。
