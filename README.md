# 项目介绍
基于MoveIT的机械臂控制和视觉兑矿，包括仿真环境和真臂部署，
内含一个硬件接口（RMArmHardwareInterface），实现cotroller与下位机的通信。

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

## 启动
Gazebo仿真:
```bash
ros2 launch launch rm_arm_moveit_config gazebo.launch.py
```
真臂控制：
```bash
ros2 launch rm_arm_moveit_config real_controll.launch.py
```