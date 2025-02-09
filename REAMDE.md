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