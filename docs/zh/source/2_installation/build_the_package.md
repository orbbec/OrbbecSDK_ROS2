### 环境配置

根据官方指南安装 ROS 2：

* [ROS 2 安装指南（Ubuntu）](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

启用 ROS 2 自动补全：

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

创建 `colcon` 工作空间：

```bash
mkdir -p ~/ros2_ws/src
```

### 二进制安装

#### Linux 二进制包安装

查看软件包：

```bash
sudo apt update
apt list | grep orbbec
```

安装 OrbbecSDK ROS2 包：

```bash
sudo apt install ros-humble-orbbec-camera ros-humble-orbbec-description
```

安装完成后，直接使用即可，无需编译。

---

### 从源码构建

#### Linux ROS2 包装器编译

克隆源代码并切换到 `v2-main` 分支：

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout v2-main
```

安装依赖项：

```bash
sudo apt install libgflags-dev nlohmann-json3-dev \
ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs ros-$ROS_DISTRO-xacro \
ros-$ROS_DISTRO-backward-ros libdw-dev libssl-dev
```

构建：

```bash
cd ~/ros2_ws
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```
