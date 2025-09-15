### Build from Source

#### Environment

Install ROS 2 according to the official guide:

* [ROS 2 installation (Ubuntu)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Enable ROS 2 auto-completion:

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

Create a `colcon` workspace:

```bash
mkdir -p ~/ros2_ws/src
```

#### Linux ROS2 Wrapper Compilation

Clone source and checkout `v2-main` branch:

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout v2-main
```

Install dependencies:

```bash
sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev libgoogle-glog0v5 libssl-dev \
ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev
```

Build:

```bash
cd ~/ros2_ws
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```

