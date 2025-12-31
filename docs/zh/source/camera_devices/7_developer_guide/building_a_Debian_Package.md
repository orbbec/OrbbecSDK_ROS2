# 构建 Debian 软件包

## 准备环境

在开始之前，安装所需的工具：

```bash
sudo apt install debhelper fakeroot python3-bloom
```

## 配置 ROS 依赖项

在系统中的 `/etc/ros/rosdep/sources.list.d/00-orbbec.yaml` 添加以下 YAML 文件。确保将 `focal` 替换为您的 Ubuntu 版本的代号，将 `humble` 替换为您的 ROS2 发行版名称：

```yaml
orbbec_camera_msgs:
  ubuntu:
    focal: [ ros-humble-orbbec-camera-msgs ]
```

接下来，创建一个新文件 `/etc/ros/rosdep/sources.list.d/50-orbbec.list` 并添加此行以指定 YAML 文件的路径：

```bash
yaml file:///etc/ros/rosdep/sources.list.d/00-orbbec.yaml
```

更新 rosdep 数据库以反映这些更改：

```bash
rosdep update
```

## 构建软件包

导航到您的工作空间并构建项目：

```bash
cd ~/ros2_ws/
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
cd src/OrbbecSDK_ROS2/
bash .make_deb.sh
```
