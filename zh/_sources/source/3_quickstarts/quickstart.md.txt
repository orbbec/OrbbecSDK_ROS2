## ROS 包快速入门

### 简介

本节提供 Orbbec ROS 2 包装器的快速入门指南。
您将学习如何：

* 启动相机节点。
* 在 **RViz2** 中可视化深度/彩色流。
* 使用 **ROS 2 CLI 工具**与话题和服务交互。

---

### 构建您的第一个相机应用

#### 步骤 1：配置 ROS 2 和工作空间环境

确保已配置 ROS 2 和工作空间环境：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
```

#### 步骤 2：启动相机节点

- 在终端 1 中

```bash
. ./install/setup.bash
ros2 run orbbec_camera list_devices_node #检查相机是否已连接
ros2 launch orbbec_camera gemini_330_series.launch.py # 或其他启动文件，见下表
```

如果您连接了多个相机，可以指定**序列号**：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py serial_number:=<您的相机序列号>
```

#### 步骤 3：在 RViz2 中可视化

启动 RViz2 并加载默认配置：

- 在终端 2 中

```bash
rviz2
```

* 添加一个 **Image** 显示，将话题设置为 `/camera/color/image_raw`。
* 为 `/camera/depth/image_raw` 添加另一个 **Image** 显示。
* 可选：为 `/camera/depth/points` 添加 **PointCloud2** 显示。

现在您应该能在 RViz2 中看到彩色流、深度流和 3D 点云。

---

### 示例功能

节点运行后，尝试一些 ROS 2 CLI 命令：

#### 列出可用的话题/服务/参数

```bash
ros2 topic list
ros2 service list
ros2 param list
```

#### 回显话题

查看深度相机数据：

```bash
ros2 topic echo /camera/depth/camera_info
```

#### 调用服务

例如，获取设备信息：

```bash
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
```

#### 使用 rosbag2 录制

```bash
ros2 bag record /camera/color/image_raw /camera/depth/image_raw
```

