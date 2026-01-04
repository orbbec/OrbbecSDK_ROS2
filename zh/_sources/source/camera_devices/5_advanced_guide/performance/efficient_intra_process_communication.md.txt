# 高效的进程内通信：

### 简介

如果我们的ROS2封装器节点与订阅者节点加载在同一进程中，它支持零拷贝通信。这可以减少图像/点云话题的拷贝时间，特别是在大帧分辨率和高FPS的情况下。

您需要启动一个组件容器，并将我们的节点作为组件与其他组件节点一起启动。有关"在单个进程中组合多个节点"的更多详细信息，请参见 [此处](https://docs.ros.org/en/rolling/Tutorials/Composition.html)。

有关高效进程内通信的更多详细信息，请参见 [此处](https://docs.ros.org/en/humble/Tutorials/Intra-Process-Communication.html#efficient-intra-process-communication)。

### 示例

**手动将多个组件加载到同一进程中**

* 启动组件：

  ```bash
  ros2 run rclcpp_components component_container
  ```

* 添加封装器：

  ```bash
  ros2 component load /ComponentManager orbbec_camera orbbec_camera::OBCameraNodeDriver -e use_intra_process_comms:=true
  ```

  以相同方式加载其他组件节点（封装器话题的消费者）。

**使用启动文件**

```bash
ros2 launch orbbec_camera gemini_intra_process_demo_launch.py
```

**限制**

* RCLPY目前不支持节点组件

* 使用 `image_transport` 的压缩图像将被禁用，因为进程内通信不支持此功能
