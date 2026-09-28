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

**使用多相机共享容器示例**

[multi_camera_shared_container](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/multi_camera_shared_container) 示例创建一个多线程组件容器，并将两个 Gemini 330 系列相机组件加载到该容器中。修改 `multi_camera_shared_container.launch.py` 中两台相机的 `usb_port` 后，运行：

默认端口为 `2-1` 和 `2-2`。可通过 `ros2 run orbbec_camera list_devices_node` 查询本机端口，并为每台相机设置不同的 `camera_name`。

```bash
ros2 launch orbbec_camera multi_camera_shared_container.launch.py
```

顶层启动文件先创建 `shared_orbbec_container`，再为两台相机分别包含示例专用的 `gemini_330_series_shared_container.launch.py`；第二台相机延迟两秒启动。两个相机必须指定相同的容器名称，并在加载组件前确保容器已经运行。

该示例向两个相机 include 传递以下参数：

* `attach_to_shared_component_container=true`：将相机组件加载到已有容器中，而不是新建容器。
* `component_container_name=shared_orbbec_container`：指定目标容器，其值必须与父 launch 文件创建的容器名称一致。
* `use_intra_process_comms=true`：为相机组件启用进程内通信。

**使用单相机进程内通信演示 launch**

```bash
ros2 launch orbbec_camera gemini_intra_process_demo_launch.py
```

**限制**

* RCLPY目前不支持节点组件

* 使用 `image_transport` 的压缩图像将被禁用，因为进程内通信不支持此功能
