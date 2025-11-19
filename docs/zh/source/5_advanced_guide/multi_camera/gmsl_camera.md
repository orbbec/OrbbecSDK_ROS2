# GMSL 相机

> 本节介绍如何在 OrbbecSDK_ROS2 中使用 GMSL 相机。目前仅支持 Gemini 335Lg GMSL 设备，其他 GMSL 设备将在不久的将来得到支持。

您可以在 [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples) 中找到示例使用代码。

## 单个 GMSL 相机

GMSL 相机在 OrbbecSDK_ROS2 中的使用与通过 USB 的 Gemini 330 系列相机相同。

```bash
ros2 launch orbbec_camera gemini_330_gmsl.launch.py
```

## 多个 GMSL 相机

要获取 GMSL 相机的 `usb_port`，插入相机并在终端中运行以下命令：

```bash
ros2 run orbbec_camera list_devices_node
```

例如，获得的 gmsl 相机 `usb_port`：`gmsl2-1`

转到 [multi_gmsl_camera.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/gmsl_camera/multi_gmsl_camera.launch.py) 文件并更改 `usb_port`。

```bash
ros2 launch orbbec_camera multi_gmsl_camera.launch.py
```

> 注意：默认情况下，multi_gmsl_camera.launch.py 仅启动 color 和 left_ir。如果您想启动其他传感器，请转到 [camera_secondary_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/camera_secondary_params.yaml) 进行修改。

## 多个 GMSL 相机同步

首先，请查看如何使用 [multi_camera_synced](./multi_camera_synced.md)。

此外，GMSL 多相机同步不需要 Multi-Camera Sync Hub Pro，因此无需设置 `primary` 模式。每个 GMSL 相机都是 `secondary`。

**额外的参数设置**

* `gmsl_trigger_fps`：设置硬件 soc 触发源帧率。
* `enable_gmsl_trigger`：启用硬件 soc 触发。

**运行启动文件**

请参考 [multi_gmsl_camera_synced.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/gmsl_camera/multi_gmsl_camera_synced.launch.py) 中的配置。

```bash
ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py
```

> 注意：默认情况下，multi_gmsl_camera_synced.launch.py 仅启动 color 和 left_ir。如果您想启动其他传感器，请转到 [camera_secondary_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/camera_secondary_params.yaml) 和 [camera_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/camera_params.yaml) 进行修改。

## GMSL 相机的使用限制

GMSL 相机与各种反序列化器芯片（如 MAX9296 和 MAX92716）接口。Orbbec GMSL 相机支持多个流，包括深度、彩色、IR 和 IMU 数据，但存在某些使用限制：

- GMSL 仅支持 V4L2 和 YUYV 格式；不支持 MJPG 格式。RGB 输出源自 YUYV 格式转换。
- Gemini-335Lg 的元数据通过单独的节点提供，而其他型号的元数据嵌入在视频帧中，这对用户保持透明。
- 当使用 Max96712 作为反序列化器芯片时，由于 Max96712 芯片的特性，在 secondary_synced 模式下必须提供多机同步触发信号。否则，在切换数据流时会发生数据流中断。
- 连接在同一个 MAX9296、MAX96712 LinkA/B 或 MAX96712 LinkC/D 上的两个相机有以下限制：
  - 在驱动程序版本 v1.2.02 之前，存在一个限制，即一个相机的 RGB 和另一个相机的右 IR 不能同时流式传输。在驱动程序版本 v1.2.02 之后，限制修改为一个相机的 RGB 和另一个相机的左 IR 不能同时流式传输。
  - 在驱动程序版本 v1.2.02 之前，存在一个限制，即一个相机的 DEPTH 和另一个相机的左 IR 不能同时流式传输。在驱动程序版本 v1.2.02 之后，限制修改为一个相机的 DEPTH 和另一个相机的右 IR 不能同时流式传输。
  - 两个相机的活动流的组合最大数量限制为四个（满足上述两个条件即可确保合规）。

有关更多已知限制，请参考 [Orbbec GMSL 相机的使用限制](https://github.com/orbbec/MIPI_Camera_Platform_Driver/blob/main/doc/Instructions%20for%20Using%20GMSL%20Camera.md)
