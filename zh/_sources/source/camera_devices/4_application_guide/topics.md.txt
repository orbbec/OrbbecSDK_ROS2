# 可用话题

话题按数据流和功能组织。默认情况下，所有话题都发布在 `/camera` 命名空间下，可以通过 `camera_name` 启动参数进行更改。

> **注意：** 特定数据流的话题（例如 `/camera/color/...`）只有在相应的启动参数（例如 `enable_color`）设置为 `true` 时才会发布。

### 图像流

这些话题提供每个启用的相机数据流的原始图像数据和相应的校准信息。对于 `color`、`depth`、`ir`、`left_ir` 和 `right_ir` 数据流，模式是一致的。

*   `/camera/color/image_raw`
    *   彩色流的原始图像数据。
*   `/camera/color/camera_info`
    *   彩色流的相机校准数据和元数据。
*   `/camera/color/metadata`
    *   来自彩色流固件的底层元数据。

*   `/camera/depth/image_raw`
    *   深度流的原始图像数据。
*   `/camera/depth/camera_info`
    *   深度流的相机校准数据和元数据。
*   `/camera/depth/metadata`
    *   来自深度流固件的底层元数据。

*   `/camera/ir/image_raw`
    *   红外（IR）流的原始图像数据。
*   `/camera/ir/camera_info`
    *   IR流的相机校准数据和元数据。
*   `/camera/ir/metadata`
    *   来自IR流固件的底层元数据。

### 点云话题

*   `/camera/depth/points`
    *   从深度流生成的点云数据。
    *   **条件：** 仅在 `enable_point_cloud` 为 `true` 时发布。

*   `/camera/depth_registered/points`
    *   彩色点云数据，其中深度点配准到彩色图像帧。
    *   **条件：** 仅在 `enable_colored_point_cloud` 为 `true` 时发布。

### IMU话题

惯性测量单元（IMU）话题提供加速度计和陀螺仪数据。其行为取决于同步设置。

*   `/camera/accel/sample`
    *   单独的加速度计数据流。
    *   **条件：** 在 `enable_accel` 为 `true` 且 `enable_sync_output_accel_gyro` 为 `false` 时发布。

*   `/camera/gyro/sample`
    *   单独的陀螺仪数据流。
    *   **条件：** 在 `enable_gyro` 为 `true` 且 `enable_sync_output_accel_gyro` 为 `false` 时发布。

*   `/camera/gyro_accel/sample`
    *   包含加速度计和陀螺仪数据的同步数据流（单条消息）。
    *   **条件：** 在 `enable_sync_output_accel_gyro` 为 `true` 时发布。

### 设备状态与诊断

*   `/camera/device_status`
    *   报告相机设备的当前状态。

*   `/camera/depth_filter_status`
    *   报告深度传感器后处理滤波器的状态。

*   `/diagnostics`
    *   发布相机节点的诊断信息。目前包括设备温度。
