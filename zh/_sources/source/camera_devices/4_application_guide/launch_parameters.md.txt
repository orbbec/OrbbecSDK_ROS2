# 启动参数

> 如果您不确定如何设置参数，可以连接orbbec相机并打开 [OrbbecViewer](https://github.com/orbbec/OrbbecSDK/releases)。

以下是可用的启动参数：

### 核心与数据流配置

*   **`camera_name`**
    *   启动节点的命名空间。
*   **`serial_number`**
    *   相机的序列号。当使用多个相机时需要此参数。
*   **`usb_port`**
    *   相机的USB端口。当使用多个相机时需要此参数。
*   **`device_num`**
    *   设备数量。如果需要多个相机，必须填写此参数。
*   **`device_preset`**
    *   默认值为 `Default`。可以使用下面命令查看可设置模式
    ```bash
    ros2 run orbbec_camera list_camera_profile_mode_node
    ```
*   **`[color|depth|left_ir|right_ir|ir]_[width|height|fps|format]`**
    *   传感器流的分辨率和帧率。
*   **`[color|depth|left_ir|right_ir|ir]_rotation`**
    *   设置流图像旋转。
    *   可能的值为 `0`、`90`、`180`、`270`。
*   **`[color|depth|left_ir|right_ir|ir]_flip`**
    *   启用流图像翻转。
*   **`[color|depth|left_ir|right_ir|ir]_mirror`**
    *   启用流图像镜像。
*   **`enable_point_cloud`**
    *   启用点云。
*   **`enable_colored_point_cloud`**
    *   启用RGB点云。
*   **`cloud_frame_id`**
    *   修改ros消息中的 `frame_id` 名称。
*   **`ordered_pc`**
    *   启用无效点云过滤。
*   **`point_cloud_qos`、`[stream]_qos`、`[stream]_camera_info_qos`**
    *   ROS 2消息服务质量（QoS）设置。可能的值为 `SYSTEM_DEFAULT`、`DEFAULT`、`PARAMETER_EVENTS`、`SERVICES_DEFAULT`、`PARAMETERS`、`SENSOR_DATA`，不区分大小写。这些分别对应 `rmw_qos_profile_system_default`、`rmw_qos_profile_default`、`rmw_qos_profile_parameter_events`、`rmw_qos_profile_services_default`、`rmw_qos_profile_parameters` 和 `SENSOR_DATA`。
*   **`color.image_raw.enable_pub_plugins`**
    *   启用彩色图像传输插件。默认值：`["image_transport/compressed", "image_transport/raw", "image_transport/theora"]`。
*   **`depth.image_raw.enable_pub_plugins`**
    *   启用深度图像传输插件。默认值：`["image_transport/compressedDepth", "image_transport/raw"]`。
*   **`left_ir.image_raw.enable_pub_plugins`**
    *   启用左红外图像传输插件。默认值：`["image_transport/compressed", "image_transport/raw", "image_transport/theora"]`。
*   **`right_ir.image_raw.enable_pub_plugins`**
    *   启用右红外图像传输插件。默认值：`["image_transport/compressed", "image_transport/raw", "image_transport/theora"]`。
*   **`point_cloud_decimation_filter_factor`**
    *   点云下采样因子。范围：`1–8`，`1`表示不下采样，数值越大下采样倍数越大。

### 传感器控制

#### 彩色流
*   **`enable_color_auto_exposure`**
    *   启用彩色自动曝光。
*   **`enable_color_auto_exposure_priority`**
    *   启用彩色自动曝光优先级。
*   **`color_exposure`**
    *   设置彩色曝光。
*   **`color_gain`**
    *   设置彩色增益。
*   **`enable_color_auto_white_balance`**
    *   启用彩色自动白平衡。
*   **`color_white_balance`**
    *   设置彩色白平衡。
*   **`color_ae_max_exposure`**
    *   设置彩色自动曝光的最大曝光值。
*   **`color_brightness`**、**`color_sharpness`**、**`color_gamma`**、**`color_saturation`**、**`color_contrast`**、**`color_hue`**
    *   设置彩色亮度、锐度、伽马、饱和度、对比度和色调。
*   **`color_backlight_compensation`**
    *   启用彩色相机的背光补偿功能。**范围**：`0–6`，**默认值**：`3`。
*   **`color_powerline_freq`**
    *   设置电源线频率。可能的值为 `disable`、`50hz`、`60hz`、`auto`。
*   **`enable_color_decimation_filter`** / **`color_decimation_filter_scale`**
    *   启用彩色抽取滤波器并设置其比例。
*   **`color_ae_roi_[left|right|top|bottom]`**
    *   设置彩色自动曝光ROI。
*   **`color_denoising_level`**
    *   启用Gemini 330系列设备的ISP降噪功能。**范围：** `0–8`，**默认值：** `0`（自动）。


#### 深度流
*   **`enable_depth_auto_exposure_priority`**
    *   启用深度自动曝光优先级。
*   **`mean_intensity_set_point`**
    *   设置深度图像的目标平均强度。例如：`mean_intensity_set_point:=100`。
    > **注意：** 这取代了已弃用的 `depth_brightness`，后者仍支持以保持向后兼容性。
*   **`enable_depth_scale`**
    *   设置D2C后是否启用深度缩放。`true`表示启用，默认为`true`。
*   **`depth_precision`**
    *   深度精度应为 `1mm` 格式。默认值为 `1mm`。
*   **`depth_ae_roi_[left|right|top|bottom]`**
    *   设置深度自动曝光ROI。

#### 红外流
*   **`enable_ir_auto_exposure`**
    *   启用红外自动曝光。
*   **`ir_exposure`** / **`ir_gain`**
    *   设置红外曝光和增益。
*   **`ir_ae_max_exposure`**
    *   设置红外自动曝光的最大曝光值。
*   **`ir_brightness`**
    *   设置红外亮度。

#### 激光 / LDP
*   **`enable_laser`**
    *   启用激光。默认值为 `true`。
*   **`laser_energy_level`**
    *   设置激光能量级别。
*   **`enable_ldp`** / **`ldp_power_level`**
    *   启用LDP并设置其功率级别。

### 设备、同步与高级功能

#### 多相机同步
*   **`sync_mode`**
    *   设置同步模式。默认值为 `standalone`。
*   **`depth_delay_us`** / **`color_delay_us`**
    *   接收捕获命令或触发信号后深度/彩色图像捕获的延迟时间（微秒）。
*   **`trigger2image_delay_us`**
    *   接收捕获命令或触发信号后图像捕获的延迟时间（微秒）。
*   **`trigger_out_delay_us`**
    *   接收捕获命令或触发信号后触发信号输出的延迟时间（微秒）。
*   **`trigger_out_enabled`**
    *   启用触发输出信号。
*   **`software_trigger_enabled`** / **`software_trigger_period`**
    *   启用软件触发输出信号 / 设置软件触发周期（毫秒）。
*   **`frames_per_trigger`**
    *   触发模式下每次触发后每个流的帧数。

> 用于 [多相机同步](../5_advanced_guide/multi_camera/multi_camera_synced.md)。

#### 网络相机
*   **`enumerate_net_device`**
    *   启用自动枚举网络设备。
*   **`net_device_ip`** / **`net_device_port`**
    *   设置网络设备的IP地址和端口（通常为 `8090`）。
*   **`force_ip_enable`**
    *   启用强制IP功能。**默认值：** `false`
*   **`force_ip_mac`**
    *   连接多个相机时的目标设备MAC地址（例如，`"54:14:FD:06:07:DA"`）。您可以使用 `list_devices_node` 查找每个设备的MAC。**默认值：** `""`
*   **`force_ip_address`**
    *   要分配的静态IP地址。**默认值：** `192.168.1.10`
*   **`force_ip_subnet_mask`**
    *   静态IP的子网掩码。**默认值：** `255.255.255.0`
*   **`force_ip_gateway`**
    *   静态IP的网关地址。**默认值：** `192.168.1.1`

> 用于 [网络相机](../5_advanced_guide/configuration/net_camera.md)。

#### 设备特定
*   **`enable_gmsl_trigger`** / **`gmsl_trigger_fps`**
    *   启用gmsl触发输出信号 / 设置gmsl触发fps。用于 [gmsl相机](../5_advanced_guide/multi_camera/gmsl_camera.md)。
*   **`preset_resolution_config`**
    * 摄像头设备的预设分辨率配置。格式: "width,height,ir_decimation_factor,depth_decimation_factor". Example: "1280,720,4,4". 仅在 Gemini435Le 设备上受支持。留空禁用。

#### 视差
*   **`disparity_to_depth_mode`**
    *   `HW`：使用硬件视差到深度转换。`SW`：使用软件视差到深度转换。
*   **`disparity_range_mode`**、**`disparity_search_offset`**、**`disparity_offset_config`**
    *   视差搜索偏移参数。用于 [视差搜索偏移](../5_advanced_guide/configuration/disparity_search_offset.md)。

#### 交错AE模式
*   **`interleave_ae_mode`**
    *   设置 `laser` 或 `hdr` 交错。
*   **`interleave_frame_enable`**、**`interleave_skip_enable`**、**`interleave_skip_index`**
    *   控制交错帧模式的参数。
*   **`[hdr|laser]_index[0|1]_[...]`**
    *   在交错帧模式下，设置hdr或laser交错帧的第0和第1帧参数。
*   *所有交错参数用于 [交错ae模式](../5_advanced_guide/configuration/interleave_ae_mode.md)。*

#### 相机内同步

- **`depth_registration`**
  *   启用深度帧与彩色帧的对齐。当 `enable_colored_point_cloud` 设置为 `true` 时需要此字段。
- **`align_mode`**
  *   要使用的对齐模式。选项为 `HW`（硬件对齐）和 `SW`（软件对齐）。
- **`align_target_stream`**
  *   设置对齐目标流模式。
  *   可能的值为 `COLOR`、`DEPTH`。
  *   `COLOR`：将深度对齐到彩色。
  *   `DEPTH`：将彩色对齐到深度。
- **`intra_camera_sync_reference`**
  - 设置相机内同步的参考点。适用于Gemini 330系列设备，当 `sync_mode` 设置为**软件**或**硬件触发**模式时。**选项：** `Start`、`Middle`、`End`。设置为空时，长基线设备默认End，短基线设备默认Middle。

### 基础与通用参数

#### 固件与后端
*   **`upgrade_firmware`**
    *   输入参数为固件路径。
*   **`preset_firmware_path`**
    *   输入参数为预设固件路径。如果输入多个路径，每个路径需要用 `,` 分隔，最多可输入3个固件路径。
*   **`uvc_backend`**
    *   可选值：`v4l2`、`libuvc`。
*   **`connection_delay`**
    *   重新打开设备的延迟时间（毫秒）。某些设备（如Astra mini）需要较长时间初始化，热插拔时立即重新打开设备可能导致固件崩溃。
*   **`retry_on_usb3_detection_failure`**
    *   如果相机连接到USB 2.0端口且未检测到，系统将尝试重置相机最多三次。使用USB 2.0连接时建议将此参数设置为 `false`，以避免不必要的重置。

#### TF、外参与校准
*   **`publish_tf`** / **`tf_publish_rate`**
    *   启用TF发布并设置其发布速率。
*   **`enable_publish_extrinsic`**
    *   启用外参发布。
*   **`ir_info_url`** / **`color_info_url`**
    *   设置IR/彩色相机信息的URL。
*   **`enable_color_undistortion`**
    *   启用彩色去畸变。

#### 时间同步
*   **`enable_sync_host_time`**
    *   启用主机时间与相机时间的同步。默认值为 `true`。如果使用全局时间，设置为 `false`。
*   **`time_domain`**
    *   选择时间戳类型：`device`、`global` 和 `system`。
*   **`time_sync_period`**

    *   相机时间与主机系统同步的间隔（秒）。
    > **注意**：仅当 **`enable_sync_host_time = true`** 且 **`time_domain = device`** 时需要设置此参数。
*   **`enable_ptp_config`**
    *   启用PTP时间同步。仅适用于Gemini 335Le。需要 `enable_sync_host_time` 设置为 `false`。
*   **`enable_frame_sync`**
    *   启用帧同步。

#### 日志与诊断
*   **`log_level`**
    *   SDK日志级别。默认为 `info`。可选值：`debug`、`info`、`warn`、`error`、`fatal`。
*   **`log_file_name`**
    *   保存的SDK日志文件名。当`log_level`为`debug`时生效。
*   **`diagnostic_period`**
    *   诊断周期（秒）。
*   **`enable_heartbeat`**
    *   启用心跳功能。默认为 `false`。如果为 `true`，相机节点将向固件发送心跳信号。

#### 其他
*   **`config_file_path`**
    *   YAML配置文件的路径。默认为 `""`。如果未指定，将使用启动文件中的默认参数。
*   **`frame_aggregate_mode`**
    *   设置帧聚合输出模式。可选值：`full_frame`、`color_frame`、`ANY`、`disable`。
*   **`enable_d2c_viewer`**
    *   发布D2C叠加图像（仅用于测试）。

### IMU

*   **`enable_accel`** / **`enable_gyro`**
    *   启用加速度计/陀螺仪并输出其信息话题数据。
*   **`enable_sync_output_accel_gyro`**
    *   启用同步 `accel_gyro`，并输出IMU话题实时数据。
*   **`accel_rate`** / **`gyro_rate`**
    *   加速度计/陀螺仪的频率。值范围从 `1.5625hz` 到 `32khz`。
*   **`accel_range`** / **`gyro_range`**
    *   加速度计（`2g`、`4g`、`8g`、`16g`）和陀螺仪（`16dps` 到 `2000dps`）的范围。
*   **`enable_accel_data_correction`** / **`enable_gyro_data_correction`**
    *   启用加速度计/陀螺仪的数据校正。
*   **`linear_accel_cov`** / **`angular_vel_cov`**
    *   线性加速度和角速度的协方差。

### 深度滤波器

*   **`enable_decimation_filter`**
    *   启用深度抽取滤波器。使用 `decimation_filter_scale` 设置。
*   **`enable_hdr_merge`**
    *   启用深度hdr合并滤波器。使用 `hdr_merge_exposure_1` 等设置。
*   **`enable_sequence_id_filter`**
    *   启用深度序列id滤波器。使用 `sequence_id_filter_id` 设置。
*   **`enable_threshold_filter`**
    *   启用深度阈值滤波器。使用 `threshold_filter_max`、`threshold_filter_min` 设置。
*   **`enable_hardware_noise_removal_filter`**
    *   启用深度硬件降噪滤波器。
*   **`enable_noise_removal_filter`**
    *   启用深度软件降噪滤波器。使用 `noise_removal_filter_min_diff` 等设置。
*   **`enable_spatial_filter`**
    *   启用深度空间滤波器。使用 `spatial_filter_alpha` 等设置。
*   **`enable_temporal_filter`**
    *   启用深度时间滤波器。使用 `temporal_filter_diff_threshold` 等设置。
*   **`enable_hole_filling_filter`**
    *   启用深度孔洞填充滤波器。使用 `hole_filling_filter_mode` 设置。
*   **`enable_spatial_fast_filter`**
    *   启用深度空间快速滤波器。使用 `spatial_fast_filter_radius` 设置。
*   **`enable_spatial_moderate_filter`**
    *   启用深度空间中等滤波器。使用 `spatial_moderate_filter_diff_threshold` 等设置。

---

> **_重要_**：请仔细阅读 [此链接](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/) 中有关软件滤波设置的说明。如果不确定，请勿修改这些设置。
