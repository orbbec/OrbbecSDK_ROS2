# 从 main 分支迁移到开源 v2-main 分支

## **引言**

最初，奥比中光提供了一个**闭源 SDK — Orbbec SDK v1**，它构成了 [OrbbecSDK ROS2 Wrapper main 分支](https://github.com/orbbec/OrbbecSDK_ROS2/tree/main)的基础。虽然 ROS 封装层本身是开源的，但它依赖于闭源的底层 SDK。这种架构限制了灵活性，阻碍了社区驱动的改进。

随着开发者越来越需要透明性、可维护性和更广泛的设备支持，奥比中光发布了全新的开源 v2-main **— Orbbec SDK_v2** ([GitHub 链接](https://github.com/orbbec/OrbbecSDK/tree/v2-main))。基于此 SDK，[OrbbecSDK ROS2 的开源 v2-main 分支](https://github.com/orbbec/OrbbecSDK_ROS2)现在完全开源，提供了更好的可扩展性，并与奥比中光未来的产品路线图保持一致。

本文档介绍了将 ROS 包从 main 分支（基于 SDK v1）迁移到 v2-main 分支（基于 Orbbec SDK_v2）的动机和好处。它重点介绍了**启动文件、参数、话题和服务**的主要差异，并提供迁移指南帮助开发者顺利过渡。

**注意：** 在以下内容中，**main** 指闭源分支，而 **v2-main** 指开源 v2-main 分支。

## **从 main 迁移到 v2-main 的优势**

2024年10月，奥比中光发布了重大更新：**OrbbecSDK ROS2 Wrapper v2**，它完全基于开源的 Orbbec SDK_v2。与传统的 main 分支（SDK v1.x）相比，v2-main 分支（Orbbec SDK_v2.x）提供了更大的灵活性和可扩展性，同时为所有符合 UVC 标准的奥比中光 USB 产品提供全面支持。从 main --> v2-main 的迁移带来以下关键优势：

### **全面的设备支持**

v2-main 分支支持所有符合 UVC 标准的奥比中光 USB 相机，并将成为支持所有新发布设备的主要平台。

### **透明性和可扩展性**

Orbbec SDK_v2 完全开源，允许开发者直接访问底层实现，便于调试、优化和二次开发。相比之下，SDK v1 是闭源的，引入了"黑盒"约束。

### 维护和更新优势

v2-main 分支提供全功能支持，包括新功能开发、性能优化和错误修复。main 分支已进入仅维护模式，只有关键错误可能获得有限更新，不再开发新功能。

### **社区和生态系统支持**

有了开源 SDK，开发者可以直接在 GitHub 或 Gitee 上提交问题和拉取请求，为功能演进做出贡献。这不仅加速了问题解决，还促进了更开放和活跃的奥比中光生态系统。

## main 和 v2-main 分支的比较

### **启动文件差异**

1. 在 v2-main 中，为 Gemini 330 系列添加了新的低功耗启动文件：
   - `gemini_330_series_low_cpu.launch.py`
2. v2-main 引入了对 **Gemini 435Le**、**Gemini 345** 和 **Gemini 345Lg** 相机的支持。
3. 由于 **OrbbecSDK_v2 仅支持 UVC 设备**，v2-main 中支持的相机型号范围比 main 略窄。详细信息见下表。

| **相机**                           | **main**                             | **v2-main**                                        |
| ---------------------------------- | ------------------------------------ | -------------------------------------------------- |
| Gemini 435Le                       | 不支持                               | gemini435_le.launch.py                             |
| Gemini 345                         | 不支持                               | gemini345.launch.py                                |
| Gemini 345Lg                       | 不支持                               | gemini345_lg.launch.py                             |
| Gemini 330 系列                    | gemini_330_series.launch.py          | gemini_330_series.launch.py                        |
| Gemini 330 低CPU                   | -                                    | gemini_330_series_low_cpu.launch.py                |
| Gemini 210                         | gemini210.launch.py                  | gemini210.launch.py                                |
| Gemini 2                           | gemini2.launch.py                    | gemini2.launch.py                                  |
| Gemini 2L                          | gemini2L.launch.py                   | gemini2.launch.py                                  |
| Gemini 2XL                         | gemini2XL.launch.py                  | -                                                  |
| Femto Bolt                         | femto_bolt.launch.py                 | femto_bolt.launch.py                               |
| Femto Mega                         | femto_mega.launch.py                 | femto_mega.launch.py                               |
| Femto                              | femto.launch.py                      | femto.launch.py                                    |
| Astra 2                            | astra2.launch.py                     | astra2.launch.py                                   |
| Astra                              | astra.launch.py                      | astra.launch.py                                    |
| Astra Mini Pro / S Pro             | astra_mini_pro.launch.py ...         | astra.launch.py                                    |
| 多相机（同步）                     | multi_camera_synced.launch.py        | multi_camera_synced.launch.py                      |
| 多相机（通用/通用）                | multi_camera.launch.py               | multi_camera.launch.py 或 orbbec_multicamera.launch.py |
| 单相机通用启动                     | ob_camera.launch.py                  | orbbec_camera.launch.py                            |
| OpenNI 设备（大白、得雅）          | 对应型号独立文件                     | 不支持                                             |

### **参数差异**

**v2-main 中的新参数（main 中不可用）**

| **参数名称**                           | **main** | **v2-main** | **描述**                                       |
| -------------------------------------- | -------- | ----------- | ---------------------------------------------- |
| upgrade_firmware                       | -        | 已添加      | 固件升级路径                                   |
| preset_firmware_path                   | -        | 已添加      | 预设固件文件路径                               |
| load_config_json_file_path             | -        | 已添加      | 加载 JSON 配置                                 |
| export_config_json_file_path           | -        | 已添加      | 导出 JSON 配置                                 |
| uvc_backend                            | -        | 已添加      | libuvc / v4l2 后端选择                         |
| enable_color_auto_exposure_priority    | -        | 已添加      | AE 优先级控制                                  |
| color_flip                             | -        | 已添加      | 彩色图像垂直翻转                               |
| color_mirror                           | -        | 已添加      | 彩色图像水平镜像                               |
| depth_flip                             | -        | 已添加      | 深度图像垂直翻转                               |
| depth_mirror                           | -        | 已添加      | 深度图像水平镜像                               |
| left_ir_flip                           | -        | 已添加      | 左红外垂直翻转                                 |
| left_ir_mirror                         | -        | 已添加      | 左红外水平镜像                                 |
| right_ir_flip                          | -        | 已添加      | 右红外垂直翻转                                 |
| right_ir_mirror                        | -        | 已添加      | 右红外水平镜像                                 |
| enable_left_ir_sequence_id_filter      | -        | 已添加      | 左红外序列 ID 滤波器                           |
| enable_right_ir_sequence_id_filter     | -        | 已添加      | 右红外序列 ID 滤波器                           |
| enable_accel_data_correction           | -        | 已添加      | 加速度计数据校正                               |
| enable_gyro_data_correction            | -        | 已添加      | 陀螺仪数据校正                                 |
| enumerate_net_device                   | -        | 已添加      | 自动枚举网络设备                               |
| net_device_ip                          | -        | 已添加      | 网络设备 IP 地址                               |
| net_device_port                        | -        | 已添加      | 网络设备端口                                   |
| exposure_range_mode                    | -        | 已添加      | 曝光范围模式：default / ultimate / regular     |
| disparity_to_depth_mode                | -        | 已添加      | 硬件视差到深度转换                             |
| ldp_power_level                        | -        | 已添加      | LDP 功率级别                                   |
| time_sync_period                       | -        | 已添加      | 时间同步周期                                   |
| gmsl_trigger_fps                       | -        | 已添加      | GMSL 触发帧率                                  |
| enable_gmsl_trigger                    | -        | 已添加      | GMSL 触发启用                                  |
| disparity_range_mode                   | -        | 已添加      | 视差范围模式                                   |
| disparity_search_offset                | -        | 已添加      | 视差搜索偏移                                   |
| disparity_offset_config                | -        | 已添加      | 视差偏移配置                                   |
| offset_index0                          | -        | 已添加      | 视差偏移索引 0                                 |
| offset_index1                          | -        | 已添加      | 视差偏移索引 1                                 |
| interleave_ae_mode                     | -        | 已添加      | AE 交错模式                                    |
| interleave_frame_enable                | -        | 已添加      | 交错帧启用                                     |
| interleave_skip_enable                 | -        | 已添加      | 跳过红外帧启用                                 |
| interleave_skip_index                  | -        | 已添加      | 跳过红外帧索引                                 |
| hdr_index1_laser_control               | -        | 已添加      | HDR 激光控制参数                               |
| hdr_index1_depth_exposure              | -        | 已添加      | HDR 深度曝光                                   |
| hdr_index1_depth_gain                  | -        | 已添加      | HDR 深度增益                                   |
| hdr_index1_ir_brightness               | -        | 已添加      | HDR 红外亮度                                   |
| hdr_index1_ir_ae_max_exposure          | -        | 已添加      | HDR 红外最大 AE                                |
| hdr_index0_laser_control               | -        | 已添加      | HDR 激光控制参数                               |
| hdr_index0_depth_exposure              | -        | 已添加      | HDR 深度曝光                                   |
| hdr_index0_depth_gain                  | -        | 已添加      | HDR 深度增益                                   |
| hdr_index0_ir_brightness               | -        | 已添加      | HDR 红外亮度                                   |
| hdr_index0_ir_ae_max_exposure          | -        | 已添加      | HDR 红外最大 AE                                |
| laser_index1_laser_control             | -        | 已添加      | 激光交错控制                                   |
| laser_index1_depth_exposure            | -        | 已添加      | 激光深度曝光                                   |
| laser_index1_depth_gain                | -        | 已添加      | 激光深度增益                                   |
| laser_index1_ir_brightness             | -        | 已添加      | 激光红外亮度                                   |
| laser_index1_ir_ae_max_exposure        | -        | 已添加      | 激光红外最大 AE                                |
| laser_index0_laser_control             | -        | 已添加      | 激光交错控制                                   |
| laser_index0_depth_exposure            | -        | 已添加      | 激光深度曝光                                   |
| laser_index0_depth_gain                | -        | 已添加      | 激光深度增益                                   |
| laser_index0_ir_brightness             | -        | 已添加      | 激光红外亮度                                   |
| laser_index0_ir_ae_max_exposure        | -        | 已添加      | 激光红外最大 AE                                |
| software_trigger_enabled               | -        | 已添加      | 软件触发启用                                   |
| enable_ptp_config                      | -        | 已添加      | PTP 配置（Gemini 335Le）                       |
| align_target_stream                    | -        | 已添加      | 对齐目标流                                     |
| spatial_fast_filter_radius             | -        | 已添加      | 快速空间滤波器半径                             |
| spatial_moderate_filter_diff_threshold | -        | 已添加      | 中等空间滤波器差异阈值                         |
| spatial_moderate_filter_magnitude      | -        | 已添加      | 中等空间滤波器幅度                             |
| spatial_moderate_filter_radius         | -        | 已添加      | 中等空间滤波器半径                             |
| color.image_raw.enable_pub_plugins     | -        | 已添加      | 彩色图像传输插件                               |
| depth.image_raw.enable_pub_plugins     | -        | 已添加      | 深度图像传输插件                               |
| left_ir.image_raw.enable_pub_plugins   | -        | 已添加      | 左红外传输插件                                 |
| right_ir.image_raw.enable_pub_plugins  | -        | 已添加      | 右红外传输插件                                 |
| force_ip_enable                        | -        | 已添加      | 强制 IP 功能                                   |
| force_ip_mac                           | -        | 已添加      | 强制 IP MAC 地址                               |
| force_ip_dhcp                          | -        | 已添加      | DHCP 自动分配                                  |
| force_ip_address                       | -        | 已添加      | 强制 IP 静态地址                               |
| force_ip_subnet_mask                   | -        | 已添加      | 强制 IP 子网掩码                               |
| force_ip_gateway                       | -        | 已添加      | 强制 IP 网关                                   |

**已移除参数（仅 main 有，v2-main 中已移除）**

| **参数**                             | **描述**                                                     |
| ------------------------------------ | ------------------------------------------------------------ |
| enable_3d_reconstruction_mode        | 3D 重建模式已弃用                                            |
| enable_hardware_reset                | 硬件重置接口已弃用                                           |
| enable_hardware_noise_removal_filter | 硬件噪声去除滤波器已弃用                                     |
| laser_on_off_mode                    | 旧激光开关接口，由 interleave / laser_index 替换            |
| enable_3d_reconstruction_mode        | 3D 重建模式重复；v2-main 中不再使用                         |
| device_preset                        | 部分逻辑迁移到新固件 / 交错参数                              |
| enable_trigger_out                   | 旧软件触发接口由 software_trigger_enabled 替换              |
| retry_on_usb3_detection_failure      | 可选；v2-main 中逻辑已调整或移除                            |
| enable_color_undistortion            | 旧接口逻辑集成到其他地方；v2-main 中仍存在但使用可能已调整  |

### **话题差异**

**v2-main** 在 main 基础上添加了以下话题：

| **话题**              | **main** | **v2-main** | **描述**                                                     |
| --------------------- | -------- | ----------- | ------------------------------------------------------------ |
| /camera/device_status | -        | 已添加      | 发布设备状态（帧率延迟、设备连接状态等）                     |

### **服务差异**

**v2-main** 在 main 基础上添加了以下服务：

| **服务**                  | **main** | **v2-main** | **描述**                 |
| ------------------------- | -------- | ----------- | ------------------------ |
| get_ptp_config            | -        | 已添加      | 获取 PTP 配置            |
| set_ptp_config            | -        | 已添加      | 设置 PTP 配置            |
| get_streams_enable        | -        | 已添加      | 获取每个流的启用状态     |
| set_streams_enable        | -        | 已添加      | 设置每个流的启用状态     |
| get_user_calib_params     | -        | 已添加      | 获取用户校准参数         |
| set_user_calib_params     | -        | 已添加      | 设置用户校准参数         |
| read_customer_data        | -        | 已添加      | 读取用户存储的自定义数据 |
| write_customer_data       | -        | 已添加      | 写入用户存储的自定义数据 |
| send_software_trigger     | -        | 已添加      | 发送软件触发             |
| set_color_ae_roi          | -        | 已添加      | 设置彩色图像的 AE ROI    |
| set_depth_ae_roi          | -        | 已添加      | 设置深度图像的 AE ROI    |
| set_color_flip            | -        | 已添加      | 设置彩色图像垂直翻转     |
| set_depth_flip            | -        | 已添加      | 设置深度图像垂直翻转     |
| set_color_rotation        | -        | 已添加      | 设置彩色图像旋转         |
| set_depth_rotation        | -        | 已添加      | 设置深度图像旋转         |
| set_left_ir_ae_roi        | -        | 已添加      | 设置左红外的 AE ROI      |
| set_right_ir_ae_roi       | -        | 已添加      | 设置右红外的 AE ROI      |
| set_left_ir_flip          | -        | 已添加      | 左红外垂直翻转           |
| set_right_ir_flip         | -        | 已添加      | 右红外垂直翻转           |
| set_left_ir_rotation      | -        | 已添加      | 左红外旋转               |
| set_right_ir_rotation     | -        | 已添加      | 右红外旋转               |
| set_reset_timestamp       | -        | 已添加      | 重置时间戳               |
| set_sync_hosttime         | -        | 已添加      | 同步主机时间             |
| set_sync_interleaverlaser | -        | 已添加      | 交错激光同步             |
| set_filter                | -        | 已添加      | 设置深度/点云滤波器      |
