# OrbbecSDK ROS2 Wrapper

> [!重要]
>
> 欢迎使用 OrbbecSDK ROS2 Wrapper。在开始使用本版本的 ROS2 封装前，请务必查看以下的 [设备支持列表](#支持的设备)，以确认您的设备是否兼容。

OrbbecSDK ROS2 封装包为 Orbbec 相机与 ROS 2 环境之间提供了无缝集成，支持 ROS2 的 Foxy、Humble 和 Jazzy 发行版。

2024 年 10 月，我们发布了重大更新版本 —— [OrbbecSDK ROS2 Wrapper v2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main)，该版本基于开源的[OrbbecSDK v2](https://github.com/orbbec/OrbbecSDK_v2/releases)，具有更强的灵活性与可扩展性。

此更新确保了对所有符合 UVC 标准的 Orbbec USB 产品的兼容性，但不再支持使用 Orbbec 传统 OpenNI 协议的设备。如果您的设备在支持列表中，我们强烈建议您使用 v2-main 分支。

以下是 main 分支（v1.x）与 v2-main 分支（v2.x）对应的设备支持列表：

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr style="background-color: #1f4e78; color: white; text-align: center;">
      <th>产品系列</th>
      <th>产品</th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/main" style="color: black; text-decoration: none;">main分支</a></th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main" style="color: black; text-decoration: none;">v2-main分支</a></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 336</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 330</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 335L</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 336L</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 330L</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td>不支持</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 335Le</td>
      <td>不支持</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 2 L</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td>推荐用于新设计</td>
      <td>即将支持</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td>
      <td>Femto Bolt</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td>全面维护</td>
      <td>即将支持</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td>有限的维护</td>
      <td>不支持</td>
    </tr>
    <tr>
      <td>Astra Pro Plus</td>
      <td>有限的维护</td>
      <td>不支持</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini Pro</td>
      <td>全面维护</td>
      <td>不支持</td>
    </tr>
  </tbody>
</table>

注意：如果您未在列表中找到您的设备，请联系我们的 FAE 或销售代表获取支持。

**定义说明**:

1. 推荐用于新设计: 我们将提供全面支持，包括新功能、漏洞修复以及性能优化。
2. 全面维护: 我们将提供漏洞修复支持。
3. 有限的维护: 我们仅提供关键性问题的修复支持。
4. 不支持: 本版本中不支持该设备。
5. 即将支持: 我们将在近期添加对该设备的支持。

## 目录

- [OrbbecSDK ROS2 Wrapper v2](#orbbecsdk-ros2-wrapper-v2)
  - [目录](#目录)
  - [安装说明](#安装说明)
  - [入门](#入门)
  - [高效的进程内通信](#高效的进程内通信)
    - [例子](#例子)
      - [手动将多个组件加载到同一进程中](#手动将多个组件加载到同一进程中)
      - [使用launch文件启动](#使用launch文件启动)
    - [限制](#限制)
  - [使用V4L2后端](#使用v4l2后端)
  - [Launch参数](#launch参数)
  - [预设presets](#预设presets)
  - [深度工作模式开关](#深度工作模式开关)
  - [深度 NFOV 和 WFOV 模式的配置](#深度-nfov-和-wfov-模式的配置)
  - [所有可用的摄像机控制服务](#所有可用的摄像机控制服务)
  - [所有可用主题](#所有可用主题)
  - [网络相机设备枚举](#网络相机设备枚举)
  - [多相机](#多相机)
  - [压缩图像](#压缩图像)
  - [使用硬件解码器解码JPEG](#使用硬件解码器解码jpeg)
    - [rockchip and Amlogic](#rockchip-and-amlogic)
    - [Nvidia Jetson](#nvidia-jetson)
  - [检查相机支持哪些配置文件](#检查相机支持哪些配置文件)
  - [构建Debian软件包](#构建debian软件包)
    - [准备环境](#准备环境)
    - [配置ROS依赖项](#配置ros依赖项)
    - [构建包](#构建包)
  - [支持的设备](#支持的设备)
  - [DDS调优](#dds调优)
  - [常见问题](#常见问题)
    - [意外崩溃](#意外崩溃)
    - [没有来自多个摄像机的数据流](#没有来自多个摄像机的数据流)
    - [其他故障排除](#其他故障排除)
    - [为什么有这么多启动文件？](#为什么有这么多启动文件)
  - [其他有用链接](#其他有用链接)
  - [License](#license)

## 安装说明

下载ROS2

- 请根据您所使用的 ROS 2 版本，参考官方安装指南进行安装：[ROS 2 安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

> 如果你的 ROS 2 命令无法自动补全，请将以下两行添加到你的 `.bashrc `或 `.zshrc` 文件中：

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

创建 `colcon` 工作空间

```bash
mkdir -p ~/ros2_ws/src
```

获取OrbbecSDK_ROS2源码

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
```

安装 deb 依赖项

```bash
# assume you have sourced ROS environment, same blow
sudo apt install libgflags-dev nlohmann-json3-dev  \
ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev
```

安装 udev 规则

```bash
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 入门

```bash
cd ~/ros2_ws/
# build release, Default is Debug
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

启动相机节点

- 第一个终端

```bash
. ./install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py # or other launch file, see below table
```

- 第二个终端

```bash
. ./install/setup.bash
rviz2
```

选择您想要显示的topic

- 获取topics / services/ parameters列表 (第3个终端)

```bash
ros2 topic list
ros2 service list
ros2 param list
```

- 获取设备信息

```bash
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
```

- 获取SDK版本号

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'

```

- 获取曝光

```bash
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
```

> 如果你选择的是 `ir` 或 `depth` 模式，请将 `/camera/get_color_exposure` 修改为 `/camera/get_ir_exposure` 或 `/camera/get_depth_exposure`，下文同理。

- 获取增益

```bash
ros2 service call /camera/get_color_gain orbbec_camera_msgs/srv/GetInt32 '{}'
```

- 获取白平衡

```bash
ros2 service call /camera/get_white_balance orbbec_camera_msgs/srv/GetInt32 '{}'
```

- 设置自动曝光

```bash
ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}'
```

* 设置自动曝光ROI区域

> 在 data_param 中，第一个值表示 左侧 `Left` 设置，第二个值表示 右侧 `Right` 设置，第三个值表示 上方 `Top` 设置，第四个值表示 下方 `Bottom` 设置。

```bash
ros2 service call /camera/set_color_ae_roi orbbec_camera_msgs/srv/SetArrays '{data_param: [0,1279,0,719]}'
```

* 设置白平衡

```bash
ros2 service call /camera/set_white_balance orbbec_camera_msgs/srv/SetInt32 '{data: 4600}'
```

- 设置激光开关

```bash
ros2 service call  /camera/set_laser_enable std_srvs/srv/SetBool "{data: true}"
```

- 切换传感器状态

```bash
ros2 service call /camera/toggle_ir std_srvs/srv/SetBool "{data : true}"

```

- 保存点云

```bash
ros2 service call /camera/save_point_cloud std_srvs/srv/Empty "{}"
```

## 高效的进程内通信:

我们的 ROS2 Wrapper 节点在与订阅节点加载到同一进程中时，支持 零拷贝通信。这可以显著减少图像或点云等大数据主题的复制次数，尤其是在高分辨率和高帧率的场景下。

您需要启动一个组件容器（Component Container），并将本节点与其他组件节点一同作为组件方式加载。有关“在同一进程中组合多个节点”的更多详细信息，请参考：[官方文档说明](https://docs.ros.org/en/rolling/Tutorials/Composition.html)

关于高效的进程内通信（Intra-process communication），请参考此文档：[高效的 Intra-Process通信](https://docs.ros.org/en/humble/Tutorials/Intra-Process-Communication.html#efficient-intra-process-communication)。

### 例子

#### 手动将多个组件加载到同一进程中

* 启动 容器（component）:

  ```bash
  ros2 run rclcpp_components component_container
  ```
* 添加组件:

  ```bash
  ros2 component load /ComponentManager orbbec_camera orbbec_camera::OBCameraNodeDriver -e use_intra_process_comms:=true
  ```

  以相同的方式加载其他组件节点 。

#### 使用launch文件启动

```bash
ros2 launch orbbec_camera gemini_intra_process_demo_launch.py
```

### 限制

* Node组件目前不支持 on RCLPY
* 使用 `image_transport`压缩图像话题将被禁止，因为这不支持进程内通信。

## 使用V4L2后端

要为 Gemini2 系列相机启用 V4L2 后端，请按以下步骤操作：

1. Gemini2 系列相机支持 V4L2 后端。
2. 打开 `config/OrbbecSDKConfig_v1.0.xml` 文件。
3. 将导航选项设置为 `LinuxUVCBackend`。
4. 将后端设置更改为 `V4L2`。

注意：默认情况下未启用 V4L2 后端。

## Launch参数

以下是可用的launch启动参数：

- `connection_delay`: 重新打开设备的延迟时间（以毫秒为单位）。
- `enable_point_cloud`: 深度点云开关。
- `enable_colored_point_cloud`: RGB点云开关。
- `cloud_frame_id`:修改ros消息中frame_id名称。
- `ordered_pc`:启动无效点云过滤开关。
- `point_cloud_qos`, `[color|depth|ir]_qos`, `[color|depth|ir]_camera_info_qos`: ROS2消息服务质量(QoS)设置
  可能的值包括： `SYSTEM_DEFAULT`, `DEFAULT`, `PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA` 并且不区分大小写。这些值分别对应： `rmw_qos_profile_system_default`, `rmw_qos_profile_default`, `rmw_qos_profile_parameter_events`, `rmw_qos_profile_services_default`, `rmw_qos_profile_parameters`,和 `SENSOR_DATA`。
- `color_ae_roi_[left|right|top|bottom]`,`depth_ae_roi_[left|right|top|bottom]`:设置Color或Depth的自动曝光ROI区域。
- `enable_d2c_viewer`: 发布depth图像叠加到彩色图像的D2C话题（仅提供测试）。
- `device_num`: 设备数量。
- `color_width`, `color_height`, `color_fps`: 彩色流的分辨率和帧率。
- `ir_width`, `ir_height`, `ir_fps`: 红外流的分辨率和帧速率。
- `depth_width`, `depth_height`, `depth_fps`: 深度流的分辨率和帧速率。
- `enable_color`: 启用彩色流。
- `enable_depth`: 启用深度流。
- `enable_ir`: 启动红外流。
- `depth_registration`: 启用深度帧与颜色帧的对齐。当 `enable_colored_point_cloud` 设置为 `true` 时，此字段为必填项。
- `usb_port`: 相机的 USB 端口。使用多个相机时需要此端口。
- `enable_accel`: 启用加速度计。
- `accel_rate`: 加速度计的频率，可选值 `1.5625hz`, `3.125hz`, `6.25hz`, `12.5hz`, `25hz`, `50hz`, `100hz`, `200hz`, `500hz`, `1khz`, `2khz`, `4khz`, `8khz`, `16khz`, `32khz`。
  具体值取决于当前的相机。
- `accel_range`: 加速度计的范围，可选值为 `2g`, `4g`, `8g`, `16g`. 具体值取决于当前相机。
- `enable_gyro`: 启用陀螺仪。
- `gyro_rate`: 陀螺仪的频率，可选值 `1.5625hz`, `3.125hz`, `6.25hz`, `12.5hz`, `25hz`, `50hz`, `100hz`, `200hz`, `500hz`, `1khz`, `2khz`, `4khz`, `8khz`, `16khz`, `32khz`。
  具体值取决于当前的相机。
- `gyro_range`: 陀螺仪的范围，可选值为 `16dps`, `31dps`, `62dps`, `125dps`, `250dps`, `500dps`, `1000dps`, `2000dps`. 具体值取决于当前相机。
- `enumerate_net_device`: 启用枚举网络设备的功能。true 表示启用，false 表示禁用。
  此功能仅支持 Femto Mega 和 Gemini 2 XL 设备。通过网络访问这些设备时，需要提前配置设备的 IP 地址。启用开关需要设置为 true。
- `depth_filter_config`: 配置深度过滤配置文件的加载路径。默认情况下，深度过滤配置文件位于 /config/depthfilter 目录。仅 Gemini2 支持。
- `depth_precision`: 深度精度应采用 `1mm`格式。默认值为 `1mm`。
- `enable_laser`: 启用激光。默认值为 `true`。
- `device_preset`: 默认值为 `Default`。仅支持 G330 系列。更多信息，请参阅 [G330 文档](https://www.orbbec.com/docs/g330-use-depth-presets/)。请参考下表，根据您的使用情况设置 `device_preset` 值。该值应为[表格](#预设presets) 中列出的预设名称之一。
- `enable_decimation_filter`: 此滤波器有效降低了深度场景的复杂度。滤波器在核尺寸 [2x2] 至 [8x8] 像素上运行。图像尺寸在两个维度上均按比例缩小，以保持纵横比。
- `enable_hdr_merge`: 此滤镜与深度 HDR 功能配合使用。通过合并交替曝光值的连续深度图像，我们可以克服同时获取欠照和过照物体深度值的挑战。
- `enable_sequence_id_filter`: 此过滤器与深度 HDR 函数联合使用，仅输出具有指定序列 ID 的序列。
- `enable_threshold_filter`: 该过滤器保留感兴趣的深度值并省略范围之外的深度值。
- `enable_noise_removal_filter`: 该滤波器可消除簇状斑点噪声，并生成填充程度较低的深度图。
- `enable_spatial_filter`: 此滤波器根据幅值参数的指定执行多次迭代处理，以增强深度数据的平滑度。它还能够填充深度图中的小孔。
- `enable_temporal_filter`: 此滤镜旨在通过基于先前帧操纵每个像素的值来提高深度数据的持久性。该滤镜对数据执行单次传递，在调整深度值的同时更新追踪历史记录。
- `enable_hole_filling_filter`: 此过滤器使用指定的模式填充深度图中的所有孔洞。
- `retry_on_usb3_detection_failure`:如果摄像头连接到 USB 2.0 端口但未被检测到，系统将尝试重置摄像头最多三次。此设置旨在防止 USB 3.0 设备被错误地识别为 USB 2.0。建议在使用 USB 2.0 连接时将此参数设置为 `false`，以避免不必要的重置。
- `tf_publish_rate`: 相机发布动态变换的速率。默认值为 0.0，表示发布静态变换。
- `time_domain`: 帧时间域，字符串类型，可以是 `device`、`global`或 `system`。`device`表示使用相机的硬件时间戳；`system`表示使用PC接收到第一个数据包或帧的时间戳；`global`用于跨多个设备同步时间，将来自不同来源的数据对齐到公共时间基准。
- `enable_sync_host_time`: 启用主机时间与相机时间的同步。默认值为 `true`，如果使用全局时间，请设置为 `false`。某些旧设备可能不支持此功能。
- `config_file_path`: YAML 配置文件的路径。如果未指定配置文件，则将使用启动文件中的默认参数。如果您想使用自定义配置文件，请参阅 `gemini_330_series.launch.py`。
- `enable_heartbeat`启用心跳功能，默认设置为 `false`。如果设置为 `true`，摄像头节点将向固件发送心跳信号；如果需要硬件日志记录，也应设置为 `true`。
- `log_level` : SDK 日志级别，默认值为 `info`，可选值为 `debug`, `info`, `warn`, `error`, `fatal`。
- `enable_color_undistortion`: 启用颜色去畸变功能，默认值为 `false`。请注意，我们的彩色相机畸变程度极小，通常无需进行去畸变设置。
- `color_brightness`: 设置彩色流亮度。
- `ir_brightness`: 设置红外流亮度。
- `color_ae_max_exposure`: 设置彩色流自动曝光最大曝光值。
- `ir_ae_max_exposure`: 设置红外流自动曝光最大曝光值。

**重要提示**：*请仔细阅读[此链接](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/)中关于软件过滤设置的说明。如果您不确定，请勿修改这些设置。*

## 预设presets

| Preset         | 特征                                                                                                            | 推荐用例                                                                          |
| -------------- | --------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| Default        | - 最佳视觉感知 - 准确度、填充率、微小物体等方面的整体表现良好。                                                 | - 通用 - 机器人                                                                  |
| Hand           | - 清除手和手指边缘                                                                                              | - 手势识别                                                                       |
| High Accuracy  | - 高置信度的深度 - 几乎没有噪声的深度值  - 较低的填充率                                                        | - 防撞 - 物体扫描                                                                |
| High Density   | - 更高的填充率 - 更多微小物体  - 可能受到噪声深度值的影响                                                      | - 物体识别 - 拾取和放置 - 前景和背景动画                                         |
| Medium Density | - 填充率和准确度的平衡性能 - 与默认设置相比：填充率较低，边缘质量更好                                           | - 通用且可替代默认                                                               |
| Custom         | - 用户定义预设 - 从上述预设衍生，并经过自定义修改，例如后处理管道的新配置、深度 AE 函数的修改平均强度设定点等。 | - 与使用预定义预设相比，使用自定义配置可实现更好的深度性能 - 对于完善的自定义配置 |

根据您的具体用例选择合适的预设名称，并将其设置为 `device_preset` 参数的值。

## 深度工作模式开关

Orbbec SDK ROS2 支持深度工作模式切换。Gemini 2、Gemini 2 L、Femto 和 Femto Bolt 相机均支持深度工作模式切换。

- 在启动相机之前，可以配置深度工作模式（depth_work_mode），以获得相应 xxx.launch.py 文件的支持。
- 深度工作模式切换支持 Gemini 2、Gemini 2 L 和 Gemini 2 XL 相机。
- xxx.launch.py 默认的深度工作模式配置是相机的默认配置，如果需要修改，可以根据需要切换到相应的模式。
- 具体的相机深度工作模式支持类型可以参见深度模式的注释。

```python
# 深度工作模式支持如下：
# 未分箱密集默认值
# Unbinned Sparse Default
# Binned Sparse Default
# Obstacle Avoidance
DeclareLaunchArgument('depth_work_mode', default_value='')
```

- 查看深度工作模式：

```bash
ros2 run orbbec_camera list_depth_work_mode_node
```

## 深度 NFOV 和 WFOV 模式的配置

对于 Femto Mega 和 Femto Bolt 设备，NFOV 和 WFOV 模式是通过在启动文件中配置深度和红外分辨率来实现的。
在启动文件中，depth_width、depth_height、ir_width、ir_height 分别表示深度分辨率和红外分辨率。
红外的帧率和分辨率必须与深度分辨率一致。不同模式与分辨率的对应关系如下：

- NFOV unbinned: 640 x 576.
- NFOV binned: 320 x 288.
- WFOV unbinned: 1024 x 1024.
- WFOV binned: 512 x 512.

## 所有可用的摄像机控制服务

以下服务的名称已经表达了其功能。
但需要注意的是，相应的 `set_[ir|depth|color]`
和 `get[ir|depth|color]*`服务只有在启动文件参数对应的流中将 `enable[ir|depth|color]`
设置为 `true` 时才可用。

- `/camera/get_auto_white_balance`
- `/camera/get_color_exposure`
- `/camera/get_color_gain`
- `/camera/get_depth_exposure`
- `/camera/get_depth_gain`
- `/camera/get_device_info`
- `/camera/get_ir_exposure`
- `/camera/get_ir_gain`
- `/camera/get_ldp_status`
- `/camera/get_sdk_version`
- `/camera/get_white_balance`
- `/camera/set_auto_white_balance`
- `/camera/set_color_auto_exposure`
- `/camera/set_color_exposure`
- `/camera/set_color_gain`
- `/camera/set_depth_auto_exposure`
- `/camera/set_depth_exposure`
- `/camera/set_depth_gain`
- `/camera/set_fan_work_mode`
- `/camera/set_floor_enable`
- `/camera/set_ir_auto_exposure`
- `/camera/set_ir_exposure`
- `/camera/set_ir_gain`
- `/camera/set_laser_enable`
- `/camera/set_ldp_enable`
- `/camera/set_white_balance`
- `/camera/toggle_color`
- `/camera/toggle_depth`
- `/camera/toggle_ir`

## 所有可用主题

- `/camera/color/camera_info` : 彩色相机信息。
- `/camera/color/image_raw`: 彩色流图像。
- `/camera/depth/camera_info`: 深度相机信息。
- `/camera/depth/image_raw`: 深度流图像。
- `/camera/depth/points` :点云，仅当 `enable_point_cloud`为 `true`时可用。
- `/camera/depth_registered/points`: 彩色点云，仅当 `enable_colored_point_cloud` 为 `true` 时可用。
- `/camera/ir/camera_info`: 红外相机信息。
- `/camera/ir/image_raw`: 红外流图像
- `/camera/accel/sample`: 加速度数据流 `enable_sync_output_accel_gyro`关闭，`enable_accel`开启
- `/camera/gyro/sample`: 陀螺仪数据流，`enable_sync_output_accel_gyro` 关闭，`enable_gyro`开启
- `camera/gyro_accel/sample`: 加速度计和陀螺仪数据流同步，`enable_sync_output_accel_gyro` 已开启
- `/diagnostics`: 摄像机的诊断信息。目前，诊断信息仅包括摄像机的温度。

## 网络相机设备枚举

目前，仅 Femto Mega 设备支持网络设备枚举功能。通过网络访问此设备时，如果将 `enumerate_net_device` 设置为 `true`，则会自动枚举该设备，无需提前配置 IP 地址或将启用开关设置为 true。具体配置方法如下：

- `enumerate_net_device`: 自动枚举网络设备，仅 Femto Mega 支持。
  如果 `enumerate_net_device` 设置为 `true`，则将自动枚举设备，无需设置 `net_device_ip`
  和 `net_device_port` 参数。
- `net_device_ip`: 设备的 IP 地址。
- `net_device_port`: 设备的端口号。

## 多相机

- 要获取相机的 `usb_port`，请插入相机并在终端中运行以下命令：

```bash
ros2 run orbbec_camera list_devices_node
```

- 将“device_num”参数设置为您拥有的摄像机数量。
- 进入 `OrbbecSDK_ROS2/launch/multi_xxx.launch.py`文件，修改 `usb_port`。
- 不要忘记将 `include` 标签放在 `group` 标签内。
  否则，不同摄像机的参数值可能会受到污染。

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_01',
            'usb_port': '6-2.4.4.2',  # replace your usb port here
            'device_num': '2'
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_02',
            'usb_port': '6-2.4.1',  # replace your usb port here
            'device_num': '2'
        }.items()
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
    ])

    return ld

```

- 要启动摄像头，请运行以下命令：

```bash
ros2 launch orbbec_camera multi_camera.launch.py
```

## 压缩图像

您可以使用 `image_transport` 来使用 `jpeg` 格式压缩图像。以下是使用示例：

要访问压缩后的彩色图像，您可以使用以下命令：

```bash
ros2 topic echo /camera/color/image_raw/compressed --no-arr
```

此命令将允许您从指定主题接收压缩的彩色图像。

## 使用硬件解码器解码JPEG

### rockchip and Amlogic

依赖于 `rockchip-mpp-dev` 和 `rockchip-rga-dev`，并非所有系统都包含这两个软件包，名称可能不同，请自行搜索。
打开 `CMakeLists.txt`，并将 `USE_RK_HW_DECODER` 设置为 `ON`。

### Nvidia Jetson

依赖于：`jetson_multimedia_api`、`libyuv`。
打开 `CMakeLists.txt` 并将 `USE_NV_HW_DECODER` 设置为 `ON`。

## 检查相机支持哪些配置文件

```bash
ros2 run orbbec_camera list_camera_profile_mode_node
```

## 构建Debian软件包

### 准备环境

开始之前，请安装所需的工具：

```bash
sudo apt install debhelper fakeroot python3-bloom
```

### 配置ROS依赖项

将以下 YAML 文件添加到您的系统 `/etc/ros/rosdep/sources.list.d/00-orbbec.yaml`。请确保将 `focal` 替换为您的 Ubuntu 版本的代号，将 `humble` 替换为您的 ROS2 发行版名称：

```yaml
orbbec_camera_msgs:
  ubuntu:
    focal: [ ros-humble-orbbec-camera-msgs ]
```

接下来，创建一个新文件 `/etc/ros/rosdep/sources.list.d/50-orbbec.list` 并添加此行以指定
YAML file:

```bash
yaml file:///etc/ros/rosdep/sources.list.d/00-orbbec.yaml
```

更新 rosdep 数据库以反映这些更改：

```bash
rosdep update
```

### 构建包

导航到您的工作区并构建项目：

```bash
cd ~/ros2_ws/
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
cd src/OrbbecSDK_ROS2/
bash .make_deb.sh
```

## 支持的设备

OrbbecSDK ROS2 Wrapper 支持以下设备。

| 产品列表                              | 最低固件版本 | **Launch File**       |
| :------------------------------------ | :----------- | :-------------------------- |
| Astra+                                | 1.0.19       | astra_adv.launch.py         |
| Astra Mini /Astra Mini Pro /Astra Pro | 1007         | astra.launch.py             |
| Astra Mini Pro S                      | 1005         | astra.launch.py             |
| Astra 2                               | 2.8.20       | astra2.launch.py            |
| Dabai                                 | 2436         | dabai.launch.py             |
| Dabai DW                              | 2606         | dabai_dw.launch.py          |
| Dabai DCW                             | 2460         | dabai_dcw.launch.py         |
| Femto/Femto W                         | 1.1.8        | femto.launch.py             |
| Femto Mega                            | 1.1.7        | femto_mega.launch.py        |
| Femto Bolt                            | 1.0.6        | femto_bolt.launch.py        |
| Femto Mega I                          | 2.0.2        | femto_net_camera.launch.py  |
| Gemini                                | 3018         | gemini.launch.py            |
| Gemini 2 / Dabai DCL                  | 1.4.60       | gemini2.launch.py           |
| Gemini 2 L                            | 1.4.32       | gemini2L.launch.py          |
| Gemini E                              | 3460         | gemini_e.launch.py          |
| Gemini E Lite                         | 3606         | gemini_e_lite.launch.py     |
| Gemini 335/335L/336/336L              | 1.2.20       | gemini_330_series.launch.py |

> [!重要]
>
> 我们建议使用 [OrbbecSDK ROS2 Wrapper v2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main)，以便为以下设备提供更好的灵活性和可扩展性：Astra 2、Femto Mega、Femto Bolt、Gemini 2、Gemini 2 L、Gemini 335、Gemini 335L、Gemini 336、Gemini 336L、Gemini 335Lg（仅支持 v2-main 分支）。

所有启动文件本质上都类似，主要区别在于同一系列不同型号的参数默认值。USB 标准的差异（例如 USB 2.0 和 USB 3.0）可能需要调整这些参数。如果您遇到启动失败，请仔细阅读规格手册。请特别注意启动文件中的分辨率设置以及其他参数，以确保兼容性和最佳性能。

## DDS调优

默认的 DDS 设置 (Galactic) 可能并非数据传输的最佳选择。不同的 DDS 设置可能会带来不同的性能。本例中使用 CycloneDDS。更多详细信息，请参阅
[ROS DDS 调优](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)。

● 编辑cyclonedds配置文件

```bash
sudo gedit /etc/cyclonedds/config.xml
```

增加

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/confighttps://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Internal>
            <MinimumSocketReceiveBufferSize>16MB</MinimumSocketReceiveBufferSize>
        </Internal>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
            <Peers>
                <Peer address="localhost"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

● 设置环境变量，添加到 `.zshrc`或 `.bashrc`

```bash
export ROS_DOMAIN_ID=42 # Numbers from 0 to 232
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
```

提示：要了解为什么 ROS_DOMAIN_ID 的最大值为 232，请访问 [ROS DOMAIN ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)
● 增加 UDP 接收缓冲区大小
编辑

```bash
/etc/sysctl.d/10-cyclone-max.conf
```

增加

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
```

如果您使用 Fast DDS，可以参考 [Fast DDS 配置](./docs/fastdds_tuning.md) 文件。

## 常见问题

### 意外崩溃

如果摄像头节点意外崩溃，它将在当前运行目录中生成崩溃日志：`Log/camera_crash_stack_trace_xx.log`。请将此日志发送给支持团队或将其提交至 GitHub 问题以获取进一步帮助。

### 没有来自多个摄像机的数据流

**电源不足**:

- 确保每个摄像头都连接到单独的集线器。
- 使用有源集线器为每个摄像头提供足够的电力。

**高分辨率**:

- 尝试降低分辨率来解决数据流问题。

**增加 usbfs_memory_mb 值**:

- 运行以下命令，将 `usbfs_memory_mb` 值增加到 128MB（这是一个参考值，可以根据系统需求进行调整）：

```bash
    echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

- 要使此更改永久生效，请检查[此链接](https://github.com/OpenKinect/libfreenect2/issues/807)。

### 其他故障排除

- 如果您遇到其他问题，请将 `log_level` 参数设置为 `debug`。这将在运行目录中生成一个 SDK 日志文件：`Log/OrbbecSDK.log.txt`。
  请将此文件提供给支持团队以获取进一步帮助。
- 如果需要固件日志，请将 `enable_heartbeat`设置为 `true`以激活此功能。

### 为什么有这么多启动文件？

- 不同的相机有不同的默认分辨率和图像格式。
- 为了简化使用，每个相机都有自己的启动文件。

## 其他有用链接

- [OrbbecSDK_v2](https://github.com/orbbec/OrbbecSDK_v2/releases)
- [Gemini 330 depth presets](https://www.orbbec.com/docs/g330-use-depth-presets/)

## License

版权所有 2024 Orbbec Ltd。

根据 Apache 许可证 2.0 版（"许可证"）获得许可；除非遵守许可证，否则您不得使用此项目。您可以在以下网址获取许可证副本：

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

除非适用法律要求或书面同意，否则根据本许可证分发的软件均按“原样”分发，不附带任何明示或暗示的保证或条件。请参阅许可证，了解本许可证下特定语言的权限和限制规定。

**Other names and brands may be claimed as the property of others**
