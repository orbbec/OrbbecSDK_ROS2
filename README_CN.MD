# orbbec_camera

**请注意：本文档为机器翻译，仅供参考。翻译可能存在遗漏或错误，且不会及时更新。请以英文原文为准。**

OrbbecSDK ROS 2 是一个用于 Orbbec 3D 相机的 ROS 2 环境无缝集成的包装器。它支持 ROS 2 Foxy、Galactic 和 Humble 版本。

## 安装说明

安装 ROS 2

- 请参考官方的 [ROS 2 安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

> 如果您的 ROS 2 命令没有自动补全，请将以下两行放入您的 `.bashrc` 或 `.zshrc` 中

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

创建 `colcon` 工作区

```bash
mkdir -p ~/ros2_ws/src
```

获取源代码

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
```

安装 deb 依赖项

```bash
# 假设您已经 source 了 ROS 环境，下同
sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs
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
# 构建 Release 版本，默认为 Debug
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

启动相机节点

- 在终端 1 中

```bash
. ./install/setup.bash
ros2 launch orbbec_camera astra.launch.py # 或者其他启动文件，参见下表
```

- 在终端 2 中

```bash
. ./install/setup.bash
rviz2
```

选择要显示的主题

- 列出主题 / 服务 / 参数（在终端 3 中）

```bash
ros2 topic list
ros2 service list
ros2 param list
```

- 获取设备信息

```bash
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
```

- 获取 SDK 版本

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'
```

- 获取曝光

```bash
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
```

> 如果您查看的是 `ir` 或 `depth`，请将 `/camera/get_color_exposure` 改为 `/camera/get_ir_exposure` 或 `/camera/get_depth_exposure`，下同。

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

- 设置白平衡

```bash
ros2 service call /camera/set_white_balance orbbec_camera_msgs/srv/SetInt32 '{data: 4600}'
```

- 设置激光器启用

```bash
ros2 service call  /camera/set_laser_enable std_srvs/srv/SetBool "{data: true}"
```

- 切换传感器

```bash
ros2 service call /camera/toggle_ir std_srvs/srv/SetBool "{data : true}"
```

- 保存点云

```bash
ros2 service call /camera/save_point_cloud std_srvs/srv/Empty "{}"
```

### 所有可用的相机控制服务

以下服务的名称已经表明了它的功能。但是，需要注意的是，相应的 `set_[ir|depth|color]*` 和 `get[ir|depth|color]*` **服务只有在启动文件的对应流的参数中将** `enable[ir|depth|color]` 设置为 `true` 时才可用。

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

### 所有可用的话题

- `/camera/color/camera_info` : 彩色相机信息。
- `/camera/color/image_raw`: 彩色流图像。
- `/camera/depth/camera_info`: 深度流图像。
- `/camera/depth/image_raw`: 深度流图像。
- `/camera/depth/points` : 点云，仅在 `enable_point_cloud` 为 `true` 时可用。
- `/camera/depth_registered/points`: 彩色点云，仅在 `enable_colored_point_cloud` 为 `true` 时可用。
- `/camera/ir/camera_info`: 红外相机信息。
- `/camera/ir/image_raw`: 红外流图像。
- `/camera/accel/sample`: 加速度数据流，`enable_sync_output_accel_gyro` 关闭，`enable_accel` 打开。
- `/camera/gyro/sample`: 陀螺仪数据流，`enable_sync_output_accel_gyro` 关闭，`enable_gyro` 打开。
- `camera/gyro_accel/sample`: 加速度和陀螺仪同步数据流，`enable_sync_output_accel_gyro` 打开。

### 网络设备枚举

目前，网络设备枚举功能仅由 Femto Mega 设备支持。当通过网络访问此设备时，如果将 `enumerate_net_device` 设置为 `true`，则设备将自动枚举，无需预先配置 IP 地址或将启用开关设置为 true。具体配置方法如下：

- `enumerate_net_device`：自动枚举网络设备，仅 Femto Mega 支持。如果将 `enumerate_net_device` 设置为 `true`，设备将自动枚举，无需设置 `net_device_ip` 和 `net_device_port` 参数。
- `net_device_ip`：设备的 IP 地址。
- `net_device_port`：设备的端口号。

### 多相机

- 要获取相机的 `usb_port`，请插入相机并在终端中运行以下命令：

```bash
ros2 run orbbec_camera list_devices_node
```

- 将 `device_num` 参数设置为您拥有的相机数量。
- 转到 `OrbbecSDK_ROS2/launch/multi_xxx.launch.py` 文件并更改 `usb_port`。
- 不要忘记将 `include` 标签放在 `group` 标签内。否则，不同相机的参数值可能会被污染。

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 包含启动文件
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_01',
            'usb_port': '6-2.4.4.2',  # 在此替换您的 USB 端口
            'device_num': '2'
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_02',
            'usb_port': '6-2.4.1',  # 在此替换您的 USB 端口
            'device_num': '2'
        }.items()
    )

    # 如果您需要更多相机，只需在此处添加更多 launch_include，并更改 usb_port 和 device_num

    # 启动描述
    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
    ])

    return ld

```

- 要启动相机，请运行以下命令：

```bash
ros2 launch orbbec_camera multi_camera.launch.py
```

## 使用硬件解码器解码 JPEG

### 瑞芯微和晶晨

依赖于 `rockchip-mpp-dev` 和 `rockchip-rga-dev`，并非所有系统都有这两个包，名称可能不同，请自行搜索。
打开 `CMakeLists.txt` 并将 `USE_RK_HW_DECODER` 设置为 `ON`。

### Nvidia Jetson

依赖于：`jetson_multimedia_api`、`libyuv`。
打开 `CMakeLists.txt` 并将 `USE_NV_HW_DECODER` 设置为 `ON`。

## 启动参数

以下是可用的启动参数：

- `connection_delay`：重新打开设备的延迟时间，单位为毫秒。一些设备（如 Astra mini）需要较长时间才能初始化，立即重新打开设备会导致固件崩溃。
- `enable_point_cloud`：启用点云。
- `enable_colored_point_cloud`：启用 RGB 点云。
- `point_cloud_qos`、`[color|depth|ir]_qos`、`[color|depth|ir]_camera_info_qos`：ROS 2 消息服务质量 (QoS) 设置。可能的值为 `SYSTEM_DEFAULT`、`DEFAULT`、`PARAMETER_EVENTS`、`SERVICES_DEFAULT`、`PARAMETERS` 和 `SENSOR_DATA`，不区分大小写。它们分别对应于 `rmw_qos_profile_system_default`、`rmw_qos_profile_default`、`rmw_qos_profile_parameter_events`、`rmw_qos_profile_services_default`、`rmw_qos_profile_parameters` 和 `SENSOR_DATA`。
- `enable_d2c_viewer`：发布 D2C 叠加图像（仅用于测试）。
- `device_num`：设备数量。如果需要多个相机，则必须填写此参数。
- `color_width`、`color_height`、`color_fps`：彩色流的分辨率和帧率。
- `ir_width`、`ir_height`、`ir_fps`：红外流的分辨率和帧率。
- `depth_width`、`depth_height`、`depth_fps`：深度流的分辨率和帧率。
- `enable_color`：启用 RGB 相机。
- `enable_depth`：启用深度相机。
- `enable_ir`：启用红外相机。
- `depth_registration`：启用将深度帧对齐到彩色帧的硬件。当设置 `enable_colored_point_cloud` 为 `true` 时，需要此字段。
- `usb_port`：相机的 USB 端口。使用多个相机时需要此参数。
- `enable_accel`：启用加速度计。
- `accel_rate`：加速度计的频率，可选值为 `1.5625hz`、`3.125hz`、`6.25hz`、`12.5hz`、`25hz`、`50hz`、`100hz`、`200hz`、`500hz`、`1khz`、`2khz`、`4khz`、`8khz`、`16khz`、`32khz`。具体值取决于当前相机。
- `accel_range`：加速度计的范围，可选值为 `2g`、`4g`、`8g`、`16g`。具体值取决于当前相机。
- `enable_gyro`：是否启用陀螺仪。
- `gyro_rate`：陀螺仪的频率，可选值为 `1.5625hz`、`3.125hz`、`6.25hz`、`12.5hz`、`25hz`、`50hz`、`100hz`、`200hz`、`500hz`、`1khz`、`2khz`、`4khz`、`8khz`、`16khz`、`32khz`。具体值取决于当前相机。
- `gyro_range`：陀螺仪的范围，可选值为 `16dps`、`31dps`、`62dps`、`125dps`、`250dps`、`500dps`、`1000dps`、`2000dps`。具体值取决于当前相机。
- `enumerate_net_device`：是否启用枚举网络设备的功能。True 表示启用，false 表示禁用。此功能仅由 Femto Mega 和 Gemini 2 XL 设备支持。当通过网络访问这些设备时，需要预先配置设备的 IP 地址。启用开关需要设置为 true。
- `depth_filter_config`：配置深度过滤配置文件的加载路径。默认情况下，深度过滤配置文件位于 /config/depthfilter 目录下，仅 Gemini2 支持。
- `depth_precision`：深度精度，格式应为 `1mm`。默认值为 `1mm`。
- `enable_laser`：是否启用激光器。默认值为 `true`。
- `laser_on_off_mode`：激光器开关交替模式，0：关闭，1：开-关交替，2：关-开交替。默认值为 `0`。
- `device_preset`：设备预设选项为 `Default` 或 `High Accuracy`。
- `enable_decimation_filter`：此滤波有效地降低了深度场景的复杂度。滤波以 [2x2] 到 [8x8] 像素的内核大小运行。图像大小在两个维度上按比例缩小，以保持长宽比。
- `enable_hdr_merge`：此滤波与深度 HDR 功能一起使用。通过合并连续的具有交替曝光值的深度图像，我们可以同时克服获取低照度和过度照明物体的深度值的挑战。
- `enable_sequence_id_filter`：此滤波与深度 HDR 功能一起使用，仅输出具有指定序列 ID 的序列。
- `enable_threshold_filter`：此滤波保留感兴趣的深度值，并忽略超出范围的深度值。
- `enable_noise_removal_filter`：此滤波以簇的形式去除斑点噪声，并产生一个填充较少的深度图。
- `enable_spatial_filter`：此滤波根据幅度参数指定的多次迭代处理，以增强深度数据的平滑度。它还能够填充深度图中的小孔。
- `enable_temporal_filter`：此滤波旨在通过基于先前帧操纵每像素值来改善深度数据的持久性。该滤波对数据执行单次处理，调整深度值的同时更新跟踪历史。
- `enable_hole_filling_filter`：此滤波使用指定的模式填充深度图中的所有孔。
- `retry_on_usb3_detection_failure`：如果相机连接到 USB 2.0 端口且未被检测到，系统将尝试最多三次重置相机。此设置旨在防止 USB 3.0 设备被错误地识别为 USB 2.0。建议在使用 USB 2.0 连接时将此参数设置为 `false`，以避免不必要的重置。

## 深度工作模式切换

- 在启动相机之前，可以为相应的 xxx.launch.py 文件支持配置深度工作模式（depth_work_mode）。
- Gemini 2、Gemini 2 L 和 Gemini 2 XL 相机支持深度工作模式切换。
- xxx.launch.py 的默认深度工作模式配置是相机的默认配置。如果需要修改，可以根据需要切换到相应的模式。
- 具体相机深度工作模式支持类型可以在深度模式的注释中找到。

```python
# 深度工作模式支持如下：
# Unbinned Dense Default
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

对于 Femto Mega 和 Femto Bolt 设备，NFOV 和 WFOV 模式是通过在启动文件中配置 Depth 和 IR 的分辨率来实现的。
在启动文件中，depth_width、depth_height、ir_width、ir_height 分别表示深度的分辨率和 IR 的分辨率。
IR 的帧率和分辨率必须与深度一致。不同模式与分辨率的对应关系如下：

- NFOV unbinned：640 x 576。
- NFOV binned：320 x 288。
- WFOV unbinned：1024 x 1024。
- WFOV binned：512 x 512。

## 检查相机支持的配置文件

```bash
ros2 run orbbec_camera list_camera_profile_mode_node
```

## 使用FFmpeg解码H264/265的工具

对于Femto Mega这种能够输出H264/H265编码格式的，能够使用mega_h26x_decode_node节点来实现H264/H265解码功能。
mega_h26x_decode_node节点的解码功能是通过FFmpeg来实现的。

* 首先启动Femto Mega相机并将color的编码格式设置为H264或者H265：

  ```bash
  ros2 launch orbbec_camera femto_mega.launch.py color_format:=H264
  ```
* 启动解码节点：

  ```bash
  ros2 run orbbec_camera mega_h26x_decode_node
  ```

mega_h26x_decode_node节点可用的话题：

* /camera/color/h26x_encoded_data ：Femto Mega输出原始color的H264/H265编码数据
* /camera/color/h26x_decoder/image_raw ：经过mega_h26x_decode_node节点解码后的RGB图像数据

## 构建 Debian 包

### 准备环境

在开始之前，请安装所需的工具：

```bash
sudo apt install debhelper fakeroot python3-bloom
```

### 配置 ROS 依赖项

将以下 YAML 文件添加到系统的 `/etc/ros/rosdep/sources.list.d/00-orbbec.yaml` 中。确保将 `focal` 替换为您的 Ubuntu 版本的代号，将 `humble` 替换为您的 ROS2 发行版名称：

```yaml
orbbec_camera_msgs:
  ubuntu:
    focal: [ros-humble-orbbec-camera-msgs]
```

接下来，创建一个新文件 `/etc/ros/rosdep/sources.list.d/50-orbbec.list`，并添加这一行以指定 YAML 文件的路径：

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

## 启动文件

| 产品系列                              | 启动文件                    |
| ------------------------------------- | --------------------------- |
| astra+                                | astra_adv.launch.py         |
| astra mini /astra mini pro /astra pro | astra.launch.py             |
| astra mini pro s                      | astra.launch.py             |
| astra2                                | astra2.launch.py            |
| astra stereo s                        | stereo_s_u3.launch.py       |
| astra pro2                            | astra_pro2.launch.py        |
| dabai                                 | dabai.launch.py             |
| dabai d1                              | dabai_d1.launch.py          |
| dabai dcw                             | dabai_dcw.launch.py         |
| dabai dw                              | dabai_dw.launch.py          |
| dabai pro                             | dabai_pro.launch.py         |
| deeya                                 | deeya.launch.py             |
| femto /femto w                        | femto.launch.py             |
| femto mega                            | femto_mega.launch.py        |
| femto bolt                            | femto_bolt.launch.py        |
| gemini                                | gemini.launch.py            |
| gemini                                | gemini.launch.py            |
| gemini2 / dabai DCL                   | gemini2.launch.py           |
| gemini2L                              | gemini2L.launch.py          |
| gemini e                              | gemini_e.launch.py          |
| gemini e lite                         | gemini_e_lite.launch.py     |
| dabai max                             | dabai_max.launch.py         |
| dabai max pro                         | dabai_max_pro.launch.py     |
| gemini uw                             | gemini_uw.launch.py         |
| dabai dcw2                            | dabai_dcw2.launch.py        |
| dabai dw2                             | dabai_dw2.launch.py         |
| gemini ew                             | gemini_ew.launch.py         |
| gemini ew lite                        | gemini_ew_lite.launch.py    |
| gemini 330 系列                       | gemini_330_series.launch.py |

**所有启动文件本质上都是相似的，主要区别在于为同一系列中不同型号设置的参数的默认值。USB 标准的差异，例如 USB 2.0 与 USB 3.0，可能需要调整这些参数。如果遇到启动失败，请仔细查看规格手册。特别注意启动文件中的分辨率设置以及其他参数，以确保兼容性和最佳性能。**

## 产品支持

| **产品列表** | **固件版本**          |
| ------------------ | --------------------------- |
| Gemini 335         | 1.2.20                      |
| Gemini 335L        | 1.2.20                      |
| Femto Bolt         | 1.0.6/1.0.9                 |
| Femto Mega         | 1.1.7/1.2.7                 |
| Gemini 2 XL        | Obox：V1.2.5 VL：1.4.54     |
| Astra 2            | 2.8.20                      |
| Gemini 2 L         | 1.4.32                      |
| Gemini 2           | 1.4.60 /1.4.76              |
| Astra+             | 1.0.22/1.0.21/1.0.20/1.0.19 |
| Femto              | 1.6.7                       |
| Femto W            | 1.1.8                       |
| DaBai              | 2436                        |
| DaBai DCW          | 2460                        |
| DaBai DW           | 2606                        |
| Astra Mini Pro     | 1007                        |
| Gemini E           | 3460                        |
| Gemini E Lite      | 3606                        |
| Gemini             | 3.0.18                      |
| Astra Mini S Pro   | 1.0.05                      |

## DDS 调优

默认的 DDS 设置（Galactic）可能不是数据传输的最佳选择。不同的 DDS 设置可能具有不同的性能。在此示例中，我们使用 CycloneDDS。有关更多详细信息，请参阅 [ROS DDS 调优](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)。

● 编辑 cyclonedds 配置文件

```bash
sudo gedit /etc/cyclonedds/config.xml
```

添加

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

● 设置环境变量，添加到 `.zshrc` 或 `.bashrc`

```bash
export ROS_DOMAIN_ID=42 # 0 到 232 之间的数字
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
```

提示：要了解为什么最大 ROS_DOMAIN_ID 为 232，请访问 [ROS 域 ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)
● 增加 UDP 接收缓冲区大小
编辑

```bash
/etc/sysctl.d/10-cyclone-max.conf
```

添加

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
```

## 常见问题

多个相机无图像

- 可能是电源供应不足。为避免这种情况，请不要将所有相机连接到同一个集线器，并使用带电源的集线器。
- 也可能是分辨率太高。要解决此问题，请尝试降低分辨率。

为什么这里有那么多启动文件

- 存在多个启动文件的原因是不同相机的默认分辨率和图像格式不同。为了方便使用，为每个相机单独设置了启动文件。

## 许可证

版权所有 2023 Orbbec Ltd。

根据 Apache 许可证 2.0 版（"许可证"）获得许可；除非遵守许可证，否则您不得使用此项目。您可以在以下网址获取许可证副本：

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)
