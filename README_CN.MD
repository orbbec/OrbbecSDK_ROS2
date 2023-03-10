# orbbec_camera

---
OrbbecSDK ROS2是OrbbecSDK ROS2 Wrapper。

## 安装指南

安装 ROS2

- 请参考官方
  [ROS2 安装指南](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

> 如果你的ROS2 命令不能自动补全，把下面两行代码放在你的 `.bashrc`
> 或者 `.zshrc`

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

创建 `colcon` 工作空间

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

获取源代码
> 请访问
> [Orbbec 开发者社区](https://developer.orbbec.com.cn/download.html?id=75&menushow=1)
> 获取最新版本的代码

安装deb依赖

```bash
sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
ros-galactic-image-transport ros-galactic-image-publisher
```

安装 udev rules.

```bash
cd `~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts`
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 入门指南

```bash
cd ~/ros2_ws/
# build release, Default is Debug
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

启动相机。

- 在终端 1。

```bash
. ./install/setup.bash 
ros2 launch orbbec_camera astra.launch.xml
```

- 在终端 2。

```bash
. ./install/setup.bash 
rviz2
```

选择你想展示的topic。

- 列出 topics / services/ parameters (新开一个终端)。

```bash
ros2 topic list
ros2 service list
ros2 param list
```

- 显示深度到彩色的外参。

```bash
ros2 topic echo --qos-durability=transient_local /camera/extrinsic/depth_to_color  --qos-profile=services_default
```

- 获取设备信息

```bash
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
```

- 查看SDK版本

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'

```

- 查看曝光值

```bash
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
```

> 如果你需要查看 `ir` 或者 `depth`的曝光, 请把 `/camera/get_color_exposure`
> 改成 `/camera/get_ir_exposure` 或者 `/camera/get_depth_exposure`,下同.

- 查看增益

```bash
ros2 service call /camera/get_color_gain orbbec_camera_msgs/srv/GetInt32 '{}'
```

- 查看白平衡

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

- 开关激光

```bash
ros2 service call  /camera/set_laser_enable std_srvs/srv/SetBool "{data: true}" 
```

- 开关senseor

```bash
ros2 service call /camera/toggle_ir std_srvs/srv/SetBool "{data : true}"

```

### 所有相机控制的service

下面的这些服务的名字已经表明了它的功能，但是要注意的是只有当在launch文件里面把相关数据流打开的时候才能用。

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

### 所有的topic

- `/camera/color/camera_info` : 彩色相机信息。
- `/camera/color/image_raw`: 彩色数据流图像。
- `/camera/depth/camera_info`: 深度相机信息。
- `/camera/depth/image_raw`: 深度相机数据流图像。
- `/camera/depth/points` : 点云，只有  `enable_point_cloud` 设置为 `true`才可用。
- `/camera/depth_registered/points`: 彩色点云  `enable_colored_point_cloud` 还有 `depth_registration`
都设置为 `true`才可用。
- `/camera/ir/camera_info`:  IR相机信息。
- `/camera/ir/image_raw`: 红外数据流图像。

### 多相机

- 首先需要获取所有相机的序列号, 插入所有的相机在终端执行以下命令:

```bash
ros2 run orbbec_camera list_devices_node
```

- 把参数 `device_num` 设置成为你的相机数量。
- 编辑 `OrbbecSDK_ROS2/launch/multi_xxx.launch.xml` 修改相机的序列号，当前只能通过序列号来区分相机。
- 注意不要忘记在`include` 加上 `group` 标签，不然前面的相机的参数会污染后面的相机的参数。

```xml

<launch>
  <!-- unique camera name-->
  <arg name="camera_name" default="camera"/>
  <!-- Hardware depth registration -->
  <arg name="3d_sensor" default="astra_mini"/>
  <!-- stereo_s_u3, astrapro, astra -->
  <arg name="camera1_prefix" default="01"/>
  <arg name="camera2_prefix" default="02"/>
  <arg name="camera1_serila_number" default="AY3JB20003C"/>
  <arg name="camera2_serila_number" default="CH26420001B"/>
  <arg name="device_num" default="2"/>
  <node name="camera" pkg="orbbec_camera" exec="ob_cleanup_shm_node" output="screen"/>
  <group>
    <include file="$(find-pkg-share orbbec_camera)/launch/gemini2.launch.xml">
      <arg name="camera_name" value="camera_$(var camera1_prefix)"/>
      <arg name="serial_number" value="$(var camera1_serila_number)"/>
      <arg name="device_num" value="$(var device_num)"/>
    </include>
  </group>
  <group>
    <include file="$(find-pkg-share orbbec_camera)/launch/dabai_dcw.launch.xml">
      <arg name="camera_name" value="camera_$(var camera2_prefix)"/>
      <arg name="serial_number" value="$(var camera2_serila_number)"/>
      <arg name="device_num" value="$(var device_num)"/>
    </include>
  </group>
  <node pkg="tf2_ros" exec="static_transform_publisher" name="camera_tf"
        args="0 0 0 0 0 0 camera01_link camera02_link"/>
</launch>

```

- 请注意，Astra相机使用信号量进行进程同步。如果相机启动失败，信号量文件可能会留在`/dev/shm`目录下，导致下一次启动卡住。为了避免这种情况，请在启动前运行以下命令：

```bash
ros2 run orbbec_camera ob_cleanup_shm_node
```

这个命令会清理掉`/dev/shm/`下面的文件

- 执行下面的命令启动多相机:

```bash
ros2 launch orbbec_camera multi_camera.launch.xml
```

## 启动参数

以下是可用的启动参数

- `connection_delay`: 重新打开设备的延迟时间（以毫秒为单位）。某些设备，例如Astra mini，需要更长的时间来初始化，立即重新打开设备可能会在热插拔时导致固件崩溃。
- `enable_point_cloud`: 是否开启点云。
- `enable_colored_point_cloud`: 是否开启彩色点云。
- `point_cloud_qos`, `[color|depth|ir]_qos,``[color|depth|ir]_camera_info_qos`: ROS2消息质量服务（QoS）设置。可能的值是`SYSTEM_DEFAULT`，`DEFAULT`，`PARAMETER_EVENTS`，`SERVICES_DEFAULT`，`PARAMETERS`，`SENSOR_DATA`，并且不区分大小写。这些对应于`rmw_qos_profile_system_default`，`rmw_qos_profile_default`，`rmw_qos_profile_parameter_events`，`rmw_qos_profile_services_default`，`rmw_qos_profile_parameters`。
- `enable_d2c_viewer`: 发布D2C叠加图像（仅用于测试）。
- `device_num`: 设备数量。如果需要多个相机，则必须填写此项。
- `color_width`, `color_height`, `color_fps`: 彩色流的分辨率和帧率。
- `ir_width`, `ir_height`, `ir_fps`: 红外流的分辨率和帧率。
- `depth_width`, `depth_height`, `depth_fps`: 深度流的分辨率和帧率。
- `enable_color`: 是否启用RGB相机。
- `enable_depth`: 启用深度相机。
- `enable_ir`: 是否开启IR相机。
- `depth_registration`: 是否启用将深度帧硬件对齐到彩色帧。当`enable_colored_point_cloud`设置为`true`时，此字段也要设置为`true`。
- `enable_publish_extrinsic`: 是否启用发布外参.

## DDS Tuning

默认的DDS设置（Galactic）可能不适用于数据传输。不同的DDS设置可能具有不同的性能。在此示例中，我们使用CycloneDDS。有关更详细的信息，请参考[ROS DDS Tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)。

● 编辑CycloneDDS配置文件.

```bash
sudo gedit /etc/cyclonedds/config.xml
```

Add

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

● 设置环境变量，把下面变量添加到.zshrc或.bashrc文件中。

```bash
export ROS_DOMAIN_ID=42 # Numbers from 0 to 232
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
```

提示：要了解为什么最大的ROS_DOMAIN_ID是232 请参考[The ROS DOMAIN ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)
● 增加UDP接收缓冲区大小。编辑

```bash
/etc/sysctl.d/10-cyclone-max.conf
```

添加

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
```

## 常见问题

多相机没有图像

- 可能是USB供电不足导致，可以尝试用供电的USB hub。

- 也可能是分辨率设置太高，降低分辨率后再尝试。

为什么这里这么多launch文件

- 存在多个launch文件的原因是因为不同相机的默认分辨率和图像格式不同。为了更方便使用，已将启动文件分开为每个相机一个。

## License

Copyright 2023 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

**Other names and brands may be claimed as the property of others**
