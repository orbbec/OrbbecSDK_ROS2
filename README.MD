# orbbec_camera

---
OrbbecSDK ROS2 is a OrbbecSDK wrapper for the Orbbec 3D camera, providing seamless integration with the ROS2
environment.

## Installation Instructions

- Please refer to the
  official [ROS 2 installation guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
  guidance

> If your ROS2 command does not auto-complete, put the following two lines into your `.bashrc`
> or `.zshrc`

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

Create `colcon` workspace Ubuntu

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Get source code
> Please visit
> [Orbbec developerwebsite](https://developer.orbbec.com.cn/download.html?id=75&menushow=1) to find
> the latest version code

Install deb dependencies

```bash
sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
ros-galactic-image-transport ros-galactic-image-publisher
```

Install udev rules.

```bash 
cd `~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts`
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Getting start

```bash
cd ~/ros2_ws/
# build release, Default is Debug
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

Launch camera node

* On terminal 1

```bash 
. ./install/setup.bash 
ros2 launch orbbec_camera ob_camera.launch.py
```

* On terminal 2

```bash
. ./install/setup.bash 
rviz2 -d src/orbbec_camera/rviz/pointcloud.rviz
```

* List topics / services/ parameters ( on terminal 3)

```bash
ros2 topic list
ros2 service list
ros2 param list
```

* Show depth to color extrinsic

```bash
ros2 topic echo --qos-durability=transient_local /camera/extrinsic/depth_to_color  --qos-profile=services_default
```

* Get device info

```bash 
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
```

* Get SDK version

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'

```

* Get exposure

```bash
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
```

> If your check `ir` or `depth`, please change ` /camera/get_color_exposure`
> to  `/camera/get_ir_exposure` or `/camera/get_depth_exposure`, Same below.

* Get gain

```bash
ros2 service call /camera/get_color_gain orbbec_camera_msgs/srv/GetInt32 '{}'
```

* Get white balance

```bash
ros2 service call /camera/get_white_balance orbbec_camera_msgs/srv/GetInt32 '{}'
```

* Set auto exposure

```bash
ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}'
```

* Set white balance

```bash
ros2 service call /camera/set_white_balance orbbec_camera_msgs/srv/SetInt32 '{data: 4600}'
```

* Set laser enable

```bash
ros2 service call  /camera/set_laser_enable std_srvs/srv/SetBool "{data: true}" 
* ```

* toggle sensor

```bash
ros2 service call  /camera/toggle_ir std_srvs/srv/SetBool "{data : true}"
* ```

### Multi-Camera

* To get the serial number of the camera, plug in the camera and run the following command in the terminal:

```bash
ros2 run orbbec_camera list_devices_node
```

* Set the `device_num` parameter to the number of cameras you have.
* Go to the `OrbbecSDK_ROS2/launch/multi_xxx.launch` file and change the serial number. Currently,
  different cameras can only be differentiated by their serial number.

```xml

<launch>
    <!-- Unique camera name -->
    <arg name="camera_name" default="camera"/>
    <!-- Hardware depth registration -->
    <arg name="3d_sensor" default="astra"/>
    <!-- Stereo_s_u3, astrapro, astra -->
    <arg name="camera1_prefix" default="01"/>
    <arg name="camera2_prefix" default="02"/>
    <arg name="camera1_serial_number" default="ADA611300CE"/>
    <arg name="camera2_serial_number" default="sn123456789"/>
    <arg name="device_num" default="2"/>
    <node name="camera" pkg="orbbec_camera" type="ob_cleanup_shm_node" output="screen"/>
    <include file="$(find-pkg-share orbbec_camera)/launch/$(arg 3d_sensor).launch.xml">
        <arg name="camera_name" value="$(var camera_name)_$(var camera1_prefix)"/>
        <arg name="serial_number" value="$(var camera1_serial_number)"/>
        <arg name="device_num" value="$(var device_num)"/>
    </include>

    <include file="$(find-pkg-share orbbec_camera)/launch/$(var 3d_sensor).launch.xml">
        <arg name="camera_name" value="$(var camera_name)_$(var camera2_prefix)"/>
        <arg name="serial_number" value="$(var camera2_serial_number)"/>
        <arg name="device_num" value="$(var device_num)"/>
    </include>
    <node pkg="tf2_ros" exec="static_transform_publisher" name="camera_tf"
          args="0 0 0 0 0 0 camera01_link camera02_link"/>
</launch>

```

* Note that the astra camera uses semaphores for process synchronization.
  If the camera start fails, the semaphore file may be left in `/dev/shm`,
  causing the next start to become stuck. To avoid this, run the following command before launching:

```bash
ros2 run orbbec_camera ob_cleanup_shm_node
```

This will clean up `/dev/shm/`.

* To launch the cameras, run the following command:

```bash
ros2 launch orbbec_camera multi_camera.launch.xml
```

## Launch parameters

The following are the launch parameters available:

- `connection_delay`: The delay time in milliseconds for reopening the device.
  Some devices, such as Astra mini, require a longer time to initialize and
  reopening the device immediately can cause
  firmware crashes when hot plugging.
- `enable_point_cloud`: Enables the point cloud.
- `enable_colored_point_cloud`: Enables the RGB point cloud.
- `point_cloud_qos`, `[color|depth|ir]_qos,``[color|depth|ir]_camera_info_qos`: ROS2 Message Quality of Service (QoS)
  settings. The possible values
  are `SYSTEM_DEFAULT`, `DEFAULT`,`PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA`
  and are case-insensitive. These correspond to `rmw_qos_profile_system_default`, `rmw_qos_profile_default`,
  `rmw_qos_profile_parameter_events`, `rmw_qos_profile_services_default`, `rmw_qos_profile_parameters`,
  and `SENSOR_DATA`,
  respectively.
- `enable_d2c_viewer`: Publishes the D2C overlay image (for testing only).
- `device_num`: The number of devices. This must be filled in if multiple cameras are required.
- `color_width`, `color_height`, `color_fps`: The resolution and frame rate of the color stream.
- `ir_width`, `ir_height`, `ir_fps`: The resolution and frame rate of the IR stream.
- `depth_width`, `depth_height`, `depth_fps`: The resolution and frame rate of the depth stream.
- `enable_color`: Enables the RGB camera.
- `enable_depth`: Enables the depth camera.
- `enable_ir`: Enables the IR camera.
- `depth_registration`: Enables hardware alignment of depth and color. This requires the RGB point cloud to be open.
- `enable_publish_extrinsic`: Enables the publishing of camera extrinsic information.

## DDS Tuning

The default DDS settings (Galactic) may not be optimal for data transmission. Different DDS settings can have varying
performance. In this example, we use CycloneDDS. For more detailed information, please refer to the
[ROS DDS Tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)。

● Edit cyclonedds configuration file

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

● Set the environment variables, add to `.zshrc` or `.bashrc`

```bash
export ROS_DOMAIN_ID=42 # Numbers from 0 to 232
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
```

Tip：to understand why the maximum ROS_DOMAIN_ID is 232, please
visit [The ROS DOMAIN ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)
● Increase UDP receive buffer size
Edit

```bash
/etc/sysctl.d/10-cyclone-max.conf
```

Add

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
```

## Frequently Asked Questions

No Picture from Multiple Cameras

- it's possible that the power supply is insufficient.
  To avoid this, do not connect all cameras to the same hub and use a powered hub instead.

- It's also possible that the resolution is too high.
  To resolve this, try lowering the resolution.

Why are there so many launch files here

- The reason for the presence of multiple launch
  files is due to the fact that the default resolutions and image formats of different cameras vary.
  To make it easier to use, the launch files have been separated for each camera.

## License

Copyright 2023 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

**Other names and brands may be claimed as the property of others**
