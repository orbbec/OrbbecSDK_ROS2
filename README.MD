# OrbbecSDK ROS2 Wrapper v2

> [!IMPORTANT]
>
> Welcome to the OrbbecSDK ROS2 Wrapper v2. Before you begin using this version of ROS2 wrapper, it's crucial to check the following [device support list](#supported-devices) to verify the compatibility.

OrbbecSDK ROS2 Wrapper provides seamless integration of Orbbec cameras with ROS 2 environment. It supports ROS2 Foxy, Humble, and Jazzy distributions.

With a major update in October 2024, we release the [OrbbecSDK ROS2 Wrapper v2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main) connected to the open source [OrbbecSDK v2](https://github.com/orbbec/OrbbecSDK_v2/releases) with enhanced flexibility and extensibility. This update ensures compatibility with all Orbbec USB products adhering to UVC standard. However, it no longer supports Orbbec's traditional OpenNI protocol devices. We strongly encourage you to use the v2-main branch if your device is supported.

Here is the device support list of main branch (v1.x) and v2-main branch (v2.x):

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr style="background-color: #1f4e78; color: white; text-align: center;">
      <th>Product Series</th>
      <th>Product</th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/main" style="color: black; text-decoration: none;">Branch main</a></th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main" style="color: black; text-decoration: none;">Branch v2-main</a></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335L</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336L</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330L</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td>Not supported</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335Le</td>
      <td>Not supported</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 L</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td>Recommended for new designs</td>
      <td>To be supported</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td>
      <td>Femto Bolt</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td>Full maintenance</td>
      <td>To be supported</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td>Limited maintenance</td>
      <td>Not supported</td>
    </tr>
    <tr>
      <td>Astra Pro Plus</td>
      <td>Limited maintenance</td>
      <td>Not supported</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini Pro</td>
      <td>Full maintenance</td>
      <td>Not supported</td>
    </tr>
  </tbody>
</table>

**Note**: If you do not find your device, please contact our FAE or sales representative for help.

**Definition**:

1. Recommended for new designs: we will provide full supports with new features,  bug fix and performance optimization;
2. Full maintenance: we will provide bug fix support;
3. Limited maintenance: we will provide critical bug fix support;
4. Not supported: we will not support specific device in this version;
5. To be supported: we will add support in the near future.

## Table of Contents

- [OrbbecSDK ROS2 Wrapper v2](#orbbecsdk-ros2-wrapper-v2)
  - [Table of Contents](#table-of-contents)
  - [Installation Instructions](#installation-instructions)
  - [Supported Devices](#supported-devices)
  - [Getting start](#getting-start)
  - [Usage](#usage)
    - [Launch parameters](#launch-parameters)
    - [All available service for camera control](#all-available-service-for-camera-control)
    - [All available topics](#all-available-topics)
    - [Network device enumeration](#network-device-enumeration)
    - [GMSL device enumeration](#gmsl-device-enumeration)
    - [Multi-Camera](#multi-camera)
    - [Check which profiles the camera supports](#check-which-profiles-the-camera-supports)
    - [Predefined presets](#predefined-presets)
      - [Optional depth presets](#optional-depth-presets)
    - [Depth work mode switch](#depth-work-mode-switch)
    - [Configuration of depth NFOV and WFOV modes](#configuration-of-depth-nfov-and-wfov-modes)
  - [Advanced Usage](#advanced-usage)
    - [Use V4L2 backend](#use-v4l2-backend)
    - [DDS Tuning](#dds-tuning)
    - [Efficient intra-process communication](#efficient-intra-process-communication)
    - [ROS2(Robot) vs Optical(Camera) Coordination Systems](#ros2robot-vs-opticalcamera-coordination-systems)
    - [Camera sensor structure](#camera-sensor-structure)
    - [TF from coordinate A to coordinate B](#tf-from-coordinate-a-to-coordinate-b)
    - [Compressed Image](#compressed-image)
    - [Building a Debian Package](#building-a-debian-package)
  - [Examples](#examples)
  - [Frequently Asked Questions](#frequently-asked-questions)
    - [Unexpected Crash](#unexpected-crash)
    - [No Data Stream from Multiple Cameras](#no-data-stream-from-multiple-cameras)
    - [Additional Troubleshooting](#additional-troubleshooting)
    - [Why Are There So Many Launch Files?](#why-are-there-so-many-launch-files)
  - [Other useful links](#other-useful-links)
  - [License](#license)

## Installation Instructions

Install ROS 2

- Please refer to the
  official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  guidance

> If your ROS 2 command does not auto-complete, put the following two lines into your `.bashrc`
> or `.zshrc`

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

Create `colcon` workspace

```bash
mkdir -p ~/ros2_ws/src
```

Get source code

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout v2-main
git branch  #Check whether the branch switch is successful
```

Install deb dependencies

```bash
# assume you have sourced ROS environment, same blow
sudo apt install libgflags-dev nlohmann-json3-dev  \
ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev
```

Install udev rules.

```bash
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Supported Devices

Currently, the following devices are supported by the OrbbecSDK ROS2 Wrapper v2-main branch.  More devices support will be added in the near future. If you can not find your device in the table below, try the [main](https://github.com/orbbec/OrbbecSDK_ROS2)  branch.

For optimal performance, we strongly recommend updating to the latest firmware version. This ensures that you benefit from the most recent enhancements and bug fixes.

| Product List | Minimal Firmware Version | **Launch File**       |
| :----------- | :----------------------- | :-------------------------- |
| Gemini 330   | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 330L  | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 335   | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 335L  | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 335Lg | 1.3.46                   | gemini_330_series.launch.py |
| Gemini 336   | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 336L  | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 335Le | 1.5.31                   | gemini_330_series.launch.py |
| Femto Bolt   | 1.1.2                    | femto_bolt.launch.py        |
| Femto Mega   | 1.3.0                    | femto_mega.launch.py        |
| Astra 2      | 2.8.20                   | astra2.launch.py            |
| Gemini 2 L   | 1.4.53                   | gemini2L.launch.py          |
| Gemini 2     | 1.4.92                   | gemini2.launch.py           |

All launch files are essentially similar, with the primary difference being the default values of the parameters set
for different models within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these parameters. If you encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution settings in the launch file, as well as other parameters, to ensure compatibility and optimal performance.

## Getting start

```bash
cd ~/ros2_ws/
# build release, Default is Debug
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

Launch camera node

- On terminal 1

```bash
. ./install/setup.bash
ros2 run orbbec_camera list_devices_node #Check if the camera is connected
ros2 launch orbbec_camera gemini_330_series.launch.py # Or other launch file, see below table
```

- On terminal 2

```bash
. ./install/setup.bash
rviz2
```

Select the topic you want to display

- List topics / services/ parameters ( On terminal 3)

```bash
ros2 topic list
ros2 service list
ros2 param list
```

- Get device info

```bash
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
```

- Get SDK version

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'
```

- Get exposure

```bash
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
```

> If your check `ir` or `depth`, please change `/camera/get_color_exposure`
> to `/camera/get_ir_exposure` or `/camera/get_depth_exposure`, Same below.

- Get gain

```bash
ros2 service call /camera/get_color_gain orbbec_camera_msgs/srv/GetInt32 '{}'
```

- Get white balance

```bash
ros2 service call /camera/get_white_balance orbbec_camera_msgs/srv/GetInt32 '{}'
```

- Set auto exposure

```bash
ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}'
```

- Set white balance

```bash
ros2 service call /camera/set_white_balance orbbec_camera_msgs/srv/SetInt32 '{data: 4600}'
```

- Set laser enable

```bash
ros2 service call  /camera/set_laser_enable std_srvs/srv/SetBool "{data: true}"
```

- toggle sensor

```bash
ros2 service call /camera/toggle_ir std_srvs/srv/SetBool "{data : true}"

```

- save point cloud

```bash
ros2 service call /camera/save_point_cloud std_srvs/srv/Empty "{}"
```

## Usage

### Launch parameters

For the entire list of parameters type `ros2 param list`.

**Modify parameters when launching launch**:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_color:=true color_width:=640 color_height:=480
```

* **[color|depth|left_ir|right_ir|ir]_width,[color|depth|left_ir|right_ir|ir]_height,[color|depth|left_ir|right_ir|ir]_fps,[color|depth|left_ir|right_ir|ir]_format**
  * The resolution and frame rate of the sensor stream
  * Run `ros2 run orbbec_camera list_camera_profile_mode_node` to get the list of supported profiles
  * For example:`color_width:=640 color_height:=480 color_fps:=30 color_format:=MJPG depth_width:=640 depth_height:=480 depth_fps:=30 depth_format:=Y16 ir_width:=640 ir_height:=480 ir_fps:=30 ir_format:=Y8`
* **enable_color_auto_exposure_priority**
  * Enable the Color auto exposure priority
  * For example:`enable_color_auto_exposure_priority:=true`
* **enable_color_auto_exposure**
  * Enable the Color auto exposure
  * For example:`enable_color_auto_exposure:=true`
* **color_exposure**
  * Set the Color exposure
  * For example:`color_exposure:=30`
  * Note:To ensure that the `color_exposure` setting takes effect, make sure to set `enable_color_auto_exposure:=false`.
* **color_gain**
  * Set the Color gain
  * For example:`color_gain:=16`
  * Note:To ensure that the `color_gain` setting takes effect, make sure to set `enable_color_auto_exposure:=false`.
* **enable_depth_auto_exposure_priority**
  * Enable the Depth auto exposure priority
  * For example:`enable_depth_auto_exposure_priority:=true`
* **depth_brightness**
  * Set the Depth brightness
  * For example:`depth_brightness:=100`

[......](./docs/launch_parameters.md)

Explanation of parameters.Reference:[Launch parameters documentation](./docs/launch_parameters.md)

### All available service for camera control

For the entire list of service `ros2 service list`.

* `/camera/get_device_info`

```bash
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo
```

* `/camera/get_sdk_version`

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString
```

* `/camera/reboot_device`

```bash
ros2 service call /camera/reboot_device std_srvs/srv/Empty '{}'
```

[......](./docs/all_available_service_for_camera_control.md)

Explanation of service.Reference:[All available service fo camera control documentation](./docs/all_available_service_for_camera_control.md)

### All available topics

For the entire list of topic `ros2 topic list`.

- `/camera/color/camera_info` : The color camera info.
- `/camera/color/image_raw`: The color stream image.
- `/camera/depth/camera_info`: The depth stream info.
- `/camera/depth/image_raw`: The depth stream image.

[......](./docs/all_available_topics.md)

Explanation of topic.Reference:[All available topics documentation](./docs/all_available_topics.md)

### Network device enumeration

When using Femto Mega and Gemini 335Le, you can use the network to connect the device.

**Femto Mega:**

```bash
ros2 launch orbbec_camera femto_mega.launch.py enumerate_net_device:=true
```

**Gemini 335Le:**

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enumerate_net_device:=true
```

For more information about network device enumeration.Reference:[Network device documentation](./orbbec_camera/examples/net_camera/)

### GMSL device enumeration

When using Gemini 335Lg, you can use the GMSL to connect the device.

```bash
ros2 launch orbbec_camera gemini_330_gmsl.launch.py
```

For more information about GMSL device enumeration.Reference:[GMSL device documentation](./orbbec_camera/examples/gmsl_camera/)

### Multi-Camera

When you have multiple cameras, you can activate all of them at the same time.

For more information about Multi camera.Reference:[Multi camera documentation](./orbbec_camera/examples/multi_camera/)

### Check which profiles the camera supports

```bash
ros2 run orbbec_camera list_camera_profile_mode_node
```

### Predefined presets

About the mode settings of Predefined presets.Reference:[Predefined presets documentation](./docs/predefined_presets.md)

#### Optional depth presets

> You can pass the firmware path of the Optional preset into the `preset_firmware_path` launch param

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py preset_firmware_path:=/home/orbbec/G336X_Dimensioning_Accurate_0.0.1_78C743B9.bin,/home/orbbec/G336X_Dimensioning_Dense_0.0.1_4E70D227.bin device_preset:=G336X Dimensioning Dense
```

### Depth work mode switch

Gemini 2, Gemini 2 L,and Femto and Femto Bolt cameras setting depth work mode.Reference:[Depth work mode switch documentation](./docs/depth_work_mode_switch.md)

### Configuration of depth NFOV and WFOV modes

For the Femto Mega and Femto Bolt devices setting NFOV and WFOV modes.Reference:[Configuration of depth NFOV and WFOV modes documentation](./docs/configuration_of_depth_NFOV_and_WFOV_modes.md)

## Advanced Usage

### Use V4L2 backend

[Setting uvc_backend](./docs/launch_parameters.md)

Note: The V4L2 backend is not enabled by default.

* Example:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py uvc_backend:=v4l2
```

### DDS Tuning

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

If you use Fast DDS, you can refer to the [Fast DDS Configuration](./docs/fastdds_tuning.md) file.

### Efficient intra-process communication

Our ROS2 Wrapper node supports zero-copy communications if loaded in the same process as a subscriber node. This can reduce copy times on image/pointcloud topics, especially with big frame resolutions and high FPS.Reference:[Efficient intra-process communication documentation](./docs/efficient_intra_process_communication.md)

### ROS2(Robot) vs Optical(Camera) Coordination Systems

* Point Of View:
  * Imagine we are standing behind of the camera, and looking forward.
  * Always use this point of view when talking about coordinates, left vs right IRs, position of sensor, etc..

![ROS2 and Camera Coordinate System](./docs/images/image0.png)

* ROS2 Coordinate System: (X: Forward, Y:Left, Z: Up)
* Camera Optical Coordinate System: (X: Right, Y: Down, Z: Forward)
* All data published in our wrapper topics is optical data taken directly from our camera sensors.
* static and dynamic TF topics publish optical CS and ROS CS to give the user the ability to move from one CS to other CS.

### Camera sensor structure

![module in rviz2](./docs/images/image3.png)

![module in rviz2](./docs/images/image1.png)

### TF from coordinate A to coordinate B:

In Orbbec cameras, the origin point (0,0,0) is taken from the camera_link position

Our wrapper provide static TFs between each sensor coordinate to the camera base (camera_link)

Also, it provides TFs from each sensor ROS coordinates to its corrosponding optical coordinates.

Example of static TFs of RGB sensor and right infra sensor of Gemini335 module as it shown in rviz2:

```bash
ros2 launch orbbec_description view_model.launch.py model:=gemini_335_336.urdf.xacro
```

![module in rviz2](./docs/images/image2.png)

### Compressed Image

You can use `image_transport` to compress the image using `jpeg`. Below is an example of how to use it:

To access the compressed color image, you can use the following command:

```bash
ros2 topic echo /camera/color/image_raw/compressed --no-arr
```

This command will allow you to receive the compressed color image from the specified topic.

### Building a Debian Package

If you want to build the Debian package of OrbbecSDK_ROS2.Reference:[Building a Debian Package documentation](./docs/building_a_Debian_Package.md)

## Examples

To explore practical examples and gain insight into how to use the camera in ROS, please navigate to the [Examples](./orbbec_camera/examples/) section for more information.

## Frequently Asked Questions

### Unexpected Crash

If the camera node crashes unexpectedly, it will generate a crash log in the current running directory: `Log/camera_crash_stack_trace_xx.log`.
Please send this log to the support team or submit it to a GitHub issue for further assistance.

### No Data Stream from Multiple Cameras

**Insufficient Power Supply**:

- Ensure that each camera is connected to a separate hub.
- Use a powered hub to provide sufficient power to each camera.

**High Resolution**:

- Try lowering the resolution to resolve data stream issues.

**Increase usbfs_memory_mb Value**:

- Increase the `usbfs_memory_mb` value to 128MB (this is a reference value and can be adjusted based on your system’s needs)
  by running the following command:

```bash
    echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

- To make this change permanent, check [this link](https://github.com/OpenKinect/libfreenect2/issues/807).

### Additional Troubleshooting

- If you encounter other issues, set the `log_level` parameter to `debug`. This will generate an SDK log file in the running directory: `Log/OrbbecSDK.log.txt`.
  Please provide this file to the support team for further assistance.
- If firmware logs are required, set `enable_heartbeat` to `true` to activate this feature.

### Why Are There So Many Launch Files?

- Different cameras have varying default resolutions and image formats.
- To simplify usage, each camera has its own launch file.

## Other useful links

- [OrbbecSDK_v2](https://github.com/orbbec/OrbbecSDK_v2/releases)
- [Gemini 330 depth presets](https://www.orbbec.com/docs/g330-use-depth-presets/)

## License

Copyright 2024 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

**Other names and brands may be claimed as the property of others**
