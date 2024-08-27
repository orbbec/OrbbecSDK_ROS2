# Orbbec ROS2 SDK
 [![stable](http://badges.github.io/stability-badges/dist/stable.svg)](http://github.com/badges/stability-badges) ![version](https://img.shields.io/badge/version-1.5.11-green)

Orbbec SDK ROS 2 is a wrapper for the Orbbec 3D camera that provides seamless integration with the ROS 2 environment. It
supports ROS 2 Foxy, Humble, and Jazzy distributions.

## Table of Contents

<!-- TOC -->
* [Orbbec ROS2 SDK](#orbbec-ros2-sdk)
  * [Table of Contents](#table-of-contents)
  * [Installation Instructions](#installation-instructions)
  * [Getting start](#getting-start)
  * [Efficient intra-process communication:](#efficient-intra-process-communication)
    * [Example](#example)
      * [Manually loading multiple components into the same process](#manually-loading-multiple-components-into-the-same-process)
    * [Limitations](#limitations)
  * [Use V4L2 backend](#use-v4l2-backend)
  * [Launch parameters](#launch-parameters)
  * [Predefined presets](#predefined-presets)
  * [Depth work mode switch](#depth-work-mode-switch)
  * [Configuration of depth NFOV and WFOV modes](#configuration-of-depth-nfov-and-wfov-modes)
  * [All available service for camera control](#all-available-service-for-camera-control)
  * [All available topics](#all-available-topics)
  * [Network device enumeration](#network-device-enumeration)
  * [Multi-Camera](#multi-camera)
  * [Compressed Image](#compressed-image)
  * [Use hardware decoder to decode JPEG](#use-hardware-decoder-to-decode-jpeg)
    * [rockchip and Amlogic](#rockchip-and-amlogic)
    * [Nvidia Jetson](#nvidia-jetson)
  * [Check which profiles the camera supports](#check-which-profiles-the-camera-supports)
  * [Building a Debian Package](#building-a-debian-package)
    * [Preparing the Environment](#preparing-the-environment)
    * [Configuring ROS Dependencies](#configuring-ros-dependencies)
    * [Building the Package](#building-the-package)
  * [Launch files](#launch-files)
  * [Product support](#product-support)
  * [DDS Tuning](#dds-tuning)
  * [Frequently Asked Questions](#frequently-asked-questions)
    * [No Data Stream from Multiple Cameras](#no-data-stream-from-multiple-cameras)
    * [Why Are There So Many Launch Files Here?](#why-are-there-so-many-launch-files-here)
    * [If You Encounter Additional Issues](#if-you-encounter-additional-issues)
  * [Other useful links](#other-useful-links)
  * [License](#license)
<!-- TOC -->

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
```

Install deb dependencies

```bash
# assume you have sourced ROS environment, same blow
sudo apt install libgflags-dev nlohmann-json3-dev  \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs
```

Install udev rules.

```bash
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

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
ros2 launch orbbec_camera astra.launch.py # or other launch file, see below table
```

- On terminal 2

```bash
. ./install/setup.bash
rviz2
```

Select the topic you want to display

- List topics / services/ parameters ( on terminal 3)

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

## Efficient intra-process communication:
  
Our ROS2 Wrapper node supports zero-copy communications if loaded in the same process as a subscriber node. This can reduce copy times on image/pointcloud topics, especially with big frame resolutions and high FPS.

You will need to launch a component container and launch our node as a component together with other component nodes. Further details on "Composing multiple nodes in a single process" can be found [here](https://docs.ros.org/en/rolling/Tutorials/Composition.html).

Further details on efficient intra-process communication can be found [here](https://docs.ros.org/en/humble/Tutorials/Intra-Process-Communication.html#efficient-intra-process-communication).

### Example
#### Manually loading multiple components into the same process
* Start the component:
  ```bash
  ros2 run rclcpp_components component_container
  ```

* Add the wrapper:
  ```bash
  ros2 component load /ComponentManager orbbec_camera orbbec_camera::OBCameraNodeDriver -e use_intra_process_comms:=true
  ```
  Load other component nodes (consumers of the wrapper topics) in the same way.

#### Using a launch file

```bash
ros2 launch orbbec_camera gemini_intra_process_demo_launch.py
```

### Limitations

* Node components are currently not supported on RCLPY
* Compressed images using `image_transport` will be disabled as this isn't supported with intra-process communication

## Use V4L2 backend
To enable the V4L2 backend for the Gemini2 series cameras, follow these steps:

1. The Gemini2 series cameras support the V4L2 backend.
2. Open the `config/OrbbecSDKConfig_v1.0.xml` file.
3. Set the navigation option to `LinuxUVCBackend`.
4. Change the backend setting to `V4L2`.

Note: The V4L2 backend is not enabled by default.

## Launch parameters

The following are the launch parameters available:

- `connection_delay`: The delay time in milliseconds for reopening the device. Some devices, such as Astra mini, require
  a longer time to initialize and reopening the device immediately can cause firmware crashes when hot plugging.
- `enable_point_cloud`: Enables the point cloud.
- `enable_colored_point_cloud`: Enables the RGB point cloud.
- `point_cloud_qos`, `[color|depth|ir]_qos`, `[color|depth|ir]_camera_info_qos`: ROS 2 Message Quality of Service (QoS)
  settings. The possible values
  are `SYSTEM_DEFAULT`, `DEFAULT`, `PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA` and are
  case-insensitive. These correspond
  to `rmw_qos_profile_system_default`, `rmw_qos_profile_default`, `rmw_qos_profile_parameter_events`, `rmw_qos_profile_services_default`, `rmw_qos_profile_parameters`,
  and `SENSOR_DATA`, respectively.
- `enable_d2c_viewer`: Publishes the D2C overlay image (for testing only).
- `device_num`: The number of devices. This must be filled in if multiple cameras are required.
- `color_width`, `color_height`, `color_fps`: The resolution and frame rate of the color stream.
- `ir_width`, `ir_height`, `ir_fps`: The resolution and frame rate of the IR stream.
- `depth_width`, `depth_height`, `depth_fps`: The resolution and frame rate of the depth stream.
- `enable_color`: Enables the RGB camera.
- `enable_depth`: Enables the depth camera.
- `enable_ir`: Enables the IR camera.
- `depth_registration`: Enables alignment of the depth frame to the color frame. This field is required when
  the `enable_colored_point_cloud` is set to `true`.
- `usb_port`: The USB port of the camera. This is required when multiple cameras are used.
- `enable_accel`: Enables the accelerometer.
- `accel_rate`: The frequency of the accelerometer, the optional values
  are `1.5625hz`, `3.125hz`, `6.25hz`, `12.5hz`, `25hz`, `50hz`, `100hz`, `200hz`, `500hz`, `1khz`, `2khz`, `4khz`, `8khz`, `16khz`, `32khz`.
  The specific value depends on the current camera.
- `accel_range`: The range of the accelerometer, the optional values are `2g`, `4g`, `8g`, `16g`. The specific value
  depends on the current camera.
- `enable_gyro`: Enables the gyroscope.
- `gyro_rate`: The frequency of the gyroscope, the optional values
  are `1.5625hz`, `3.125hz`, `6.25hz`, `12.5hz`, `25hz`, `50hz`, `100hz`, `200hz`, `500hz`, `1khz`, `2khz`, `4khz`, `8khz`, `16khz`, `32khz`.
  The specific value depends on the current camera.
- `gyro_range`: The range of the gyroscope, the optional values
  are `16dps`, `31dps`, `62dps`, `125dps`, `250dps`, `500dps`, `1000dps`, `2000dps`. The specific value depends on the
  current camera.
- `enumerate_net_device`: Enables the function of enumerating network devices. True means enabled, false means disabled.
  This feature is only supported by Femto Mega and Gemini 2 XL devices. When accessing these devices through the
  network, the IP address of the device needs to be configured in advance. The enable switch needs to be set to true.
- `depth_filter_config`: Configures the loading path for the depth filtering configuration file. By default, the depth
  filtering configuration file is located in the /config/depthfilter directory. Supported only on Gemini2.
- `depth_precision`: The depth precision should be in the format `1mm`. The default value is `1mm`.
- `enable_laser`: Enables the laser. The default value is `true`.
- `laser_on_off_mode`: Laser on/off alternate mode, 0: off, 1: on-off alternate, 2: off-on alternate. The default value
  is `0`.
- `device_preset`: The default value is `Default`. Only the G330 series is supported. For more information, refer to
  the [G330 documentation](https://www.orbbec.com/docs/g330-use-depth-presets/). Please refer to the table below to set
  the `device_preset` value based on your use case. The value should be one of the preset names
  listed [in the table](#predefined-presets).
- `enable_decimation_filter`: This filter effectively reduces the depth scene complexity. The filter runs on kernel
  sizes [2x2] to [8x8] pixels. The image size is scaled down proportionally in both dimensions to preserve the aspect
  ratio.
- `enable_hdr_merge`: This filter is used jointly with the depth HDR function. By merging consecutive depth images of
  alternating exposure values, we can overcome challenges in acquiring depth values for under-illuminated and
  over-illuminated objects simultaneously.
- `enable_sequence_id_filter`: This filter is used jointly with the depth HDR function and outputs only the sequence
  with the specified sequence ID.
- `enable_threshold_filter`: This filter preserves depth values of interest and omits depth values out of scope.
- `enable_noise_removal_filter`: This filter removes speckle noise in clusters and gives rise to a less-filled depth
  map.
- `enable_spatial_filter`: This filter performs multiple iterations of processing as specified by the magnitude
  parameter to enhance the smoothness of depth data. It is also capable of filling small holes in depth maps.
- `enable_temporal_filter`: This filter is intended to improve the depth data persistency by manipulating per-pixel
  values based on previous frames. The filter performs a single pass on the data, adjusting the depth values while also
  updating the tracking history.
- `enable_hole_filling_filter`: This filter fills all holes in the depth map using the specified mode.
- `retry_on_usb3_detection_failure`: If the camera is connected to a USB 2.0 port and is not detected, the system will
  attempt to reset the camera up to three times. This setting aims to prevent USB 3.0 devices from being incorrectly
  recognized as USB 2.0. It is recommended to set this parameter to `false` when using a USB 2.0 connection to avoid
  unnecessary resets.

- `enable_3d_reconstruction_mode`: Enables 3D reconstruction mode. Default is `false`. When set to `true`, the camera 
laser operates in on-off mode, capturing IR images (laser off) for VSLAM localization and depth images (laser on) for point cloud computation.
- `tf_publish_rate`: The rate at which the camera publishes dynamic transforms. The default value is `0.0`, which means static transforms are published.
- `time_domain`: The frame time domain, string type, can be `device`, `global`, or `system`. `device` means using the hardware timestamp from the camera,
`system` means using the timestamp when the PC received the first packet of data or frame, and `global` is used for synchronized time across multiple
devices, aligning data from different sources to a common time base.
- `enable_sync_host_time`: Enables synchronization of the host time with the camera time. The default value is `true`, if
use global time, set to `false`. Some old devices may not support this feature.
- `config_file_path`: The path to the YAML configuration file. The default value is `""`. If the configuration file is not specified,
the default parameters from the launch file will be used. If you want to use a custom configuration file, please refer to `gemini_330_series.launch.py`.
  `enable_heartbeat` enables the heartbeat function, which is set to `false` by default. If set to `true`, the camera node will send heartbeat signals to 
the firmware, and if hardware logging is desired, it should also be set to `true`.
- `log_level` : SDK log level, the default value is `info`, the optional values are `debug`, `info`, `warn`, `error`, `fatal`.
- `enable_color_undistortion`: Enables color undistortion, the default value is `false`. Note that our color cameras exhibit minimal distortion, and typically, undistortion is not necessary.

**IMPORTANT**: *Please carefully read the instructions regarding software filtering settings
at [this link](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/). If you are uncertain, do not modify
these settings.*

## Predefined presets

| Preset         | Features                                                                                                                                                                                                      | Recommended use cases                                                                                                                                         |
|----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Default        | - Best visual perception<br/>- Overall good performance in accuracy, fill rate, tiny objects, etc.                                                                                                            | - Generic<br>- Robotics                                                                                                                                       |
| Hand           | - Clear hand and finger edges                                                                                                                                                                                 | - Gesture recognition                                                                                                                                         |
| High Accuracy  | - Depth of high confidence<br>- Barely noise depth values<br>- Lower fill rate                                                                                                                                | - Collision avoidance<br>- Object scanning                                                                                                                    |
| High Density   | - Higher fill rate<br>- More tiny objects<br>- May suffer from noise depth values                                                                                                                             | - Object recognition<br>- Pick & place<br>- Foreground & background animation                                                                                 |
| Medium Density | - Balanced performance in fill rate and accuracy<br>- In comparison to Default: lower fill rate, better edge quality                                                                                          | - Generic and alternative to Default                                                                                                                          |
| Custom         | - User defined Preset<br>- Derived from Presets above, with customized modifications, e.g. a new configuration for the post-processing pipeline, modified mean intensity set point of depth AE function, etc. | - Better depth performance achieved using customized configurations in comparison to using predefined presets<br>- For well-established custom configurations |

Choose the appropriate preset name based on your specific use case and set it as the value for the `device_preset`
parameter.

## Depth work mode switch

Orbbec SDK ROS 2 supports the depth work mode switch. The depth work mode switch is supported by Gemini 2, Gemini 2 L,
and Femto and Femto Bolt cameras.

- Before starting the camera, depth work mode (depth_work_mode) can be configured for the corresponding xxx.launch.py
  file's support.
- The depth work mode switch is supported by Gemini 2, Gemini 2 L, and Gemini 2 XL cameras.
- The default depth work mode configuration of xxx.launch.py is the camera's default configuration. If you need to
  modify it, you can switch to the corresponding mode as needed.
- The specific camera depth work mode support types can be found in the comments of the depth mode.

```python
# Depth work mode support is as follows:
# Unbinned Dense Default
# Unbinned Sparse Default
# Binned Sparse Default
# Obstacle Avoidance
DeclareLaunchArgument('depth_work_mode', default_value='')
```

- View depth work modes:

```bash
ros2 run orbbec_camera list_depth_work_mode_node
```

## Configuration of depth NFOV and WFOV modes

For the Femto Mega and Femto Bolt devices, the NFOV and WFOV modes are implemented by configuring the resolution of
Depth and IR in the launch file.
In launch file, depth_width、depth_height、ir_width、ir_height represents the resolution of the depth and the resolution of
the IR.
The frame fps and resolution of IR must be consistent with the depth. The correspondence between different modes and
resolutions is as follows:

- NFOV unbinned: 640 x 576.
- NFOV binned: 320 x 288.
- WFOV unbinned: 1024 x 1024.
- WFOV binned: 512 x 512.

## All available service for camera control

The name of the following service already expresses its function.
However, it should be noted that the corresponding `set_[ir|depth|color]*`
and `get[ir|depth|color]*` **services are only available if you set** `enable[ir|depth|color]`
to `true` in the stream that corresponds to the argument of the launch file.

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

## All available topics

- `/camera/color/camera_info` : The color camera info.
- `/camera/color/image_raw`: The color stream image.
- `/camera/depth/camera_info`: The depth stream image.
- `/camera/depth/image_raw`: The depth stream image
- `/camera/depth/points` : The point cloud, only available when `enable_point_cloud` is `true`.
- `/camera/depth_registered/points`: The colored point cloud, only available when `enable_colored_point_cloud`
  is `true`.
- `/camera/ir/camera_info`: The IR camera info.
- `/camera/ir/image_raw`: The IR stream image
- `/camera/accel/sample`: Acceleration data stream `enable_sync_output_accel_gyro`turned off，`enable_accel`turned on
- `/camera/gyro/sample`: Gyroscope data stream，enable_sync_output_accel_gyro`turned off，`enable_gyro`turned on
- `camera/gyro_accel/sample`: Synchronized data stream of acceleration and gyroscope，`enable_sync_output_accel_gyro`
  turned on
- `/diagnostics`: The diagnostic information of the camera, Currently, the diagnostic information only includes the
  temperature of the camera.

## Network device enumeration

Currently, the network device enumeration function is supported only by the Femto Mega device. When accessing this
device over the network, if `enumerate_net_device` is set to `true`, the device will be automatically enumerated,
eliminating the need to configure the IP address in advance or set the enable switch to true. The specific configuration
methods are as follows:

- `enumerate_net_device`: enumeration network device automatically, only supported by Femto Mega.
  if `enumerate_net_device` set to `true`, the device will be enumerated automatically,No need to set
  the `net_device_ip`
  and `net_device_port` parameters.
- `net_device_ip`: The IP address of the device.
- `net_device_port`: The port number of the device.

## Multi-Camera

- To get the `usb_port` of the camera, plug in the camera and run the following command in the terminal:

```bash
ros2 run orbbec_camera list_devices_node
```

- Set the `device_num` parameter to the number of cameras you have.
- Go to the `OrbbecSDK_ROS2/launch/multi_xxx.launch.py` file and change the `usb_port`.
- Don't forget to put the `include` tag inside the `group` tag.
  Otherwise, the parameter values of different cameras may become contaminated.

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

- To launch the cameras, run the following command:

```bash
ros2 launch orbbec_camera multi_camera.launch.py
```

## Compressed Image
You can use `image_transport` to compress the image using `jpeg`. Below is an example of how to use it:

To access the compressed color image, you can use the following command:
```bash
ros2 topic echo /camera/color/image_raw/compressed --no-arr
```

This command will allow you to receive the compressed color image from the specified topic.

## Use hardware decoder to decode JPEG

### rockchip and Amlogic

Depends on `rockchip-mpp-dev` and `rockchip-rga-dev`, not all systems have these two packages, the names may be
different, please search by yourself.
Open `CMakeLists.txt` and set `USE_RK_HW_DECODER` to `ON`.

### Nvidia Jetson

Depends on: `jetson_multimedia_api`,`libyuv`.
Open `CMakeLists.txt` and set `USE_NV_HW_DECODER` to `ON`.

## Check which profiles the camera supports

```bash
ros2 run orbbec_camera list_camera_profile_mode_node
```

## Building a Debian Package

### Preparing the Environment

Before starting, install the required tools:

```bash
sudo apt install debhelper fakeroot python3-bloom
```

### Configuring ROS Dependencies

Add the following YAML file to your system at `/etc/ros/rosdep/sources.list.d/00-orbbec.yaml`. Make sure to
replace `focal` with the codename of your Ubuntu version and `humble` with your ROS2 distribution name:

```yaml
orbbec_camera_msgs:
  ubuntu:
    focal: [ ros-humble-orbbec-camera-msgs ]
```

Next, create a new file `/etc/ros/rosdep/sources.list.d/50-orbbec.list` and add this line to specify the path to the
YAML file:

```bash
yaml file:///etc/ros/rosdep/sources.list.d/00-orbbec.yaml
```

Update the rosdep database to reflect these changes:

```bash
rosdep update
```

### Building the Package

Navigate to your workspace and build the project:

```bash
cd ~/ros2_ws/
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
cd src/OrbbecSDK_ROS2/
bash .make_deb.sh
```

## Launch files

| product serials                       | launch file                 |
|---------------------------------------|-----------------------------|
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
| gemini 330 series                     | gemini_330_series.launch.py |

**All launch files are essentially similar, with the primary difference being the default values of the parameters set
for different models
within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these
parameters. If you
encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution
settings in the launch
file, as well as other parameters, to ensure compatibility and optimal performance.**

## Product support

Please refer to the OrbbecSDK supported
products: [Product Support](https://github.com/orbbec/OrbbecSDK?tab=readme-ov-file#product-support)

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

If you use Fast DDS, you can refer to the [Fast DDS Configuration](./docs/fastdds_tuning.md) file.

## Frequently Asked Questions

### No Data Stream from Multiple Cameras

**Insufficient Power Supply**:
  - Ensure that all cameras are not connected to the same hub.
  - Use a powered hub to provide sufficient power to each camera.
**High Resolution**:
  - Try lowering the resolution to resolve data stream issues.

**Increase usbfs_memory_mb Value**:
  - Increase the `usbfs_memory_mb` value to 128MB by running the following command:
    ```bash
    echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
    ```
  - For making this change permanent, check [this link](https://github.com/OpenKinect/libfreenect2/issues/807).

### Why Are There So Many Launch Files Here?

- Multiple launch files exist because different cameras have varying default resolutions and image formats.
- To simplify usage, each camera has its own launch file.

### If You Encounter Additional Issues

- If you encounter other issues, set the `log_level` parameter to `debug`. This will generate an SDK Log file in the running directory. Provide this file to the support team for further assistance.
- If firmware logs are required, set `enable_heartbeat` to `true` to activate this feature.

## Other useful links

- [Orbbec SDK](https://orbbec.github.io/OrbbecSDK/)
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
