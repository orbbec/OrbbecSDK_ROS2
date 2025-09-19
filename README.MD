# OrbbecSDK ROS2 Wrapper v2

> [!IMPORTANT]
>
> Welcome to the OrbbecSDK ROS2 Wrapper v2. Before you begin using this version of ROS2 wrapper, it's crucial to check the following [device support list](#supported-devices) to verify the compatibility.

[English](./README.MD) | [中文](./README_CN.MD)

OrbbecSDK ROS2 Wrapper provides seamless integration of Orbbec cameras with ROS 2 environment. It supports ROS2 Foxy, Humble, and Jazzy distributions.

By default, we recommend using the **v2-main** branch. For older OpenNI devices not supported by v2-main, please use the **main** branch. Device models that are only supported by the main branch are listed in the table below.

If you are a user in China, it is recommended to use [gitee Repo](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2).

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
      <td style="text-align: center; font-weight: bold;">Gemini 435Le</td>
      <td>Gemini 435Le</td>
      <td>Not supported</td>
      <td>Recommended for new designs</td>
    </tr>
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
      <td>Gemini 330</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330L</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="5" style="text-align: center; font-weight: bold;">Gemini 2</td>
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
      <td>Gemini 215</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 210</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
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
      <td rowspan="1" style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini Pro</td>
      <td>Full maintenance</td>
      <td>Full maintenance</td>
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
sudo apt install libgflags-dev nlohmann-json3-dev \
ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev
```

Optional dependencies:

```bash
# 435Le writeCustomerDate feature:
sudo apt install libssl-dev
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

| Product List   | Minimal Firmware Version | **Launch File**       |
| :------------- | :----------------------- | :-------------------------- |
| Gemini 435Le   | 1.2.04                   | gemini435_le.launch.py      |
| Gemini 335     | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 336     | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 335L    | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 336L    | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 335Lg   | 1.3.46                   | gemini_330_series.launch.py |
| Gemini 335Le   | 1.5.31                   | gemini_330_series.launch.py |
| Gemini 330     | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 330L    | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 2       | 1.4.92                   | gemini2.launch.py           |
| Gemini 2 L     | 1.4.53                   | gemini2L.launch.py          |
| Femto Bolt     | 1.1.2                    | femto_bolt.launch.py        |
| Femto Mega     | 1.3.0                    | femto_mega.launch.py        |
| Astra 2        | 2.8.20                   | astra2.launch.py            |
| Astra Mini Pro | 2.0.01                   | astra.launch.py             |

All launch files are essentially similar, with the primary difference being the default values of the parameters set
for different models within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these parameters. If you encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution settings in the launch file, as well as other parameters, to ensure compatibility and optimal performance.

## Getting start

```bash
cd ~/ros2_ws/
source /opt/ros/$ROS_DISTRO/setup.bash
# build release, Default is Debug
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

Launch camera node

- On terminal 1

```bash
source ~/ros2_ws/install/setup.bash
ros2 run orbbec_camera list_devices_node #Check if the camera is connected
ros2 launch orbbec_camera gemini_330_series.launch.py # Or other launch file, see below table
```

- On terminal 2

```bash
source ~/ros2_ws/install/setup.bash
rviz2
```

Select the topic you want to display

- List topics / services/ parameters ( On terminal 3)

```bash
ros2 topic list
ros2 service list
ros2 param list
```

- Echo a topic

```bash
ros2 topic echo /camera/depth/camera_info
```

- Call a service

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'
```

- Record with rosbag2

```bash
ros2 bag record /camera/color/image_raw /camera/depth/image_raw
```

For more usage details, please refer to the official [OrbbecSDK ROS2 documentation](https://orbbec.github.io/OrbbecSDK_ROS2/source/4_application_guide/application_guide.html)

## Usage

### Check which profiles the camera supports

```bash
ros2 run orbbec_camera list_camera_profile_mode_node
```

### Optional depth presets

> You can pass the firmware path of the Optional preset into the `preset_firmware_path` launch param.Users can download the appropriate Preset from the [Depth Preset Release](https://www.orbbec.com/docs/depth-preset-release/) page provided by Orbbec based on their specific application scenarios and update it to the camera for better depth performance in corresponding scenarios.

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py preset_firmware_path:=/home/orbbec/G336X_AMR_Default_v0.0.5.bin
```

### Use V4L2 backend

Note: The V4L2 backend is not enabled by default.

* Example:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py uvc_backend:=v4l2
```

For more usage details, please refer to the official [OrbbecSDK ROS2 documentation](https://orbbec.github.io/OrbbecSDK_ROS2/source/5_advanced_guide/advanced_guide.html)

## Examples

To explore practical examples and gain insight into how to use the camera in ROS, please navigate to the [Examples](./orbbec_camera/examples/) section for more information.

## Other useful links

- [OrbbecSDK ROS2 documentation](https://orbbec.github.io/OrbbecSDK_ROS2/index.html)
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
