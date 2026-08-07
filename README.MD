<div align="center">
<!-- Title: -->
  <a href="https://www.orbbec.com/products/">
    <img src="./docs/images/orbbec-logo-horizontal.png" width="500" alt="Orbbec">
  </a>
  <h1><a href="https://github.com/orbbec/OrbbecSDK_ROS2">OrbbecSDK ROS2 Wrapper v2</a></h1>

<!-- Language: -->
  <p align="center">
    <a href="./README.MD">English</a> | <a href="./README_CN.MD">中文</a>
  </p>

<!-- Labels: -->
  <!-- First row: -->
  <a href="https://docs.ros.org/en/foxy/">
    <img src="https://img.shields.io/badge/ROS-Foxy-f5792a?style=flat-square&amp;logo=ros&amp;logoColor=white" height="20" alt="ROS Foxy">
  </a>
  <a href="https://docs.ros.org/en/humble/">
    <img src="https://img.shields.io/badge/ROS-Humble-f5792a?style=flat-square&amp;logo=ros&amp;logoColor=white" height="20" alt="ROS Humble">
  </a>
  <a href="https://docs.ros.org/en/jazzy/">
    <img src="https://img.shields.io/badge/ROS-Jazzy-f5792a?style=flat-square&amp;logo=ros&amp;logoColor=white" height="20" alt="ROS Jazzy">
  </a>
  <a href="https://docs.ros.org/en/lyrical/">
    <img src="https://img.shields.io/badge/ROS-Lyrical-f5792a?style=flat-square&amp;logo=ros&amp;logoColor=white" height="20" alt="ROS Lyrical">
  </a>
  <a href="https://docs.ros.org/en/rolling/">
    <img src="https://img.shields.io/badge/ROS-Rolling%20(experimental)-e5c07b?style=flat-square&amp;logo=ros&amp;logoColor=white" height="20" alt="ROS Rolling (experimental)">
  </a>
  <img src="https://img.shields.io/badge/Ubuntu-20.04-0078d4?style=flat-square&amp;logo=ubuntu&amp;logoColor=white" height="20" alt="Ubuntu 20.04">
  <img src="https://img.shields.io/badge/Ubuntu-22.04-0078d4?style=flat-square&amp;logo=ubuntu&amp;logoColor=white" height="20" alt="Ubuntu 22.04">
  <img src="https://img.shields.io/badge/Ubuntu-24.04-0078d4?style=flat-square&amp;logo=ubuntu&amp;logoColor=white" height="20" alt="Ubuntu 24.04">
  <img src="https://img.shields.io/badge/Ubuntu-26.04-0078d4?style=flat-square&amp;logo=ubuntu&amp;logoColor=white" height="20" alt="Ubuntu 26.04">

  <!-- Second row: -->
  <br>
  <a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main">
    <img src="https://img.shields.io/badge/branch-v2--main-555555?style=flat-square" height="20" alt="branch v2-main">
  </a>
  <a href="http://www.apache.org/licenses/LICENSE-2.0">
    <img src="https://img.shields.io/badge/license-Apache--2.0-6bcf00?style=flat-square" height="20" alt="license Apache-2.0">
  </a>
  <a href="https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2">
    <img src="https://img.shields.io/badge/Gitee-mirror-c71d23?style=flat-square" height="20" alt="Gitee mirror">
  </a>
</div>

## Introduction

The **OrbbecSDK ROS2 Wrapper** provides seamless integration of Orbbec cameras with the ROS 2 ecosystem and supports the **ROS2 Foxy**, **Humble**, **Jazzy**, and **Lyrical** distributions, with experimental support for **Rolling**.

- Default branch: **v2-main**
- For legacy **OpenNI** devices: use the **main** branch
- For users in China: use the [Gitee repository](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2)

## branch selection

Here is the device support list of main branch (v1.x) and v2-main branch (v2.x):

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr style="background-color: #1f4e78; color: black; text-align: center;">
      <th>Product Series</th>
      <th>Product</th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/main" style="color: black; text-decoration: none;">Branch main</a></th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main" style="color: black; text-decoration: none;">Branch v2-main</a></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 430</td>
      <td>Gemini 435Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">Gemini 301</td>
      <td>Gemini 305</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 305g</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="10" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 338Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 338Lg</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 340</td>
      <td>Gemini 345Lg</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="5" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td>recommended for new designs</td>
      <td>not supported</td>
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
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td>limited maintenance</td>
      <td>not supported</td>
    </tr>
    <tr>
      <td>Astra Pro Plus</td>
      <td>limited maintenance</td>
      <td>not supported</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini Pro</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Astra Mini S Pro</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">LiDAR</td>
      <td>Pulsar ME450</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Pulsar SL450</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
  </tbody>
</table>

**Note**: LiDAR devices are supported in v2.6.3 and later versions. For usage details, please refer to the [LiDAR documentation](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/lidar_devices/index.html). If you do not find your device, please contact our FAE or sales representative for help.

**Definition**:

1. Recommended for new designs: we will provide full supports with new features,  bug fix and performance optimization;
2. Full maintenance: we will provide bug fix support;
3. Limited maintenance: we will provide critical bug fix support;
4. Not supported: we will not support specific device in this version;
5. To be supported: we will add support in the near future.

**Migration Guide**

If you need to migrate existing projects from the main (v1.x) branch to the v2-main (v2.x) branch, please refer to the official [migration guide](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/7_developer_guide/migration_guide.html)

## Installation Instructions

**Environment configuration**

Install ROS 2 according to the official guide:

* [ROS 2 installation (Ubuntu)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Install dependencies:

```bash
sudo apt install libgflags-dev nlohmann-json3-dev \
ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs ros-$ROS_DISTRO-xacro \
ros-$ROS_DISTRO-backward-ros libdw-dev libssl-dev mesa-utils libgl1 libgoogle-glog-dev
```

Enable ROS 2 auto-completion:

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

**Linux Binary Package Installation**

Check available packages:

```bash
sudo apt update
apt list | grep orbbec
```

Install OrbbecSDK ROS2 package:

```bash
sudo apt install ros-humble-orbbec-camera ros-humble-orbbec-description
```

After installation, you can use it directly without compilation.

**Build from Source**

Create a `colcon` workspace:

```bash
mkdir -p ~/ros2_ws/src
```

Clone source and checkout `v2-main` branch:

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout v2-main
```

Build:

```bash
cd ~/ros2_ws
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Registration Script (Required)

To allow the Orbbec cameras to be recognized correctly on Linux, install the udev rules.

**Binary Installation**

```bash
sudo cp /opt/ros/$ROS_DISTRO/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Build from Source**

```bash
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This step is **mandatory** for Linux users.

`Notes:` If this script is not executed, open the device will fail due to permission issues. You need to run the sample with sudo (administrator privileges).

## Getting start

Launch camera node

- On terminal 1

```bash
source ~/ros2_ws/install/setup.bash
ros2 run orbbec_camera list_devices_node # Check if the camera is connected
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

For more usage details, please refer to the official [OrbbecSDK ROS2 documentation](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/application_guide.html)

## Supported Devices

Currently, the following devices are supported by the OrbbecSDK ROS2 Wrapper v2-main branch.  More devices support will be added in the near future. If you can not find your device in the table below, try the [main](https://github.com/orbbec/OrbbecSDK_ROS2)  branch.

For optimal performance, we strongly recommend updating to the latest firmware version. This ensures that you benefit from the most recent enhancements and bug fixes.

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr>
      <th>Product Series</th>
      <th>Products List</th>
      <th>Recommended FW Version</th>
      <th>Launch File</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 430</td>
      <td>Gemini 435Le</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemin435Le-Firmware">1.3.19</a></td>
      <td>gemini435_le.launch.py</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">Gemini 301</td>
      <td>Gemini 305</td>
      <td><a href="https://doc.orbbec.com/documentation/Gemini%20305%20Documentation/Firmware%20Release%20of%20Gemini%20305">1.0.70</a></td>
      <td>gemini_301_series.launch.py</td>
    </tr>
    <tr>
      <td>Gemini 305g</td>
      <td><a href="https://doc.orbbec.com/documentation/Gemini%20305%20Documentation/Firmware%20Release%20of%20Gemini%20305">1.0.70</a></td>
      <td>gemini_301_series.launch.py</td>
    </tr>
    <tr>
      <td rowspan="10" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335</td>
      <td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.8.10.zip">1.8.10</a></td>
      <td>gemini_330_series.launch.py</td>
    </tr>
    <tr><td>Gemini 336</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.8.10.zip">1.8.10</a></td><td>gemini_330_series.launch.py</td></tr>
    <tr><td>Gemini 330</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.8.10.zip">1.8.10</a></td><td>gemini_330_series.launch.py</td></tr>
    <tr><td>Gemini 335L</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.8.10.zip">1.8.10</a></td><td>gemini_330_series.launch.py</td></tr>
    <tr><td>Gemini 336L</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.8.10.zip">1.8.10</a></td><td>gemini_330_series.launch.py</td></tr>
    <tr><td>Gemini 330L</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.8.10.zip">1.8.10</a></td><td>gemini_330_series.launch.py</td></tr>
    <tr><td>Gemini 335Lg</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.8.10.zip">1.8.10</a></td><td>gemini_330_series.launch.py</td></tr>
    <tr><td>Gemini 335Le</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.8.10.zip">1.8.10</a></td><td>gemini_330_series.launch.py</td></tr>
    <tr><td>Gemini 338Le</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_v1.8.17.zip">1.8.17</a></td><td>gemini_330_series.launch.py</td></tr>
    <tr><td>Gemini 338Lg</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_v1.8.17.zip">1.8.17</a></td><td>gemini_330_series.launch.py</td></tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 340</td>
      <td>Gemini 345Lg</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini340-Firmware">1.9.03</a></td>
      <td>gemini345_lg.launch.py</td>
    </tr>
    <tr>
      <td rowspan="4" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2-Firmware">1.4.98</a></td>
      <td>gemini2.launch.py</td>
    </tr>
    <tr>
      <td>Gemini 2 L</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2L-Firmware">1.5.2</a></td>
      <td>gemini2L.launch.py</td>
    </tr>
    <tr>
      <td>Gemini 215</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini215-Firmware">1.0.9</a></td>
      <td>gemini210.launch.py</td>
    </tr>
    <tr>
      <td>Gemini 210</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini210-Firmware">1.0.9</a></td>
      <td>gemini210.launch.py</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td>
      <td>Femto Bolt</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Bolt-Firmware">1.1.3</a></td>
      <td>femto_bolt.launch.py</td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-Firmware">1.3.1</a></td>
      <td>femto_mega.launch.py</td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-I-Firmware">2.0.4</a></td>
      <td>femto_mega.launch.py</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Astra2_Release_2.8.20.zip">2.8.20</a></td>
      <td>astra2.launch.py</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini Pro</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-Pro">2.0.03</a></td>
      <td>astra.launch.py</td>
    </tr>
    <tr>
      <td>Astra Mini S Pro</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-S-Pro">2.0.03</a></td>
      <td>astra.launch.py</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">LiDAR</td>
      <td>Pulsar ME450</td>
      <td>1.0.0.6</td>
      <td>lidar.launch.py</td>
    </tr>
    <tr>
      <td>Pulsar SL450</td>
      <td>2.2.4.5</td>
      <td>lidar.launch.py</td>
    </tr>
  </tbody>
</table>

All launch files are essentially similar, with the primary difference being the default values of the parameters set
for different models within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these parameters. If you encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution settings in the launch file, as well as other parameters, to ensure compatibility and optimal performance.

## Tutorial

please refer to the official [OrbbecSDK ROS2 documentation](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/advanced_guide.html)

## Examples

To explore practical examples and gain insight into how to use the camera in ROS, please navigate to the [Examples](./orbbec_camera/examples/) section for more information.

## Other useful links

- [OrbbecSDK ROS2 documentation](https://orbbec.github.io/OrbbecSDK_ROS2/en/index.html)
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
