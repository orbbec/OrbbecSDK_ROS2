# OrbbecSDK ROS2 Wrapper v2

[English](./README.MD) | [中文](./README_CN.MD)

## 简介

OrbbecSDK ROS2 Wrapper 提供 Orbbec 相机与 ROS 2 环境的无缝集成，支持 **ROS2 Foxy、Humble 和 Jazzy** 发行版。

- 默认推荐使用 **v2-main** 分支。
- 对于旧的 **OpenNI** 设备（v2-main 不支持），请使用 **main** 分支。
- 如果您在中国使用，建议使用 [gitee 仓库](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2)。

## 分支选择

以下是 main 分支 (v1.x) 和 v2-main 分支 (v2.x) 的设备支持列表：

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
      <td style="text-align: center; font-weight: bold;">Gemini 435Le</td>
      <td>Gemini 435Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
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
      <td>to be supported</td>
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
      <td style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini (S) Pro</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini (S) Pro</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">LiDAR</td>
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
**注意**: 如果您没有找到对应的设备，请联系 FAE 或销售代表获取帮助。

**定义**:

1. 新设计推荐：我们将提供完整支持，包括新功能、bug 修复和性能优化；
2. 完全维护：我们将提供 bug 修复支持；
3. 有限维护：我们将提供关键 bug 修复支持；
4. 不支持：此版本不会支持该设备；
5. 即将支持：我们将在近期添加支持。

**迁移指南**

如果您需要将现有项目从 main (v1.x) 分支迁移到 v2-main (v2.x) 分支，请参考官方[迁移指南](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/7_developer_guide/migration_guide.html)

## 安装说明

**环境配置**

根据官方指南安装 ROS 2：

* [ROS 2 安装指南（Ubuntu）](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

安装依赖项：

```bash
sudo apt install libgflags-dev nlohmann-json3-dev \
ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs ros-$ROS_DISTRO-xacro \
ros-$ROS_DISTRO-backward-ros libdw-dev libssl-dev mesa-utils libgl1
```

启用 ROS 2 自动补全：

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

**二进制安装**

查看软件包：

```bash
sudo apt update
apt list | grep orbbec
```

安装 OrbbecSDK ROS2 包：

```bash
sudo apt install ros-humble-orbbec-camera ros-humble-orbbec-description
```

安装完成后，直接使用即可，无需编译。

**从源码构建**

创建 `colcon` 工作空间：

```bash
mkdir -p ~/ros2_ws/src
```

克隆源代码并切换到 `v2-main` 分支：

```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout v2-main
```

构建：

```bash
cd ~/ros2_ws
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```
## 注册脚本（必需）

为了让 Orbbec 相机在 Linux 上被正确识别，请安装 udev 规则。

**二进制安装**

```bash
sudo cp /opt/ros/$ROS_DISTRO/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**从源码构建**

```bash
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

此步骤对于 Linux 用户是**必需的**。

`注意：` 如果不执行此脚本，由于权限问题，打开设备将会失败。您需要使用 sudo（管理员权限）运行示例程序。

## 快速开始

启动相机节点

- 终端 1

```bash
source ~/ros2_ws/install/setup.bash
ros2 run orbbec_camera list_devices_node #检查相机是否已连接
ros2 launch orbbec_camera gemini_330_series.launch.py # 或其他启动文件，见下表
```

- 终端 2

```bash
source ~/ros2_ws/install/setup.bash
rviz2
```

选择需要显示的 topic

- 列出 topics / services / parameters (终端 3)

```bash
ros2 topic list
ros2 service list
ros2 param list
```

- 查看 topic

```
ros2 topic echo /camera/depth/camera_info
```

- 调用服务

```
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'
```

更多使用详情，请参考官方 [OrbbecSDK ROS2 文档](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/4_application_guide/application_guide.html)

## 支持的设备

目前 v2-main 分支支持以下设备。更多设备支持将陆续增加。如果没有找到您的设备，请尝试 [main](https://github.com/orbbec/OrbbecSDK_ROS2) 分支。

为获得最佳性能，强烈建议更新至最新固件版本，以确保您能获得最新的改进和 bug 修复。

| **产品列表** | **推荐固件版本**                                                             |  **启动文件**                                          |
| ----------------------- | -------------------------------------------------------------------------------------------- | --------------------------------------------- |
| Astra Mini Pro           | [2.0.03](https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-Pro)             |                                               astra.launch.py   |
| Astra Mini S Pro            | [2.0.03](https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-S-Pro)             |                                               astra.launch.py   |
| Gemini 435Le            | [1.3.6](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemin435Le-Firmware)             |                                               gemini435_le.launch.py   |
| Gemini 330 series       | [1.6.00](https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip) | gemini_330_series.launch.py |
| Gemini 215              | [1.0.9](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini215-Firmware)               |                                               gemini210.launch.py   |
| Gemini 210              | [1.0.9](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini210-Firmware)               |                                               gemini210.launch.py    |
| Gemini 2                | [1.4.98](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2-Firmware)                |                                               gemini2.launch.py     |
| Gemini 2 L              | [1.5.2](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2L-Firmware)                |                                               gemini2L.launch.py  |
| Femto Bolt              | [1.1.2](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Bolt-Firmware)              |                               femto_bolt.launch.py |
| Femto Mega              | [1.3.1](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-Firmware)              |                                               femto_mega.launch.py  |
| Femto Mega I             | [2.0.4](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-I-Firmware)              |                                               femto_mega.launch.py  |
| Astra 2                 | [2.8.20](https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Astra2_Release_2.8.20.zip)    |                                               astra2.launch.py    |
| Pulsar SL450            | 2.2.4.5 | lidar.launch.py |
| Pulsar ME450            | 1.0.0.6 | lidar.launch.py |

所有启动文件基本相似，主要区别在于针对不同型号的默认参数设置。不同的 USB 标准（如 USB 2.0 和 USB 3.0）可能需要调整这些参数。如果遇到启动失败，请仔细查看规格说明书，特别是启动文件中的分辨率设置以及其他参数，以确保兼容性和最佳性能。

## 教程

请参考官方 [OrbbecSDK ROS2 文档](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/5_advanced_guide/advanced_guide.html)

## 示例

要查看更多实践示例并了解如何在 ROS 中使用相机，请参阅 [Examples](./orbbec_camera/examples/) 部分。

## 其他有用链接

- [OrbbecSDK ROS2 文档](https://orbbec.github.io/OrbbecSDK_ROS2/zh/index.html)
- [OrbbecSDK_v2](https://github.com/orbbec/OrbbecSDK_v2/releases)
- [Gemini 330 深度预设](https://www.orbbec.com/docs/g330-use-depth-presets/)

## 许可证

Copyright 2024 Orbbec Ltd.

本项目遵循 Apache License, Version 2.0 协议；您只能在符合该协议的情况下使用。您可以在以下地址获取该协议副本：

http://www.apache.org/licenses/LICENSE-2.0

除非适用法律要求或书面同意，否则本项目按“原样”分发，不带任何明示或暗示的保证。具体内容请查看协议中的相关条款。

**其他名称和品牌可能归属于其各自所有者**
