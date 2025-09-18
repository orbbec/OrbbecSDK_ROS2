# OrbbecSDK ROS2 Wrapper v2

> [!IMPORTANT]
>
> 欢迎使用 OrbbecSDK ROS2 Wrapper v2。在开始使用此版本的 ROS2 wrapper 之前，请务必查看以下 [设备支持列表](#supported-devices)，以确认兼容性。

[English](./README.MD) | [中文](./README_CN.MD)

OrbbecSDK ROS2 Wrapper 提供 Orbbec 相机与 ROS 2 环境的无缝集成。它支持 ROS2 Foxy、Humble 和 Jazzy 发行版。

默认情况下，我们推荐使用 **v2-main** 分支。对于旧的 OpenNI 设备（v2-main 分支不支持），请使用 **main** 分支。仅由 main 分支支持的设备型号在下表中列出。

如果您在中国使用，建议使用 [gitee 仓库](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2)。

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
      <td rowspan="7" style="text-align: center; font-weight: bold;">Gemini 330</td>
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
**注意**: 如果您没有找到对应的设备，请联系 FAE 或销售代表获取帮助。

**定义**:

1. 新设计推荐：我们将提供完整支持，包括新功能、bug 修复和性能优化；
2. 完全维护：我们将提供 bug 修复支持；
3. 有限维护：我们将提供关键 bug 修复支持；
4. 不支持：此版本不会支持该设备；
5. 即将支持：我们将在近期添加支持。

## 安装说明

安装 ROS 2

- 请参考官方 [ROS 2 安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

> 如果您的 ROS 2 命令无法自动补全，请将以下两行加入到 `.bashrc` 或 `.zshrc` 中

```
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

创建 `colcon` 工作区

```
mkdir -p ~/ros2_ws/src
```

获取源码

```
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout v2-main
git branch  #检查分支是否切换成功
```

安装依赖包

```
# 假设您已经 source ROS 环境，下同
sudo apt install libgflags-dev nlohmann-json3-dev \
ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev
```

可选依赖：

```
# 435Le writeCustomerDate 功能需要：
sudo apt install libssl-dev
```

安装 udev 规则

```
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 支持的设备

目前 v2-main 分支支持以下设备。更多设备支持将陆续增加。如果没有找到您的设备，请尝试 [main](https://github.com/orbbec/OrbbecSDK_ROS2) 分支。

为获得最佳性能，强烈建议更新至最新固件版本，以确保您能获得最新的改进和 bug 修复。

| 产品列表       | 最低固件版本 | **启动文件**                |
| -------------- | ------------ | --------------------------- |
| Gemini 435Le   | 1.2.04       | gemini435_le.launch.py      |
| Gemini 335     | 1.2.20       | gemini_330_series.launch.py |
| Gemini 336     | 1.2.20       | gemini_330_series.launch.py |
| Gemini 335L    | 1.2.20       | gemini_330_series.launch.py |
| Gemini 336L    | 1.2.20       | gemini_330_series.launch.py |
| Gemini 335Lg   | 1.3.46       | gemini_330_series.launch.py |
| Gemini 335Le   | 1.5.31       | gemini_330_series.launch.py |
| Gemini 330     | 1.2.20       | gemini_330_series.launch.py |
| Gemini 330L    | 1.2.20       | gemini_330_series.launch.py |
| Gemini 2       | 1.4.92       | gemini2.launch.py           |
| Gemini 2 L     | 1.4.53       | gemini2L.launch.py          |
| Femto Bolt     | 1.1.2        | femto_bolt.launch.py        |
| Femto Mega     | 1.3.0        | femto_mega.launch.py        |
| Astra 2        | 2.8.20       | astra2.launch.py            |
| Astra Mini Pro | 2.0.01       | astra.launch.py             |

所有启动文件基本相似，主要区别在于针对不同型号的默认参数设置。不同的 USB 标准（如 USB 2.0 和 USB 3.0）可能需要调整这些参数。如果遇到启动失败，请仔细查看规格说明书，特别是启动文件中的分辨率设置以及其他参数，以确保兼容性和最佳性能。

## 快速开始

```
cd ~/ros2_ws/
source /opt/ros/$ROS_DISTRO/setup.bash
# 编译 release，默认是 Debug
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

启动相机节点

- 终端 1

```
source ~/ros2_ws/install/setup.bash
ros2 run orbbec_camera list_devices_node #检查相机是否已连接
ros2 launch orbbec_camera gemini_330_series.launch.py # 或其他启动文件，见下表
```

- 终端 2

```
source ~/ros2_ws/install/setup.bash
rviz2
```

选择需要显示的 topic

- 列出 topics / services / parameters (终端 3)

```
ros2 topic list
ros2 service list
ros2 param list
```

- 查看某个 topic

```
ros2 topic echo /camera/depth/camera_info
```

- 调用某个服务

```
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'
```

- 使用 rosbag2 录制

```
ros2 bag record /camera/color/image_raw /camera/depth/image_raw
```

更多使用详情，请参考官方 [OrbbecSDK ROS2 文档](https://orbbec.github.io/OrbbecSDK_ROS2/source/4_application_guide/application_guide.html)

## 使用方法

### 检查相机支持的 profile

```
ros2 run orbbec_camera list_camera_profile_mode_node
```

### 可选深度预设

> 您可以将可选预设的固件路径传入 `preset_firmware_path` 启动参数。用户可以从 Orbbec 提供的 [Depth Preset Release](https://www.orbbec.com/docs/depth-preset-release/) 页面下载合适的预设，并将其更新到相机中，以在对应场景下获得更好的深度性能。

```
ros2 launch orbbec_camera gemini_330_series.launch.py preset_firmware_path:=/home/orbbec/G336X_AMR_Default_v0.0.5.bin
```

### 使用 V4L2 backend

注意：V4L2 backend 默认未启用。

- 示例：

```
ros2 launch orbbec_camera gemini_330_series.launch.py uvc_backend:=v4l2
```

更多使用详情，请参考官方 [OrbbecSDK ROS2 文档](https://orbbec.github.io/OrbbecSDK_ROS2/source/5_advanced_guide/advanced_guide.html)

## 示例

要查看更多实践示例并了解如何在 ROS 中使用相机，请参阅 [Examples](./orbbec_camera/examples/) 部分。

## 其他有用链接

- [OrbbecSDK ROS2 文档](https://orbbec.github.io/OrbbecSDK_ROS2/index.html)
- [OrbbecSDK_v2](https://github.com/orbbec/OrbbecSDK_v2/releases)
- [Gemini 330 深度预设](https://www.orbbec.com/docs/g330-use-depth-presets/)

## 许可证

Copyright 2024 Orbbec Ltd.

本项目遵循 Apache License, Version 2.0 协议；您只能在符合该协议的情况下使用。您可以在以下地址获取该协议副本：

http://www.apache.org/licenses/LICENSE-2.0

除非适用法律要求或书面同意，否则本项目按“原样”分发，不带任何明示或暗示的保证。具体内容请查看协议中的相关条款。

**其他名称和品牌可能归属于其各自所有者**
