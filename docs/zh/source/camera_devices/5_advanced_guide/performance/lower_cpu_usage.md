## 使用Orbbec ROS包降低CPU使用率

您可以在 [示例](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples) 中找到使用示例代码。

本文档概述了在使用 **Gemini 330系列相机** 的 **OrbbecSDK_ROS2 v2** 环境中最小化CPU使用率的策略。固件版本必须 **不低于1.4.10**，且 `device` 应设置为 **Default**。

### 降低CPU使用率的推荐设置

要在OrbbecSDK_ROS2中实现最低的CPU使用率，建议配置以下参数。

|    参数    |             推荐值             |                      说明                      |
| :--------------: | :------------------------------------: | :--------------------------------------------: |
| `uvc_backend` |                `v4l2`                |     与 `libuvc` 相比CPU使用率更低     |
| `color_format` |                `RGB`                |         CPU使用率低于 `MJPG`         |
|    `filter`    | 仅使用 `hardware_noise_removal_filter` | 其他滤波器会显著增加CPU使用率 |

### 用于测试的启动文件

* `gemini_330_series_lower_cpu_usage.launch.py`
* `multi_camera_lower_cpu_usage.launch.py`

### 测试环境

**硬件配置**

* **CPU**: Intel i7-8700 @ 3.20GHz

* **内存**: 24 GB

* **存储**: Micron 2200S NVMe 256GB

* **GPU**: NVIDIA GeForce GTX 1660Ti

* **操作系统**: Ubuntu22.04

**ROS配置**

* **ROS版本**: ROS2 Humble

* **SDK版本**: OrbbecSDK_ROS2 v2.2.1

**相机设置**

* 设备: 2x Gemini 335, 1x Gemini 336, 1x Gemini 336L

* 固件版本: 1.4.10


### 测试设置

**数据流设置：**

* 深度/左IR/右IR: 848×480 @ 30fps
* 彩色: 848×480 @ 30fps

注意：以下CPU使用率数据重点关注 `uvc_backend`、`color_format` 和各种滤波器组合。

### 测试结果

**`uvc_backend` 对比（RGB格式）**

| libuvc CPU使用率 | v4l2 CPU使用率 | 绝对变化 |
| :--------------: | :------------: | :-------------: |
|      182.8%      |     118.8%     |     -64.0%     |

使用v4l2后端可以显著降低CPU使用率。在我们的实现中，v4l2无需对Linux内核进行任何补丁即可工作，允许用户轻松在v4l2和libuvc之间切换，并保持与标准Linux发行版的完全兼容性。

**`color_format` 对比（MJPG vs RGB）**

| 后端 | MJPG CPU使用率 | RGB CPU使用率 | 绝对变化 |
| :-----: | :------------: | :-----------: | :-------------: |
| libuvc |     347.7%     |    182.8%    |     -164.9%     |
|  v4l2  |     170.0%     |    118.8%    |     -51.2%     |

如果选择RGB格式而不是MJPG，可以降低CPU使用率，因为MJPG图像的解码会消耗主机CPU资源。

**滤波器配置影响**

| 应用的滤波器                                       | libuvc CPU使用率 | CPU使用率增加 | v4l2 CPU使用率 | CPU使用率增加 |
| ----------------------------------------------------- | ---------------- | ------------------ | -------------- | ------------------ |
| 无滤波器（基准）                                 | 182.8%           | 0.0%（基准）    | 118.8%         | 0.0%（基准）    |
| `（软件）noise_removal_filter`                    | 218.0%           | +35.2%             | 128.5%         | +9.7%              |
| `（软件）noise_removal_filter + spatial_filter` | 469.6%           | +286.8%            | 336.7%         | +217.9%            |
| `hardware_noise_removal_filter`                     | 186.3%           | +3.5%              | 115.4%         | -3.4%              |
| `hardware_noise_removal_filter + spatial_filter`  | 251.3%           | +68.5%             | 152.5%         | +33.7%             |

根据测试结果，仅使用 `hardware_noise_removal_filter` 相比无滤波器基准，对 `libuvc`（+3.5%）和 `v4l2`（-3.4%）的CPU使用率变化可以忽略不计，因为此滤波器在相机硬件内部运行。相比之下，其他滤波器在主机系统上执行。将 `spatial_filter` 添加到硬件滤波器会导致CPU使用率适度增加，而应用基于软件的 `noise_removal_filter`——无论是单独使用还是与 `spatial_filter` 结合——都会显著增加CPU负载。为保持较低的CPU使用率，建议避免使用基于软件的滤波器，仅依赖 `hardware_noise_removal_filter`。

### 进一步优化

|           参数           |                  推荐值                  |                      说明                      |
| :----------------------------: | :----------------------------------------------: | :---------------------------------------------: |
|     `depth_registration`     | `false` 或 `true` 配合 `align_mode=HW` |      软件对齐消耗更多CPU      |
|     `enable_point_cloud`     |                    `false`                    |     禁用点云可降低CPU使用率     |
| `enable_colored_point_cloud` |                    `false`                    | 禁用彩色点云可降低CPU使用率 |
