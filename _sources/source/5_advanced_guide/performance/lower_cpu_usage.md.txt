## Reducing CPU Usage with Orbbec ROS Package

This document outlines strategies for minimizing CPU usage in the **OrbbecSDK_ROS2 v2** environment when using **Gemini 330 series cameras**. The firmware version must be **no lower than 1.4.10**, and `device` should be set to **Default**.

### Recommended Settings for Lower CPU Usage

To achieve the lowest possible CPU usage in OrbbecSDK_ROS2, it is recommended to configure the following parameters.

|    Parameter    |             Recommendation             |                      Note                      |
| :--------------: | :------------------------------------: | :--------------------------------------------: |
| `uvc_backend` |                `v4l2`                |     Lower CPU usage compared to `libuvc`     |
| `color_format` |                `RGB`                |         Lower CPU usage than `MJPG`         |
|    `filter`    | Only `hardware_noise_removal_filter` | Other filters significantly increase CPU usage |

### Launch Files Used for Testing

* `gemini_330_series_lower_cpu_usage.launch.py`
* `multi_camera_lower_cpu_usage.launch.py`

### Test environment

**Hardware Configuration**

* **CPU**: Intel i7-8700 @ 3.20GHz

* **Memory**: 24 GB

* **Storage**: Micron 2200S NVMe 256GB

* **GPU**: NVIDIA GeForce GTX 1660Ti

* **OS**: Ubuntu22.04

**ROS Configuration**

* **ROS Version**: ROS2 Humble

* **SDK Version**: OrbbecSDK_ROS2 v2.2.1

**Camera Setup**

* Devices: 2x Gemini 335, 1x Gemini 336, 1x Gemini 336L

* Firmware Version: 1.4.10


### Test Setup

**Stream Settings:**

* Depth / IR Left / IR Right: 848×480 @ 30fps
* Color: 848×480 @ 30fps

Note: The following CPU usage data focuses on `uvc_backend`, `color_format` and various filter combinations.

### Test Results

**`uvc_backend` Comparison (RGB format)**

| libuvc CPU Usage | v4l2 CPU Usage | Absolute Change |
| :--------------: | :------------: | :-------------: |
|      182.8%      |     118.8%     |     -64.0%     |

The CPU usage can be significantly reduced with v4l2 backend. In our implementation, v4l2 works without requiring any patches to the Linux kernel, allowing users to easily switch between v4l2 and libuvc and maintaining full compatibility with standard Linux distributions.

**`color_format` Comparison (MJPG vs RGB)**

| Backend | MJPG CPU Usage | RGB CPU Usage | Absolute Change |
| :-----: | :------------: | :-----------: | :-------------: |
| libuvc |     347.7%     |    182.8%    |     -164.9%     |
|  v4l2  |     170.0%     |    118.8%    |     -51.2%     |

The CPU usage can be reduced if the RGB format is selected instead of MJPG, since the decoding of MJPG image will consume the host CPU resource.

**Filter Configuration Impact**

| Filters Applied                                       | libuvc CPU Usage | CPU Usage Increase | v4l2 CPU Usage | CPU Usage Increase |
| ----------------------------------------------------- | ---------------- | ------------------ | -------------- | ------------------ |
| No Filter (benchmark)                                 | 182.8%           | 0.0%(benchmark)    | 118.8%         | 0.0%(benchmark)    |
| `(software)noise_removal_filter`                    | 218.0%           | +35.2%             | 128.5%         | +9.7%              |
| `(software)noise_removal_filter + spatial_filter` | 469.6%           | +286.8%            | 336.7%         | +217.9%            |
| `hardware_noise_removal_filter`                     | 186.3%           | +3.5%              | 115.4%         | -3.4%              |
| `hardware_noise_removal_filter + spatial_filter`  | 251.3%           | +68.5%             | 152.5%         | +33.7%             |

Based on the test results, using only the `hardware_noise_removal_filter` results in a negligible change in CPU usage for both `libuvc` (+3.5%) and `v4l2` (-3.4%) compared to the no-filter benchmark, as this filter runs internally on the camera hardware. In contrast, other filters execute on the host system. Adding the `spatial_filter` to the hardware filter leads to a moderate increase in CPU usage, while applying the software-based `noise_removal_filter` —either alone or combined with `spatial_filter` —significantly increases CPU load. To maintain low CPU usage, it is recommended to avoid software-based filters and rely solely on the `hardware_noise_removal_filter`.

### Further Optimizationa

|           Parameter           |                  Recommendation                  |                      Note                      |
| :----------------------------: | :----------------------------------------------: | :---------------------------------------------: |
|     `depth_registration`     | `false` or `true` with `align_mode=HW` |      Software alignment consumes more CPU      |
|     `enable_point_cloud`     |                    `false`                    |     Disabling point cloud reduces CPU usage     |
| `enable_colored_point_cloud` |                    `false`                    | Disabling colored point cloud reduces CPU usage |

