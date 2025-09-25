# Performance & Optimization

## Ob_benchmark tool

> The goal of this tool is to benchmark the performance of various OrbbecSDK_ROS2 camera configurations. The benchmark results depend on the camera and settings used.(Currently only works with ROS2 Humble)

You can find example usage code in the [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples).

### Tool Configuration ([start_benchmark_params.json](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/tools/startbenchmark/start_benchmark_params.json))

```json
{
    "start_benchmark_params": {
        "camera_name": [
            "camera_01",
            "camera_02",
            "camera_03",
            "camera_04"
        ],
        "process_name": "component_conta",
        "switch_cycle": 300,
        "test_cycle": 1,
        "skip_number": 30
    }
}
```

- `camera_name`: Names of the cameras to be configured. Example: `"camera_01"`, `"camera_02"`, etc.
- `process_name`: The name of the process to be monitored. For example, `"component_conta"` will monitor the data of the container process.
- `switch_cycle`: The cycle time for switching configurations, in seconds. For example, setting it to `300` means the configuration will switch every 300 seconds.
- `test_cycle`: The testing cycle, in seconds. For example, setting it to `1` means the tool will collect data for the monitored process every 1 second.
- `skip_number`: The number of data points to skip. For example, setting it to `30` means that the first 30 data points will be ignored.

### Camera configuration (launch files)

In the launch folder, there are multiple.launch.py files (`ob_benchmark_0.launch.py`, `ob_benchmark_1.launch.py`, ..., `ob_benchmark_19.launch.py`). Each file corresponds to a different camera configuration.

### Running the ob_benchmark tool

To run the tool, use the following commands:

```bash
source install/setup.bash
ros2 run orbbec_camera ob_benchmark_node
```

### Output Data Files

The output data files will be stored in the ob_benchmark folder with filenames like `0.csv`, `1.csv`, ..., 19.csv. For example:

- `0.csv` contains data from the `ob_benchmark_0.launch.py` configuration.
- `1.csv` contains data from the `ob_benchmark_1.launch.py` configuration.

## Reducing CPU Usage with Orbbec ROS Package

You can find example usage code in the [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples).

This document outlines strategies for minimizing CPU usage in the **OrbbecSDK_ROS2 v2** environment when using **Gemini 330 series cameras**. The firmware version must be **no lower than 1.4.10**, and `device` should be set to **Default**.

### Recommended Settings for Lower CPU Usage

To achieve the lowest possible CPU usage in OrbbecSDK_ROS2, it is recommended to configure the following parameters.

| Parameter        | Recommendation                         | Note                                           |
| ---------------- | -------------------------------------- | ---------------------------------------------- |
| `uvc_backend`  | `v4l2`                               | Lower CPU usage compared to `libuvc`         |
| `color_format` | `RGB`                                | Lower CPU usage than `MJPG`                  |
| `filter`       | Only `hardware_noise_removal_filter` | Other filters significantly increase CPU usage |

### Launch Files Used for Testing

- `gemini_330_series_lower_cpu_usage.launch.py`
- `multi_camera_lower_cpu_usage.launch.py`

### Test environment

**Hardware Configuration**

- **CPU**: Intel i7-8700 @ 3.20GHz
- **Memory**: 24 GB
- **Storage**: Micron 2200S NVMe 256GB
- **GPU**: NVIDIA GeForce GTX 1660Ti
- **OS**: Ubuntu22.04

**ROS Configuration**

- **ROS Version**: ROS2 Humble
- **SDK Version**: OrbbecSDK_ROS2 v2.2.1

**Camera Setup**

- Devices: 2x Gemini 335, 1x Gemini 336, 1x Gemini 336L
- Firmware Version: 1.4.10

### Test Setup

**Stream Settings:**

- Depth / IR Left / IR Right: 848×480 @ 30fps
- Color: 848×480 @ 30fps

Note: The following CPU usage data focuses on `uvc_backend`, `color_format` and various filter combinations.

### Test Results

**`uvc_backend` Comparison (RGB format)**

| libuvc CPU Usage | v4l2 CPU Usage | Absolute Change |
| ---------------- | -------------- | --------------- |
| 182.8%           | 118.8%         | -64.0%          |

The CPU usage can be significantly reduced with v4l2 backend. In our implementation, v4l2 works without requiring any patches to the Linux kernel, allowing users to easily switch between v4l2 and libuvc and maintaining full compatibility with standard Linux distributions.

**`color_format` Comparison (MJPG vs RGB)**

| Backend | MJPG CPU Usage | RGB CPU Usage | Absolute Change |
| ------- | -------------- | ------------- | --------------- |
| libuvc  | 347.7%         | 182.8%        | -164.9%         |
| v4l2    | 170.0%         | 118.8%        | -51.2%          |

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

| Parameter                      | Recommendation                                   | Note                                            |
| ------------------------------ | ------------------------------------------------ | ----------------------------------------------- |
| `depth_registration`         | `false` or `true` with `align_mode=HW` | Software alignment consumes more CPU            |
| `enable_point_cloud`         | `false`                                        | Disabling point cloud reduces CPU usage         |
| `enable_colored_point_cloud` | `false`                                        | Disabling colored point cloud reduces CPU usage |

## Efficient intra-process communication

### Introduction

Our ROS2 Wrapper node supports zero-copy communications if loaded in the same process as a subscriber node. This can reduce copy times on image/pointcloud topics, especially with big frame resolutions and high FPS.

You will need to launch a component container and launch our node as a component together with other component nodes. Further details on "Composing multiple nodes in a single process" can be found [here](https://docs.ros.org/en/rolling/Tutorials/Composition.html).

Further details on efficient intra-process communication can be found [here](https://docs.ros.org/en/humble/Tutorials/Intra-Process-Communication.html#efficient-intra-process-communication).

### Example

**Manually loading multiple components into the same process**

- Start the component:

  ```bash
  ros2 run rclcpp_components component_container
  ```
- Add the wrapper:

  ```bash
  ros2 component load /ComponentManager orbbec_camera orbbec_camera::OBCameraNodeDriver -e use_intra_process_comms:=true
  ```

  Load other component nodes (consumers of the wrapper topics) in the same way.

**Using a launch file**

```bash
ros2 launch orbbec_camera gemini_intra_process_demo_launch.py
```

**Limitations**

- Node components are currently not supported on RCLPY
- Compressed images using `image_transport` will be disabled as this isn't supported with intra-process communication

## Fast DDS Optimization for Orbbec Camera with ROS2

When operating with the default configuration, Fast DDS exhibits suboptimal transmission efficiency, resulting in
significant image transmission delays when used with the Orbbec camera in ROS2. This document provides guidance on
optimizing Fast DDS to enhance image transfer efficiency.

### Adjusting System Parameters

**IP Fragmentation Time**

- **Path**: `/proc/sys/net/ipv4/ipfrag_time` (default: 30 seconds)
- **Purpose**: Defines the duration that IP fragments are kept in memory.
- **Adjustment**: Decrease this value to reduce the time window where no fragments are received, which can help reduce
  delays. Consider the specific needs of your environment as this setting affects all incoming fragments.

  **Example**: Set to 3 seconds.

  ```bash
  sudo sysctl net.ipv4.ipfrag_time=3
  ```

**IP Fragmentation Memory Threshold**

- **Path**: `/proc/sys/net/ipv4/ipfrag_high_thresh` (default: 262144 bytes)
- **Purpose**: Sets the maximum memory used to reassemble IP fragments.
- **Adjustment**: Increase this value to allow more memory for fragment reassembly, which can improve handling of larger
  data packets.

  **Example**: Increase to 128 MB.

  ```bash
  sudo sysctl net.ipv4.ipfrag_high_thresh=134217728
  ```

**Maximum Buffer Sizes**

- **Purpose**: Configures the maximum buffer sizes for receiving and sending data, which is critical for high-throughput
  data transmission.
- **Adjustment**: Set the maximum buffer sizes for both receiving and sending operations.

  **Commands**:

  ```bash
  sudo sysctl -w net.core.rmem_max=2147483647
  sudo sysctl -w net.core.rmem_default=2147483647
  sudo sysctl -w net.core.wmem_max=2147483647
  sudo sysctl -w net.core.wmem_default=2147483647
  ```

Alternatively, make these settings permanent by adding them to the `/etc/sysctl.d/10-fastrtps-max.conf` file.

```bash
sudo gedit /etc/sysctl.d/10-fastrtps-max.conf
```

add blow lines to the file:

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
net.core.wmem_max=2147483647
net.core.wmem_default=2147483647
```

then save and exit the file. run `sudo sysctl -p` to apply the changes.

For detailed guidance, refer
to [ROS 2 DDS Tuning Documentation](https://docs.ros.org/en/foxy/How-To-Guides/DDS-tuning.html).

### Fast DDS Configuration

Below is an example of a Fast DDS configuration file optimized for ROS2 usage with the Orbbec camera. This configuration
enhances the overall data transmission by adjusting buffer sizes and transport settings.

**Configuration File:** `shm_fastdds.xml`

Place this file in the `$HOME` directory.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UDP_transport</transport_id>
            <type>UDPv4</type>
            <maxInitialPeersRange>10</maxInitialPeersRange>
            <maxMessageSize>65000</maxMessageSize>
            <sendBufferSize>1048576</sendBufferSize>
            <receiveBufferSize>1048576</receiveBufferSize>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="participant_profile_ros2" is_default_profile="true">
        <rtps>
            <name>profile_for_ros2_context</name>
            <userTransports>
                <transport_id>UDP_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
            <sendSocketBufferSize>1048576</sendSocketBufferSize>
            <listenSocketBufferSize>1048576</listenSocketBufferSize>
            <builtin>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>127.0.0.1</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
    <data_writer profile_name="default publisher profile" is_default_profile="true">
        <qos>
            <publishMode>
                <kind>ASYNCHRONOUS</kind>
            </publishMode>
            <latencyBudget>
                <duration>
                    <sec>0</sec>
                    <nanosec>1000000</nanosec>
                </duration>
            </latencyBudget>
        </qos>
        <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </data_writer>
    <data_reader profile_name="default subscription profile" is_default_profile="true">
        <qos>
            <data_sharing>
                <kind>AUTOMATIC</kind>
            </data_sharing>
            <latencyBudget>
                <duration>
                    <sec>0</sec>
                    <nanosec>1000000</nanosec>
                </duration>
            </latencyBudget>
        </qos>
        <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </data_reader>
</profiles>
```

**Environment Variables**

Set the following environment variables to use the custom Fast DDS profile:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

This configuration aims to optimize the data flow and reduce transmission delays, improving the responsiveness and
reliability of the Orbbec camera system in a ROS2 environment.
