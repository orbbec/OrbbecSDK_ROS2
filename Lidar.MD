# OrbbecSDK ROS2 LiDAR Driver

This ROS2 driver supports your use of Orbbec single-line/multi-line LiDAR. This document provides installation instructions## 4. IMU Data

### 4.1 IMU Topics

When IMU is enabled, the following topics will be published:

- **`/lidar/imu/sample`**: Unified IMU topic containing synchronized accelerometer and gyroscope data in `sensor_msgs/Imu` format.
- **`/lidar/lidar_to_imu`**: Transform from LiDAR frame to IMU frame.

### 4.2 Using IMU Dataides, and other important information to help you quickly get started using this driver.

## 1. Installation

### 1.1 Prerequisites

Before using the OrbbecSDK ROS2 LiDAR driver, please ensure that the following dependencies are installed on your system:

- **ROS2**: A valid installation of ROS2 (Humble, Foxy, or other supported distributions).
  - If you need help, please refer to the [ROS2 Installation Guide](https://docs.ros.org/en/foxy/Installation.html).

### 1.2 Install deb Dependencies

```bash
# assume you have sourced ROS environment, same blow
sudo apt install libgflags-dev nlohmann-json3-dev  \
ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev
```

### 1.3 Install udev Rules

```bash
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 1.4 Build the Package

```bash
cd ~/ros2_ws/
# Build release version, default is Debug
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

### 1.5 Launch the LiDAR Node

* First terminal

```bash
. ./install/setup.bash
ros2 launch orbbec_camera lidar.launch.py
```

* Second terminal

```bash
. ./install/setup.bash
rviz2
```

1. Open Rviz2.
2. Add a `PointCloud2` or `LaserScan` display.
3. For `PointCloud2`, select the `/lidar/cloud/points` topic; for `LaserScan`, select the `/lidar/scan/points` topic.
4. Set the `Fixed Frame` to `lidar_lidar_frame` to properly align the data.

* `PointCloud2` visualization example:

![module in rviz2](./docs/images/lidar0.jpg)

* `LaserScan` visualization example:

![module in rviz2](./docs/images/lidar1.png)

## 2. Usage

### 2.1 Running the Driver

To start the driver, launch the provided ROS2 launch file:

```bash
source install/setup.bash
# Launch the driver with point cloud data
ros2 launch orbbec_camera lidar.launch.py lidar_format:=LIDAR_POINT
# Launch the driver with sphere point cloud data
ros2 launch orbbec_camera lidar.launch.py lidar_format:=LIDAR_SPHERE_POINT
# Launch the driver with laser scan data
ros2 launch orbbec_camera lidar.launch.py lidar_format:=LIDAR_SCAN
# Launch the driver with IMU enabled
ros2 launch orbbec_camera lidar.launch.py enable_imu:=true imu_rate:=50hz
# Launch the driver with both point cloud and IMU data
ros2 launch orbbec_camera lidar.launch.py lidar_format:=LIDAR_POINT enable_imu:=true imu_rate:=100hz
```

This command will start the node that interfaces with the Orbbec LiDAR device. Please ensure that the LiDAR hardware is properly connected before running this command.

### 2.2 Get Device Information for Connected LiDARs

```bash
ros2 run orbbec_camera list_devices_node
```

This command will list the connected LiDAR devices and display their respective IP addresses and ports. You can use this information to configure the driver to connect to specific devices.

### 2.3 Check Which Configurations the LiDAR Supports

```bash
ros2 run orbbec_camera list_camera_profile_mode_node
```

### 2.4 Parameters and Configuration

The `lidar.launch.py` file contains default parameters for the driver. You can customize these settings by modifying the launch file or creating a custom configuration file. Key parameters include:

- **device_type**: The type of device to launch. Optional values: `lidar`, `camera`. Setting this parameter to `lidar` launches the LiDAR device, setting it to `camera` launches the camera device.
- **camera_name**: Launch node namespace.
- **device_num**: Number of devices. This must be filled if you need to launch multiple devices.
- **upgrade_firmware**: Firmware upgrade function. The input parameter is the firmware path.
- **connection_delay**: Delay time (in milliseconds) for reopening the device. Immediately reopening the device during hot-plugging may cause firmware crashes.
- **publish_tf**: Enable TF publishing.
- **tf_publish_rate**: TF publishing frequency.
- **lidar_format**: Data format for the LiDAR. Optional values: `LIDAR_POINT`, `LIDAR_SPHERE_POINT`, `LIDAR_SCAN`
- **lidar_rate**: Scan rate of the LiDAR.
- **publish_n_pkts**: Number of frames to accumulate before publishing merged point cloud. Range: 1-12000. Only effective when lidar_format is `LIDAR_POINT` or `LIDAR_SPHERE_POINT`, used to merge specified number of frames before publishing. Default value: `1`
- **enable_scan_to_point**: Enable conversion of scan data to point cloud data, publishing PointCloud2 data type topics.
- **repetitive_scan_mode**: Repetitive scan mode parameter.
- **filter_level**: Add filter level parameter.
- **vertical_fov**: Vertical angle parameter.
- **min_angle**: Minimum angle of the LiDAR scan range in degrees (e.g., `-135.0`). Default value: `-135.0`.
- **max_angle**: Maximum angle of the LiDAR scan range in degrees (e.g., `135.0`). Default value: `135.0`.
- **min_range**: Minimum distance the LiDAR can measure in meters. Default value: `0.05`.
- **max_range**: Maximum distance the LiDAR can measure in meters. Default value: `30.0`.
- **echo_mode**: Echo mode of the LiDAR. Optional values: `Last Echo`, `First Echo`
- **point_cloud_qos**: ROS 2 message Quality of Service (QoS) settings. Possible values include `SYSTEM_DEFAULT`, `DEFAULT`, `PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, and `SENSOR_DATA`, and are case-insensitive. These values correspond to `rmw_qos_profile_system_default`, `rmw_qos_profile_default`, `rmw_qos_profile_parameter_events`, `rmw_qos_profile_services_default`, `rmw_qos_profile_parameters`, and `SENSOR_DATA` respectively.
- **enumerate_net_device**: Enable automatic enumeration of network devices.
- **net_device_ip**: IP address of the network device.
- **net_device_port**: Port number on the network side.
- **log_level**: SDK log level, default value is `none`, optional values are `debug`, `info`, `warn`, `error`, `fatal`
- **time_domain**: Timestamp type of the device. Optional values are `device`, `global`, `system`
- **config_file_path**: Path to the YAML configuration file. Default value is "". If no configuration file is specified, default parameters from the launch file will be used.
- **enable_heartbeat**: Enable heartbeat function, default is `false`. If set to `true`, the camera node will send heartbeat signals to the firmware; this should also be set to `true` if hardware logging is required.
- **enable_imu**: Enable IMU (accelerometer + gyroscope) and output unified IMU topic data.
- **imu_rate**: Unified frequency of the IMU (both accelerometer and gyroscope).
- **accel_range**: Range of the accelerometer.
- **gyro_range**: Range of the gyroscope.
- **linear_accel_cov**: Linear acceleration covariance value, default is `0.0001`.
- **angular_vel_cov**: Angular velocity covariance value, default is `0.0001`.

## 3. Point Cloud Data Details

### 3.1 Point Cloud Format

PointCloud2 (PointXYZITO) point cloud format is as follows:

```
float32 x               # X axis, unit: meters
float32 y               # Y axis, unit: meters
float32 z               # Z axis, unit: meters
uint8   intensity       # LiDAR intensity
uint8   tag             # LiDAR tag
uint32  offset_time     # Point cloud offset relative to topic time, unit nanoseconds
```

### 3.2 Point Cloud Aggregation Functionality

The `publish_n_pkts` parameter enables point cloud aggregation functionality, which allows the LiDAR to accumulate a specified number of frames before publishing, then merge these frames into a larger point cloud data package for publishing.

#### Features:

- **Parameter Range**: 1-12000 frames
- **Applicable Formats**: Only effective when lidar format is `LIDAR_POINT` or `LIDAR_SPHERE_POINT`
- **Default Value**: 1 (no aggregation, each frame published individually)
- **Purpose**: Improve point cloud density, suitable for applications requiring denser point cloud data

#### Usage Examples:

```bash
# Aggregate 10 frames before publishing
ros2 launch orbbec_camera lidar.launch.py lidar_format:=LIDAR_POINT publish_n_pkts:=10

# Aggregate 100 frames before publishing
ros2 launch orbbec_camera lidar.launch.py lidar_format:=LIDAR_SPHERE_POINT publish_n_pkts:=100
```

**Note**: Increasing the `publish_n_pkts` value will improve point cloud density but will also increase latency and memory usage. Please adjust according to actual application requirements.

## 4. IMU Data

### 4.1 IMU Topics

When IMU is enabled, the following topics will be published:

- **`/lidar/imu/sample`**: Unified IMU topic containing synchronized accelerometer and gyroscope data in `sensor_msgs/Imu` format.
- **`/lidar/lidar_to_imu`**: Transform from LiDAR frame to IMU frame.

### 4.2 Using IMU Data

To enable IMU data collection:

```bash
# Launch with IMU enabled
ros2 launch orbbec_camera lidar.launch.py enable_imu:=true imu_rate:=50hz

# Check IMU topics
ros2 topic list | grep imu

# View IMU data
ros2 topic echo /lidar/imu/sample
```

The IMU data includes:

- **linear_acceleration**: 3D acceleration data (x, y, z) in m/s²
- **angular_velocity**: 3D angular velocity data (x, y, z) in rad/s
- **orientation**: Quaternion orientation (not provided by hardware, set to zero)
