# 435le\_example\_node

This example node demonstrates how to interact with an Orbbec camera in ROS2 using the Orbbec SDK.
Users can learn how to retrieve camera information, control camera streams, and read/write camera calibration parameters through this node.

---

## 1. How to Run

After building the workspace, run the example node directly:

```bash
ros2 run orbbec_camera 435le_example_node
```

Once successfully started, the following menu will appear. Enter the corresponding number to select a feature:

```bash
========= Example Menu =========
1. Write camera parameters
2. Read camera parameters
3. Enable camera streams
4. Disable camera streams
5. Get device info
6. Show device status
7. Set Color AE ROI
0. Exit
```

---

## 2. Features

### 2.1 Camera Parameters

Used to manage camera calibration parameters, including:

* **k**: Camera intrinsic matrix (3×3)
* **d**: Distortion coefficients (1x8)
* **rotation**: Extrinsic rotation matrix (3×3)
* **translation**: Extrinsic translation vector (3×1)

#### Example Commands

```bash
# Write parameters
ros2 service call /camera/set_user_calib_params orbbec_camera_msgs/srv/SetUserCalibParams \
"{k: [614.9613647460938, 0.0, 634.91552734375,
      0.0, 614.65771484375, 391.407470703125,
      0.0, 0.0, 1.0],
  d: [-0.03131488710641861,
       0.032955970615148544,
       9.096559369936585e-05,
      -0.0003368517500348389,
      -0.01115430984646082,
      0.0, 0.0, 0.0],
  rotation: [0.9999880790710449,  0.0003024190664291382, -0.004874417092651129,
            -0.0002965621242765337, 0.9999992251396179,   0.001202247804030776,
             0.004874777048826218, -0.0012007878394797444, 0.9999874234199524],
  translation: [-0.023897956848144532,
                -9.439220279455185e-05,
                -6.804073229432106e-06]}"
```

```bash
# Read parameters
ros2 service call /camera/get_user_calib_params orbbec_camera_msgs/srv/GetUserCalibParams '{}'
```

> MD5 verification is performed during both writing and reading to ensure data integrity.

---

### 2.2 Stream Control

The master switch for camera streams allows enabling or disabling all streams during runtime.

#### Example Commands

* **Enable camera streams**:

  ```bash
  ros2 service call /camera/set_streams_enable std_srvs/srv/SetBool "{data: true}"
  ```
* **Disable camera streams**:

  ```bash
  ros2 service call /camera/set_streams_enable std_srvs/srv/SetBool "{data: false}"
  ```

- **Get streams status**

  ```bash
  ros2 service call /camera/get_streams_enable std_srvs/srv/GetBool '{}'
  ```

### 2.3 Device Info

Get camera device information, including Name, Serial Number, Firmware Version, Supported Minimum SDK Version, Current SDK Version, and Hardware Version.

#### Example Command

```bash
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo "{}"
```

---

### 2.4 Device Status

`/camera/device_status` is a real-time topic provided by the Orbbec camera for monitoring device status and performance metrics.

#### Example Command

```bash
ros2 topic echo /camera/device_status
```

---

### 2.5 Set Color AE ROI
In data_param, the first value is the Left setting, the second value is the Right setting, the third value is the Top setting, and the fourth value is the Bottom setting.

```bash
ros2 service call /camera/set_color_ae_roi orbbec_camera_msgs/srv/SetArrays '{data_param: [0,1279,0,719]}'
```
---


## 3. Additional Features

### 3.1 Optimized Color Stream Latency

In the previous version, enabling the color point cloud caused noticeable latency in color image publishing.
In this version, data synchronization and publishing have been optimized so that enabling the color point cloud no longer affects the depth stream publishing latency.

---


### 3.2 Added `time_sync_period`

A new parameter **time_sync_period** is introduced to configure the time synchronization interval between the camera module and the host.

* Unit: **seconds**
* Function: The camera aligns its internal timestamp with the host at the specified interval, ensuring consistent timestamps.

>  **Note** : This parameter only needs to be set when **`enable_sync_host_time = true`** and  **`time_domain = device`** .

---

### 3.3 Added `camera_info_url` and `ir_info_url`

Support specifying calibration file paths through parameters to load external camera intrinsic and extrinsic configurations:

* **camera\_info\_url**: Path to the color camera calibration file
* **ir\_info\_url**: Path to the IR camera calibration file

> Note: This method does not write the file information to the device.

#### Usage Example

Specify the path in the launch file or when launching:

```launch
DeclareLaunchArgument('ir_info_url', default_value='file:///home/user/calibration/color.yaml'),
DeclareLaunchArgument('color_info_url', default_value='file:///home/user/calibration/ir.yaml'),
```

#### Calibration File Example (YAML Format)

```yaml
image_width: 1280
image_height: 800
camera_name: orbbec_color
camera_matrix:
  rows: 3
  cols: 3
  data: [614.9613647460938, 0.0, 634.91552734375,
         0.0, 614.65771484375, 391.407470703125,
         0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 8
  data: [-0.03131488710641861,
          0.032955970615148544,
          9.096559369936585e-05,
         -0.0003368517500348389,
         -0.01115430984646082,
          0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [614.9613647460938, 0.0, 634.91552734375, 0.0,
         0.0, 614.65771484375, 391.407470703125, 0.0,
         0.0, 0.0, 1.0, 0.0]
```
