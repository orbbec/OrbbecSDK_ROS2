# Benchmark Usage

This section introduces how to use the benchmark tool in C++ and Python, and provides an example YAML configuration file.

## Using common benchmark node

```
ros2 run orbbec_camera common_benchmark_node.py \
    --run_time 2h  \
    --csv_file /path/to/log.csv
```



* **Parameters**
  * **--run_time**: Duration for monitoring, specified as time strings like `"10s"`, `"5m"`, `"1h"`, `"2d"`. Default is 10 seconds.
  *  **--csv_file**: Path to the output CSV file. By default, it is saved in the workspace directory with the name "camera_monitor_log.csv".

## Using service benchmark node

### ROS2 C++

* **Single service benchmark**

```
ros2 run orbbec_camera service_benchmark_node \
    --ros-args \
    -p service_name:=/camera/get_depth_gain \
    -p service_type:=orbbec_camera_msgs/srv/GetInt32 \
    -p count:=10
```

* ****Multiple services benchmark (YAML config)****

```
ros2 run orbbec_camera service_benchmark_node \
    --ros-args \
    -p yaml_file:=/path/to/default_service_cpp.yaml
```

### ROS2 Python

* **Single service benchmark**

```
ros2 run orbbec_camera service_benchmark_node.py --service /camera/get_depth_gain --count 10
```

* ****Multiple services benchmark (YAML config)****

```
ros2 run orbbec_camera service_benchmark_node.py --yaml_file /path/to/default_service.yaml
```



### **Example YAML configuration**

We provide an example YAML configuration, located in the `scripts` directory as `service_default.yaml`.

```yaml
default_count: 50

services:
- name: /camera/get_auto_white_balance
  type: orbbec_camera_msgs/srv/GetInt32
- name: /camera/get_color_exposure
  type: orbbec_camera_msgs/srv/GetInt32
- name: /camera/get_color_gain
  type: orbbec_camera_msgs/srv/GetInt32
- name: /camera/get_depth_exposure
  type: orbbec_camera_msgs/srv/GetInt32
- name: /camera/get_depth_gain
  type: orbbec_camera_msgs/srv/GetInt32
- name: /camera/get_device_info
  type: orbbec_camera_msgs/srv/GetDeviceInfo
- name: /camera/send_software_trigger
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_auto_white_balance
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_color_ae_roi
  type: orbbec_camera_msgs/srv/SetArrays
  request: {data_param: [0,1279,0,719]}
- name: /camera/set_color_auto_exposure
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_color_exposure
  type: orbbec_camera_msgs/srv/SetInt32
  request: {data: 30}
- name: /camera/set_color_flip
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_color_gain
  type: orbbec_camera_msgs/srv/SetInt32
  request: {data: 20}
- name: /camera/set_color_mirror
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_color_rotation
  type: orbbec_camera_msgs/srv/SetInt32
  request: {data: 90}
- name: /camera/set_depth_ae_roi
  type: orbbec_camera_msgs/srv/SetArrays
  request: {data_param: [0,1279,0,719]}
- name: /camera/set_depth_auto_exposure
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_depth_exposure
  type: orbbec_camera_msgs/srv/SetInt32
  request: {data: 3000}
- name: /camera/set_depth_flip
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_depth_gain
  type: orbbec_camera_msgs/srv/SetInt32
  request: {data: 200}
```
