# 性能基准测试使用

本节介绍如何在 C++ 和 Python 中使用性能基准测试工具，并提供示例 YAML 配置文件。

## 使用通用基准测试节点

```
ros2 run orbbec_camera common_benchmark_node.py \
    --run_time 2h  \
    --csv_file /path/to/log.csv
```

* **参数**
  * **--run_time**：监控持续时间，指定为时间字符串，如 `"10s"`、`"5m"`、`"1h"`、`"2d"`。默认为 10 秒。
  *  **--csv_file**：输出 CSV 文件的路径。默认情况下，它保存在工作空间目录中，名称为 "camera_monitor_log.csv"。

## 使用服务基准测试节点

### ROS2 C++

* **单个服务基准测试**

```
ros2 run orbbec_camera service_benchmark_node \
    --ros-args \
    -p service_name:=/camera/get_depth_gain \
    -p service_type:=orbbec_camera_msgs/srv/GetInt32 \
    -p count:=10
```

* **多个服务基准测试（YAML 配置）**

```
ros2 run orbbec_camera service_benchmark_node \
    --ros-args \
    -p yaml_file:=/path/to/default_service_cpp.yaml
```

### ROS2 Python

* **单个服务基准测试**

```
ros2 run orbbec_camera service_benchmark_node.py --service /camera/get_depth_gain --count 10
```

* **多个服务基准测试（YAML 配置）**

```
ros2 run orbbec_camera service_benchmark_node.py --yaml_file /path/to/default_service.yaml
```

### **示例 YAML 配置**

我们提供了一个示例 YAML 配置，位于 `scripts` 目录中，名为 `service_default.yaml`。

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
