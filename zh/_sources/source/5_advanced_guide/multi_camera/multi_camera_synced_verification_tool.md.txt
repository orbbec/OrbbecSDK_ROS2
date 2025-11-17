# 多相机同步验证节点

文件路径：[image_sync_example_node.cpp](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp)

此示例节点用于 **4 台 Orbbec 相机** 的同步采集与时间戳验证。
它可用于验证多相机主从同步（Primary / Secondary Synced）模式下的帧对齐精度。

---

## 使用教程

### **在 `multi_camera_synced.launch.py` 的基础上做以下修改**

   - 增加 `launch_include` 以适配 4 台相机

   - 根据设备型号选择对应的 launch 文件，如 330 系列选择 gemini_330_series.launch.py

   - 命名规范：camera_name 命名方式 camera_01、camera_02、camera_03 ...

   - 设置 device_num 数量为 4

   - USB 端口配置 usb_port，使用以下命令查看并绑定正确的端口
   `ros2 run orbbec_camera list_devices_node`

   - 同步模式设置为主从模式，一个 primary 相机，其他设置为 secondary_synced

   - 启动顺序：主相机（Primary）应最后启动，以确保同步信号建立正确。

---

### **参数文件修改**

   修改 `config` 目录下的以下两个配置文件：

   - `camera_params.yaml`

   - `camera_secondary_params.yaml`

   确保以下内容统一：

   - 启用 **depth** 与 **color** 流；

   - 统一各相机的 **帧率（fps）**。

---

### **启动与验证**
   - 启动多机同步 launch： `multi_camera_synced.launch.py`

   - 打开新终端运行同步验证节点： `ros2 run orbbec_camera image_sync_example_node`
      该节点会输出多相机图像的时间戳差异信息，用于验证同步效果。

---

### **参考 launch 文件**

```
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LoadComposableNodes



def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(package_dir, "launch")
    config_file_dir = os.path.join(package_dir, "config")
    config_file_path = os.path.join(config_file_dir, "camera_params.yaml")
    secondary_config_file_path = os.path.join(config_file_dir, "camera_secondary_params.yaml")

    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_01",
            "usb_port": "2-2.3",
            "device_num": "4",
            "sync_mode": "primary",
            "config_file_path": config_file_path,
            "trigger_out_enabled": "true"
        }.items(),
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_02",
            "usb_port": "2-1",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "config_file_path": secondary_config_file_path,
            "trigger_out_enabled": "false"
        }.items(),
    )

    launch3_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_03",
            "usb_port": "2-3",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "config_file_path": secondary_config_file_path,
            "trigger_out_enabled": "false"
        }.items(),
    )

    launch4_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_04",
            "usb_port": "2-4",
            "device_num": "4",
            "sync_mode": "secondary_synced",
            "config_file_path": secondary_config_file_path,
            "trigger_out_enabled": "false"
        }.items(),
    )



    # Launch description
    ld = LaunchDescription(
        [

            TimerAction(period=0.0, actions=[GroupAction([launch2_include])]),
            TimerAction(period=2.0, actions=[GroupAction([launch3_include])]),
            TimerAction(period=4.0, actions=[GroupAction([launch4_include])]),
            TimerAction(period=6.0, actions=[GroupAction([launch1_include])]),
            # The primary camera should be launched at last
        ]
    )

    return ld
```

---

## 必要系统配置

### **提升 USB 缓冲区容量**

```
echo 512 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

### **配置 Fast DDS**

优化 ROS2 节点间通信延迟，可以有效减少图像传输延迟，[配置过程见此章节](../performance/fastdds_tuning.md)。
