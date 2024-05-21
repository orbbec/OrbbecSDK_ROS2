## 使用Orbbec ROS 2包配置多个摄像头

本节介绍如何在ROS 2环境中同时配置和使用多个Orbbec摄像头。

### 识别摄像头USB端口

#### 列出连接的摄像头的脚本

要确定摄像头连接到哪些USB端口，您可以使用以下bash脚本。该脚本列出了连接到系统的所有Orbbec设备及其USB端口和序列号。

```bash
#!/bin/bash

VID="2bc5"

for dev in /sys/bus/usb/devices/*; do
  if [ -e "$dev/idVendor" ]; then
    vid=$(cat "$dev/idVendor")
    if [ "$vid" == "${VID}" ]; then
      port=$(basename $dev)
      product=$(cat "$dev/product" 2>/dev/null) # 产品名称
      serial=$(cat "$dev/serial" 2>/dev/null) # 序列号
      echo "发现Orbbec设备 $product，usb端口 $port，序列号 $serial"
    fi
  fi
done
```

将此脚本保存为一个文件，并在您的终端中执行，以输出连接的摄像头列表。

### 启动多个摄像头

#### 多摄像头启动配置

您可以通过为每个摄像头指定不同的USB端口来启动多个摄像头。下面是一个使用ROS 2启动系统启动两个摄像头的Python脚本示例。

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')

    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'gemini_330_series.launch.py')),
        launch_arguments={'camera_name': 'camera_01', 'usb_port': '2-3.4.4.4.1', 'device_num': '2', 'sync_mode': 'free_run'}.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'gemini_330_series.launch.py')),
        launch_arguments={'camera_name': 'camera_02', 'usb_port': '2-3.4.4.4.3', 'device_num': '2', 'sync_mode': 'free_run'}.items()
    )

    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include])
    ])

    return ld
```

#### 运行启动文件

要执行多摄像头的启动配置，请使用命令：

```bash
ros2 launch orbbec_camera multi_camera.launch.py
```

### 配置多摄像头的TF树

#### 两个摄像头的TF配置示例

使用多个摄像头时，校准它们并为每个摄像头发布静态TF树是必不可少的。以下Python脚本基于您的校准结果配置TF树：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_01_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_01_link'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_02_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_02_link'],
            output='screen'
        )
    ])

    return ld
```

将此配置保存为 `multi_camera_tf.launch.py` 在Orbbec摄像头包的启动目录中。运行它，请使用：

```bash
ros2 launch orbbec_camera multi_camera_tf.launch.py
```