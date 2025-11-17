# Multi-Camera Synchronization Verification Node

**File path:** [image_sync_example_node.cpp](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp)

This example node is designed for **synchronized capture and timestamp verification** across **four Orbbec cameras**.
It can be used to validate frame alignment accuracy under the multi-camera **Primary / Secondary Synced** mode.

---

## Usage Guide

### **Modify `multi_camera_synced.launch.py` as follows**

   - Add `launch_include` to support four cameras.

   - Select the appropriate launch file based on your camera model.
     For example, use `gemini_330_series.launch.py` for the Gemini 330 series.

   - Naming convention:
     `camera_name` should follow the format `camera_01`, `camera_02`, `camera_03`, ...

   - Set `device_num` to **4**.

   - Configure the USB ports using:

     `ros2 run orbbec_camera list_devices_node`

     to view and bind the correct ports.

   - Set synchronization mode to **Primary/Secondary Synced**,
     with one camera as **primary** and the others as **secondary_synced**.

   - **Startup sequence:**
     The **Primary camera should always be launched last** to ensure the synchronization signal is established correctly.

---

### **Modify configuration files**

   Edit the following two files under the `config` directory:

   - `camera_params.yaml`

   - `camera_secondary_params.yaml`

   Ensure the following settings are consistent across all cameras:

   - Enable both **depth** and **color** streams.

   - Use the same **frame rate (fps)** for all cameras.

---

### **Launch and Verification**

   - Start the multi-camera synchronization launch file:

     `ros2 launch orbbec_camera multi_camera_synced.launch.py`

   - In a new terminal, run the synchronization verification node:

     `ros2 run orbbec_camera image_sync_example_node`

     This node outputs timestamp differences between multiple camera streams for synchronization validation.

---

### **Reference Launch File**

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

## System Configuration Requirements

### **Increase USB Buffer Memory**

   Prevent frame drops caused by concurrent data transmission:

   `echo 512 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`

### **Configure Fast DDS**

   Optimize ROS2 node communication latency to reduce image transmission delay.
   See [this section](../performance/fastdds_tuning.md) for detailed configuration instructions.
