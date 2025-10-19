# Multi-Camera

- To get the `usb_port` of the camera, plug in the camera and run the following command in the terminal:

```bash
ros2 run orbbec_camera list_devices_node
```

- Set the `device_num` parameter to the number of cameras you have.
- Go to the `OrbbecSDK_ROS2/launch/multi_xxx.launch.py` file and change the `usb_port`.
- Don't forget to put the `include` tag inside the `group` tag.
  Otherwise, the parameter values of different cameras may become contaminated.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_01',
            'usb_port': '6-2.4.4.2',  # replace your usb port here
            'device_num': '2'
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2L.launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera_02',
            'usb_port': '6-2.4.1',  # replace your usb port here
            'device_num': '2'
        }.items()
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
    ])

    return ld

```

- To launch the cameras, run the following command:

```bash
ros2 launch orbbec_camera multi_camera.launch.py
```

## No Data Stream from Multiple Cameras

**Insufficient Power Supply**:

- Ensure that each camera is connected to a separate hub.
- Use a powered hub to provide sufficient power to each camera.

**High Resolution**:

- Try lowering the resolution to resolve data stream issues.

**Increase usbfs_memory_mb Value**:

- Increase the `usbfs_memory_mb` value to 128MB (this is a reference value and can be adjusted based on your system’s needs)
  by running the following command:

```bash
    echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

- To make this change permanent, check [this link](https://github.com/OpenKinect/libfreenect2/issues/807).

## Image topic frame rate too low from Multiple Cameras

Refer to the [Fast DDS Configuration](../performance/fastdds_tuning.md) file.
