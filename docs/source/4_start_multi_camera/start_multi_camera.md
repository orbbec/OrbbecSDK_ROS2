<!-- docs/source/4_start_multi_camera/start_multi_camera.md -->

# Multiple cameras

This section describes how to configure and use multiple Orbbec cameras simultaneously in a ROS 2 environment.

- Table of contents
  - [Script to list connected cameras](#script-to-list-connected-cameras)
  - [Setup for multiple camera launch](#setup-for-multiple-camera-launch)
  - [Running the launch file](#running-the-launch-file)
  - [Configuring the TF tree for multiple cameras](#configuring-the-tf-tree-for-multiple-cameras)
  - [Example TF configuration for two cameras](#example-tf-configuration-for-two-cameras)


## List connected cameras

To determine which USB ports the cameras are connected to, you can execute the following command.
This Command lists all Orbbec devices attached to the system along with their USB port and serial number:

```bash
$ ros2 run orbbec_camera list_devices_node

```

As follows:

```
USB port_id: 4-1.1-3
Modified USB port_id: 4-1.1
[INFO]serial: CP7X54P0004D
[INFO]usb port: 4-1.1
[INFO]usb connect type: USB3.2
USB port_id: 4-1.2-5
Modified USB port_id: 4-1.2
[INFO]serial: CP7X54P000AA
[INFO]usb port: 4-1.2
[INFO]usb connect type: USB3.2
```

From the log above, it appears that the two USB cameras you are connected to are using USB ports **4-1.1 and 4-1.2**.

## Setup for multiple camera launch

You can launch multiple cameras by specifying different USB ports for each camera. You can refer to orbbec_multicamera.launch.py to implement the multicamera launch script you need..

## Running the launch file

To execute the launch configuration for multiple cameras, use the command:

```
ros2 launch orbbec_camera orbbec_multicamera.launch.py config_file_path:=multicamera.yaml
```

**Note:**

1. Multiple devices default config reference config/multicamera.yaml
2. Multiple cameras synced  config reference config/multicamera_synced.yaml

## Example TF configuration for two cameras

When using multiple cameras, it's essential to calibrate them and publish a static TF tree for each camera. The following Python script configures the TF tree based on your calibration results:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

# Define the extrinsics for each camera (x, y, z, roll, pitch, yaw)
camera_01_transform = ['0.1', '0', '0.2', '0', '0', '1.57']  # Example parameters
camera_02_transform = ['-0.1', '0', '0.2', '0', '0', '-1.57']  # Example parameters

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_01_tf',
            arguments=camera_01_transform + ['base_link', 'camera_01_link'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_02_tf',
            arguments=camera_02_transform + ['base_link', 'camera_02_link'],
            output='screen'
        )
    ])

    return ld

```

Save this configuration as `multi_camera_tf.launch.py` in the launch directory of the Orbbec camera package. To run it, use:

```bash
ros2 launch orbbec_camera multi_camera_tf.launch.py
```
