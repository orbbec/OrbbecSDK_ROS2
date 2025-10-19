## ROS Package QuickStarts

### Introduction

This section provides a quick start to using the Orbbec ROS 2 wrapper.
You will learn how to:

* Launch a camera node.
* Visualize depth/color streams in **RViz2**.
* Interact with topics and services using **ROS 2 CLI tools**.

---

### Build your First Camera Application

#### Step 1: Source ROS 2 and Workspace

Make sure ROS 2 and your workspace environment are sourced:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
```

#### Step 2: Launch the Camera Node

- On terminal 1

```bash
. ./install/setup.bash
ros2 run orbbec_camera list_devices_node #Check if the camera is connected
ros2 launch orbbec_camera gemini_330_series.launch.py # Or other launch file, see below table
```

If you have multiple cameras connected, you can specify the **serial number**:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py serial_number:=<YourCameraSN>
```

#### Step 3: Visualize in RViz2

Launch RViz2 and load the default config:

- On terminal 2

```bash
rviz2
```

* Add an **Image** display, set topic to `/camera/color/image_raw`.
* Add another **Image** display for `/camera/depth/image_raw`.
* Optionally, add a **PointCloud2** display for `/camera/depth/points`.

You should now see the color stream, depth stream, and 3D point cloud in RViz2.

---

### Sample Features

After the node is running, try some ROS 2 CLI commands:

#### List available topics / services/ parameters

```bash
ros2 topic list
ros2 service list
ros2 param list
```

#### Echo a topic

View depth camera data:

```bash
ros2 topic echo /camera/depth/camera_info
```

#### Call a service

For example, get device Information:

```bash
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
```

#### Record with rosbag2

```bash
ros2 bag record /camera/color/image_raw /camera/depth/image_raw
```
