# Efficient intra-process communication:

### Introduction

Our ROS2 Wrapper node supports zero-copy communications if loaded in the same process as a subscriber node. This can reduce copy times on image/pointcloud topics, especially with big frame resolutions and high FPS.

You will need to launch a component container and launch our node as a component together with other component nodes. Further details on "Composing multiple nodes in a single process" can be found [here](https://docs.ros.org/en/rolling/Tutorials/Composition.html).

Further details on efficient intra-process communication can be found [here](https://docs.ros.org/en/humble/Tutorials/Intra-Process-Communication.html#efficient-intra-process-communication).

### Example

**Manually loading multiple components into the same process**

* Start the component:

  ```bash
  ros2 run rclcpp_components component_container
  ```

* Add the wrapper:

  ```bash
  ros2 component load /ComponentManager orbbec_camera orbbec_camera::OBCameraNodeDriver -e use_intra_process_comms:=true
  ```

  Load other component nodes (consumers of the wrapper topics) in the same way.

**Using a launch file**

```bash
ros2 launch orbbec_camera gemini_intra_process_demo_launch.py
```

**Limitations**

* Node components are currently not supported on RCLPY

* Compressed images using `image_transport` will be disabled as this isn't supported with intra-process communication
