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

**Using the shared multi-camera container example**

The [multi_camera_shared_container](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/multi_camera_shared_container) example creates one multithreaded component container and loads two Gemini 330 Series camera components into it. Update the two `usb_port` values in `multi_camera_shared_container.launch.py`, then run:

The default ports are `2-1` and `2-2`. Use `ros2 run orbbec_camera list_devices_node` to find the ports on your system. Give each camera a unique `camera_name`.

```bash
ros2 launch orbbec_camera multi_camera_shared_container.launch.py
```

The example passes the following arguments to both camera includes:

The top-level launch file starts `shared_orbbec_container` first, then includes the example-specific `gemini_330_series_shared_container.launch.py` for each camera. The second camera starts two seconds after the first. Both includes must use the same container name, and the container must be running before components are loaded.

* `attach_to_shared_component_container=true`: Loads the camera component into an existing container instead of creating another container.
* `component_container_name=shared_orbbec_container`: Selects the target container. The value must match the name of the container created by the parent launch file.
* `use_intra_process_comms=true`: Enables intra-process communication for the camera component.

**Using the single-camera intra-process demonstration launch**

```bash
ros2 launch orbbec_camera gemini_intra_process_demo_launch.py
```

**Limitations**

* Node components are currently not supported on RCLPY

* Compressed images using `image_transport` will be disabled as this isn't supported with intra-process communication
