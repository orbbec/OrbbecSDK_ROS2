# Zero-copy communications

## Efficient intra-process communication:

[](https://github.com/orbbec/OrbbecSDK_ROS2?tab=readme-ov-file#efficient-intra-process-communication)

Our ROS2 Wrapper node supports zero-copy communications if loaded in the same process as a subscriber node. This can reduce copy times on ../image/pointcloud topics, especially with big frame resolutions and high FPS.

You will need to launch a component container and launch our node as a component together with other component nodes. Further details on "Composing multiple nodes in a single process" can be found [here](https://docs.ros.org/en/rolling/Tutorials/Composition.html).

Further details on efficient intra-process communication can be found [here](https://docs.ros.org/en/humble/Tutorials/Intra-Process-Communication.html#efficient-intra-process-communication).

## Zero-copy example

[](https://github.com/orbbec/OrbbecSDK_ROS2?tab=readme-ov-file#example)

## Manually loading multiple components into the same process

[](https://github.com/orbbec/OrbbecSDK_ROS2?tab=readme-ov-file#manually-loading-multiple-components-into-the-same-process)

* Start the component:

  ```shell
  ros2 run rclcpp_components component_container
  ```
* Add the wrapper:

  ```shell
  ros2 component load /ComponentManager orbbec_camera orbbec_camera::OBCameraNodeDriver -e use_intra_process_comms:=true
  ```

  Load other component nodes (consumers of the wrapper topics) in the same way.

## Using a launch file

[](https://github.com/orbbec/OrbbecSDK_ROS2?tab=readme-ov-file#using-a-launch-file)

```shell
ros2 launch orbbec_camera orbbec_camera.launch.py use_intra_process_comms:=true
```

```bash
$ ros2 component list
/camera/camera_container
  1  /camera/camera
  2  /camera/frame_latency
```

## Limitations

[](https://github.com/orbbec/OrbbecSDK_ROS2?tab=readme-ov-file#limitations)

* Node components are currently not supported on RCLPY
* Compressed images using `image_transport` will be disabled as this isn't supported with intra-process communication
