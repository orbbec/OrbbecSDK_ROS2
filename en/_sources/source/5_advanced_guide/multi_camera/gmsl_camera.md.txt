# GMSL_camera

> This section describes how to use GMSL camera in OrbbecSDK_ROS2.Currently, only Gemini 335Lg GMSL devices are supported, and other GMSL devices will be supported in the near future.

You can find example usage code in the [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples).

## Single GMSL camera

The usage of GMSL camera in OrbbecSDK_ROS2 is the same as that of Gemini 330 series camera via USB.

```bash
ros2 launch orbbec_camera gemini_330_gmsl.launch.py
```

## Multi GMSL camera

To get the `usb_port` of the GMSL camera, plug in the camera and run the following command in the terminal:

```bash
ros2 run orbbec_camera list_devices_node
```

For example, the obtained gmsl camera `usb_port`: `gmsl2-1`

Go to the [multi_gmsl_camera.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/gmsl_camera/multi_gmsl_camera.launch.py) file and change the `usb_port`.

```bash
ros2 launch orbbec_camera multi_gmsl_camera.launch.py
```

> Note: By default, multi_gmsl_camera.launch.py only starts color and left_ir. If you want to start other sensors, please go to [camera_secondary_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/camera_secondary_params.yaml) to modify them.

## Multi GMSL camera synced

First, please see how to use [multi_camera_synced](./multi_camera_synced.md).

In addition, GMSL multi-camera synced does not require Multi-Camera Sync Hub Pro, so there is no need to set the `primary` mode. Each GMSL camera is `secondary`.

**Additional Parameter Settings**

* `gmsl_trigger_fps` : set hardware soc trigger source frame rate.
* `enable_gmsl_trigger` : enable hardware soc trigger.

**Run the launch**

Please refer to the configuration in [multi_gmsl_camera_synced.launch.py.](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/gmsl_camera/multi_gmsl_camera_synced.launch.py)

```bash
ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py
```

> Note: By default, multi_gmsl_camera_synced.launch.py only starts color and left_ir. If you want to start other sensors, please go to [camera_secondary_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/camera_secondary_params.yaml) and [camera_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/camera_params.yaml) to modify them.



## Usage Limitations of GMSL Cameras

[](https://github.com/orbbec/MIPI_Camera_Platform_Driver/tree/main?tab=readme-ov-file#usage-limitations-of-gmsl-cameras)

GMSL cameras interface with various deserializer chips such as MAX9296 and MAX92716. Orbbec GMSL cameras support multiple streams including depth, color, IR, and IMU data, but certain usage limitations apply:

- GMSL only supports V4L2 and YUYV format; MJPG format is not supported. RGB output is derived from YUYV format conversion.
- Metadata for Gemini-335Lg is provided via a separate node, while metadata for other models is embedded within video frames, which remains transparent to users.
- When using the Max96712 as a deserializer chip, due to the characteristics of the Max96712 chip, a multi - machine synchronous trigger signal must be provided in the secondary_synced mode. Otherwise, data flow interruption will occur when switching the data stream
- Two cameras connected on the same MAX9296, MAX96712 LinkA/B, or MAX96712 LinkC/D have the following limitations:
  - Before driver version v12.02, there was a restriction that the RGB of one camera and the right IR of another camera could not stream simultaneously. After driver version v12.02, the restriction was modified to that the RGB of one camera and the left IR of another camera cannot stream simultaneously.
  - Before driver version v12.02, there was a restriction that the DEPTH of one camera and the left IR of another camera could not stream simultaneously. After driver version v12.02, the restriction was modified to that the DEPTH of one camera and the right IR of another camera cannot stream simultaneously.
  - The combined maximum number of active streams from both cameras is limited to four (satisfying the above two conditions ensures compliance).

For further known limitations, please refer to [Usage Limitations of Orbbec GMSL Cameras](https://github.com/orbbec/MIPI_Camera_Platform_Driver/blob/main/doc/Instructions%20for%20Using%20GMSL%20Camera.md)
