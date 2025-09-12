# GMSL_camera

> This section describes how to use GMSL camera in OrbbecSDK_ROS2.Currently, only Gemini 335Lg GMSL devices are supported, and other GMSL devices will be supported in the near future.

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

### Additional Parameter Settings

* `gmsl_trigger_fps` : set hardware soc trigger source frame rate.
* `enable_gmsl_trigger` : enable hardware soc trigger.

### Run the launch

Please refer to the configuration in [multi_gmsl_camera_synced.launch.py.](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/gmsl_camera/multi_gmsl_camera_synced.launch.py)

```bash
ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py
```

> Note: By default, multi_gmsl_camera_synced.launch.py only starts color and left_ir. If you want to start other sensors, please go to [camera_secondary_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/camera_secondary_params.yaml) and [camera_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/camera_params.yaml) to modify them.
