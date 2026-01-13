# Frequently Asked Questions

### Unexpected Crash

If the camera node crashes unexpectedly, it will generate a crash log in the current running directory: `Log/camera_crash_stack_trace_xx.log`. Please send this log to the support team or submit it to a GitHub issue for further assistance.

### No Data Stream from Multiple Cameras

**Insufficient Power Supply**:

- Ensure that each camera is connected to a separate hub.
- Use a powered hub to provide sufficient power to each camera.

**High Resolution**:

- Try lowering the resolution to resolve data stream issues.

**Increase usbfs_memory_mb Value**:

- Increase the `usbfs_memory_mb` value to 128MB (this is a reference value and can be adjusted based on your system’s needs) by running the following command:

```
    echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

- To make this change permanent, check [this link](https://github.com/OpenKinect/libfreenect2/issues/807).

### Additional Troubleshooting

- If you encounter other issues, set the `log_level` parameter to `debug`. This will generate an SDK log file in the running directory: `Log/OrbbecSDK.log.txt`. Please provide this file to the support team for further assistance.
- If firmware logs are required, set the `log_level` parameter to `debug` and set `enable_heartbeat` to `true` to activate this feature.
- If you set the `log_level` parameter to `debug` and do not want the terminal to refresh too many logs, you can change `output="screen"` to `output="log"` in `launch`, and the logs will be saved in the `~/.ros/log` directory.

### Why Are There So Many Launch Files?

- Different cameras have varying default resolutions and image formats.
- To simplify usage, each camera has its own launch file.

### How to Launch a Specific Camera When Multiple Cameras Are Connected

While the launch file did not explicitly specify which device to use. In that case, the driver will connect to the default device.

You can check the serial number of your device by running:
```bash
ros2 run orbbec_camera list_devices_node
```

Then launch with the serial number explicitly set, for example:
```bash
ros2 launch orbbec_camera femto_bolt.launch.py serial_number:=CL8H741005J
```

### Why Is It Necessary to Add Delays When Starting Multiple Cameras or Switching Streams?

Multi-camera systems place high demands on USB bandwidth and device initialization timing. If multiple camera streams are started or switched simultaneously, it may cause temporary bandwidth congestion, leading to device initialization failures, stream startup errors, or frame drops. To ensure system stability, the following practices are recommended:

- **Multi-camera startup phase**

  When starting multiple cameras, it is recommended to introduce an appropriate delay between each camera startup (e.g., **2 seconds**) to avoid instantaneous bandwidth overload or low-level device initialization conflicts.

- **Stream enable/disable and mode switching phase**

  When invoking stream control services (such as `set_streams_enable`, `toggle_depth`, and `toggle_color`), avoid triggering multiple service calls at the same time. Instead, introduce a reasonable interval between operations (e.g., **20 ms**) to ensure reliable stream state transitions.

Following these timing control guidelines can significantly improve the stability of multi-camera systems during startup and runtime, reducing errors and unexpected behavior.
### femto bolt depth stream no data

This module depends on the OpenGL library at runtime. If OpenGL is not installed or the graphics driver is incomplete, the depth stream may output no data. Please make sure to install the necessary OpenGL libraries first (Ubuntu example below):

```bash
  sudo apt update && sudo apt install -y mesa-utils libgl1-mesa-glx libglu1-mesa
```

  After installation, you can check whether OpenGL is available through the following command:

```bash
  glxinfo -B
```

### The image does not reach the preset frame rate

First you need to confirm whether the image does not reach the preset frame rate. There are several ways to view framerate in ROS 2, such as:

* `ros2 topic hz`
* `rqt`
* Custom tools (such as the `benchmark` tool provided by this ROS package)

It should be noted that different tools have different statistical methods and QoS configurations, so the frame rate results obtained may be different. When you find that the frame rate is lower than expected, please prioritize whether the error is caused by the frame rate statistics tool itself.

If you confirm that the image frame rate does not reach the preset value, you can try the following troubleshooting steps:

1. **Reduce the resolution or frame rate** to determine whether the frame rate is reduced due to USB/network bandwidth limitations;
2. **Confirm whether the camera firmware version and ROS package version are the latest**. Older versions may have performance or compatibility issues.

If the above methods still cannot solve the problem, please contact our company **FAE**, or submit an issue in **GitHub Issue** for further support.


### Issues related to soft trigger mode

* **Each sensor does not flow out at the same time when the signal is triggered**
  Please enable the frame aggregation function and set the parameter `frame_aggregate_mode` to `full_frame` to ensure that multiple sensor data are output synchronously under the same trigger.

* **The preset frame rate cannot be reached in auto trigger mode**
  When setting `software_trigger_period`, you need to consider the actual open stream frame rate and exposure time. For example, when `color_fps` is set to 10 FPS, `software_trigger_period` cannot be lower than the following calculated value:

  ```
  software_trigger_period ≥ 1000000 / fps × N + 2 × expo
  ```

  Among them:

  * `fps`: sensor frame rate
  * `N`: The number of frames collected in a single trigger
  * `expo`: exposure time
  * `Unit`: µs

  If `software_trigger_period` is set too small, the trigger frequency will be limited, resulting in frame loss.


