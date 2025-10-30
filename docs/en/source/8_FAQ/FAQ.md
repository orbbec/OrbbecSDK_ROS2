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
- If firmware logs are required, set `enable_heartbeat` to `true` to activate this feature.

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