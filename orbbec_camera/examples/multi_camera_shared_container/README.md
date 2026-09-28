# Shared Component Container

This example loads two Gemini 330 Series camera components into one ROS 2 component container.

## Before running

Update both `usb_port` values in `multi_camera_shared_container.launch.py` to match your cameras. The defaults are `2-1` and `2-2`. Give each camera a unique `camera_name`.

## Run

```bash
ros2 launch orbbec_camera multi_camera_shared_container.launch.py
```

## Full guide

[Efficient intra-process communication (English)](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/performance/efficient_intra_process_communication.html) · [中文指南](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/5_advanced_guide/performance/efficient_intra_process_communication.html)
