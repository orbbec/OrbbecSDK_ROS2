# GigE Action Command

This example starts two Gemini 335Le cameras in Group Actions synchronization mode and one host-side Action Command sender.

## Before running

Use Gemini 335Le firmware 1.8.24 or later and Orbbec SDK 2.10.2 or later. Connect both cameras and the host to the same network, then set the two `net_device_ip` values in `multi_gige_action_command.launch.py` to the camera IP addresses.

## Run

```bash
ros2 launch orbbec_camera multi_gige_action_command.launch.py
```

## Full guide

[GigE Action Command (English)](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/multi_camera/gige_action_command.html) · [中文指南](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/5_advanced_guide/multi_camera/gige_action_command.html)
