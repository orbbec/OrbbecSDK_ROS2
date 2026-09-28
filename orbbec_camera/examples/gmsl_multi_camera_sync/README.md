# GMSL Multi-Camera Synchronization

This example synchronizes two GMSL-connected Gemini 330 Series cameras using the standard `gemini_330_series.launch.py` file.

## Before running

Grant access to `/dev/camsync` and update the `usb_port` values in `multi_gmsl_camera_synced.launch.py` to match your GMSL links. The default ports are `gmsl2-1` and `gmsl2-3`.

## Run

```bash
sudo chmod 777 /dev/camsync
ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py
```

## Full guide

[GMSL multi-camera synchronization (English)](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/multi_camera/gmsl_camera.html) · [中文指南](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/5_advanced_guide/multi_camera/gmsl_camera.html)
