# AE/AWB Lock Test

This test-only ROS 2 action checks the AE/AWB capture and manual lock-in flow through the camera driver's services and color-frame metadata.

## Before running

Start the camera driver with the color stream enabled. The sample must run in the same namespace as the driver, and `color/metadata` must be available.

## Run

```bash
ros2 run orbbec_camera ae_awb_lock_test_node --ros-args -r __ns:=/camera
ros2 action send_goal /camera/run_ae_awb_lock_test \
  orbbec_camera_msgs/action/RunAeAwbLockTest \
  "{timeout_ms: 10000}" --feedback
```

## Full guide

[AE/AWB lock test (English)](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/examples/ae_awb_lock.html) · [中文指南](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/4_application_guide/examples/ae_awb_lock.html)
