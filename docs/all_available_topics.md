# All available topics

- `/camera/color/camera_info` : The color camera info.
- `/camera/color/image_raw`: The color stream image.
- `/camera/depth/camera_info`: The depth stream image.
- `/camera/depth/image_raw`: The depth stream image
- `/camera/depth/points` : The point cloud, only available when `enable_point_cloud` is `true`.
- `/camera/depth_registered/points`: The colored point cloud, only available when `enable_colored_point_cloud`
  is `true`.
- `/camera/ir/camera_info`: The IR camera info.
- `/camera/ir/image_raw`: The IR stream image
- `/camera/accel/sample`: Acceleration data stream `enable_sync_output_accel_gyro`turned off，`enable_accel`turned on
- `/camera/gyro/sample`: Gyroscope data stream，enable_sync_output_accel_gyro `turned off，`enable_gyro`turned on
- `camera/gyro_accel/sample`: Synchronized data stream of acceleration and gyroscope，`enable_sync_output_accel_gyro`
  turned on
- `/diagnostics`: The diagnostic information of the camera, Currently, the diagnostic information only includes the
  temperature of the camera.
