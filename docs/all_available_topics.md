# All available topics

> The names of the following topics already express their functions.
> However, it should be noted that the corresponding `[ir|right_ir|left_ir|depth|color]/[image_raw|camera_info|metadata]`
> topics are only available when `enable[ir|right_ir|left_ir|depth|color]` is set to true in the stream corresponding to the startup file parameters.

- `/camera/color/camera_info` : The color camera info.
- `/camera/color/image_raw`: The color stream image.
- `/camera/color/metadata`: The color stream firmware data.
- `/camera/depth/camera_info`: The depth camera info.
- `/camera/depth/image_raw`: The depth stream image.
- `/camera/depth/metadata`:The depth stream firmware data.
- `/camera/depth/points` : The point cloud, only available when `enable_point_cloud` is `true`.
- `/camera/depth_registered/points`: The colored point cloud, only available when `enable_colored_point_cloud`
  is `true`.
- `/camera/depth_filter_status`: The depth sensor filter status.
- `/camera/ir/camera_info`: The IR camera info.
- `/camera/ir/image_raw`: The IR stream image.
- `/camera/ir/metadata`: The IR stream firmware data.
- `/camera/imu/sample`: Synchronized IMU data stream (accelerometer and gyroscope), when `enable_imu` is turned on
- `/diagnostics`: The diagnostic information of the camera, Currently, the diagnostic information only includes the
  temperature of the camera.
