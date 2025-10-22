# Available Topics

Topics are organized by stream and function. By default, all topics are published under the `/camera` namespace, which can be changed with the `camera_name` launch parameter.

> **Note:** Topics for a specific stream (e.g., `/camera/color/...`) are only published if their corresponding launch parameter (e.g., `enable_color`) is set to `true`.

### Image Streams

These topics provide the raw image data and corresponding calibration information for each enabled camera stream. The pattern is consistent for `color`, `depth`, `ir`, `left_ir`, and `right_ir` streams.

*   `/camera/color/image_raw`
    *   Raw image data from the color stream.
*   `/camera/color/camera_info`
    *   Camera calibration data and metadata for the color stream.
*   `/camera/color/metadata`
    *   Low-level metadata from the color stream firmware.

*   `/camera/depth/image_raw`
    *   Raw image data from the depth stream.
*   `/camera/depth/camera_info`
    *   Camera calibration data and metadata for the depth stream.
*   `/camera/depth/metadata`
    *   Low-level metadata from the depth stream firmware.

*   `/camera/ir/image_raw`
    *   Raw image data from the infrared (IR) stream.
*   `/camera/ir/camera_info`
    *   Camera calibration data and metadata for the IR stream.
*   `/camera/ir/metadata`
    *   Low-level metadata from the IR stream firmware.

### Point Cloud Topics

*   `/camera/depth/points`
    *   Point cloud data generated from the depth stream.
    *   **Condition:** Published only when `enable_point_cloud` is `true`.

*   `/camera/depth_registered/points`
    *   Colored point cloud data, where the depth points are registered to the color image frame.
    *   **Condition:** Published only when `enable_colored_point_cloud` is `true`.

### IMU Topics

The Inertial Measurement Unit (IMU) topics provide accelerometer and gyroscope data. Their behavior depends on the synchronization setting.

*   `/camera/accel/sample`
    *   Individual accelerometer data stream.
    *   **Condition:** Published when `enable_accel` is `true` AND `enable_sync_output_accel_gyro` is `false`.

*   `/camera/gyro/sample`
    *   Individual gyroscope data stream.
    *   **Condition:** Published when `enable_gyro` is `true` AND `enable_sync_output_accel_gyro` is `false`.

*   `/camera/gyro_accel/sample`
    *   Synchronized data stream containing both accelerometer and gyroscope data in a single message.
    *   **Condition:** Published when `enable_sync_output_accel_gyro` is `true`.

### Device Status & Diagnostics

*   `/camera/device_status`
    *   Reports the current status of the camera device.

*   `/camera/depth_filter_status`
    *   Reports the status of the depth sensor's post-processing filters.

*   `/diagnostics`
    *   Publishes diagnostic information about the camera node. Currently, this includes the device temperature.
