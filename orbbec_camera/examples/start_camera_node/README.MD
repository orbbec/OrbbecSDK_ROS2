## Starting the Camera Node in ROS 2

This guide provides instructions on how to launch the camera node with a colored point cloud feature enabled using ROS 2.

### Command to Start the Node

To start the camera node, execute the following command in your terminal:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_colored_point_cloud:=true
```

This command initiates the camera node and enables the colored point cloud.

### Published Topics

Once the camera node is running, it will publish data on several ROS topics. Below is a list of the available topics:

- **IMU Data and IMU Information:**
  - `camera/accel/imu_info`
  - `camera/gyro/imu_info`
  - `camera/gyro_accel/sample`

- **Color Camera Topics:**
  - `/camera/color/camera_info`
  - `/camera/color/image_raw`
  - `/camera/color/image_raw/compressed`
  - `/camera/color/image_raw/compressedDepth`
  - `/camera/color/image_raw/theora`
  - `/camera/color/metadata`

- **Depth Camera Topics:**
  - `/camera/depth/camera_info`
  - `/camera/depth/image_raw`
  - `/camera/depth/image_raw/compressed`
  - `/camera/depth/image_raw/compressedDepth`
  - `/camera/depth/image_raw/theora`
  - `/camera/depth/metadata`
  - `/camera/depth/points`
  - `/camera/depth_filter_status`
  - `/camera/depth_registered/points`
  - `/camera/depth_to_color`
  - `/camera/depth_to_left_ir`
  - `/camera/depth_to_right_ir`

- **Infrared Camera Topics:**
  - `/camera/left_ir/camera_info`
  - `/camera/left_ir/image_raw`
  - `/camera/left_ir/image_raw/compressed`
  - `/camera/left_ir/image_raw/compressedDepth`
  - `/camera/left_ir/image_raw/theora`
  - `/camera/left_ir/metadata`
  - `/camera/right_ir/camera_info`
  - `/camera/right_ir/image_raw`
  - `/camera/right_ir/image_raw/compressed`
  - `/camera/right_ir/image_raw/compressedDepth`
  - `/camera/right_ir/image_raw/theora`
  - `/camera/right_ir/metadata`

- **Miscellaneous Topics:**
  - `/diagnostics`
  - `/parameter_events`
  - `/rosout`
  - `/rosout_agg`

### Visualizing Data in RViz2

To view the PointCloud or Image data, use RViz2:

1. Launch RViz2.
2. Select the topic you wish to visualize from the list of published topics.
3. Add the selected topic to RViz2 to start viewing the data.

### Example Visualizations

Here are examples of how the visualization might appear in RViz2:

- **PointCloud Visualization**
  ![PointCloud View](./images/image1.jpg)
  
- **Image Data Visualization**
  ![Image Data View](./images/image2.jpg)

