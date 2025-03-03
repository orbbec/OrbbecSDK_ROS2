# Launch parameters

The following are the launch parameters available:

- `connection_delay`: The delay time in milliseconds for reopening the device. Some devices, such as Astra mini, require
  a longer time to initialize and reopening the device immediately can cause firmware crashes when hot plugging.
- `enable_point_cloud`: Enables the point cloud.
- `enable_colored_point_cloud`: Enables the RGB point cloud.
- `cloud_frame_id`:Modifying the frame_id name within the ros message.
- `ordered_pc`:Enable filtering of invalid point clouds.
- `point_cloud_qos`, `[color|depth|ir]_qos`, `[color|depth|ir]_camera_info_qos`: ROS 2 Message Quality of Service (QoS)
  settings. The possible values
  are `SYSTEM_DEFAULT`, `DEFAULT`, `PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA` and are
  case-insensitive. These correspond
  to `rmw_qos_profile_system_default`, `rmw_qos_profile_default`, `rmw_qos_profile_parameter_events`, `rmw_qos_profile_services_default`, `rmw_qos_profile_parameters`,
  and `SENSOR_DATA`, respectively.
- `enable_d2c_viewer`: Publishes the D2C overlay image (for testing only).
- `device_num`: The number of devices. This must be filled in if multiple cameras are required.
- `uvc_backend`:Optional values: v4l2, libuvc
- `color_width`, `color_height`, `color_fps`: The resolution and frame rate of the color stream.
- `ir_width`, `ir_height`, `ir_fps`: The resolution and frame rate of the IR stream.
- `depth_width`, `depth_height`, `depth_fps`: The resolution and frame rate of the depth stream.
- `enable_color`: Enables the RGB camera.
- `enable_depth`: Enables the depth camera.
- `enable_ir`: Enables the IR camera.
- `depth_registration`: Enables alignment of the depth frame to the color frame. This field is required when
  the `enable_colored_point_cloud` is set to `true`.
- `usb_port`: The USB port of the camera. This is required when multiple cameras are used.
- `enable_accel`: Enables the accelerometer.
- `accel_rate`: The frequency of the accelerometer, the optional values
  are `1.5625hz`, `3.125hz`, `6.25hz`, `12.5hz`, `25hz`, `50hz`, `100hz`, `200hz`, `500hz`, `1khz`, `2khz`, `4khz`, `8khz`, `16khz`, `32khz`.
  The specific value depends on the current camera.
- `accel_range`: The range of the accelerometer, the optional values are `2g`, `4g`, `8g`, `16g`. The specific value
  depends on the current camera.
- `enable_gyro`: Enables the gyroscope.
- `gyro_rate`: The frequency of the gyroscope, the optional values
  are `1.5625hz`, `3.125hz`, `6.25hz`, `12.5hz`, `25hz`, `50hz`, `100hz`, `200hz`, `500hz`, `1khz`, `2khz`, `4khz`, `8khz`, `16khz`, `32khz`.
  The specific value depends on the current camera.
- `gyro_range`: The range of the gyroscope, the optional values
  are `16dps`, `31dps`, `62dps`, `125dps`, `250dps`, `500dps`, `1000dps`, `2000dps`. The specific value depends on the
  current camera.
- `enumerate_net_device`: Enables the function of enumerating network devices. True means enabled, false means disabled.
  This feature is only supported by Femto Mega and Gemini 2 XL devices. When accessing these devices through the
  network, the IP address of the device needs to be configured in advance. The enable switch needs to be set to true.
- `depth_filter_config`: Configures the loading path for the depth filtering configuration file. By default, the depth
  filtering configuration file is located in the /config/depthfilter directory. Supported only on Gemini2.
- `depth_precision`: The depth precision should be in the format `1mm`. The default value is `1mm`.
- `enable_laser`: Enables the laser. The default value is `true`.
- `device_preset`: The default value is `Default`. Only the G330 series is supported. For more information, refer to
  the [G330 documentation](https://www.orbbec.com/docs/g330-use-depth-presets/). Please refer to the table below to set
  the `device_preset` value based on your use case. The value should be one of the preset names
  listed [in the table](#predefined-presets).
- `enable_decimation_filter`: This filter effectively reduces the depth scene complexity. The filter runs on kernel
  sizes [2x2] to [8x8] pixels. The image size is scaled down proportionally in both dimensions to preserve the aspect
  ratio.
- `enable_hdr_merge`: This filter is used jointly with the depth HDR function. By merging consecutive depth images of
  alternating exposure values, we can overcome challenges in acquiring depth values for under-illuminated and
  over-illuminated objects simultaneously.
- `enable_sequence_id_filter`: This filter is used jointly with the depth HDR function and outputs only the sequence
  with the specified sequence ID.
- `enable_threshold_filter`: This filter preserves depth values of interest and omits depth values out of scope.
- `enable_noise_removal_filter`: This filter removes speckle noise in clusters and gives rise to a less-filled depth
  map.
- `enable_spatial_filter`: This filter performs multiple iterations of processing as specified by the magnitude
  parameter to enhance the smoothness of depth data. It is also capable of filling small holes in depth maps.
- `enable_temporal_filter`: This filter is intended to improve the depth data persistency by manipulating per-pixel
  values based on previous frames. The filter performs a single pass on the data, adjusting the depth values while also
  updating the tracking history.
- `enable_hole_filling_filter`: This filter fills all holes in the depth map using the specified mode.
- `retry_on_usb3_detection_failure`: If the camera is connected to a USB 2.0 port and is not detected, the system will
  attempt to reset the camera up to three times. This setting aims to prevent USB 3.0 devices from being incorrectly
  recognized as USB 2.0. It is recommended to set this parameter to `false` when using a USB 2.0 connection to avoid
  unnecessary resets.
- `tf_publish_rate`: The rate at which the camera publishes dynamic transforms. The default value is `0.0`, which means static transforms are published.
- `time_domain`: The frame time domain, string type, can be `device`, `global`, or `system`. `device` means using the hardware timestamp from the camera,
  `system` means using the timestamp when the PC received the first packet of data or frame, and `global` is used for synchronized time across multiple
  devices, aligning data from different sources to a common time base.
- `enable_sync_host_time`: Enables synchronization of the host time with the camera time. The default value is `true`, if
  use global time, set to `false`. Some old devices may not support this feature.
- `config_file_path`: The path to the YAML configuration file. The default value is `""`. If the configuration file is not specified,
  the default parameters from the launch file will be used. If you want to use a custom configuration file, please refer to `gemini_330_series.launch.py`.
  `enable_heartbeat` enables the heartbeat function, which is set to `false` by default. If set to `true`, the camera node will send heartbeat signals to
  the firmware, and if hardware logging is desired, it should also be set to `true`.
- `log_level` : SDK log level, the default value is `info`, the optional values are `debug`, `info`, `warn`, `error`, `fatal`.
- `enable_color_undistortion`: Enables color undistortion, the default value is `false`. Note that our color cameras exhibit minimal distortion, and typically, undistortion is not necessary.
- `interleave_ae_mode` : Set laser or hdr interleave.
- `interleave_frame_enable` : Whether to enable interleave frame mode.
- `interleave_skip_enable` : Whether to enable skip frames.
- `interleave_skip_index` : Set skip pattern IR or flood IR.
- `[hdr|laser]_index[0|1]_[laser_control|depth_exposure|depth_gain|ir_brightness|ae_max_exposure]`:In interleave frame mode, set the 0th and 1st frame parameters of hdr or laser interleaving frames

**IMPORTANT**: *Please carefully read the instructions regarding software filtering settings
at [this link](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/). If you are uncertain, do not modify
these settings.*
