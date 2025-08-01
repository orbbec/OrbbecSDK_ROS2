# Launch parameters

> If you are not sure how to set the parameters, you can connect the orbbec camera and open the [OrbbecViewer](https://github.com/orbbec/OrbbecSDK_v2/releases/tag/v2.3.5) parameter settings.

The following are the launch parameters available:

* **camera_name**
  * Start the node namespace
* **depth_registration**
  * Enable alignment of the depth frame to the color frame. This field is required when the `enable_colored_point_cloud` is set to `true`.
* **serial_number**
  * The serial number of the camera. This is required when multiple cameras are used
* **usb_port**
  * The USB port of the camera. This is required when multiple cameras are used
* **device_num**
  * The number of devices. This must be filled in if multiple cameras are required
* **upgrade_firmware**
  * The input parameter is the firmware path
* **preset_firmware_path**
  * The input parameter is the perset firmware path. If multiple paths are input, each path needs to be separated by `,`and a maximum of 3 firmware paths can be input
* **uvc_backend**
  * Optional values: v4l2, libuvc
* **point_cloud_qos, [color|depth|left_ir|right_ir|ir]_qos, [color|depth|left_ir|right_ir|ir]_camera_info_qos**
  * ROS 2 Message Quality of Service (QoS) settings. The possible values are `SYSTEM_DEFAULT`, `DEFAULT`, `PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA` and are case-insensitive. These correspond to `rmw_qos_profile_system_default`, `rmw_qos_profile_default`, `rmw_qos_profile_parameter_events`, `rmw_qos_profile_services_default`, `rmw_qos_profile_parameters`, and `SENSOR_DATA`, respectively.
* **color_ae_roi_[left|right|top|bottom],depth_ae_roi_[left|right|top|bottom]**
  * Set Color and Depth auto exposure ROI.
* **[color|depth|left_ir|right_ir|ir]_rotation**
  * Set stream image rotation
  * The possible values are `0`,`90`,`180`,`270`
* **[color|depth|left_ir|right_ir|ir]_flip**
  * Enable the stream image flip
* **[color|depth|left_ir|right_ir|ir]_mirror**
  * Enable the stream image mirror
* **enable_point_cloud**
  * Enable the point cloud
* **enable_colored_point_cloud**
  * Enable the RGB point cloud
* **cloud_frame_id**
  * Modify the frame_id name within the ros message
* **connection_delay**
  * The delay time in milliseconds for reopening the device. Some devices, such as Astra mini, require a longer time to initialize and reopening the device immediately can cause firmware crashes when hot plugging
* **[color|depth|left_ir|right_ir|ir]_width,[color|depth|left_ir|right_ir|ir]_height,[color|depth|left_ir|right_ir|ir]_fps,[color|depth|left_ir|right_ir|ir]_format**
  * The resolution and frame rate of the sensor stream
* **enable_color_auto_exposure_priority**
  * Enable the Color auto exposure priority
* **enable_color_auto_exposure**
  * Enable the Color auto exposure
* **color_exposure**
  * Set the Color exposure
* **color_gain**
  * Set the Color gain
* **enable_color_auto_white_balance**
  * Enable the Color auto white balance
* **color_white_balance**
  * Set the Color white balance
* **color_ae_max_exposure**
  * Set the maximum exposure value for Color auto exposure
* **color_brightness**
  * Set the Color brightness
* **color_sharpness**
  * Set the Color sharpness
* **color_gamma**
  * Set the Color gamma
* **color_saturation**
  * Set the Color saturation
* **color_constrast**
  * Set the Color constrast
* **color_hue**
  * Set the Color hue
* **enable_color_backlight_compenstation**
  * Enable the Color backlight compenstation
* **color_powerline_freq**
  * Set the power line freq
  * The possible values are `disable`,`50hz`,`60hz`,`auto`
* **enable_color_decimation_filter**
  * Enable the Color decimation filter
* **color_decimation_filter_scale**
  * Set the Color decimation filter scale
* **enable_depth_auto_exposure_priority**
  * Enable the Depth auto exposure priority
* **mean_intensity_set_point**
  * Set the target mean intensity of the Depth image  
  * **Note:** This parameter replaces the deprecated `depth_brightness`.  
* **enable_ir_auto_exposure**
  * Enable the IR auto exposure
* **ir_exposure**
  * Set the IR exposure
* **ir_gain**
  * Set the IR gain
* **ir_ae_max_exposure**
  * Set the maximum exposure value for IR auto exposure
* **ir_brightness**
  * Set the IR brightness
* **enable_sync_output_accel_gyro**
  * Enable the sync accel_gyro,and output IMU topic real-time data
* **enable_accel**
  * Enable the Accelerometer,and output Accelerometer info topic data
* **enable_accel_data_correction**
  * Enable the Accelerometer data correction
* **accel_rate**
  * The frequency of the accelerometer, the optional values are `1.5625hz`, `3.125hz`, `6.25hz`, `12.5hz`, `25hz`, `50hz`, `100hz`, `200hz`, `500hz`, `1khz`, `2khz`, `4khz`, `8khz`, `16khz`, `32khz`
* **accel_range**
  * The range of the accelerometer, the optional values are `2g`, `4g`, `8g`, `16g`. The specific value depends on the current camera
* **enable_gyro**
  * Enable the gyroscope,and output gyroscope info topic data
* **enable_gyro_data_correction**
  * Enable the gyroscope data correction
* **gyro_rate**
  * The frequency of the gyroscope, the optional values are `1.5625hz`, `3.125hz`, `6.25hz`, `12.5hz`, `25hz`, `50hz`, `100hz`, `200hz`, `500hz`, `1khz`, `2khz`, `4khz`, `8khz`, `16khz`, `32khz`.The specific value depends on the current camera
* **gyro_range**
  * The range of the gyroscope, the optional values are `16dps`, `31dps`, `62dps`, `125dps`, `250dps`, `500dps`, `1000dps`, `2000dps`. The specific value depends on the
    current camera
* **liner_accel_cov**
  * Covariance of the linear acceleration
* **angular_vel_cov**
  * Covariance of the angular velocity
* **publish_tf**
  * Enable the TF publish
* **tf_publish_rate**
  * Set Rate of the TF publication
* **ir_info_url**
  * Set URL of the IR image info
* **color_info_url**
  * Set URL of the color image info
* **enumerate_net_device**
  * Enable automatically enumerate network devices,this parameter is usually used [net camera](../orbbec_camera/examples/net_camera/README.MD)
* **net_device_ip**
  * Set net device's IP address,this parameter is usually used [net camera](../orbbec_camera/examples/net_camera/README.MD)
* **net_device_port**
  * Set net device's port.Usually, you can set it to 8090,this parameter is usually used [net camera](../orbbec_camera/examples/net_camera/README.MD)
* **log_level**
  * SDK log level, the default value is `info`, the optional values are `debug`, `info`, `warn`, `error`, `fatal`
* **enable_publish_extrinsic**
  * Enable the extrinsics publish
* **enable_d2c_viewer**
  * Publishes the D2C overlay image (for testing only)
* **enable_hardware_d2d**
  * `false`: switch to software disparity convert to depth, true: switch to hardware disparity convert to depth
* **enable_ldp**
  * Enable the LDP
* **ldp_power_level**
  * Set power level of the LDP
* **sync_mode**
  * Set sync mode.The default value is standalone,this parameter is usually used [multi camera synced](../orbbec_camera/examples/multi_camera_synced/README.MD)
* **depth_delay_us**
  * The delay time of the depth image capture after receiving the capture command or trigger signal in microseconds,this parameter is usually used [multi camera synced](../orbbec_camera/examples/multi_camera_synced/README.MD)
* **color_delay_us**
  * The delay time of the color image capture after receiving the capture command or trigger signal in microseconds,this parameter is usually used [multi camera synced](../orbbec_camera/examples/multi_camera_synced/README.MD)
* **trigger2image_delay_us**
  * The delay time of the image capture after receiving the capture command or trigger signal in microseconds,this parameter is usually used [multi camera synced](../orbbec_camera/examples/multi_camera_synced/README.MD)
* **trigger_out_delay_us**
  * The delay time of the trigger signal output after receiving the capture command or trigger signal in microseconds,this parameter is usually used [multi camera synced](../orbbec_camera/examples/multi_camera_synced/README.MD)
* **trigger_out_enabled**
  * Enable the trigger out signal,this parameter is usually used [multi camera synced](../orbbec_camera/examples/multi_camera_synced/README.MD)
* **software_trigger_enabled**
  * Enable the software trigger out signal,this parameter is usually used [multi camera synced](../orbbec_camera/examples/multi_camera_synced/README.MD)
* **frames_per_trigger**
  * The frame number of each stream after each trigger in triggering mode,this parameter is usually used [multi camera synced](../orbbec_camera/examples/multi_camera_synced/README.MD)
* **software_trigger_period**
  * Software trigger period in ms,this parameter is usually used [multi camera synced](../orbbec_camera/examples/multi_camera_synced/README.MD)
* **enable_ptp_config**
  * Enable PTP time synchronization
  * Only for Gemini 335Le
  * Note: To enable PTP time synchronization, you must set the **enable_sync_host_time** parameter to `false`.
* **enable_frame_sync**
  * Enable the frame synchronization
* **ordered_pc**
  * Enable filtering of invalid point clouds
* **enable_depth_scale**
  * Enable the depth scale
* **enable_decimation_filter**
  * Enable the Depth decimation filter.The Depth decimation filter setting parameter is `decimation_filter_scale`
* **enable_hdr_merge**
  * Enable the Depth hdr merge filter.The Depth hdr merge filter setting parameter is `hdr_merge_exposure_1`,`hdr_merge_gain_1`,`hdr_merge_exposure_2`,`hdr_merge_gain_2`
* **enable_sequence_id_filter**
  * Enable the Depth sequence id filter.The Depth sequence id filter setting parameter is `sequence_id_filter_id`
* **enable_threshold_filter**
  * Enable the Depth threshold filter.The Depth threshold filter setting parameter is `threshold_filter_max`,`threshold_filter_min`
* **enable_hardware_noise_removal_filter**
  * Enable the Depth hardware noise removal filter
* **enable_noise_removal_filter**
  * Enable the Depth software noise removal_filter.The Depth noise removal filter setting parameter is `noise_removal_filter_min_diff`,`noise_removal_filter_max_size`
* **enable_spatial_filter**
  * Enable the Depth spatial filter.The Depth spatial filter setting parameter is `spatial_filter_alpha`,`spatial_filter_diff_threshold`,`spatial_filter_magnitude`,`spatial_filter_radius`
* **enable_temporal_filter**
  * Enable the Depth temporal filter.The Depth temporal filter setting parameter is `temporal_filter_diff_threshold`,`temporal_filter_weight`
* **enable_hole_filling_filter**
  * Enable the Depth hole filling filter.The Depth hole filling filter setting parameter is `hole_filling_filter_mode`
* **enable_spatial_fast_filter**
  - Enable the Depth spatial fast filter. The Depth spatial fast filter setting parameter is `spatial_fast_filter_radius`.
* **enable_spatial_moderate_filter**
  - Enable the Depth spatial moderate filter. The Depth spatial moderate filter setting parameters are `spatial_moderate_filter_diff_threshold`, `spatial_moderate_filter_magnitude`, `spatial_moderate_filter_radius`.
* **align_mode**
  * The alignment mode to be used. Options are `HW` for hardware alignment and `SW` for software alignment
* **align_target_stream**
  * Set align target stream mode
  * The possible values are `COLOR`,`DEPTH`
  * `COLOR`:Align depth to color
  * `DEPTH`:Align color to depth
* **diagnostic_period**
  * Diagnostic period in seconds
* **enable_laser**
  * Enable the laser. The default value is `true`
* **depth_precision**
  * The depth precision should be in the format `1mm`. The default value is `1mm`
* **device_preset**
  * The default value is `Default`. Only the G330 series is supported. For more information, refer to the [G330 documentation](https://www.orbbec.com/docs/g330-use-depth-presets/). Please refer to the table below to set the `device_preset` value based on your use case. The value should be one of the preset names listed [in the table](./predefined_presets.md)
* **retry_on_usb3_detection_failure**
  * If the camera is connected to a USB 2.0 port and is not detected, the system will attempt to reset the camera up to three times. This setting aims to prevent USB 3.0 devices from being incorrectly recognized as USB 2.0. It is recommended to set this parameter to `false` when using a USB 2.0 connection to avoid unnecessary resets
* **laser_energy_level**
  * Set the laser energy level
* **enable_sync_host_time**
  * Enable synchronization of the host time with the camera time. The default value is `true`, if use global time, Set to `false`. Some old devices may not support this feature
* **time_domain**
  * Select timestamp type: device , global and system
* **enable_color_undistortion**
  * Enable the Color undistortion
* **config_file_path**
  * The path to the YAML configuration file. The default value is `""`. If the configuration file is not specified,the default parameters from the launch file will be used. If you want to use a custom configuration file, please refer to `gemini_330_series.launch.py`.`enable_heartbeat` enables the heartbeat function, which is set to `false` by default. If set to `true`, the camera node will send heartbeat signals to the firmware, and if hardware logging is desired, it should also be set to `true`
* **enable_heartbeat**
  * Enable the heartbeat function, which is set to `false` by default. If set to `true`, the camera node will send heartbeat signals to the firmware, and if hardware logging is desired, it should also be set to `true`
* **gmsl_trigger_fps**
  * Set gmsl trigger fps,this parameter is usually used [gmsl camera](../orbbec_camera/examples/gmsl_camera/README.MD)
* **enable_gmsl_trigger**
  * Enable the gmsl trigger out signal,this parameter is usually used [gmsl camera](../orbbec_camera/examples/gmsl_camera/README.MD)
* **disparity_range_mode**
  * Disparity search length,this parameter is usually used [disparity search offset](../orbbec_camera/examples/disparity_search_offset/README.MD)
* **disparity_search_offset**
  * Set Disparity search offset value,this parameter is usually used [disparity search offset](../orbbec_camera/examples/disparity_search_offset/README.MD)
* **disparity_offset_config**
  * Disparity search offset interleave frames,this parameter is usually used [disparity search offset](../orbbec_camera/examples/disparity_search_offset/README.MD)
* **frame_aggregate_mode**
  * Set frame aggregate output mode.The optional values are `full_frame`,`color_frame`,`ANY`,`disable`
* **interleave_ae_mode**
  * Set laser or hdr interleave,this parameter is usually used [interleave ae mode](../orbbec_camera/examples/interleave_ae_mode/README.MD)
* **interleave_frame_enable**
  * Whether to enable interleave frame mode,this parameter is usually used [interleave ae mode](../orbbec_camera/examples/interleave_ae_mode/README.MD)
* **interleave_skip_enable**
  * Whether to enable skip frames,this parameter is usually used [interleave ae mode](../orbbec_camera/examples/interleave_ae_mode/README.MD)
* **interleave_skip_index**
  * Set skip pattern IR or flood IR,this parameter is usually used [interleave ae mode](../orbbec_camera/examples/interleave_ae_mode/README.MD)
* **[hdr|laser]_index[0|1]_[laser_control|depth_exposure|depth_gain|ir_brightness|ae_max_exposure]**
  * In interleave frame mode, set the 0th and 1st frame parameters of hdr or laser interleaving frames,this parameter is usually used [interleave ae mode](../orbbec_camera/examples/interleave_ae_mode/README.MD)

**IMPORTANT**: *Please carefully read the instructions regarding software filtering settings
at [this link](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/). If you are uncertain, do not modify
these settings.*
