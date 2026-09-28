# Launch parameters

> If you are not sure how to set the parameters, connect the Orbbec camera and open [OrbbecViewer](https://github.com/orbbec/OrbbecSDK/releases), or refer to the [camera datasheet](../1_overview/introduction.md) in Chapter 1.

## How to modify launch parameters

Launch parameters can be modified in two ways:

1. **Override parameters in the launch command**

   This is recommended for debugging, testing a parameter, or applying a setting only for the current launch. Use the format `parameter_name:=parameter_value`. You can append multiple parameters to the same command.

   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera_02
   ```

   Example: change the camera name and enable point cloud output at the same time.

   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera_02 enable_point_cloud:=true
   ```

2. **Modify the default value in a launch file**

   This is useful when you want a setting to become a long-term default, such as a fixed resolution, frame rate, camera name, or synchronization mode. Device launch files are located in [orbbec_camera/launch](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/launch). Choose the `*.launch.py` file that matches your camera model.

   For example, modify the default camera name to `camera_02`:

   ```python
   DeclareLaunchArgument('camera_name', default_value='camera_02')
   ```

   If you build from source and do not use `--symlink-install`, you usually need to rebuild the workspace and source it again after modifying a launch file:

   ```bash
   cd ~/ros2_ws
   colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

   If you use `--symlink-install`, restarting the launch file is usually enough. If you installed with apt/deb, it is not recommended to edit files in the system installation directory directly. Prefer command-line parameter overrides, or copy the launch file and maintain your own launch configuration.

The following are the launch parameters available:

> **About defaults:** Device launch files may assign different defaults to the same parameter. A default is listed below only when it can be confirmed consistently. An empty value or `-1` usually means that the node does not actively change the device's current value.

## Core & Stream Configuration

*   **`camera_name`**
    *   Start the node namespace.
*   **`serial_number`**
    *   The serial number of the camera. This is required when multiple cameras are used. See [multi camera](../5_advanced_guide/multi_camera/multi_camera.md) for multi-camera startup.
*   **`usb_port`**
    *   The USB port of the camera. This is required when multiple cameras are used. See [multi camera](../5_advanced_guide/multi_camera/multi_camera.md) for multi-camera startup.
*   **`device_num`**
    *   The number of devices. This must be filled in if multiple cameras are required. See [multi camera](../5_advanced_guide/multi_camera/multi_camera.md) for multi-camera startup.
* **`device_preset`**
    * Depth preset. See [predefined presets](../5_advanced_guide/configuration/predefined_presets.md) for available presets and recommended scenarios. You can use the following command to view the configurable modes; the tool also prints the preset list and preset version information.
    ```bash
    ros2 run orbbec_camera list_devices_node
    ```
* **`preset_resolution_config`**
  * Preset resolution configuration for the camera device. Format: "width,height,ir_decimation_factor,depth_decimation_factor". Example: "1280,720,4,4". Leave empty to disable.
*   **`[color|depth|left_ir|right_ir|ir]_[width|height|fps|format]`**
    *   The resolution and frame rate of the sensor stream.
    *   For Femto Mega / Femto Bolt, depth NFOV and WFOV modes are configured by combining depth and IR resolutions. See [Configuration of depth NFOV and WFOV modes](../5_advanced_guide/configuration/configuration_of_depth_NFOV_and_WFOV_modes.md).
    *   For lower CPU usage, see the `color_format` recommendations in [Lower CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md).
*   **`enable_[color|depth|left_ir|right_ir|ir]`**
    *   Enable or disable the corresponding image stream.

> **Gemini 301 series limitation:** All enabled image streams with an FPS greater than `0` must use the same FPS. The node validates this requirement at startup and when profiles are switched at runtime through `/camera/set_stream_profile`.

* **`depth_decimation_factor`** / **`left_ir_decimation_factor`** / **`right_ir_decimation_factor`**
  * Set the downsampling multiple. You can use `ros2 run orbbec_camera list_camera_profile_mode_node` to view the settable resolution. **Default value:** `1`.
*   **`color_frame_queue_max_frames`**, **`left_color_frame_queue_max_frames`**, **`right_color_frame_queue_max_frames`**
    *   Set the maximum number of color frames buffered by the corresponding color-frame worker. The default is `10`; when the queue is full, the oldest frame is discarded and the overflow counter is incremented. The current queue size and overflow counters can be queried with `/camera/get_color_queue_stats`.
*   **`[color|depth|left_ir|right_ir|ir]_rotation`**
    *   Set stream image rotation.
    *   The possible values are `0`, `90`, `180`, `270`.
*   **`[color|depth|left_ir|right_ir|ir]_flip`**
    *   Enable the stream image flip.
*   **`[color|depth|left_ir|right_ir|ir]_mirror`**
    *   Enable the stream image mirror.
*   **`enable_point_cloud`**
    *   Enable the point cloud. See [Point Cloud](point_cloud.md) for usage and RViz2 visualization.
*   **`enable_colored_point_cloud`**
    *   Enable the RGB point cloud. See [Point Cloud](point_cloud.md) for usage and RViz2 visualization.
*   **`cloud_frame_id`**
    *   Modify the `frame_id` name within the ros message.
*   **`ordered_pc`**
    *   Enable filtering of invalid point clouds.
*   **`point_cloud_qos`, `[stream]_qos`, `[stream]_camera_info_qos`**
    *   ROS 2 Message Quality of Service (QoS) settings. The possible values are `SYSTEM_DEFAULT`, `DEFAULT`, `PARAMETER_EVENTS`, `SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA` and are case-insensitive. These correspond to `rmw_qos_profile_system_default`, `rmw_qos_profile_default`, `rmw_qos_profile_parameter_events`, `rmw_qos_profile_services_default`, `rmw_qos_profile_parameters`, and `SENSOR_DATA`, respectively.
*   **`[stream]_qos_history`, `[stream]_qos_depth`**
    *   Override the image publisher History and Depth. Common parameters include `color_qos_history`, `color_qos_depth`, `depth_qos_history`, and `depth_qos_depth`; depending on the selected launch file, `stream` can also be `left_color`, `right_color`, `ir`, `left_ir`, or `right_ir`.
    *   `qos_history` accepts `DEFAULT`, `KEEP_LAST`, or `KEEP_ALL` (case-insensitive). The default is `default`, which keeps the history policy from `[stream]_qos`. `qos_depth` defaults to `-1`, which keeps the base QoS depth; a positive value overrides it.
* **`color.image_raw.enable_pub_plugins`**
  * Enable Color image transport plugins. The enabled list is determined by the device launch file. See [Compressed Image](compressed_image.md) for subscribing to compressed images.
* **`depth.image_raw.enable_pub_plugins`**
  * Enable Depth image transport plugins. The enabled list is determined by the device launch file. See [Compressed Image](compressed_image.md) for subscribing to compressed images.
* **`left_ir.image_raw.enable_pub_plugins`**
  * Enable Left IR image transport plugins. The enabled list is determined by the device launch file. See [Compressed Image](compressed_image.md) for subscribing to compressed images.
* **`right_ir.image_raw.enable_pub_plugins`**
  * Enable Right IR image transport plugins. The enabled list is determined by the device launch file. See [Compressed Image](compressed_image.md) for subscribing to compressed images.
* **`point_cloud_decimation_filter_factor`**
  * Point cloud downsampling factor. Range: `1–8`. `1` means no downsampling.
* **`bag_record_filename`**
  * Record device data to the specified SDK `.bag` file after startup. Leave empty to disable automatic recording. When recording starts, a JSON preset file with the same base name is also exported, for example `record.bag` creates `record.json`.
* **`bag_filename`**
  * Play back the specified SDK `.bag` file. When set, the node creates a playback device from the bag file instead of connecting to a physical camera.
  * During playback, image and IMU streams use the profiles recorded in the bag file. The launch parameters do not select new image resolution, frame rate, format, or IMU range/sample rate profiles.
* **`bag_loop`**
  * Loop SDK bag playback after the file reaches the end. Default: `false`. This only takes effect when `bag_filename` is set.
* **`enable_fps_boost`**
  * Enable device FPS Boost. The default is `false`; this parameter only takes effect when the device supports the `FPS Boost` property.

## Sensor Controls

### Color Stream
*   **`enable_color_auto_exposure`**
    *   Enable the Color auto exposure.
*   **`enable_color_auto_exposure_priority`**
    *   Enable the Color auto exposure priority.
*   **`color_exposure`**
    *   Set the Color exposure.
*   **`color_gain`**
    *   Set the Color gain.
*   **`enable_color_auto_white_balance`**
    *   Enable the Color auto white balance.
*   **`color_white_balance`**
    *   Set the Color white balance.
*   **`color_ae_max_exposure`**
    *   Set the maximum exposure value for Color auto exposure.
*   **`color_ae_max_gain`**
    *   Set the maximum gain for Color auto exposure. Supported by Gemini 2 firmware `1.5.04` and above, and Gemini 2L firmware `1.5.09` and above.
* **`ae_reference_stream`**
  * Set the auto-exposure reference stream. Options: `depth`, `color`. Default: `depth`.
  * This replaces the old `ae_mode` parameter. The old values `depthbased/colorbased` map to `depth/color`.
* **`ae_strategy`**
  * Set the auto-exposure strategy. Options: `default`, `motion`. Default: `motion`.
  * This replaces the old `enable_sports_mode` parameter.
*   **`color_brightness`**, **`color_sharpness`**, **`color_gamma`**, **`color_saturation`**, **`color_contrast`**, **`color_hue`**
    *   Set the Color brightness, sharpness, gamma, saturation, contrast, and hue.
*   **`color_backlight_compensation`**
    *   Set the Color camera's backlight compensation level. Valid values are `0–6`; the launch default is `-1`, which leaves the current device value unchanged.
*   **`color_powerline_freq`**
    *   Set the power line freq. The possible values are `disable`, `50hz`, `60hz`, `auto`.
* **`color_mjpeg_quality`**
  * Set the color MJPEG encoding quality. **Range:** `1–100`; **Default:** `-1` (leave the current device value unchanged). Firmware version `1.8.11` or later is required.
*   **`color_preset`**
    *   Set the Color preset by name. Supported on Gemini 330 series and Gemini 301 series devices. Common options include `Default`, `Warm Biased AWB`, and `Cold Biased AWB`; the exact list is reported by the device. The name comparison is case-insensitive.
*   **`color_anti_flicker`**
    *   Enable Color anti-flicker. Supported by Gemini 330 series firmware `1.7.13` and above, and Gemini 301 series firmware `1.0.54` and above.
*   **`enable_color_decimation_filter`** / **`color_decimation_filter_scale`**
    *   Enable the Color decimation filter and set its scale.
*   **`color_ae_roi_[left|right|top|bottom]`**
    *   Set Color auto exposure ROI.
*   **`color_denoising_level`**
    *   Enable ISP Color denoising. **Range:** `0–8`; `0` means auto. Supported by Gemini 330 series, Gemini 2 firmware `1.5.04` and above, and Gemini 2L firmware `1.5.09` and above. This feature requires Color auto exposure and new firmware support.


### Depth Stream
* **`enable_depth_auto_exposure_priority`**
  * Enable the Depth auto exposure priority.
* **`mean_intensity_set_point`**
  * Set the target average intensity of the depth image when auto-exposure is turned on. For example: `mean_intensity_set_point:=100`.
  > **Note:** In wrapper version 2.4.7 and later, this parameter replaces the deprecated `depth_brightness`, but `depth_brightness` will still be supported for backward compatibility.
*   **`enable_depth_scale`**
    *   Whether to enable depth scaling after setting D2C. `true` means enabled, the default is `true`.
*   **`depth_precision`**
    *   Set the depth precision, using a value such as `1mm`. When the launch argument is empty, the node does not actively change the device's current depth precision.
*   **`depth_work_mode`**
    *   Set the depth work mode. See [Depth Work Mode Switch](../5_advanced_guide/configuration/depth_work_mode_switch.md) for supported devices, mode query commands, and launch examples.
*   **`depth_ae_roi_[left|right|top|bottom]`**
    *   Set Depth auto exposure ROI.

### IR Stream
*   **`enable_ir_auto_exposure`**
    *   Enable the IR auto exposure.
*   **`ir_exposure`** / **`ir_gain`**
    *   Set the IR exposure and gain.
*   **`ir_ae_max_exposure`**
    *   Set the maximum exposure value for IR auto exposure.
*   **`ir_brightness`**
    *   Set the target average intensity of the ir image when auto-exposure is turned on. Some device launch files no longer declare this launch argument.
*   **`enable_left_ir_sequence_id_filter`** / **`left_ir_sequence_id_filter_id`**
    *   Enable the Left IR SequenceIdFilter and select a sequence ID.
*   **`enable_right_ir_sequence_id_filter`** / **`right_ir_sequence_id_filter_id`**
    *   Enable the Right IR SequenceIdFilter and select a sequence ID.
### Laser / LDP
*   **`enable_laser`**
    *   Enable the laser. The default value is `true`.
*   **`laser_energy_level`**
    *   Set the laser energy level.
*   **`enable_ldp`** / **`ldp_power_level`**
    *   Enable the LDP and set its power level.
*   **`enable_lrm_obstacle_distance_publish`**
    *   Publish LRM obstacle distance on `/camera/lrm/obstacle_distance`. The default is `false`. Enabling this parameter also enables LDP.
*   **`lrm_obstacle_distance_publish_rate`**
    *   Set the LRM obstacle distance topic rate in Hz. The default is `10.0`; non-positive values fall back to `10.0`.

## Device, Sync & Advanced Features

### Multi-Camera Synchronization
*   **`sync_mode`**
    *   Set sync mode. The default is determined by the selected launch file. See [multi camera synced](../5_advanced_guide/multi_camera/multi_camera_synced.md) for multi-camera connection, synchronization modes, and trigger configuration.
*   **`enable_gmsl_trigger`** / **`gmsl_trigger_fps`**
    *   Enable the GMSL trigger output signal / set the GMSL trigger frame rate. See [GMSL camera](../5_advanced_guide/multi_camera/gmsl_camera.md) for the configuration.
*   **`depth_delay_us`** / **`color_delay_us`**
    *   The delay time (microseconds) of the depth/color image capture after receiving the capture command or trigger signal.
*   **`trigger2image_delay_us`**
    *   The delay time (microseconds) of the image capture after receiving the capture command or trigger signal. Us
*   **`trigger_out_delay_us`**
    *   The delay time (microseconds) of the trigger signal output after receiving the capture command or trigger signal.
*   **`trigger_out_enabled`**
    *   Enable the trigger out signal.
*   **`software_trigger_enabled`** / **`software_trigger_period`**
    *   Enable the software trigger out signal / set the software trigger period in ms.
*   **`frames_per_trigger`**
    *   The frame number of each stream after each trigger in triggering mode.
*   **`sync_io_voltage_level`**
    *   Set the sync IO voltage level. Default: `-1`, which means do not set it. This is only supported on devices that expose the property; it can also be changed at runtime with `/camera/set_sync_io_voltage_level`.

### Network Cameras
* **`enumerate_net_device`**
  * Enable automatically enumerate network devices. See [net camera](../5_advanced_guide/configuration/net_camera.md) for network camera startup, specified IP startup, and Force IP configuration.
* **`net_device_ip`** / **`net_device_port`**
  * Set net device's IP address and port (usually `8090`). See [net camera](../5_advanced_guide/configuration/net_camera.md) for network camera startup, specified IP startup, and Force IP configuration.
* **`force_ip_enable`**
  * Enable the Force IP function. **Default:** `false`
* **`force_ip_mac`**
  * Target device MAC address when multiple cameras are connected (e.g., `"54:14:FD:06:07:DA"`). You can use the `list_devices_node` to find the MAC of each device. **Default:** `""`
* **`force_ip_address`**
  * Static IP address to assign. **Default:** `192.168.1.10`
* **`force_ip_subnet_mask`**
  * Subnet mask for the static IP. **Default:** `255.255.255.0`
* **`force_ip_gateway`**
  * Gateway address for the static IP. **Default:** `192.168.1.1`

### Disparity
*   **`disparity_to_depth_mode`**
    *   `HW`: use hardware disparity to depth conversion. `SW`: use software disparity to depth conversion. Use `disable` to turn it off.
    *   This parameter is case-insensitive. Use one of the valid values listed above.
*   **`disparity_range_mode`**, **`disparity_search_offset`**, **`disparity_offset_config`**
    *   Parameters for disparity search offset. Used for [disparity search offset](../5_advanced_guide/configuration/disparity_search_offset.md).

### Interleave AE Mode
*   **`interleave_ae_mode`**
    *   Set `laser` or `hdr` interleave.
*   **`interleave_frame_enable`**, **`interleave_skip_enable`**, **`interleave_skip_index`**
    *   Parameters to control interleave frame mode.
*   **`[hdr|laser]_index[0|1]_[...]`**
    *   In interleave frame mode, set the 0th and 1st frame parameters of hdr or laser interleaving frames.
*   *All interleave parameters are used for [interleave ae mode](../5_advanced_guide/configuration/interleave_ae_mode.md).*

### Intra-Camera Synchronization

- **`depth_registration`**
  *   Enable alignment of the depth frame to the color frame. This field is required when the `enable_colored_point_cloud` is set to `true`. See [Aligning Depth to Color](../5_advanced_guide/configuration/align_depth_color.md) for startup and viewing examples.
- **`align_mode`**
  *   The alignment mode to be used. Options are `HW` for hardware alignment and `SW` for software alignment.
  *   This parameter is case-insensitive. Use one of the valid values listed above.
- **`align_target_stream`**
  *   Set align target stream mode.
  *   The possible values are `COLOR`, `DEPTH`.
  *   `COLOR`: Align depth to color.
  *   `DEPTH`: Align color to depth.
  *   This parameter is case-insensitive. Hardware D2C only supports `COLOR` as the target stream; use `align_mode:=SW` if you need to align to `DEPTH`. See [Aligning Depth to Color](../5_advanced_guide/configuration/align_depth_color.md) for startup and viewing examples.
- **`intra_camera_sync_reference`**
  - Sets the reference point for intra-camera synchronization on supported Gemini 330/335 series devices. **Options:** `Start`, `Middle`, `End`. When empty, the node leaves the device's current setting unchanged.

## Basic & General Parameters

### Firmware & Backend
* Camera nodes and launch files no longer provide `upgrade_firmware` or `preset_firmware_path`. Use the standalone `firmware_update_tool` to update firmware or burn preset files. See [firmware_update_tool Tool](../6_benchmark/firmware_update_tool.md).
* **`uvc_backend`**
  * Optional values: `v4l2`, `libuvc`. See [Lower CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md) for low-CPU scenarios.
* **`connection_delay`**
  * The delay time in milliseconds for reopening the device. Some devices, such as Astra mini, require a longer time to initialize and reopening the device immediately can cause firmware crashes when hot plugging.
* **`retry_on_usb3_detection_failure`**
  * If the camera is connected to a USB 2.0 port and is not detected, the system will attempt to reset the camera up to three times. It is recommended to set this parameter to `false` when using a USB 2.0 connection to avoid unnecessary resets.

### TF, Extrinsics & Calibration
*   **`publish_tf`** / **`tf_publish_rate`**
    *   Enable the TF publish and set its publication rate. See [Coordinate Systems and TF Transforms](coordinate_and_tf.md) for coordinate systems, TF tree inspection, and visualization.
*   **`enable_publish_extrinsic`**
    *   Enable the extrinsics publish.
*   **`ir_info_url`** / **`color_info_url`**
    *   Set URL of the IR/color camera info.
*   **`enable_[color|depth|ir|left_ir|right_ir]_undistortion`**
    *   Enable the SDK undistortion filter for the selected image stream. Dual-IR devices use `enable_left_ir_undistortion` / `enable_right_ir_undistortion`; single-IR devices use `enable_ir_undistortion`.

### Time Synchronization
* **`enable_sync_host_time`**
  * Enable synchronization of the host time with the camera time. The default is determined by the device launch file; Gemini 330 series launch files, including Gemini 336L, default to `false`. Set it to `false` when using global time.
* **`time_domain`**
  * Select timestamp type: `device`, `global`, and `system`.
  * This parameter is case-insensitive. Use one of the valid values listed above.
* **`enable_ptp_config`**
  * Enable PTP time synchronization. Requires `enable_sync_host_time` to be `false`.
* **`timestamp_clock_type`**
  * Set the SDK timestamp clock type. Optional values: `realtime`, `monotonic`. When the launch argument is empty, the node does not explicitly set the SDK clock type.
* **`time_sync_period`**
  * Interval (in seconds) for synchronizing the camera time with the host system.
  > **Note**: This parameter takes effect only when `enable_sync_host_time = true` and `time_domain` is not `global`.

* **`enable_frame_sync`**
  * Enable the frame synchronization.
* **`enable_frame_drop_log`**
  * Enable frame drop logging. The log reports drops detected at both the SDK receive stage and the ROS publish stage.
* **`frame_timestamp_csv_file`**
  * CSV output path for frame timestamp statistics. If empty, no CSV file is written; set a path such as `/tmp/frame_timestamp.csv` to save CSV data.

### Logging & Diagnostics
* **`log_level`**
  * Shared SDK and ROS node log level. The launch default is `info`; set it to `debug` for more debug logs. Optional values: `none`, `debug`, `info`, `warn`, `error`, `fatal`.
  * SDK logs and crash files are saved to `~/.ros/Log` by default. ROS logs remain in `~/.ros/log`.
* **`log_file_name`**
  * SDK log file name. When empty, the log file is named after the node startup time in the format `OrbbecSDK_<YYYYMMDD_HHMMSS>.log`. When specified, the resulting path is `~/.ros/Log/<camera_name>/<log_file_name>`.
* **`enable_firmware_log`**
  * Enable firmware logging. This switch is independent from `enable_heartbeat` and can be enabled only when firmware logs are needed.
* **`diagnostic_period`**
  * Diagnostic period in seconds.
* **`enable_heartbeat`**
  * Enable the heartbeat function. Default is `false`. If `true`, the camera node will send heartbeat signals to the firmware.
* **`monitor_poll_interval_sec`**
  * Set the SDK polling interval for the device heartbeat and firmware log, in seconds. The default is `-1`, which leaves the SDK polling interval unchanged. Valid values are `1–10`; values outside this range are clamped to the nearest boundary. This parameter controls the polling interval and does not enable heartbeat or firmware-log capture by itself.

### Miscellaneous
*   **`config_file_path`**
    *   The path to the YAML configuration file. Default is `""`. If not specified, default parameters from the launch file will be used. Some presets or special modes are configured through YAML. See [predefined presets](../5_advanced_guide/configuration/predefined_presets.md).
*   **`load_config_json_file_path`**
    *   SDK JSON configuration import path. When set, the node imports the JSON configuration during initialization. For Gemini 330 series, use `gemini_330_series_sdk_json.launch.py` as the dedicated SDK JSON launch file. See [Gemini 330 Series SDK JSON Usage Guide](../5_advanced_guide/configuration/sdk_json_config.md).
    *   If the JSON contains `application_config`, the node syncs stream enable states, resolution, frame rate, format, undistortion, point cloud, HDR merge, and device-level decimation from it when the corresponding launch / YAML parameters have not been passed to the node.
*   **`export_config_json_file_path`**
    *   SDK JSON configuration export path. When set, the node exports the current device configuration to JSON after initialization. You can also export at runtime with the `/camera/export_config_json` service. See [Gemini 330 Series SDK JSON Usage Guide](../5_advanced_guide/configuration/sdk_json_config.md).
    *   Before export, the node syncs the current ROS2 sensor stream, point cloud, and HDR merge settings into the SDK `application_config` when the device supports it.
*   **`frame_aggregate_mode`**
    *   Set frame aggregate output mode. Optional values: `full_frame`, `color_frame`, `ANY`, `disable`.
    *   This parameter is case-insensitive. Use one of the valid values listed above.
*   **`enable_d2c_viewer`**
    *   Publishes the D2C overlay image (for testing only). See [Aligning Depth to Color](../5_advanced_guide/configuration/align_depth_color.md) for examples.
*   **`depth_colorizer_mode`**
    *   Colorizes the depth image published on `/camera/depth/image_raw`. Supported values are `none`, `jet`, `jet_inv`, and `gray`. `none` keeps the raw depth image, `gray` publishes `mono8`, and `jet` / `jet_inv` publish `rgb8`.
    *   When a colorizer mode other than `none` is selected together with `enable_d2c_viewer:=true`, the node logs a warning and automatically disables `enable_d2c_viewer`, because the viewer requires a raw `16UC1` depth image.

## IMU

*   **`enable_accel`** / **`enable_gyro`**
    *   Enable the Accelerometer/gyroscope and output its info topic data.
*   **`enable_sync_output_accel_gyro`**
    *   Enable the sync `accel_gyro`, and output IMU topic real-time data.
*   **`accel_rate`** / **`gyro_rate`**
    *   The frequency of the accelerometer/gyroscope. Values range from `1.5625hz` to `32khz`.
*   **`accel_range`** / **`gyro_range`**
    *   The range of the accelerometer (`2g`, `4g`, `8g`, `16g`) and gyroscope (`16dps` to `2000dps`).
*   **`enable_accel_data_correction`** / **`enable_gyro_data_correction`**
    *   Enable data correction for the accelerometer/gyroscope.
*   **`linear_accel_cov`** / **`angular_vel_cov`**
    *   Covariance of the linear acceleration and angular velocity.

## Depth Filters

*   **`enable_decimation_filter`**
    *   Enable the Depth decimation filter. Set with `decimation_filter_scale`.
*   **`enable_hdr_merge`**
    *   Enable the Depth hdr merge filter. Set with `hdr_merge_exposure_1`, etc.
*   **`enable_sequence_id_filter`**
    *   Enable the Depth sequence id filter. Set with `sequence_id_filter_id`.
*   **`enable_threshold_filter`**
    *   Enable the Depth threshold filter. Set with `threshold_filter_max` and `threshold_filter_min`.
*   **`enable_hardware_noise_removal_filter`**
    *   Enable the Depth hardware noise removal filter. For Gemini 330 series devices, an empty value uses the SDK default. See [Lower CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md) for low-CPU configuration recommendations.
*   **`enable_noise_removal_filter`**
    *   Enable the Depth software noise removal filter. For Gemini 330 series devices, an empty value uses the SDK default. Set `noise_removal_filter_min_diff`, etc. See [Lower CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md) for low-CPU configuration recommendations.
* **`enable_false_positive_filter`**
  * Enable this option to reduce ghosting noise. For usage examples and runtime tuning, see [Gemini 330 Series FalsePositiveFilter Usage Guide](../5_advanced_guide/configuration/false_positive_filter.md).
*   **`enable_spatial_filter`**
    *   Enable the Depth spatial filter. Set with `spatial_filter_alpha`, etc. See [Lower CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md) for low-CPU configuration recommendations.
*   **`enable_temporal_filter`**
    *   Enable the Depth temporal filter. Set with `temporal_filter_diff_threshold`, etc.
*   **`enable_hole_filling_filter`**
    *   Enable the Depth hole filling filter. Set with `hole_filling_filter_mode`.
*   **`enable_spatial_fast_filter`**
    *   Enable the Depth spatial fast filter. Set with `spatial_fast_filter_radius`.
*   **`enable_spatial_moderate_filter`**
    *   Enable the Depth spatial moderate filter. Set with `spatial_moderate_filter_diff_threshold`, etc.
*   **`enable_mgc_noise_removal_filter`**
    *   Enable the MGC noise removal filter. This parameter is available for Astra Mini (S) Pro, DaBai Pro Max, and DaBai DCW2.
*   **`enable_lut_noise_removal_filter`**
    *   Enable the LUT noise removal filter. This parameter is available for Astra Mini (S) Pro, DaBai Pro Max, and DaBai DCW2.
* **`enable_enhanced_depth`**
  * Enable LingBot enhanced depth filtering. The default is `false`. Both Color and Depth must be enabled, and D2C/C2D alignment must be configured. For complete environment, startup, and image requirements, see the [EnhancedDepthFilter Usage Guide](../5_advanced_guide/configuration/enhanced_depth_filter.md).
* **`enhanced_depth_model_path`**
  * Path to the LingBot `model.sm4` file. The default is empty. This parameter is required when enhanced depth filtering is enabled; an absolute path is recommended. The model file cannot be changed at runtime.
* **`enhanced_depth_confidence_threshold`**
  * Confidence threshold for enhanced depth filtering. It must be an integer from `0` to `255`. The default is `51`.
* **`enable_edge_noise_removal_filter`**
  * Enable EdgeNoiseRemovalFilter to reduce edge noise in depth frames.
* **`enable_disp_outliers_filter`**
  * Enable DispOutliersFilter to remove disparity outliers in depth frames.
* **`disp_outliers_filter_search_mode`**
  * Set the DispOutliersFilter search mode. Leave it empty to keep the SDK default. Options: `FULL`, `OFFSET_80`. The value is case-insensitive.

---

> **_IMPORTANT_**: Please carefully read the instructions regarding software filtering settings at [this link](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/). If you are uncertain, do not modify these settings.
