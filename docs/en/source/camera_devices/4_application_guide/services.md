# Available services

> **Note:** Services related to a specific stream (e.g., `/camera/set_color_*`) are only available if that stream is enabled in the launch file (e.g., `enable_color:=true`).

### Stream Control

#### Color Stream
*   `/camera/toggle_color`
    ```bash
    ros2 service call /camera/toggle_color std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/get_color_exposure` & `/camera/get_color_gain`
    ```bash
    ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
    ros2 service call /camera/get_color_gain orbbec_camera_msgs/srv/GetInt32 '{}'
    ```
*   `/camera/set_color_auto_exposure`
    ```bash
    ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/set_color_exposure` & `/camera/set_color_gain`
    ```bash
    ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 '{data: 1}'
    ros2 service call /camera/set_color_gain orbbec_camera_msgs/srv/SetInt32 '{data: 64}'
    ```
*   `/camera/set_color_mirror`, `/camera/set_color_flip`, `/camera/set_color_rotation`
    ```bash
    ros2 service call /camera/set_color_mirror std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/set_color_flip std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/set_color_rotation orbbec_camera_msgs/srv/SetInt32 '{data: 180}'
    ```
*   `/camera/set_color_ae_roi`
    ```bash
    # data_param: [Left, Right, Top, Bottom]
    ros2 service call /camera/set_color_ae_roi orbbec_camera_msgs/srv/SetArrays '{data_param: [0,1279,0,719]}'
    ```

#### Depth Stream
*   `/camera/toggle_depth`
    ```bash
    ros2 service call /camera/toggle_depth std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/get_depth_exposure` & `/camera/get_depth_gain`
    ```bash
    ros2 service call /camera/get_depth_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
    ros2 service call /camera/get_depth_gain orbbec_camera_msgs/srv/GetInt32 '{}'
    ```
*   `/camera/set_depth_auto_exposure`
    ```bash
    ros2 service call /camera/set_depth_auto_exposure std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/set_depth_exposure` & `/camera/set_depth_gain`
    ```bash
    ros2 service call /camera/set_depth_exposure orbbec_camera_msgs/srv/SetInt32 '{data: 3000}'
    ros2 service call /camera/set_depth_gain orbbec_camera_msgs/srv/SetInt32 '{data: 64}'
    ```
*   `/camera/set_depth_mirror`, `/camera/set_depth_flip`, `/camera/set_depth_rotation`
    ```bash
    ros2 service call /camera/set_depth_mirror std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/set_depth_flip std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/set_depth_rotation orbbec_camera_msgs/srv/SetInt32 '{data: 180}'
    ```
*   `/camera/set_depth_ae_roi`
    ```bash
    # data_param: [Left, Right, Top, Bottom]
    ros2 service call /camera/set_depth_ae_roi orbbec_camera_msgs/srv/SetArrays '{data_param: [0,847,0,479]}'
    ```

#### IR Stream
*   `/camera/toggle_ir`

    ```bash
    ros2 service call /camera/toggle_ir std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/get_ir_exposure` & `/camera/get_ir_gain`
    ```bash
    ros2 service call /camera/get_ir_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
    ros2 service call /camera/get_ir_gain orbbec_camera_msgs/srv/GetInt32 '{}'
    ```
*   `/camera/set_ir_long_exposure`
    ```bash
    ros2 service call /camera/set_ir_long_exposure std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/set_ir_auto_exposure`
    ```bash
    ros2 service call /camera/set_ir_auto_exposure std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/set_ir_exposure` & `/camera/set_ir_gain`
    ```bash
    ros2 service call /camera/set_ir_exposure orbbec_camera_msgs/srv/SetInt32 '{data: 3000}'
    ros2 service call /camera/set_ir_gain orbbec_camera_msgs/srv/SetInt32 '{data: 64}'
    ```
*   `/camera/set_ir_mirror`, `/camera/set_ir_flip`, `/camera/set_ir_rotation`
    ```bash
    ros2 service call /camera/set_ir_mirror std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/set_ir_flip std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/set_ir_rotation orbbec_camera_msgs/srv/SetInt32 '{data: 180}'
    ```
*   `/camera/switch_ir`
    ```bash
    ros2 service call /camera/switch_ir orbbec_camera_msgs/srv/SetString '{data: left}'
    ```

#### All Streams
*   `/camera/get_streams_enable` & `/camera/set_streams_enable`
    ```bash
    ros2 service call /camera/get_streams_enable orbbec_camera_msgs/srv/GetBool '{}'
    ros2 service call /camera/set_streams_enable std_srvs/srv/SetBool '{data: false}'
    ```

### Color Queue Diagnostics

* `/camera/get_color_queue_stats`

This service uses `std_srvs/srv/SetBool`. Pass `false` to query the current statistics, or `true` to query and reset the accumulated statistics.

```bash
ros2 service call /camera/get_color_queue_stats std_srvs/srv/SetBool '{data: false}'
```

The JSON response in `message` contains the total `overflow_count` and a `queues` object. Each enabled queue reports `capacity_frames`, `queue_size`, `max_queue_size`, `overflow_count`, `oldest_queue_wait_ms`, and `max_queue_wait_ms`. The service also reports the node `namespace` and whether the statistics were reset in `statistics_reset`.

```bash
ros2 service call /camera/get_color_queue_stats std_srvs/srv/SetBool '{data: true}'
```

### Runtime Stream Configuration

*   `/camera/set_stream_profile`

    Switch one or more enabled image stream profiles while the node is running. `stream_name` accepts `color`, `left_color`, `right_color`, `depth`, `ir`, `left_ir`, and `right_ir`. Specify only the fields to change; use `0` for unchanged numeric fields and an empty string for an unchanged format. The node stops and restarts the streams during the switch. The service reports failure if the requested profile is already active. On Gemini 301 series devices, all enabled image streams with an FPS greater than `0` must use the same FPS; the service validates this requirement.

    ```bash
    ros2 service call /camera/set_stream_profile orbbec_camera_msgs/srv/SetStreamProfile "{profiles: [{stream_name: color, width: 1280, height: 720, fps: 30, format: MJPG}]}"
    ```

*   `/camera/set_image_registration_mode`

    Switch depth-color image registration at runtime. Valid values are `OFF`, `HW_D2C`, `SW_D2C`, and `SW_C2D`, case-insensitive. Both color and depth streams must be enabled for every mode except `OFF`. The node restarts streams automatically and restores the previous mode if the switch fails.

    ```bash
    ros2 service call /camera/set_image_registration_mode orbbec_camera_msgs/srv/SetString "{data: HW_D2C}"
    ```

### Sensor & Emitter Control

*   `/camera/set_auto_white_balance` & `/camera/get_auto_white_balance`
    ```bash
    ros2 service call /camera/set_auto_white_balance std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/get_auto_white_balance orbbec_camera_msgs/srv/GetInt32 '{}'
    ```
*   `/camera/set_white_balance` & `/camera/get_white_balance`
    ```bash
    ros2 service call /camera/set_white_balance orbbec_camera_msgs/srv/SetInt32 '{data: 2800}'
    ros2 service call /camera/get_white_balance orbbec_camera_msgs/srv/GetInt32 '{}'
    ```
*   `/camera/set_laser_enable`
    ```bash
    ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/get_laser_status`
    ```bash
    ros2 service call /camera/get_laser_status orbbec_camera_msgs/srv/GetBool '{}'
    ```
*   `/camera/set_ldp_enable` & `/camera/get_ldp_status`
    ```bash
    ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/get_ldp_status orbbec_camera_msgs/srv/GetBool '{}'
    ```
*   `/camera/set_ptp_config` & `/camera/get_ptp_config`
    ```bash
    ros2 service call /camera/set_ptp_config std_srvs/srv/SetBool '{data: true}'
    ros2 service call /camera/get_ptp_config orbbec_camera_msgs/srv/GetBool '{}'
    ```
*   `/camera/get_lrm_measure_distance`
    ```bash
    ros2 service call /camera/get_lrm_measure_distance orbbec_camera_msgs/srv/GetInt32 '{}'
    ```
*   `/camera/set_fan_work_mode`
    ```bash
    ros2 service call /camera/set_fan_work_mode orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
    ```
*   `/camera/set_flood_enable`
    ```bash
    ros2 service call /camera/set_flood_enable std_srvs/srv/SetBool '{data: true}'
    ```

### Device Information & Management

*   `/camera/get_device_info`
    ```bash
    ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo
    ```
*   `/camera/get_device_config`
    Get the currently effective device configuration state, such as preset, alignment mode, time domain, sync mode, and frame aggregate mode.
    ```bash
    ros2 service call /camera/get_device_config orbbec_camera_msgs/srv/GetDeviceConfig '{}'
    ```
*   `/camera/get_sdk_version`
    ```bash
    ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString
    ```
*   `/camera/export_config_json`
    Export the current device configuration to an SDK JSON file. For the Gemini 330 series JSON import and export workflow, see [Gemini 330 Series SDK JSON Usage Guide](../5_advanced_guide/configuration/sdk_json_config.md).
    ```bash
    ros2 service call /camera/export_config_json orbbec_camera_msgs/srv/SetString "{data: '/tmp/orbbec_camera_config.json'}"
    ```
*   `/camera/set_bag_recording`
    Record current device data with SDK bag recording. `enable: true` starts recording and `enable: false` stops recording. When `file_path` is empty, the default file name is created in the current working directory.
    ```bash
    ros2 service call /camera/set_bag_recording orbbec_camera_msgs/srv/SetBagRecording "{enable: true, file_path: '/tmp/orbbec_record.bag'}"
    ```
    ```bash
    ros2 service call /camera/set_bag_recording orbbec_camera_msgs/srv/SetBagRecording "{enable: false, file_path: ''}"
    ```
*   `/camera/reboot_device`
    ```bash
    ros2 service call /camera/reboot_device std_srvs/srv/Empty '{}'
    ```

### Synchronization & Triggering

*   `/camera/send_software_trigger`
    ```bash
    ros2 service call /camera/send_software_trigger std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/set_sync_hosttime`
    ```bash
    ros2 service call /camera/set_sync_hosttime std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/set_reset_timestamp`
    ```bash
    # Only available when time_domain param is set to device
    ros2 service call /camera/set_reset_timestamp std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/set_sync_interleaverlaser`
    ```bash
    # Only available if interleave_ae_mode is 'laser' and interleave_frame_enable is true
    ros2 service call /camera/set_sync_interleaverlaser orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
    ```
*   `/camera/set_sync_io_voltage_level`
    Set the sync IO voltage level. This is only supported on devices that expose the property.
    ```bash
    ros2 service call /camera/set_sync_io_voltage_level orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
    ```

### Depth Filter Configuration

*   `/camera/set_filter`
    For `FalsePositiveFilter` startup parameters, status checks, and named-parameter tuning examples, see [Gemini 330 Series FalsePositiveFilter Usage Guide](../5_advanced_guide/configuration/false_positive_filter.md). For `EnhancedDepthFilter` environment requirements, startup parameters, and status checks, see the [EnhancedDepthFilter Usage Guide](../5_advanced_guide/configuration/enhanced_depth_filter.md).
    ```bash
    # filter_name is the filter name, and filter_enable indicates whether the filter is enabled.
    # filter_param is the legacy positional parameter form; filter_config is the new named parameter form.
    # filter_param and filter_config cannot be used at the same time.

    # Set DecimationFilter: [scale]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: DecimationFilter, filter_enable: false, filter_param: [5]}'

    # Set SpatialAdvancedFilter: [alpha, disp_diff, magnitude, radius]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_param: [0.5,160,1,8]}'

    # Set SequenceIdFilter: [sequence_id]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SequenceIdFilter, filter_enable: true, filter_param: [1]}'

    # Set ThresholdFilter: [min, max]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: ThresholdFilter, filter_enable: true, filter_param: [0,15999]}'

    # Set NoiseRemovalFilter: [min_diff, max_size]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: NoiseRemovalFilter, filter_enable: true, filter_param: [256,80]}'

    # Set HardwareNoiseRemoval: [threshold]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: HardwareNoiseRemoval, filter_enable: true, filter_param: [0.2]}'

    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialFastFilter, filter_enable: true, filter_param: [4]}'

    # Set SpatialModerateFilter: [disp_diff, magnitude, radius]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialModerateFilter, filter_enable: true, filter_param: [160,1,3]}'

    # Set FalsePositiveFilter: []
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: FalsePositiveFilter, filter_enable: true, filter_param: []}'

    # Set EnhancedDepthFilter: [confidence_threshold]. The threshold must be an integer from 0 to 255.
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: EnhancedDepthFilter, filter_enable: true, filter_param: [60]}'

    # Set MgcNoiseRemovalFilter / LutNoiseRemovalFilter: []
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: MgcNoiseRemovalFilter, filter_enable: true, filter_param: []}'
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: LutNoiseRemovalFilter, filter_enable: true, filter_param: []}'

    # Set EdgeNoiseRemovalFilter: []
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: EdgeNoiseRemovalFilter, filter_enable: true, filter_param: []}'

    # Tune filters with named parameters
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: NoiseRemovalFilter, filter_enable: true, filter_config: [{name: min_diff, value: '256'}, {name: max_size, value: '80'}]}"
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: HardwareNoiseRemovalFilter, filter_enable: true, filter_config: [{name: threshold, value: '0.2'}]}"
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_config: [{name: alpha, value: '0.5'}, {name: disp_diff, value: '160'}, {name: magnitude, value: '1'}, {name: radius, value: '8'}]}"
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: EnhancedDepthFilter, filter_enable: true, filter_config: [{name: confidence_threshold, value: '60'}]}"

    # Set DispOutliersFilter. search_mode accepts FULL or OFFSET_80, case-insensitive.
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: DispOutliersFilter, filter_enable: true, filter_config: [{name: search_mode, value: 'FULL'}]}"
    ```

    Filter status is updated after service calls on `/camera/depth_filters/status`. The topic uses the structured `orbbec_camera_msgs/msg/DepthFiltersStatus` message and includes each filter's enabled state and parameters.

### Disparity Configuration

*   `/camera/set_disparity_range_mode`
    ```bash
    ros2 service call /camera/set_disparity_range_mode orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
    ```
*   `/camera/set_disparity_search_offset`
    ```bash
    ros2 service call /camera/set_disparity_search_offset orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
    ```

### Data Capture

*   `/camera/save_images`
    ```bash
    ros2 service call /camera/save_images std_srvs/srv/Empty '{}'
    ```
    The service saves frames from each enabled image stream, up to the configured `max_save_images_count` (default `10`). For every frame, the node writes matching `.raw`, `.png`, and metadata `.json` files under the `image` directory in the current working directory. File names include the stream, resolution, frame rate, local timestamp with microsecond precision, and frame index.
*   `/camera/save_point_cloud`
    ```bash
    ros2 service call /camera/save_point_cloud std_srvs/srv/Empty '{}'
    ```

#### Device-Specific

*   `/camera/write_customer_data` & `/camera/read_customer_data`
    ```bash
    ros2 service call /camera/write_customer_data orbbec_camera_msgs/srv/SetString '{data: "string"}'
    ros2 service call /camera/read_customer_data orbbec_camera_msgs/srv/GetString '{}'
    ```
    > **Supported Modules**: Gemini 435Le
*   `/camera/set_user_calib_params` & `/camera/get_user_calib_params`
    ```bash
    ros2 service call /camera/set_user_calib_params orbbec_camera_msgs/srv/SetUserCalibParams \
    '{k: [614.9613647460938, 0.0, 634.91552734375,
          0.0, 614.65771484375, 391.407470703125,
          0.0, 0.0, 1.0],
      d: [-0.03131488710641861,
           0.032955970615148544,
           9.096559369936585e-05,
          -0.0003368517500348389,
          -0.01115430984646082,
           0.0, 0.0, 0.0],
      rotation: [0.9999880790710449,  0.0003024190664291382, -0.004874417092651129,
                -0.0002965621242765337, 0.9999992251396179,   0.001202247804030776,
                 0.004874777048826218, -0.0012007878394797444, 0.9999874234199524],
      translation: [-0.023897956848144532,
                    -9.439220279455185e-05,
                    -6.804073229432106e-06]}'
    ros2 service call /camera/get_user_calib_params orbbec_camera_msgs/srv/GetUserCalibParams '{}'
    ```
    > **Supported Modules**: Gemini 435Le

*   `/camera/set_ae_reference_stream`
    ```bash
      # depth or color
      ros2 service call /camera/set_ae_reference_stream orbbec_camera_msgs/srv/SetString "{data: depth}"
    ```
    > **Supported Modules**: Gemini 301 series.
    > **Compatibility**: Replaces the old `/camera/set_ae_mode` service. The old values `depthbased/colorbased` map to `depth/color`.
*   `/camera/set_ae_strategy`
    ```bash
      # default or motion
      ros2 service call /camera/set_ae_strategy orbbec_camera_msgs/srv/SetString "{data: motion}"
    ```
    > **Supported Modules**: Gemini 301 series.
    > **Compatibility**: Replaces the old `/camera/set_sports_mode` service.

### Point cloud decimation
*   `/camera/set_point_cloud_decimation`
    ```bash
    ros2 service call /camera/set_point_cloud_decimation orbbec_camera_msgs/srv/SetInt32 '{data: 8}'
    ```
*   `/camera/get_point_cloud_decimation`
    ```bash
    ros2 service call /camera/get_point_cloud_decimation orbbec_camera_msgs/srv/GetInt32 '{}'
    ```
