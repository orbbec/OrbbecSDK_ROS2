# 可用服务

> **注意：** 与特定数据流相关的服务（例如 `/camera/set_color_*`）仅在启动文件中启用该数据流时可用（例如 `enable_color:=true`）。

### 数据流控制

#### 彩色流
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
    # data_param: [左, 右, 上, 下]
    ros2 service call /camera/set_color_ae_roi orbbec_camera_msgs/srv/SetArrays '{data_param: [0,1279,0,719]}'
    ```

#### 深度流
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
    # data_param: [左, 右, 上, 下]
    ros2 service call /camera/set_depth_ae_roi orbbec_camera_msgs/srv/SetArrays '{data_param: [0,847,0,479]}'
    ```

#### 红外流
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

#### 所有数据流
*   `/camera/get_streams_enable` & `/camera/set_streams_enable`
    ```bash
    ros2 service call /camera/get_streams_enable orbbec_camera_msgs/srv/GetBool '{}'
    ros2 service call /camera/set_streams_enable std_srvs/srv/SetBool '{data: false}'
    ```

### 彩色帧队列诊断

* `/camera/get_color_queue_stats`

该服务使用 `std_srvs/srv/SetBool`。请求值为 `false` 时查询当前统计信息；请求值为 `true` 时查询并重置累计统计信息。

```bash
ros2 service call /camera/get_color_queue_stats std_srvs/srv/SetBool '{data: false}'
```

响应的 `message` 字段为 JSON，包含总 `overflow_count` 和 `queues` 对象。每个已启用队列会报告 `capacity_frames`、`queue_size`、`max_queue_size`、`overflow_count`、`oldest_queue_wait_ms` 和 `max_queue_wait_ms`。响应同时包含节点 `namespace` 以及是否执行了统计重置的 `statistics_reset` 字段。

```bash
ros2 service call /camera/get_color_queue_stats std_srvs/srv/SetBool '{data: true}'
```

### 运行时数据流配置

*   `/camera/set_stream_profile`

    用于在节点运行期间切换一个或多个已启用图像流的 Profile。`stream_name` 支持 `color`、`left_color`、`right_color`、`depth`、`ir`、`left_ir` 和 `right_ir`。宽、高、帧率或格式可以只填写需要修改的字段；未修改的数值字段填写 `0`，格式填写空字符串。切换时节点会停止并重新启动数据流；如果目标 Profile 已经生效，服务会返回失败。Gemini 301 系列要求所有启用且 FPS 大于 `0` 的图像流使用相同 FPS，服务会校验此限制。

    ```bash
    ros2 service call /camera/set_stream_profile orbbec_camera_msgs/srv/SetStreamProfile "{profiles: [{stream_name: color, width: 1280, height: 720, fps: 30, format: MJPG}]}"
    ```

*   `/camera/set_image_registration_mode`

    运行时切换深度和彩色图像对齐模式。可选值为 `OFF`、`HW_D2C`、`SW_D2C` 和 `SW_C2D`，大小写不敏感。除 `OFF` 外，彩色流和深度流必须同时启用。切换时节点会自动重启数据流；失败时会恢复原对齐模式。

    ```bash
    ros2 service call /camera/set_image_registration_mode orbbec_camera_msgs/srv/SetString "{data: HW_D2C}"
    ```

### 传感器与发射器控制

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

### 设备信息与管理

*   `/camera/get_device_info`
    ```bash
    ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo
    ```
*   `/camera/get_device_config`
    获取当前生效的设备配置状态，例如 preset、对齐模式、时间域、同步模式、帧聚合模式等。
    ```bash
    ros2 service call /camera/get_device_config orbbec_camera_msgs/srv/GetDeviceConfig '{}'
    ```
*   `/camera/get_sdk_version`
    ```bash
    ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString
    ```
*   `/camera/export_config_json`
    导出当前设备配置为 SDK JSON 文件。Gemini 330 系列的 JSON 导入导出流程请参考 [Gemini 330 系列 SDK JSON 使用说明](../5_advanced_guide/configuration/sdk_json_config.md)。
    ```bash
    ros2 service call /camera/export_config_json orbbec_camera_msgs/srv/SetString "{data: '/tmp/orbbec_camera_config.json'}"
    ```
*   `/camera/set_bag_recording`
    使用 SDK bag 录制当前设备数据。`enable: true` 开始录制，`enable: false` 停止录制；`file_path` 为空时使用当前工作目录下的默认文件名。
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

### 同步与触发

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
    # 仅在time_domain参数设置为device时可用
    ros2 service call /camera/set_reset_timestamp std_srvs/srv/SetBool '{data: true}'
    ```
*   `/camera/set_sync_interleaverlaser`
    ```bash
    # 仅在interleave_ae_mode为'laser'且interleave_frame_enable为true时可用
    ros2 service call /camera/set_sync_interleaverlaser orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
    ```
*   `/camera/set_sync_io_voltage_level`
    设置同步 IO 电压等级。仅支持具备该属性的设备。
    ```bash
    ros2 service call /camera/set_sync_io_voltage_level orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
    ```

### 深度滤波器配置

*   `/camera/set_filter`
    `FalsePositiveFilter` 的启动参数、状态确认和命名参数调参示例可参考 [Gemini 330 系列 FalsePositiveFilter 使用说明](../5_advanced_guide/configuration/false_positive_filter.md)。`EnhancedDepthFilter` 的环境要求、启动参数和状态确认方法可参考 [EnhancedDepthFilter 使用说明](../5_advanced_guide/configuration/enhanced_depth_filter.md)。
    ```bash
    # filter_name 为滤波器名称，filter_enable 表示是否开启滤波器开关。
    # filter_param 为旧的按位置传参方式；filter_config 为新的命名参数方式。
    # filter_param 和 filter_config 不能同时使用。

    # 设置 DecimationFilter: [scale]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: DecimationFilter, filter_enable: false, filter_param: [5]}'

    # 设置 SpatialAdvancedFilter: [alpha, disp_diff, magnitude, radius]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_param: [0.5,160,1,8]}'

    # 设置 SequenceIdFilter: [sequence_id]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SequenceIdFilter, filter_enable: true, filter_param: [1]}'

    # 设置 ThresholdFilter: [min, max]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: ThresholdFilter, filter_enable: true, filter_param: [0,15999]}'

    # 设置 NoiseRemovalFilter: [min_diff, max_size]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: NoiseRemovalFilter, filter_enable: true, filter_param: [256,80]}'

    # 设置 HardwareNoiseRemoval: [threshold]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: HardwareNoiseRemoval, filter_enable: true, filter_param: [0.2]}'

    # 设置 SpatialFastFilter: [radius]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialFastFilter, filter_enable: true, filter_param: [4]}'

    # 设置 SpatialModerateFilter: [disp_diff, magnitude, radius]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialModerateFilter, filter_enable: true, filter_param: [160,1,3]}'

    # 设置 FalsePositiveFilter: []
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: FalsePositiveFilter, filter_enable: true, filter_param: []}'

    # 设置 EnhancedDepthFilter: [confidence_threshold]，阈值必须是 0 到 255 之间的整数
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: EnhancedDepthFilter, filter_enable: true, filter_param: [60]}'

    # 设置 MgcNoiseRemovalFilter / LutNoiseRemovalFilter: []
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: MgcNoiseRemovalFilter, filter_enable: true, filter_param: []}'
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: LutNoiseRemovalFilter, filter_enable: true, filter_param: []}'

    # 设置 EdgeNoiseRemovalFilter: []
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: EdgeNoiseRemovalFilter, filter_enable: true, filter_param: []}'

    # 使用 filter_config 按参数名调参
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: NoiseRemovalFilter, filter_enable: true, filter_config: [{name: min_diff, value: '256'}, {name: max_size, value: '80'}]}"
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: HardwareNoiseRemovalFilter, filter_enable: true, filter_config: [{name: threshold, value: '0.2'}]}"
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_config: [{name: alpha, value: '0.5'}, {name: disp_diff, value: '160'}, {name: magnitude, value: '1'}, {name: radius, value: '8'}]}"
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: EnhancedDepthFilter, filter_enable: true, filter_config: [{name: confidence_threshold, value: '60'}]}"

    # 设置 DispOutliersFilter。search_mode 支持 FULL 或 OFFSET_80，大小写不敏感。
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: DispOutliersFilter, filter_enable: true, filter_config: [{name: search_mode, value: 'FULL'}]}"
    ```

    滤波状态会在服务调用后更新到 `/camera/depth_filters/status`。该话题使用结构化消息 `orbbec_camera_msgs/msg/DepthFiltersStatus`，包含每个滤波器的使能状态和参数。

### 视差配置

*   `/camera/set_disparity_range_mode`
    ```bash
    ros2 service call /camera/set_disparity_range_mode orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
    ```
*   `/camera/set_disparity_search_offset`
    ```bash
    ros2 service call /camera/set_disparity_search_offset orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
    ```

### 数据捕获与校准管理

*   `/camera/save_images`
    ```bash
    ros2 service call /camera/save_images std_srvs/srv/Empty '{}'
    ```
    该服务会为每个已启用的图像流保存帧，数量受 `max_save_images_count` 限制（默认值为 `10`）。每帧都会在当前工作目录下的 `image` 目录中生成对应的 `.raw`、`.png` 和元数据 `.json` 文件。文件名包含流名称、分辨率、帧率、带微秒精度的本地时间戳和帧序号。
*   `/camera/save_point_cloud`
    ```bash
    ros2 service call /camera/save_point_cloud std_srvs/srv/Empty '{}'
    ```

### 特定设备

*   `/camera/write_customer_data` & `/camera/read_customer_data`
    ```bash
    ros2 service call /camera/write_customer_data orbbec_camera_msgs/srv/SetString '{data: "string"}'
    ros2 service call /camera/read_customer_data orbbec_camera_msgs/srv/GetString '{}'
    ```
    > **支持模组**：Gemini 435Le。
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
    > **支持模组**：Gemini 435Le。
*   `/camera/set_ae_reference_stream`
    ```bash
      # depth or color
      ros2 service call /camera/set_ae_reference_stream orbbec_camera_msgs/srv/SetString "{data: depth}"
    ```
    > **支持模组**：Gemini 301 系列。
    > **兼容说明**：替代旧服务 `/camera/set_ae_mode`，旧值 `depthbased/colorbased` 对应新值 `depth/color`。
*   `/camera/set_ae_strategy`
    ```bash
      # default or motion
      ros2 service call /camera/set_ae_strategy orbbec_camera_msgs/srv/SetString "{data: motion}"
    ```
    > **支持模组**：Gemini 301 系列。
    > **兼容说明**：替代旧服务 `/camera/set_sports_mode`。

### 点云下采样
*   `/camera/set_point_cloud_decimation`
    ```bash
    ros2 service call /camera/set_point_cloud_decimation orbbec_camera_msgs/srv/SetInt32 '{data: 8}'
    ```
*   `/camera/get_point_cloud_decimation`
    ```bash
    ros2 service call /camera/get_point_cloud_decimation orbbec_camera_msgs/srv/GetInt32 '{}'
    ```
