# 所有可用的相机控制服务

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
    `/camera/get_laser_status`
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
*   `/camera/set_floor_enable`
    ```bash
    ros2 service call /camera/set_floor_enable std_srvs/srv/SetBool '{data: true}'
    ```

### 设备信息与管理

*   `/camera/get_device_info`
    ```bash
    ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo
    ```
*   `/camera/get_sdk_version`
    ```bash
    ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString
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

### 深度滤波器配置

*   `/camera/set_filter`
    ```bash
    # 设置DecimationFilter
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: DecimationFilter, filter_enable: false, filter_param: [5]}'

    # 设置SpatialAdvancedFilter
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_param: [0.5,160,1,8]}'

    # 设置SequenceIdFilter
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SequenceIdFilter, filter_enable: true, filter_param: [1]}'

    # 设置ThresholdFilter
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: ThresholdFilter, filter_enable: true, filter_param: [0,15999]}'

    # 设置NoiseRemovalFilter
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: NoiseRemovalFilter, filter_enable: true, filter_param: [256,80]}'

    # 设置HardwareNoiseRemoval
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: HardwareNoiseRemoval, filter_enable: true, filter_param: []}'

    # 设置SpatialFastFilter
    # filter_param: [radius]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialFastFilter, filter_enable: true, filter_param: [4]}'

    # 设置SpatialModerateFilter
    # filter_param: [disp_diff, magnitude, radius]
    ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialModerateFilter, filter_enable: true, filter_param: [160,1,3]}'
    ```

### 数据捕获与校准管理

*   `/camera/save_images`
    ```bash
    ros2 service call /camera/save_images std_srvs/srv/Empty '{}'
    ```
*   `/camera/save_point_cloud`
    ```bash
    ros2 service call /camera/save_point_cloud std_srvs/srv/Empty '{}'
    ```

> **注意**：以下服务目前仅支持435Le模块。每个服务一次只能存储一组数据或字符串。

*   `/camera/write_customer_data` & `/camera/read_customer_data`
    ```bash
    ros2 service call /camera/write_customer_data orbbec_camera_msgs/srv/SetString '{data: "string"}'
    ros2 service call /camera/read_customer_data orbbec_camera_msgs/srv/GetString '{}'
    ```
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

### 点云下采样
*   `/camera/set_point_cloud_decimation`
    ```bash
    ros2 service call /camera/set_point_cloud_decimation orbbec_camera_msgs/srv/SetInt32 '{data: 8}'
    ```
*   `/camera/get_point_cloud_decimation`
    ```bash
    ros2 service call /camera/get_point_cloud_decimation orbbec_camera_msgs/srv/GetInt32 '{}'
    ```
