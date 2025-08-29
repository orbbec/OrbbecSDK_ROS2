# All available service for camera control

> The name of the following service already expresses its function.
> However, it should be noted that the corresponding `set_[ir|depth|color]*`
> and `get[ir|right_ir|left_ir|depth|color]*` **services are only available if you set** `enable[ir|depth|color]`
> to `true` in the stream that corresponds to the argument of the launch file.

* `/camera/get_color_exposure`

```bash
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
```

* `/camera/get_color_gain`

```bash
ros2 service call /camera/get_color_gain orbbec_camera_msgs/srv/GetInt32 '{}'
```

* `/camera/set_color_auto_exposure`

```bash
ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_color_exposure`

```bash
ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 '{data: 1}'
```

* `/camera/set_color_gain`

```bash
ros2 service call /camera/set_color_gain orbbec_camera_msgs/srv/SetInt32 '{data: 64}'
```

* `/camera/set_color_ae_roi`

```bash
#In data_param, the first value is the Left setting, the second value is the Right setting, the third value is the Top setting, and the fourth value is the Bottom setting.
ros2 service call /camera/set_color_ae_roi orbbec_camera_msgs/srv/SetArrays '{data_param: [0,1279,0,719]}'
```

* `/camera/set_color_mirror`

```bash
ros2 service call /camera/set_color_mirror std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_color_flip`

```bash
ros2 service call /camera/set_color_flip std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_color_rotation`

```bash
ros2 service call /camera/set_color_rotation orbbec_camera_msgs/srv/SetInt32 '{data: 180}'
```

* `/camera/toggle_color`

```bash
ros2 service call /camera/toggle_color std_srvs/srv/SetBool '{data: true}'
```

* `/camera/get_depth_exposure`

```bash
ros2 service call /camera/get_depth_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
```

* `/camera/get_depth_gain`

```bash
ros2 service call /camera/get_depth_gain orbbec_camera_msgs/srv/GetInt32 '{}'
```

* `/camera/set_depth_auto_exposure`

```bash
ros2 service call /camera/set_depth_auto_exposure std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_depth_exposure`

```bash
 ros2 service call /camera/set_depth_exposure orbbec_camera_msgs/srv/SetInt32 '{data: 3000}'
```

* `/camera/set_depth_gain`

```bash
ros2 service call /camera/set_depth_gain orbbec_camera_msgs/srv/SetInt32 '{data: 64}'
```

* `/camera/set_depth_ae_roi`

```bash
#In data_param, the first value is the Left setting, the second value is the Right setting, the third value is the Top setting, and the fourth value is the Bottom setting.
ros2 service call /camera/set_depth_ae_roi orbbec_camera_msgs/srv/SetArrays '{data_param: [0,847,0,479]}'
```

* `/camera/set_depth_mirror`

```bash
ros2 service call /camera/set_depth_mirror std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_depth_flip`

```bash
ros2 service call /camera/set_depth_flip std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_depth_rotation`

```bash
ros2 service call /camera/set_depth_rotation orbbec_camera_msgs/srv/SetInt32 '{data: 180}'
```

* `/camera/toggle_depth`

```bash
ros2 service call /camera/toggle_depth std_srvs/srv/SetBool '{data: true}'
```

* `/camera/get_ir_exposure`

```bash
ros2 service call /camera/get_ir_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
```

* `/camera/get_ir_gain`

```bash
ros2 service call /camera/get_ir_gain orbbec_camera_msgs/srv/GetInt32 '{}'
```

* `/camera/set_ir_long_exposure`

```bash
ros2 service call /camera/set_ir_long_exposure std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_ir_auto_exposure`

```bash
ros2 service call /camera/set_ir_auto_exposure std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_ir_exposure`

```bash
ros2 service call /camera/set_ir_exposure orbbec_camera_msgs/srv/SetInt32 '{data: 3000}'
```

* `/camera/set_ir_gain`

```bash
ros2 service call /camera/set_ir_gain orbbec_camera_msgs/srv/SetInt32 '{data: 64}'
```

* `/camera/set_ir_mirror`

```bash
ros2 service call /camera/set_ir_mirror std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_ir_flip`

```bash
ros2 service call /camera/set_ir_flip std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_ir_rotation`

```bash
ros2 service call /camera/set_ir_rotation orbbec_camera_msgs/srv/SetInt32 '{data: 180}'
```

* `/camera/switch_ir`

```bash
ros2 service call /camera/switch_ir orbbec_camera_msgs/srv/SetString '{data: left}'
```

* `/camera/toggle_ir`

```bash
ros2 service call /camera/toggle_ir std_srvs/srv/SetBool '{data: true}'
```

* `/camera/get_auto_white_balance`

```bash
ros2 service call /camera/get_auto_white_balance orbbec_camera_msgs/srv/GetInt32 '{}'
```

* `/camera/set_auto_white_balance`

```bash
ros2 service call /camera/set_auto_white_balance std_srvs/srv/SetBool '{data: true}'
```

* `/camera/get_white_balance`

```bash
ros2 service call /camera/get_white_balance orbbec_camera_msgs/srv/GetInt32 '{}'
```

* `/camera/set_white_balance`

```bash
ros2 service call /camera/set_white_balance orbbec_camera_msgs/srv/SetInt32 '{data: 2800}'
```

* `/camera/set_laser_enable`

```bash
ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_ldp_enable`

```bash
ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: true}'
```

* `/camera/get_ldp_status`

```bash
ros2 service call /camera/get_ldp_status orbbec_camera_msgs/srv/GetBool '{}'
```

* `/camera/set_ptp_config`

```bash
`ptp_config`ros2 service call /camera/set_ptp_config std_srvs/srv/SetBool '{data: true}'
```

* `/camera/get_ptp_config`

```bash
ros2 service call /camera/get_ptp_config` orbbec_camera_msgs/srv/GetBool '{}'
```

* `/camera/get_lrm_measure_distance`

```bash
ros2 service call /camera/get_lrm_measure_distance orbbec_camera_msgs/srv/GetInt32 '{}'
```

* `/camera/get_device_info`

```bash
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo
```

* `/camera/get_sdk_version`

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString
```

* `/camera/reboot_device`

```bash
ros2 service call /camera/reboot_device std_srvs/srv/Empty '{}'
```

* `/camera/save_images`

```bash
ros2 service call /camera/save_images std_srvs/srv/Empty '{}'
```

* `/camera/save_point_cloud`

```bash
ros2 service call /camera/save_point_cloud std_srvs/srv/Empty '{}'
```

* `/camera/set_fan_work_mode`

```bash
ros2 service call /camera/set_fan_work_mode orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
```

* `/camera/set_filter`

```bash
#filter_name is the filter name,filter_enable is whether to enable the filter switch,and filter_param is the filter parameter

#set DecimationFilter
ros2 service call /camera/set_filter  orbbec_camera_msgs/srv/SetFilter '{filter_name: DecimationFilter,filter_enable: false,filter_param: [5]}'

#set SpatialAdvancedFilter
ros2 service call /camera/set_filter  orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialAdvancedFilter,filter_enable: true,filter_param: [0.5,160,1,8]}'

#set SequenceIdFilter
ros2 service call /camera/set_filter  orbbec_camera_msgs/srv/SetFilter '{filter_name: SequenceIdFilter,filter_enable: true,filter_param: [1]}'

#set ThresholdFilter
ros2 service call /camera/set_filter  orbbec_camera_msgs/srv/SetFilter '{filter_name: SequenceIdFilter,filter_enable: true,filter_param: [0,15999]}'

#set NoiseRemovalFilter
ros2 service call /camera/set_filter  orbbec_camera_msgs/srv/SetFilter '{filter_name: NoiseRemovalFilter,filter_enable: true,filter_param: [256,80]}'

#set HardwareNoiseRemoval
ros2 service call /camera/set_filter  orbbec_camera_msgs/srv/SetFilter '{filter_name: HardwareNoiseRemoval,filter_enable: true,filter_param: []}'

# Set SpatialFastFilter
# filter_param: [radius]
ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialFastFilter, filter_enable: true, filter_param: [4]}'

# Set SpatialModerateFilter
# filter_param: [disp_diff, magnitude, radius, ]
ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter '{filter_name: SpatialModerateFilter, filter_enable: true, filter_param: [160,1,3]}'
```

* `/camera/set_floor_enable`

```bash
ros2 service call /camera/set_floor_enable std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_reset_timestamp`

```bash
#Only available when time_domain param is set to device
ros2 service call /camera/set_reset_timestamp std_srvs/srv/SetBool '{data: true}'
```

* `/camera/set_sync_interleaverlaser`

```bash
#Only available if interleave_ae_mode param is set to laser and interleave_frame_enable param is set to true
ros2 service call /camera/set_sync_interleaverlaser orbbec_camera_msgs/srv/SetInt32 '{data: 0}'
```

* `/camera/set_sync_hosttime`

```
ros2 service call /camera/set_sync_hosttime std_srvs/srv/SetBool '{data: true}'
```

* `/camera/send_software_trigger`

```
ros2 service call /camera/send_software_trigger std_srvs/srv/SetBool '{data: true}'
```

- `/camera/set_streams_enable`

```bash
ros2 service call /camera/set_streams_enable std_srvs/srv/SetBool '{data: false}'
```

- `/camera/get_streams_enable`

```bash
ros2 service call /camera/get_streams_enable std_srvs/srv/GetBool '{}'
```

- `/camera/write_customer_data`

```bash
ros2 service call /camera/write_customer_data orbbec_camera_msgs/srv/SetString '{data: "string"}'
```

- `/camera/read_customer_data`

```bash
ros2 service call /camera/read_customer_data orbbec_camera_msgs/srv/GetString '{}'
```

- `/camera/set_user_calib_params`

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
```

- `/camera/get_user_calib_params`

```bash
ros2 service call /camera/get_user_calib_params orbbec_camera_msgs/srv/GetUserCalibParams '{}'
```

> **Note**:
> The following four services (`/camera/write_customer_data`, `/camera/read_customer_data`, `/camera/set_user_calib_params`, `/camera/get_user_calib_params`) are currently supported only on the 435Le module.
> Each service can store only one set of data or string at a time.
