^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package orbbec_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update OrbbecSDK to v2.5.5
* add new launch files and benchmark tools
* Add new features: forceIp, ISP denoising, extrinsic control, etc.
* Improve parameters and defaults
* Fix parameter typos, remove deprecated functions, and other bug fixes

* Contributors: obyalian, slz, xiexun

2.4.7 (2025-09-01)
------------------
* Add static transform accel_gyro_optical_frame
* Change the default value of EnumerateNetDevice in the SDK config file to false‌
* Fix queryDevice thread
* Fix crash by improving lock handling
* Remove mallopt to save CPU usage
* Docs: fix comment typos
* Fix firmware update and avoid colcon symlink issues
* Fix device disconnection after reboot service call 2
* Fix device disconnection after reboot service call
* Rename 'depth_brightness' to 'mean_intensity_set_point', keep old param for compatibility
* Fix typo in setPtpConfigCallback
* Rename 'mean_intensity_set_point' to 'depth_brightness' in launch parameters and update related documentation
* Update Orbbec SDK to version 2.4.11
* Fix reboot_device service
* Bump package version to 2.4.6 in package.xml
* Update Orbbec SDK to version 2.4.10 and modify related configurations
  - Updated the OrbbecSDKConfig-release.cmake files for both arm64 and x64 architectures to reference the new SDK version 2.4.10.
  - Replaced occurrences of 'depth_brightness' with 'mean_intensity_set_point' in launch files and the camera node header to improve clarity and consistency.
  - Added new spatial filtering options (SpatialFastFilter and SpatialModerateFilter) in the camera node implementation, including parameters for radius, magnitude, and difference threshold.
  - Updated the setupDevices and setupDepthPostProcessFilter methods to handle the new mean intensity setting and spatial filter parameters.
  - Modified launch files for various examples to reflect the new argument names and added new parameters for spatial filtering.
* Add FPS counter functionality to OBCameraNode
* Contributors: datean, obyalian, xiexun, yalian

2.4.5 (2025-07-04)
------------------
* bump to v2.4.5
* Update SDK version to v2.4.8
* Add gemini210.launch.py
* Update package.xml
* bump to v2.4.4
* Update SDK version to v2.4.7
* Change the service name of set_software_trigger_enabled to send_software_trigger
* Add .camke-format.py and format CMakeLists.txt
* Add set_software_trigger_enabled param and service
* Fix compilation issues caused by modifying camera_info data content
* Update camera_info data content
* Add color decimation filter in gemini2.launch.py and gemini2L.launch.py
* Update multi_camera_synced_verification_tool README.MD
* Contributors: jj

2.4.3 (2025-05-22)
------------------
* Update README.MD
* Update examples launch
* Update OrbbecSDKConfig_v2.0.xml
* Add enable_color_auto_exposure param in gemini_330_series.launch.py
* bump to v2.4.3
* Update SDK version to v2.4.3
* bump to v2.4.2
* Update SDK verison to v2.4.2
* Fix the abnormal connection problem of multiple network cameras
* Update multi_camera_synced_verify.launch.py
* Fix the crash problem caused by starting the upgrade_firmware param for a single camera when multiple cameras are connected on the PC.
* Update frame_aggregate_mode description
* Update disaparity_to_depth_mode parameter default value
* Update gemini_synced_verify.launch.py and add disaparity_to_depth_mode write condition judgment
* Modify the noise removal filter default value
* Replace enable_hardware_d2d param with disaparity_to_depth_mode
* bump to v2.4.1
* Fix DaBai A and DaBai AL not being able to use imu
* Fix the issue that decimation_filter_scale cannot be set to 1
* Fix color_powerline_freq
* Add /camera/set_write_customer_data and /camera/set_read_customer_data service for Gemini 435Le
* Update filter parameters in astra2.launch.py , gemini2.launch.py and gemini2L.launch.py
* Add gemini435_le.launch.py
* Add hole_filling_filter_mode param in femto_bolt.launch.py and femto_mega.launch.py
* Add noise removal filter, decimation filter, spatial filter, temporal filter and hole filling filter
* bump to v2.4.0
* Update README
* Add enable_ptp_config param for Gemini 335Le
* Add dabai_al.launch.py
* Change the implementation conditions of decimation_filter for Gemini 2/Gemini 2 L
* Fix PID output of list_devices_node
* Add rotation, flip and mirror parameters and services
* Update sdk version to v2.4.1
* Update list_devices_node
* Add dabai_a.launch.py
* Add do_rectify in the camera_info topic (to determine whether to process distortion)
* Add align_target_stream param in gemini_330_series.launch.py
* Update README.MD
* Update 99-obsensor-libusb.rules
* Fix the issue that decimation_filter_scale cannot be set to 8
* Update multi_save_rgbir tool to be compatible with other camera devices (net devices are not supported yet) and removed metadata_export, metadata_save tools
* Update README.MD
* Add custom Gemini 336&Gemini 336L product id
* Add depth_precision param in gemini_330_series.launch.py
* Add enable_disaparity_to_depth param in gemini_330_series.launch.py
* Add enable_accel_data_correction and enable_gyro_data_correction parameters in gemini_330_series.launch.py
* Add color_powerline_freq param in gemini_330_series.launch.py
* Add upgrade_firmware param in gemini_330_series.launch.py
* Update setting set_color/depth_ae_roi service area range
* Update setting color and depth roi parameters area range
* Fix a crash caused by using /camera/set_filter service
* Add load_config_json_file_path and export_config_json_file_path parameters
* Contributors: jj

2.3.4 (2025-04-21)
------------------
* Update gmsl_camera example
* Update multi_camera_synced_verification_tool README.MD
* Remove noise removal filter params and add threshold filter params in femto_bolt.launch.py, femto_mega.launch.py
* Update Filter Configuration Impact in lower_cpu_usage README
* Add noise removal filter related parameters in femto_bolt.launch.py, femto_mega.launch.py
* Add roi related parameters in gemini2.launch.py
* bump to v2.3.4
* Update SDK version to v2.3.5
* bump to v2.3.3
* Update SDK version to v2.3.4
* Add exposure_range_mode param in gemini_330_series.launch.py
* bump to v2.3.2
* Update SDK version to v2.3.3
* Fixed hot-plug crash caused by diagnostic updater
* bump to v2.3.1
* Update SDK version to v2.3.2
* Fix the crash problem caused by connecting to network devices
* Update gemini_330_series.launch.py ​​network device description
* Fix connect net device bug
* Update distortion_model ouput
* Fix the failure of setting hardware_noise_removal_filter_threshold in serivce
* Update gemini2.launch.py ​​to be compatible with Gemini 210 and Gemini 215
* bump to v2.3.0
* bump to v2.2.5
* Add enable_left_ir_sequence_id_filter,left_ir_sequence_id_filter_id,enable_right_ir_sequence_id_filter and right_ir_sequence_id_filter_id in gemini_330_series.launch.py
* Add hardware_noise_removal_filter_threshold param in gemini_330_series.launch.py
* Add Gemini 210 and Gemini 215 device rules
* Update the name of the distortion model in camera_info
* Update sdk version to v2.3.1
* Update multi camera synced
* Fix Gemini2L laser shutdown caused by LDP shutdown
* Add AE ROI setting param in gemini_330_series.launch.py
* Add service to set color and depth ROI
* Fix the bug that hot-plugging and reconnecting failed when starting multi cameras using serial_number
* Update multi_camera.launch.py
* Contributors: jj

2.2.4 (2025-02-28)
------------------
* Update camera_params.yaml
* Roll back enable_lrm to enable_ldp
* bump to v2.2.4
* Fix the bug that Gemini 2 turned off LDP and caused the laser to turn off
* Add update Preset Firmware function, associated parameter preset_firmware_path
* Updated README for benchmark example
* Updated README for multi_camera_synced_verification_tool examples
* Add net_camera examples
* Add gmsl_camera example
* Add disparity_search_offset examples
* Add interleave_ae_mode examples
* Update examples README
* Add multi_camera_synced_verification_tool example
* Add enable_color_decimation_filter and color_decimation_filter_scale param
* Rename depth_mean_intensity_set_point to depth_brightness
* Add lrm_power_level param
* Rename LDP to LRM
* Add depth_mean_intensity_set_point param
* Add enable_depth_auto_exposure_priority param
* Set the device timestamp to 60 seconds
* Update multi_save_rgbir tool
* Add private instruction 2026 to interleave_frame
* Fix the bug that enable_sequence_id_filter does not work
* Change the enable_color_auto_exposure_priority and enable_color_backlight_compenstation parameter types to bool type
* Update multi_save_rgbir tool
* Contributors: jj

2.2.3 (2025-02-15)
------------------
* Update multi_camera_lower_cpu_usage.launch.py
* bump to v2.2.3
* Update sdk version to v2.2.8
* Fix the crash bug caused by enable_d2c_viewer in hotplug
* Fix the bug that the service control SequenceIdFilter setting is invalid
* Fix the bug that the launch file in examples is not available
* Remove cpuid.h in benchmark node
* Add multi_camera_synced and examples README.MD
* Update benchmark README.MD in examples
* Update benchmark and lower_cpu_usage in examples
* bump to v2.2.2
* Update sdk version to v2.2.7
* Add enable_color_backlight_compenstation param
* Add enable_color_auto_exposure_priority param
* Add color_sharpness、color_gamma、color_saturation、color_constrast and color_hue params
* Fix the problem that shutting down LDP using service causes laser to shut down
* Add service switch and parameter setting interface for DecimationFilter, HDRMerge, SequencedFilter, ThresholdFilter, NoiseRemovalFilter, HardwareNoiseRemoval, SpatialAdvancedFilter and TemporalFilter
* Fix the issue that disparity_offset_config cannot be turned off
* Modify the default value of time_domain\_ to global
* Add ob_benchmark_node tool and benchmark examples
* Delete multicamera_sync
* Delete start_multi_sync.sh
* Add decimation_filter related parameters to the launch of Gemini 2 and Gemini 2 L
* Delete dabai_a.launch and dabai_al.launch
* Add dabai_a.launch, dabai_al.launch and update 99-obsensor-libusb.rules
* Add multi-camera sync tool and readme
* Fix disparity_search_offset and offset_index valid range check
* [launch]Add enable_hardware_noise_removal_filter param in launch
* Update orbbec_multicamera.launch.py
* [service] Add set_noise_removal_filter_enable and fix setAllSoftwareFilterEnableCallback
* [service] add set_all_software_filter_enable
* [service] add set_spatial_filter_enable and set_temporal_filter_enable and set_hole_filling_filter_enable
* [service] add set_threshold_filter_enable
* [service] add set_sequence_id_filter_enable
* [service] add set_decimation_filter_enable
* [launch]Set the default values ​​of disparity_search_offset, offset_index0, and offset_index1 to -1
* bump to v2.2.1
* [SDK]Update SDK version to v2.2.1
* [launch]Add disparity_range_mode param
* bump to v2.2.0
* [launch]Add frame_aggregate_mode param in launch
* Fix device lock issue in OBCameraNodeDriver::startDevice
* [launch]Update parameter default values
* Add disparity search offset
* [SDK]Update sdk version to v2.2.0
* Remove unused YAML files
* Update orbbec_multicamera.launch.py and orbbec_camera.launch.py
* add orbbec_multi_synced.launch.py
* Update orbbec_multicamera.launch.py and yaml
* Remove enable_3d_reconstruction_mode param
* [yaml]Update camera yaml parameter configuration file
* Fixed the error when interleave_ae_mode is "hdr"
* Contributors: jj, zhuangzi

2.1.1 (2024-12-24)
------------------
* Restore docs
* Update the frame_id when enable_sync_output_accel_gyro is set to true
* Update docs and delete timed sync
* Fix the imu_info frame_id bug and initialize the quaternion's w value to 1.0
* Modify the print format
* No longer judging depth_laser_status
* Remove unnecessary judgments to avoid reducing the color frame rate
* apply setAlignToStreamProfile API
* [launch]Rename the decimation_filter_scale parameter
* Fix loading interleave_ae parameter error
* Update interleave_ae parameter setting order
* bump to v2.1.1
* [SDK]Update SDK version to v2.1.1
* [yaml]Update common.yaml and gemini330_series.yaml
* Fix does not initialize filters
* [launch]Restore parameter default values
* bump to v2.1.0
* [docs]Update docs
* [launch]Add new launch and yaml description files for v2-main-dev
* [tool]Fix the crash problem caused by multi_save_rgbir_node when saving images
* [sdk]Update SDK version to v2.1.0
* [launch]Remove laser_on_off_mode params
* [launch]Add uvc_backend params
* [tool]Fixed the crash problem when saving images in multi_save_rgbir_node
* Fix multi_save_rgbir_node crash
* [tool]Improve multi_save_rgbir_params.json structure and trigger mode
* Updated interleave_ae description
* [tool]Improve multi_save_rgbir_node
* [interleave_ae]Added parameter settings for interleave frames 0 and 1
* [launch]Modify the multi_camera_synced.launch.py ​​camera startup interval
* [launch]Modify the default value of interleave_frame_enable
* [interleave_ae]Added interleave_ae_mode
* bump to v2.0.9
* [sdk]update sdk version to v2.0.24
* [exposure]Set auto_exposure to take precedence over exposure parameters
* [SDK]Restore the SDK version to v2.0.21
* Fix the problem that the Femto series cannot set sync_mode
* [service]Fix SYNCInterleaveLaserCallback
* bump to v2.0.9
* [sdk]update sdk version to v2.0.24
* update about laser revise
* Add local variable pid of LASER_CONTROL
* [laser control]fix laser settings
* [service]add set_sync_interleaverlaser and set_sync_hosttime
* update multi_save_rgbir_params.json
* add max_size and min_diff judgment
* [config]Update OrbbecSDKConfig
* [time_domain]Fix the improper use of enableGlobalTimestamp
* [service] update multi synced reset timestamp
* Updated to version 2.0.21 with hash 8b662c1b
* Abandon use_hardware_time and use time_domain
* update SDK to version 2.0.21
* Modify the interface getRecommendedFilters to createRecommendedFilters
* bump to v2.0.8
* Refactor depth filter configuration
* d2c delete G355 judgment
* update sdk version to v2.0.20
* Write the tool node parameters into a json file
* Fix incorrect usage of getRecommendedFilters
* Delete orb_device_lock when the program exits、list_devices_node add USB port type and update multi_save
* Deleting Printouts
* add rest timestamp and  syncImmediately service
* Color flow adds rgba and bgra
* Failed to start USB setting for specified single camera
* fix laser settings
* modify rclcpp::Duration to be compatible with both Humble and Foxy
* resolve the compilation failure issue in Foxy
* Change the camera support launch
* add frame loss judgment but service switch color or depth does not work
* improve multi_save_rgbir_node
* bump to v2.0.7
* update SDK to v2.0.18
* replace service enable_laser connector
* fix g300 turning off color and depth causes other streams to behave abnormally
* update SDK to 1017 v2.0.16
* support G335Lg multi camera sync
* bump to v2.0.6
* update SDK to 1016 v2.0.15
* Update libob_frame_processor.so
* fixed jazzy compile error
* update SDK to 1012 v2.0.14
* bump to v2.0.5
* fixed color point cloud
* Add obsdk v2.0.13 and update rossdk v2.0.4
* Add SDK error judgment
* Update SDK sdk to v2.0.11
* fixed depth point cloud Y mirrored
* enable sync time to host
* fix depth point cloud image
* update OrbbecSDKConfig
* update OrbbecSDKConfig
* Modify default param
* chore: fixed compile error
* Modify the json directory
* update arm64 sdk
* update 2.0.3
* Add export json
* Add export_params
* Add some tools
* 1. fix nvblox crash bug
* 1. fix use-intra-process-comms color and depth params
* Add enable_hardware_d2d
* Add gemini_intra_process_demo
* Add zero copy
* Add zero copy
* Alignment Closed Source
* delete multi_save
* fixed missing filesystem header
* bump to v2.0.1
* remove arm32 old sdk
* update arm sdk to 0912
* Add extension path
* multi_save_rgbir
* chore: Add list_devices_node executable to orbbec_camera CMakeLists.txt
* Fix issue with initializing device and enable sync host time
* Update SDK to 0910
* fixed compile error
* add Multigmsl
* fixed filter mem leak
* update opensdk to v2.0.7
* 新增ARM64
* 添加三方库，extensions三方库文件可剪切到build的同级目录下
* 修复colored_point_cloud
* updata libOrbbecSDK.so ,  ob_camera_node and CMakeLists.txt
* updata libOrbbecSDK.so ,  ob_camera_node and CMakeLists.txt
* chore: Update OBCameraNode setupDevices() with property settings
* chore: Add TRY_EXECUTE_BLOCK macro for error handling in OBCameraNodeDriver
* chore: Add TRY_EXECUTE_BLOCK macro for error handling in OBCameraNodeDriver
* chore: Improve error handling and logging in OBCameraNodeDriver
* chore: Add check when init device failed
* chore: Add crash stack trace logging on signal handler
* chore: update SDK
* chore: remove backward-cpp use backward-ros
* chore: add backward-cpp for crash trace
* chore: add count fps
* Merge pull request #59 from orbbec/zerocopy
  chore: add gemini_intra_process_demo
* chore: add gemini_intra_process_demo
* fixed align filter
* chore: add more log
* chore: Clear distortion for color stream when color undistortion is enabled
* feat: drop depth when color not lost and D2C enable
* feat: Enable noise removal filter in camera node setup
* add connectionType in log
* add topic statics
* change: move tool node to tools dir
* fixed g330 depth
* chore: update SDK
* bump to v1.5.11
* add enable heartbeat in launch file
* chore: Refactor OBCameraNode setupTopics() method
* chore: Update connection delay default value to 10 milliseconds
* chore: Add enable_color_undistortion param
* chore: Fix start device time calculation in OBCameraNodeDriver
* chore: Update README && add enable heartbeat param
* chore: Add error handling for failed depth sensor filter retrieval
* chore: extract setupDepthPostProcessFilter()
* remove big try catch block
* Fix baseline sign
* add reboot device inteface
* Fixed wrong ir frame id
* feat: Enable HDR merge filter in OBCameraNode setupDevices()
* bump to v1.5.10
* Update SDK lib to version 1.10.11
* chore: Update multi camera launch example
* chore: Update g330 launch file
* chore: Add time domain params
* Update gemini 330 series launch file
* Add min depth and max depth limit in launch file
* Add min depth and max depth limit
* fixed bug #53
* fixed depth no laser status
* publish static tf by defulat
* add get_ldp_measure_distance
* chore: fixed path error
* add help scripts
* Fix publishing static transform order in OBCameraNode
* Add help scipts
* Fix negative translation value in OBCameraNode
* Fix left and right IR calibration error in OBCameraNode
* chore: Update time sync interface
* chore: Refactor gemini_330_series.launch.py for improved readability and maintainability
* chore: Optimal gemini 330 launch file
* chore: Disable 3D reconstruction mode by default
* Add enable_3d_reconstruction_mode params for 3d reconstruction scenario
* Fixed right ir error
* Fixed bolt tf error
* Update SDK to v1.10.9
* Fix issue #44
* disable point cloud by default
* disable point cloud by default
* chore: add license
* Contributors: Joe Dong, daiyin, datean, datean@123, jj

1.5.8 (2024-06-22)
------------------
* chore: Update Orbbec ROS2 SDK version to 1.5.8
* Update: Fixed wrong depth point cloud
* Update: Fixed the issue of point cloud distortion in femto bolt
* Update: fixed iron compile error
* chore: Add LDP switch
* chore: Fixed ros2 jazzy compile error
* chore: set bolt imu to 200hz
* chore: Remove glog deps
* chore: Add exposure && gain && white balance params
* Add laser energy level param limit
* Add laser energy level param
* Add print default filter params
* Fixed femto bolt config
* Contributors: Joe Dong

1.5.7 (2024-06-05)
------------------
* Update SDK config
* Bump to v1.5.7
* Update SDK to v1.10.8
* fixed save image crash
* fixed save point cloud crash
* fixed pid to lower case
* Bump to v1.5.6
* Fixed bolt align
* Update setupPipelineConfig: put set depth scale out
* Update SDK to v1.10.7
* update sync mode default to standalone
* update PointCloud.rviz
* Contributors: Joe Dong

1.5.5 (2024-05-22)
------------------
* bump to v1.5.5
* fixed crash
* Fixed create diagnstic
* use unique_ptr publish point cloud
* chore: Disable array bounds warning in CMakeLists.txt
* Contributors: Joe Dong

1.5.4 (2024-05-15)
------------------
* update Default sync mode
* add more debug log
* Update filter default params
* bump to v1.5.4
* chore: Refactor OBCameraNode setupPipelineConfig method
* Fixed filter params
* Fixed filter params
* Contributors: Joe Dong

1.5.2 (2024-05-12)
------------------
* Update SDK to v1.10.5
* chore: Add dependencies libgflags-dev, nlohmann-json-dev, and libgoogle-glog-dev
* Bump to v1.5.2
* Update SDK to v1.10.4
* chore: Remove unused color_frame variable in OBCameraNode
* chore: Update camera node to use microsecond timestamps for system time
* Fix depth filter process when frame is nullptr
* Bump to v1.5.1
* Update SDK to v1.10.3
* Update launch file name
* fixed race condition
* chore: update list profile
* chore: use default stream profile
* chore: rename camera launch file name
* chore: rename gemini2R to gemini_generic
* chore: Update launch file for gemini2RF
* add gemini2RF launch
* add retry_on_usb3_detection_failure param
* fixed g2r maybe crash
* fixed g2 no color point cloud
* fixed warning
* add more log
* fixed crash
* fixed depth align
* update README && make package
* update README && make package
* Bump to v1.5.0
* update utils.cpp
* Fix range validation and error handling in setGainCallback and setAutoExposureCallback
* Add extrinsics publishers for depth to accelerometer and depth to gyro
* bump to v1.4.8
* update sdk
* Update SDK to 1.10.0
* fixed maybe crash
* Add docs
* update params
* Update Log
* Add laser on off mode
* update SDK
* fixed baseline params
* update  depth precision setting
* Fixed maybe crash
* Remove device info pid && vid
* Update launch file and device rules
* add pid
* Fixed connect device
* Fixed crash
* Update build type to ReleaseWithDebInfo
* Update json meta data field name
* add laster switch
* Add temperatures diagnostics updater
* Add D2C publisher
* bump to v1.4.7
* Add support for HdrMerge filter setting pass
* add gemini2 RL
* Add processDepthFrameFilter function to OBCameraNode
* Add connection delay parameter to OBCameraNodeDriver
* add align mode param
* Update align mode parameter in OBCameraNode
* add filter params
* remove unused static TF
* update log level
* update SDK
* Adapted G2R
* Update SDK to g2r temp version
* Add setIRLongExposureCallback to OBCameraNode
* update readme
* add femto mega ip addr and port
* Enable depth scale in OBCameraNode
* Enable depth scale in OBCameraNode
* Contributors: Joe Dong

1.4.6 (2024-02-26)
------------------
* bump to v1.4.6
* upgrade SDK to v1.9.4
* add use hardware time option
* fixed save image counter
* fixed not close ofstream
* fixed compile error in unbuntu 2204
* fixed save 10 image
* save depth to raw image
* Contributors: Joe Dong

1.4.4 (2024-01-15)
------------------
* Max Pro increases the IR long exposure enable configuration.
* Update orbbecsdk library to v1.9.3
* merge master
* fixed imu frame id.
* Fix DCL IMU synchronous output abnormal issue。
* Update version to v1.4.4
* Update sdk for fix astra pro ission
* Delete dw2 launch
* Add dw2 launch
* Add code to disable frame synchronization processing.
* Delete redundant software filtering enable configuration from launch
* Update sdk library 1.9.1-dcw2-P01
* DCW2,max pro add depth filter config
* Add max pro launch
* merge master
* Fix the issue of ineffective software filtering in depth filtering configuration。
* Add astra pro2 launch.
* Gemini2 depth work mode add Obstacle Avoidance.
* Modify improperly named variables
* Print log output for gemini2 D2D and depth accuracy information.
* Print log output for gemini2 D2D and depth accuracy information.
* Merge branch 'master' of code.orbbec.com.cn:OrbbecSDK/OrbbecSDK_ROS2
* fixed frame_id name
* Resolving the issue of abnormal simultaneous output of depth point cloud and color point cloud.
* Fixing the command to control the data stream crash issue.
* Fixing the command to control the data stream crash issue.
* fixed distortion model
* fixed bolt TF
* Update version to v1.4.3
* Update gemini2.launch.py for depth filter config
* Remove softfilter config
* 1.更新orbbecSDK 1.9.1;2.gemini2增加深度滤波配置功能;3.增加IMU同时输出topic.
* Resolve Bolt's coordinate abnormality issue.
* Fix distortion parameter bugs, see #15
* Adjust initialization code logic.
* [Max_Pro]Update orbbecSDK library
* Fixing the problem of aligning simultaneous output of colored point cloud and depth point cloud.
* [Max_Pro]Update orbbecSDK for support gemini uw depth 11bit
* [Max_Pro]Adapter dabai max pro
* Contributors: Joe Dong, lixiaobin

1.4.2 (2023-12-06)
------------------
* Update version to v1.4.2
* Update orbbecSDK library to v1.8.3
* Contributors: lixiaobin

1.4.1 (2023-11-23)
------------------
* Update version to v1.4.1
* Update the method of reading intrinsic parameters.
* Update orbbecsdk library to 1.8.2
* Merge branch 'ordered_point_cloud' into 'master'
  add ordered point cloud param
  See merge request OrbbecSDK/OrbbecSDK_ROS2!7
* add ordered point cloud param
* Fix bug for callback color data
* bump to v1.4.0
* Modify the py file in the launch to fix the abnormal behavior of multiple device hot-swapping in Foxy ROS2 version.
* Fix the naming error issue in the color function.
* Modify gemini2.launch.py to resolve the abnormal program behavior of multiple device reconnection in the Foxy version.
* Using a new thread to process Color data.
* Implementing flip function for data stream.
* Contributors: Joe Dong, lixiaobin, 默存

1.3.9 (2023-10-25)
------------------
* bump to v1.3.9
* Fix the problem of abnormal internal parameter acquisition for individual USB devices.
* Adaptation for IR MJPG software decoding processing.
* Gemini2 X L add service to read left ir exposure.
* Fix camera information abnormality issue.
* Contributors: lixiaobin

1.3.8 (2023-10-17)
------------------
* Update OrbbecSDK lib
* bump to v1.3.8
* add lock in selectDeviceBySerialNumber func
* fixed start multi device
* add check start IMU
* check rgb buffer avoid crash
* fixed stop cause crash
* fixed camera info width && height
* Update femto bolt,gemini2,gemini2L launch resulation configruation.
* fixed crash
* Fixed the issue of resource null pointer during device disconnection.
* Fixed the issue with the calling of the device PID acquisition interface.
* Add dcw2 pid to 99-obsensor-libusb.rules
* Resolving the Femto Bolt D2C point cloud issue.
* Contributors: Joe Dong, lixiaobin

1.3.7 (2023-10-11)
------------------
* Add the enable switch function for network device enumeration.
* Update version to 1.3.7
* Add instructions for depth work mode in the Readme.
* merge master
* Update OrbbecSDK v1.8.1 library
* Update OrbbecSDK to v1.8.1
* Update the data stream configuration of Gemini2 series devices.
* Modify multi device sync config parameter.
* Adapter Femto bolt device.
* Contributors: lixiaobin

1.3.6 (2023-10-07)
------------------
* fixed enable frame sync param
* add gemini2 VL launch file
* bump to v1.3.6
* add help scripts
* add enable frame sync
* fixed camera info
* Contributors: Joe Dong

1.3.5 (2023-09-15 16:54)
------------------------
* fixed save point cloud
* Contributors: Joe Dong

1.3.4 (2023-09-15 15:11)
------------------------
* bump to v1.3.4
* fixed get device by uid
* change sem to pthread lock
* Contributors: Joe Dong

1.3.2 (2023-09-13)
------------------
* fixed set ir mirror
* fixed set ir mirror
* add dcw2 launch
* bump to v1.3.2
* update SDK to sdk v.1.7.4
* add astra mini s pro pid
* fix mising close sem
* update SDK to v1.7.3
* bump to v1.3.1
* print more clean log
* fixed save left && right IR image
* add imu enable parmeter
* fixed TF error
* fixed set and get ir property
* add license header
* remove jpeg end zero
* fixed use nv jpeg decoder
* add jetson hardware decoder
* remove sigal handler
* remove point cloud filter
* changed default color points topic
* remove unused parmeter
* remove unused parmeter
* Merge branch 'master' of code.orbbec.com.cn:OrbbecSDK/OrbbecSDK_ROS2
* fixed tf calc error
* Add Dabai max dabai_max.launch.py
* fixed rk mpp decoder rgb channel error
* fixed crash
* remove xml launch file
* update SDK to v1.7.2
* update SDK to v1.7.2
* fixed multi device param name
* fixed point cloud inv
* remove unused tf
* fixed camera info frame id
* add gemini2 XL launch file
* update SDK to v1.7.1
* add cmake log
* fixed file name typo
* Merge pull request #6 from orbbec/gst_decoder
  add gstreamer  decode MPEG
* remove unused code
* add python launch file
* add python launch file
* first work gst decoder
* disable Use Gstreamer hardware decoder by default
* fixed source and sink name
* add gst decoder
* fixed memory leak
* bump to v1.3.0
* Merge branch 'hw_decoder' into 'master'
  add mpp hardware decode mjpeg
  See merge request OrbbecSDK/OrbbecSDK_ROS2!6
* add mpp hardware decode mjpeg
* refactory cmake
* Add imu parameter README
* Add missing params
* Contributors: Joe Dong, bajingsi, 默存

1.2.9 (2023-08-23)
------------------
* add list device script
* add py launch file
* add py launch file
* use rclcpp componet
* add list device script
* add py launch file
* add py launch file
* use rclcpp componet
* Merge pull request #5 from jian-dong/master
  Release v1.2.9
* remove print device type
* update SDK to v1.6.3
* update sdk to v1.6.2
* bump to v1.2.8
* add astra2 launch file
* update readme
* disable enableMultiDeviceSync for OpenNI device
* bump to v1.2.7
* update SDK
* fixed tf error
* fixed tf
* fixed log level
* fixed HUB hot plug bug
* bump to v1.2.6
* uncomment IMU code
* gemini2 L launch file
* add help scripts
* update launch file
* update cmake
* update cmake
* update SDK to v1.6.1
* add read device usb port
* add uyuv to rgb888
* fixed camera link frame id
* fixed TF
* fixed TF
* fix ir image step
* remove lfs
* remove lfs
* remove lfs
* remove lfs
* slim SDK
* comment IMU
* update readme
* comment start&& stop IMU
* comment start&& stop IMU
* add more launch file
* bump to v1.2.5
* fixed log not clear
* fixed gemini2 cannot convert mjpeg
* Contributors: Joe Dong

1.2.4 (2023-04-28)
------------------
* fixed gemini e enum device
* disable gemini_e color point cloud by default
* add sync_signal_trigger_out param
* bump v1.2.4
* Merge branch 'feature/imu' into 'master'
  添加 IMU topic
  * 添加了读取IMU数据流，并发布相关topic，目前只支持gemini2
  See merge request OrbbecSDK/OrbbecSDK_ROS2!5
* add IMU topic
* fix drop bad mjpeg frame
* add soft filter max diff
* fix point cloud dist
* fix read orbbec config
* fixed depth image value
* add sync mode
* add soft filter&& AE switch
* add list depth work mode
* bump to v1.2.3
* fixed gemini2 get stuck
* update launch file
* update SDK to v1.5.7
* bump to v1.2.2
* bump to v1.2.2
* add d2c viewer topic
* add enable hardware d2d
* fix get camera info
* update SDK to v1.5.6
* add switch IR camera
* fixed broken SDK soft link
* rewrite install rules script
* fix astra mini pro launch file
* Contributors: Joe Dong, 默存

1.2.1 (2023-02-20)
------------------
* add save point cloud and image
* fixed typo
* clean .make_deb
* ignore undeclare paramer exception
* add d2c viewer
* rename camer_node_factory to camera_node_driver
* fixed device count
* refactory startDevice
* add get ldp status
* change set_fan_mode to set_fan_work_mode
* update README
* remove wrong  static_tf_broadcaster\_ init
* fixed all launch file error
* bump to v1.2.1
* fixed multi camera namespace error
* update log
* fixed get camera params
* fixed get camera params
* fixed enum openNI device
* fixed code format
* fixed crash
* fixed catch error
* add more launch file
* update SDK to v1.5.5
* bump to v1.2.0
* update usb rules
* fixed check null ptr
* Merge branch 'refactory' into 'master'
  Big refactoring code
  See merge request OrbbecSDK/OrbbecSDK_ROS2!4
* Big refactoring code
  * Removal of unnecessary files
  * Optimized multi-camera launch
  * Explicitly list the parameters in the launch file
* clean shm
* add check connection timer
* refactory get stream
* remove unused code
* fixed multi camera start
* replace SDK to v1.4.3
* replace SDK to v1.4.1
* Merge branch 'develop' into 'master'
  Small fix
  See merge request OrbbecSDK/OrbbecSDK_ROS2!3
* Small fix
* add install usb rules
* Contributors: Joe Dong, 默存

1.0.4 (2022-07-07)
------------------
* remove unuse file
* bump to v1.0.4
* add make deb scripts
* update orbbec SDK install way
* update SDK
* Contributors: Joe Dong

1.0.3 (2022-07-04 17:05)
------------------------
* bump to v1.0.3
* Contributors: Joe Dong

1.0.2 (2022-07-04 16:52)
------------------------
* roll back sdk
* update sdk
* update sdk
* remove multiple thread publish point cloud
* use single thread publish point cloud
* fix ir auto exposure
* remove openMP
* bump to v1.0.2
* remove libuvc debug flag
* add stream format as parmeters
* add rgb point cloud topic
* fix crash
* add auto white balance ctrl
* remove debug msg
* add OBFormatFromString
* fix point cloud filter
* fix typo
* Merge branch 'release/1.0.1' into 'master'
  add more config
  See merge request OrbbecSDK/orbbecros2sdk!2
* [fix] only YUYV and I420 support color point cloud
* [fix] print enum name
* [add][scripts] recv point cloud
* [add][glog] update readme
* [fix][cmake] find image_publisher
* add NOTICE
* add multi camera rivz config
* fix multi camera
* use pid adapter color format
* add toggle sesror service
* Merge branch 'dev' into 'master'
  Fix point cloud direction
  See merge request OrbbecSDK/orbbecros2sdk!1
* Fix point cloud direction
* Contributors: Joe Dong, 默存

1.0.0 (2022-06-10)
------------------
* change 'get_api_version' to 'get_sdk_version'
* update readme
* remove orbbec dir
* add dependencies
* use bash echo latch messsage
* fix ob_sdk from system
* fix get ob sdk version
* fix publish rgb point cloud
* catch publish point cloud exception
* add rbgFormatConvertRGB888
* change a better name
* add get verion api
* change topic name
* clean code
* fixed warning
* fixed stop device
* fixed hotplug
* remove debug msg
* fix use frame timestamp
* add get device info srv
* print time cost
* set response status
* catch error when ctrl camera
* add help scripts
* fix point cloud color
* fix tf frame error
* init commit
* Contributors: Joe Dong
