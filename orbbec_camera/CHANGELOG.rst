^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package orbbec_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.14 (2025-09-30)
-------------------
* Bump version to 1.5.14
* Update OrbbecSDK (v1.10.9 → v1.10.27)
* Add new params (color/IR settings, rotations, filters, etc.)
* Add reboot device interface and exception handling
* Improve device selection, lock handling, error handling & logging
* Fix device enumeration, depth filter, IMU info, etc.
* Update launch files (Gemini, Dabai, Astra, Mega, Deeya, etc.)
* Update udev rules, dependencies (opengl, tf2_eigen, libdw-dev)
* Refactor camera setup (resolutions, info, topics, post-process filters)
* Add new features: HDR merge, noise removal, heartbeat, point cloud options
* Misc bug fixes, cleanup, and stability improvements
* Contributors: Joe Dong, daiwei, datean, jjiszjj, obyalian, xiexun, zhuangzi

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
