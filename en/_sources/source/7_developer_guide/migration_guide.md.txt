# Migrating from main to Open-Source v2-main

## Introduction

Initially, Orbbec provided a **closed-source SDK — Orbbec SDK v1**, which formed the foundation of the [OrbbecSDK ROS2 Wrapper main branch](https://github.com/orbbec/OrbbecSDK_ROS2/tree/main). Although the ROS wrapper layer itself was open-source, it relied on a closed-source underlying SDK. This architecture imposed limitations on flexibility and hindered community-driven improvements.

As developers increasingly demanded transparency, maintainability, and broader device support, Orbbec released a brand-new open-source v2-main **— Orbbec SDK_v2** ([GitHub link](https://github.com/orbbec/OrbbecSDK/tree/v2-main)). Based on this SDK, the [open-source v2-main branch of OrbbecSDK ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) is now fully open source, offering greater extensibility and alignment with Orbbec’s future product roadmap.

This document introduces the motivations and benefits of migrating ROS packages from the main branch (based on SDK v1) to the v2-main branch (based on Orbbec SDK_v2). It highlights the key differences in **launch files, parameters, topics, and services**, and provides a migration guide to help developers smoothly transition.

**Note:** In the following content, **main** refers to the closed-source branch, while **v2-main** refers to the open-source v2-main branch.

## Advantages of Migrating from main to v2-main

In October 2024, Orbbec released a major update: **OrbbecSDK ROS2 Wrapper v2**, which is entirely based on the open-source Orbbec SDK_v2. Compared to the legacy main branch (SDK v1.x), the v2-main branch (Orbbec SDK_v2.x) provides greater flexibility and scalability, while offering comprehensive support for all Orbbec USB products that comply with the UVC standard.The migration from main --> v2-main brings the following key advantages:

### **Comprehensive Device Support**

The v2-main branch supports all UVC-compliant Orbbec USB cameras and will be the primary platform for supporting all newly released devices.

### **Transparency and Extensibility**

Orbbec SDK_v2 is fully open-source, allowing developers to directly access the underlying implementation for easier debugging, optimization, and secondary development. By contrast, SDK v1 was closed-source, introducing a “black-box” constraint.

### Advantages in Maintenance and Updates

The v2-main branch offers full-featured support, including new feature development, performance optimization, and bug fixes. The main branch has entered a maintenance-only mode, where only critical bugs may receive limited updates, and no new features are being developed.

### **Community and Ecosystem Support**

With an open-source SDK, developers can directly submit issues and pull requests on GitHub or Gitee, contributing to feature evolution. This not only accelerates problem resolution but also fosters a more open and active Orbbec ecosystem.

## Comparison Between main and v2-main Branches

### **Launch File Differences**

1. In v2-main, new low-power launch file has been added for the Gemini 330 series:
   - `gemini_330_series_low_cpu.launch.py`
2. v2-main introduces support for **Gemini 435Le**, **Gemini 345**, and **Gemini 345Lg** cameras.
3. Since **OrbbecSDK_v2 only supports UVC devices**, the range of camera models supported in v2-main is slightly narrower than in main. Detailed information is provided in the table below.

| **camera**                         | **main**                             | **v2-main**                                        |
| ---------------------------------- | ------------------------------------ | -------------------------------------------------- |
| Gemini 435Le                       | Not supported                        | gemini435_le.launch.py                             |
| Gemini 345                         | Not supported                        | gemini345.launch.py                                |
| Gemini 345Lg                       | Not supported                        | gemini345_lg.launch.py                             |
| Gemini 330 series                  | gemini_330_series.launch.py          | gemini_330_series.launch.py                        |
| Gemini 330 low cpu                 | -                                    | gemini_330_series_low_cpu.launch.py                |
| Gemini 210                         | gemini210.launch.py                  | gemini210.launch.py                                |
| Gemini 2                           | gemini2.launch.py                    | gemini2.launch.py                                  |
| Gemini 2L                          | gemini2L.launch.py                   | gemini2.launch.py                                  |
| Gemini 2XL                         | gemini2XL.launch.py                  | -                                                  |
| Femto Bolt                         | femto_bolt.launch.py                 | femto_bolt.launch.py                               |
| Femto Mega                         | femto_mega.launch.py                 | femto_mega.launch.py                               |
| Femto                              | femto.launch.py                      | femto.launch.py                                    |
| Astra 2                            | astra2.launch.py                     | astra2.launch.py                                   |
| Astra                              | astra.launch.py                      | astra.launch.py                                    |
| Astra Mini Pro / S Pro             | astra_mini_pro.launch.py ...         | astra.launch.py                                    |
| Multi-Camera (Synchronized)        | multi_camera_synced.launch.py        | multi_camera_synced.launch.py                      |
| Multi-Camera (Generic / Universal) | multi_camera.launch.py               | multi_camera.launch.pyorbbec_multicamera.launch.py |
| Single-Camera Generic Launch       | ob_camera.launch.py                  | orbbec_camera.launch.py                            |
| OpenNI devices (Dabai、Deeya)      | Corresponding model independent file | Not supported                                      |

### **Parameter Differences**

**New Parameters in v2-main (not available in main)**

| **Parameter Name**                     | **main** | **v2-main** | **Description**                                   |
| -------------------------------------- | -------- | ----------- | ------------------------------------------------- |
| upgrade_firmware                       | -        | Added       | Firmware upgrade path                             |
| preset_firmware_path                   | -        | Added       | Preset firmware file path                         |
| load_config_json_file_path             | -        | Added       | Load JSON configuration                           |
| export_config_json_file_path           | -        | Added       | Export JSON configuration                         |
| uvc_backend                            | -        | Added       | libuvc / v4l2 backend selection                   |
| enable_color_auto_exposure_priority    | -        | Added       | AE priority control                               |
| color_flip                             | -        | Added       | Color image vertical flip                         |
| color_mirror                           | -        | Added       | Color image horizontal mirror                     |
| depth_flip                             | -        | Added       | Depth image vertical flip                         |
| depth_mirror                           | -        | Added       | Depth image horizontal mirror                     |
| left_ir_flip                           | -        | Added       | Left IR vertical flip                             |
| left_ir_mirror                         | -        | Added       | Left IR horizontal mirror                         |
| right_ir_flip                          | -        | Added       | Right IR vertical flip                            |
| right_ir_mirror                        | -        | Added       | Right IR horizontal mirror                        |
| enable_left_ir_sequence_id_filter      | -        | Added       | Left IR sequence ID filter                        |
| enable_right_ir_sequence_id_filter     | -        | Added       | Right IR sequence ID filter                       |
| enable_accel_data_correction           | -        | Added       | Accelerometer data correction                     |
| enable_gyro_data_correction            | -        | Added       | Gyroscope data correction                         |
| enumerate_net_device                   | -        | Added       | Automatic enumeration of network devices          |
| net_device_ip                          | -        | Added       | Network device IP address                         |
| net_device_port                        | -        | Added       | Network device port                               |
| exposure_range_mode                    | -        | Added       | Exposure range mode: default / ultimate / regular |
| disparity_to_depth_mode                | -        | Added       | Hardware disparity-to-depth conversion            |
| ldp_power_level                        | -        | Added       | LDP power level                                   |
| time_sync_period                       | -        | Added       | Time synchronization period                       |
| gmsl_trigger_fps                       | -        | Added       | GMSL trigger frame rate                           |
| enable_gmsl_trigger                    | -        | Added       | GMSL trigger enable                               |
| disparity_range_mode                   | -        | Added       | Disparity range mode                              |
| disparity_search_offset                | -        | Added       | Disparity search offset                           |
| disparity_offset_config                | -        | Added       | Disparity offset configuration                    |
| offset_index0                          | -        | Added       | Disparity offset index 0                          |
| offset_index1                          | -        | Added       | Disparity offset index 1                          |
| interleave_ae_mode                     | -        | Added       | AE interleave mode                                |
| interleave_frame_enable                | -        | Added       | Interleaved frame enable                          |
| interleave_skip_enable                 | -        | Added       | Skip IR frame enable                              |
| interleave_skip_index                  | -        | Added       | Skip IR frame index                               |
| hdr_index1_laser_control               | -        | Added       | HDR laser control parameters                      |
| hdr_index1_depth_exposure              | -        | Added       | HDR depth exposure                                |
| hdr_index1_depth_gain                  | -        | Added       | HDR depth gain                                    |
| hdr_index1_ir_brightness               | -        | Added       | HDR IR brightness                                 |
| hdr_index1_ir_ae_max_exposure          | -        | Added       | HDR IR max AE                                     |
| hdr_index0_laser_control               | -        | Added       | HDR laser control parameters                      |
| hdr_index0_depth_exposure              | -        | Added       | HDR depth exposure                                |
| hdr_index0_depth_gain                  | -        | Added       | HDR depth gain                                    |
| hdr_index0_ir_brightness               | -        | Added       | HDR IR brightness                                 |
| hdr_index0_ir_ae_max_exposure          | -        | Added       | HDR IR max AE                                     |
| laser_index1_laser_control             | -        | Added       | Laser interleave control                          |
| laser_index1_depth_exposure            | -        | Added       | Laser depth exposure                              |
| laser_index1_depth_gain                | -        | Added       | Laser depth gain                                  |
| laser_index1_ir_brightness             | -        | Added       | Laser IR brightness                               |
| laser_index1_ir_ae_max_exposure        | -        | Added       | Laser IR max AE                                   |
| laser_index0_laser_control             | -        | Added       | Laser interleave control                          |
| laser_index0_depth_exposure            | -        | Added       | Laser depth exposure                              |
| laser_index0_depth_gain                | -        | Added       | Laser depth gain                                  |
| laser_index0_ir_brightness             | -        | Added       | Laser IR brightness                               |
| laser_index0_ir_ae_max_exposure        | -        | Added       | Laser IR max AE                                   |
| software_trigger_enabled               | -        | Added       | Software trigger enable                           |
| enable_ptp_config                      | -        | Added       | PTP configuration (Gemini 335Le)                  |
| align_target_stream                    | -        | Added       | Target stream for alignment                       |
| spatial_fast_filter_radius             | -        | Added       | Fast spatial filter radius                        |
| spatial_moderate_filter_diff_threshold | -        | Added       | Moderate spatial filter difference threshold      |
| spatial_moderate_filter_magnitude      | -        | Added       | Moderate spatial filter magnitude                 |
| spatial_moderate_filter_radius         | -        | Added       | Moderate spatial filter radius                    |
| color.image_raw.enable_pub_plugins     | -        | Added       | Color image transport plugins                     |
| depth.image_raw.enable_pub_plugins     | -        | Added       | Depth image transport plugins                     |
| left_ir.image_raw.enable_pub_plugins   | -        | Added       | Left IR transport plugins                         |
| right_ir.image_raw.enable_pub_plugins  | -        | Added       | Right IR transport plugins                        |
| force_ip_enable                        | -        | Added       | Force IP feature                                  |
| force_ip_mac                           | -        | Added       | Force IP MAC address                              |
| force_ip_dhcp                          | -        | Added       | DHCP auto assignment                              |
| force_ip_address                       | -        | Added       | Force IP static address                           |
| force_ip_subnet_mask                   | -        | Added       | Force IP subnet mask                              |
| force_ip_gateway                       | -        | Added       | Force IP gateway                                  |

**Removed Parameters (main only, removed in v2-main)**

| **Parameter**                        | **Description**                                              |
| ------------------------------------ | ------------------------------------------------------------ |
| enable_3d_reconstruction_mode        | 3D reconstruction mode deprecated                            |
| enable_hardware_reset                | Hardware reset interface deprecated                          |
| enable_hardware_noise_removal_filter | Hardware noise removal filter deprecated                     |
| laser_on_off_mode                    | Old laser on/off interface, replaced by interleave / laser_index |
| enable_3d_reconstruction_mode        | 3D reconstruction mode repeated; no longer used in v2-main   |
| device_preset                        | Some logic migrated to new firmware / interleave parameters  |
| enable_trigger_out                   | Old software trigger interface replaced by software_trigger_enabled |
| retry_on_usb3_detection_failure      | Optional; logic adjusted or removed in v2-main               |
| enable_color_undistortion            | Old interface logic integrated elsewhere; still exists in v2-main but usage may be adjusted |

### **Topic Differences**

**v2-main** adds the following topics based on main:

| **Topic**             | **main** | **v2-main** | **Description**                                              |
| --------------------- | -------- | ----------- | ------------------------------------------------------------ |
| /camera/device_status | -        | Added       | Publishes device status (frame rate delay, device connection status, etc.) |

### **Service Differences**

**v2-main** adds the following services based on main:

| **Service**               | **main** | **v2-main** | **Description**                      |
| ------------------------- | -------- | ----------- | ------------------------------------ |
| get_ptp_config            | -        | Added       | Get PTP configuration                |
| set_ptp_config            | -        | Added       | Set PTP configuration                |
| get_streams_enable        | -        | Added       | Get the enable status of each stream |
| set_streams_enable        | -        | Added       | Set the enable status of each stream |
| get_user_calib_params     | -        | Added       | Get user calibration parameters      |
| set_user_calib_params     | -        | Added       | Set user calibration parameters      |
| read_customer_data        | -        | Added       | Read user-stored custom data         |
| write_customer_data       | -        | Added       | Write user-stored custom data        |
| send_software_trigger     | -        | Added       | Send software trigger                |
| set_color_ae_roi          | -        | Added       | Set AE ROI for color image           |
| set_depth_ae_roi          | -        | Added       | Set AE ROI for depth image           |
| set_color_flip            | -        | Added       | Set color image vertical flip        |
| set_depth_flip            | -        | Added       | Set depth image vertical flip        |
| set_color_rotation        | -        | Added       | Set color image rotation             |
| set_depth_rotation        | -        | Added       | Set depth image rotation             |
| set_left_ir_ae_roi        | -        | Added       | Set AE ROI for left IR               |
| set_right_ir_ae_roi       | -        | Added       | Set AE ROI for right IR              |
| set_left_ir_flip          | -        | Added       | Left IR vertical flip                |
| set_right_ir_flip         | -        | Added       | Right IR vertical flip               |
| set_left_ir_rotation      | -        | Added       | Left IR rotation                     |
| set_right_ir_rotation     | -        | Added       | Right IR rotation                    |
| set_reset_timestamp       | -        | Added       | Reset timestamp                      |
| set_sync_hosttime         | -        | Added       | Synchronize host time                |
| set_sync_interleaverlaser | -        | Added       | Interleaver laser synchronization    |
| set_filter                | -        | Added       | Set depth/point cloud filters        |
