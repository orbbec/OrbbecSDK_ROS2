/*******************************************************************************
* Copyright (c) 2023 Orbbec 3D Technology, Inc
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#pragma once

#include <string>
#include <cstdlib>

#define THREAD_NUM 4

#define OB_ROS_MAJOR_VERSION 1
#define OB_ROS_MINOR_VERSION 4
#define OB_ROS_PATCH_VERSION 3

#ifndef STRINGIFY
#define STRINGIFY(arg) #arg
#endif
#ifndef VAR_ARG_STRING
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
#endif

#define STRINGIFY(arg) #arg
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
/* Return version in "X.Y.Z" format */
#define OB_ROS_VERSION_STR \
  (VAR_ARG_STRING(OB_ROS_MAJOR_VERSION.OB_ROS_MINOR_VERSION.OB_ROS_PATCH_VERSION))

namespace orbbec_camera {

const bool ALIGN_DEPTH = false;
const bool POINTCLOUD = false;
const bool ALLOW_NO_TEXTURE_POINTS = false;
const bool SYNC_FRAMES = false;
const bool ORDERED_POINTCLOUD = false;

const bool PUBLISH_TF = true;
const double TF_PUBLISH_RATE = 0;     // Static transform
const double DIAGNOSTICS_PERIOD = 0;  // Static transform

const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;
const int IMAGE_FPS = 30;

const std::string IMAGE_QOS = "SYSTEM_DEFAULT";
const std::string DEFAULT_QOS = "DEFAULT";
const std::string HID_QOS = "HID_DEFAULT";
const std::string EXTRINSICS_QOS = "EXTRINSICS_DEFAULT";

const double IMU_FPS = 0;

const bool ENABLE_DEPTH = true;
const bool ENABLE_INFRA1 = true;
const bool ENABLE_INFRA2 = true;
const bool ENABLE_COLOR = true;
const bool ENABLE_FISHEYE = true;
const bool ENABLE_IMU = true;
const bool HOLD_BACK_IMU_FOR_FRAMES = false;
const bool PUBLISH_ODOM_TF = true;

const std::string DEFAULT_BASE_FRAME_ID = "camera_link";
const std::string DEFAULT_ODOM_FRAME_ID = "odom_frame";
const std::string DEFAULT_DEPTH_FRAME_ID = "camera_depth_frame";
const std::string DEFAULT_INFRA1_FRAME_ID = "camera_infra1_frame";
const std::string DEFAULT_INFRA2_FRAME_ID = "camera_infra2_frame";
const std::string DEFAULT_COLOR_FRAME_ID = "camera_color_frame";
const std::string DEFAULT_FISHEYE_FRAME_ID = "camera_fisheye_frame";
const std::string DEFAULT_IMU_FRAME_ID = "camera_imu_frame";

const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID = "camera_depth_optical_frame";
const std::string DEFAULT_INFRA1_OPTICAL_FRAME_ID = "camera_infra1_optical_frame";
const std::string DEFAULT_INFRA2_OPTICAL_FRAME_ID = "camera_infra2_optical_frame";
const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID = "camera_color_optical_frame";
const std::string DEFAULT_FISHEYE_OPTICAL_FRAME_ID = "camera_fisheye_optical_frame";
const std::string DEFAULT_ACCEL_OPTICAL_FRAME_ID = "camera_accel_optical_frame";
const std::string DEFAULT_GYRO_OPTICAL_FRAME_ID = "camera_gyro_optical_frame";
const std::string DEFAULT_IMU_OPTICAL_FRAME_ID = "camera_imu_optical_frame";

const std::string DEFAULT_ALIGNED_DEPTH_TO_COLOR_FRAME_ID = "camera_aligned_depth_to_color_frame";
const std::string DEFAULT_ALIGNED_DEPTH_TO_INFRA1_FRAME_ID = "camera_aligned_depth_to_infra1_frame";
const std::string DEFAULT_ALIGNED_DEPTH_TO_INFRA2_FRAME_ID = "camera_aligned_depth_to_infra2_frame";
const std::string DEFAULT_ALIGNED_DEPTH_TO_FISHEYE_FRAME_ID =
    "camera_aligned_depth_to_fisheye_frame";

const std::string DEFAULT_UNITE_IMU_METHOD = "";
const std::string DEFAULT_FILTERS = "";
const std::string DEFAULT_TOPIC_ODOM_IN = "";
const std::string DEFAULT_D2C_MODE = "sw";  // sw = software mode, hw=hardware mode, none,
const float ROS_DEPTH_SCALE = 0.001;

const int32_t FEMTO_OW_PID = 0x0638;
const int32_t FEMTO_BOLT_PID = 0x066b;
const int32_t FEMTO_LIVE_PID = 0x0668;
const int32_t FEMTO_PID = 0x0635;
const int32_t ASTRA_PLUS_PID = 0x0636;
const int32_t ASTRA_PLUS_S_PID = 0x0637;
const int32_t OPENNI_START_PID = 0x0601;
const int32_t OPENNI_END_PID = 0x06FF;
const int32_t ASTRA_MINI_PID = 0x0404;
const int32_t ASTRA_MINI_S_PID = 0x0407;
const int GEMINI2_PID = 0x0670;
const std::string ORB_DEFAULT_LOCK_NAME = "orbbec_device_lock";

}  // namespace orbbec_camera
