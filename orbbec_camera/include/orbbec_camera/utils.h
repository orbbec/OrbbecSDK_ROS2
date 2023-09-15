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
#include <ostream>
#include <glog/logging.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>

#include "libobsensor/ObSensor.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "orbbec_camera_msgs/msg/extrinsics.hpp"
#include "magic_enum/magic_enum.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace orbbec_camera {
sensor_msgs::msg::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                                 OBCameraDistortion distortion, int width);

void saveRGBPointsToPly(const std::shared_ptr<ob::Frame>& frame, const std::string& fileName);

void saveRGBPointCloudMsgToPly(const sensor_msgs::msg::PointCloud2& msg, const std::string& fileName);

void saveDepthPointsToPly(const sensor_msgs::msg::PointCloud2& msg, const std::string& fileName);

void savePointsToPly(const std::shared_ptr<ob::Frame>& frame, const std::string& fileName);

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]);

std::ostream& operator<<(std::ostream& os, const OBCameraParam& rhs);

orbbec_camera_msgs::msg::Extrinsics obExtrinsicsToMsg(const OBD2CTransform& extrinsics,
                                                      const std::string& frame_id);

rclcpp::Time frameTimeStampToROSTime(uint64_t ms);

std::string getObSDKVersion();

OBFormat OBFormatFromString(const std::string& format);

std::string ObDeviceTypeToString(const OBDeviceType& type);

rmw_qos_profile_t getRMWQosProfileFromString(const std::string& str_qos);

bool isOpenNIDevice(int pid);

OB_DEPTH_PRECISION_LEVEL depthPrecisionLevelFromString(
    const std::string& depth_precision_level_str);

OBMultiDeviceSyncMode OBSyncModeFromString(const std::string& mode);

OB_SAMPLE_RATE sampleRateFromString(std::string& sample_rate);

std::string sampleRateToString(const OB_SAMPLE_RATE& sample_rate);

OB_GYRO_FULL_SCALE_RANGE fullGyroScaleRangeFromString(std::string& full_scale_range);

std::string fullGyroScaleRangeToString(const OB_GYRO_FULL_SCALE_RANGE& full_scale_range);

OBAccelFullScaleRange fullAccelScaleRangeFromString(std::string& full_scale_range);

std::string fullAccelScaleRangeToString(const OBAccelFullScaleRange& full_scale_range);

std::string parseUsbPort(const std::string& line);

bool isValidJPEG(const std::shared_ptr<ob::ColorFrame>& frame);

}  // namespace orbbec_camera
