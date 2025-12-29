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
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>

#include "libobsensor/ObSensor.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "orbbec_camera_msgs/msg/extrinsics.hpp"
#include "magic_enum/magic_enum.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/opencv.hpp>
#include <openssl/evp.h>
#include <sstream>
#include <iomanip>
#include <arpa/inet.h>

namespace orbbec_camera {
inline void LogFatal(const char* file, int line, const std::string& message) {
  std::cerr << "Check failed at " << file << ":" << line << ": " << message << std::endl;
  std::abort();
}
}  // namespace orbbec_camera

#define TRY_EXECUTE_BLOCK(block)                                                                  \
  try {                                                                                           \
    block;                                                                                        \
  } catch (const ob::Error& e) {                                                                  \
    std::string error_msg = e.getMessage() ? e.getMessage() : "Unknown OB error";                 \
    if (error_msg.find("Device is deactivated") != std::string::npos ||                           \
        error_msg.find("disconnected") != std::string::npos ||                                    \
        error_msg.find("Send control transfer failed") != std::string::npos) {                    \
      RCLCPP_WARN(logger_,                                                                        \
                  "Device communication error in %s at line %d: %s - Device may be disconnected", \
                  __FUNCTION__, __LINE__, error_msg.c_str());                                     \
    } else {                                                                                      \
      RCLCPP_ERROR(logger_, "Error in %s at line %d: %s", __FUNCTION__, __LINE__,                 \
                   error_msg.c_str());                                                            \
    }                                                                                             \
  } catch (const std::exception& e) {                                                             \
    RCLCPP_ERROR(logger_, "Exception in %s at line %d: %s", __FUNCTION__, __LINE__, e.what());    \
  } catch (...) {                                                                                 \
    RCLCPP_ERROR(logger_, "Unknown exception in %s at line %d", __FUNCTION__, __LINE__);          \
  }

#define TRY_TO_SET_PROPERTY(func, property, value)                                             \
  try {                                                                                        \
    device_->func(property, value);                                                            \
  } catch (const ob::Error& e) {                                                               \
    RCLCPP_ERROR_STREAM(logger_, "Failed to set " << property << " to " << value << " in "     \
                                                  << __FUNCTION__ << " at line " << __LINE__   \
                                                  << ": " << e.getMessage());                  \
  } catch (const std::exception& e) {                                                          \
    RCLCPP_ERROR_STREAM(logger_, "Failed to set " << property << " to " << value << " in "     \
                                                  << __FUNCTION__ << " at line " << __LINE__   \
                                                  << ": " << e.what());                        \
  } catch (...) {                                                                              \
    RCLCPP_ERROR_STREAM(logger_, "Failed to set " << property << " to " << value << " in "     \
                                                  << __FUNCTION__ << " at line " << __LINE__); \
  }

// Macros for checking conditions and comparing values
#define CHECK(condition) \
  (!(condition) ? LogFatal(__FILE__, __LINE__, "Check failed: " #condition) : (void)0)

template <typename T1, typename T2>
void CheckOp(const char* expr, const char* file, int line, T1 val1, T2 val2, bool result) {
  if (!result) {
    std::ostringstream os;
    os << "Check failed: " << expr << " (" << val1 << " vs. " << val2 << ")";
    orbbec_camera::LogFatal(file, line, os.str());
  }
}

#define CHECK_OP(opname, op, val1, val2) \
  CheckOp(#val1 " " #op " " #val2, __FILE__, __LINE__, val1, val2, (val1)op(val2))

#define CHECK_EQ(val1, val2) CHECK_OP(_EQ, ==, val1, val2)
#define CHECK_NE(val1, val2) CHECK_OP(_NE, !=, val1, val2)
#define CHECK_LE(val1, val2) CHECK_OP(_LE, <=, val1, val2)
#define CHECK_LT(val1, val2) CHECK_OP(_LT, <, val1, val2)
#define CHECK_GE(val1, val2) CHECK_OP(_GE, >=, val1, val2)
#define CHECK_GT(val1, val2) CHECK_OP(_GT, >, val1, val2)

// Overload for raw pointers
template <typename T>
T* CheckNotNull(T* ptr, const char* file, int line) {
  if (ptr == nullptr) {
    std::ostringstream os;
    os << "Null pointer passed to CheckNotNull at " << file << ":" << line;
    orbbec_camera::LogFatal(file, line, os.str());
  }
  return ptr;
}

// Template for smart pointers like std::shared_ptr, std::unique_ptr
template <typename T>
T& CheckNotNull(T& ptr, const char* file, int line) {
  if (ptr == nullptr) {
    std::ostringstream os;
    os << "Null pointer passed to CheckNotNull at " << file << ":" << line;
    orbbec_camera::LogFatal(file, line, os.str());
  }
  return ptr;
}

#if defined(CHECK_NOTNULL)
#undef CHECK_NOTNULL
#endif
#define CHECK_NOTNULL(val) CheckNotNull(val, __FILE__, __LINE__)

namespace orbbec_camera {
sensor_msgs::msg::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                                 OBCameraDistortion distortion, int width);

void saveRGBPointsToPly(const std::shared_ptr<ob::Frame>& frame, const std::string& fileName);

void saveRGBPointCloudMsgToPly(const sensor_msgs::msg::PointCloud2::UniquePtr& msg,
                               const std::string& fileName);

void saveDepthPointsToPly(const sensor_msgs::msg::PointCloud2::UniquePtr& msg,
                          const std::string& fileName);

void savePointsToPly(const std::shared_ptr<ob::Frame>& frame, const std::string& fileName);

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]);

std::ostream& operator<<(std::ostream& os, const OBCameraParam& rhs);

orbbec_camera_msgs::msg::Extrinsics obExtrinsicsToMsg(const OBD2CTransform& extrinsics,
                                                      const std::string& frame_id);

rclcpp::Time fromMsToROSTime(uint64_t ms);

rclcpp::Time fromUsToROSTime(uint64_t us);

std::string getObSDKVersion();

OBFormat OBFormatFromString(const std::string& format);

OBLiDARScanRate OBScanRateFromInt(const int rate);

std::string OBFormatToString(const OBFormat& format);

std::ostream& operator<<(std::ostream& os, const OBFormat& rhs);

std::string ObDeviceTypeToString(const OBDeviceType& type);

rmw_qos_profile_t getRMWQosProfileFromString(const std::string& str_qos);

bool isOpenNIDevice(int pid);

OB_DEPTH_PRECISION_LEVEL depthPrecisionLevelFromString(
    const std::string& depth_precision_level_str);

float depthPrecisionFromString(const std::string& depth_precision_level_str);

OBMultiDeviceSyncMode OBSyncModeFromString(const std::string& mode);

OB_SAMPLE_RATE sampleRateFromString(std::string& sample_rate);

std::string sampleRateToString(const OB_SAMPLE_RATE& sample_rate);

std::ostream& operator<<(std::ostream& os, const OB_SAMPLE_RATE& rhs);

OB_GYRO_FULL_SCALE_RANGE fullGyroScaleRangeFromString(std::string& full_scale_range);

std::string fullGyroScaleRangeToString(const OB_GYRO_FULL_SCALE_RANGE& full_scale_range);

std::ostream& operator<<(std::ostream& os, const OB_GYRO_FULL_SCALE_RANGE& rhs);

OBAccelFullScaleRange fullAccelScaleRangeFromString(std::string& full_scale_range);

std::string fullAccelScaleRangeToString(const OBAccelFullScaleRange& full_scale_range);

std::ostream& operator<<(std::ostream& os, const OBAccelFullScaleRange& rhs);

std::string parseUsbPort(const std::string& line);

bool isValidJPEG(const std::shared_ptr<ob::ColorFrame>& frame);

std::string metaDataTypeToString(const OBFrameMetadataType& meta_data_type);

std::ostream& operator<<(std::ostream& os, const OBFrameMetadataType& rhs);

OBHoleFillingMode holeFillingModeFromString(const std::string& hole_filling_mode);

bool isGemini2R(int pid);

OBStreamType obStreamTypeFromString(const std::string& stream_type);

struct UndistortedImageResult {
  cv::Mat image;
  OBCameraIntrinsic new_intrinsic;
};

UndistortedImageResult undistortImage(const cv::Mat& image, const OBCameraIntrinsic& intrinsic,
                                      const OBCameraDistortion& distortion);

std::string getDistortionModels(OBCameraDistortion distortion);

std::string calcMD5(const std::string& data);
double getScanAngleIncrement(OBLiDARScanRate fps);

double deg2rad(double deg);

double rad2deg(double rad);
}  // namespace orbbec_camera
