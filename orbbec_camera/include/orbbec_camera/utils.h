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
#include <memory>
#include <sstream>
#include <iostream>
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
#include "gaemi_base_libs/logger.hpp"


inline void LogFatal(const char* file, int line, const std::string& message,
                     std::shared_ptr<gaemi::base_libs::Logger> logger = nullptr) {
  if (logger) {
    logger->get_spd_logger()->critical("Fatal error at {}:{}: {}", file, line, message);
  } else {
    std::cerr << "Fatal error at " << file << ":" << line << ": " << message << std::endl;
  }
  std::abort();
}

// Macro for executing blocks with exception handling using spdlog
#define TRY_EXECUTE_BLOCK(block, logger)                                                      \
  try {                                                                                        \
    block;                                                                                     \
  } catch (const ob::Error& e) {                                                              \
    if (logger) {                                                                             \
      logger->get_spd_logger()->error("Error in {} at line {}: {}", __FUNCTION__, __LINE__, e.getMessage()); \
    }                                                                                          \
  } catch (const std::exception& e) {                                                         \
    if (logger) {                                                                             \
      logger->get_spd_logger()->error("Exception in {} at line {}: {}", __FUNCTION__, __LINE__, e.what()); \
    }                                                                                          \
  } catch (...) {                                                                             \
    if (logger) {                                                                             \
      logger->get_spd_logger()->error("Unknown exception in {} at line {}", __FUNCTION__, __LINE__); \
    }                                                                                          \
  }

// Macro for setting properties with exception handling using spdlog
#define TRY_TO_SET_PROPERTY(func, property, value, device, logger)                           \
  try {                                                                                       \
    device->func(property, value);                                                           \
  } catch (const ob::Error& e) {                                                             \
    if (logger) {                                                                            \
      logger->get_spd_logger()->error("Failed to set {} to {} in {} at line {}: {}",       \
                                       property, value, __FUNCTION__, __LINE__, e.getMessage()); \
    }                                                                                         \
  } catch (const std::exception& e) {                                                        \
    if (logger) {                                                                            \
      logger->get_spd_logger()->error("Failed to set {} to {} in {} at line {}: {}",       \
                                       property, value, __FUNCTION__, __LINE__, e.what());  \
    }                                                                                         \
  } catch (...) {                                                                            \
    if (logger) {                                                                            \
      logger->get_spd_logger()->error("Failed to set {} to {} in {} at line {}",           \
                                       property, value, __FUNCTION__, __LINE__);            \
    }                                                                                         \
  }

// Check condition macro with optional logger support
#define CHECK_1(condition) \
  (!(condition) ? LogFatal(__FILE__, __LINE__, "Check failed: " #condition, nullptr) : (void)0)
#define CHECK_2(condition, logger) \
  (!(condition) ? LogFatal(__FILE__, __LINE__, "Check failed: " #condition, logger) : (void)0)

#define GET_CHECK_MACRO(_1, _2, NAME, ...) NAME
#define CHECK(...) GET_CHECK_MACRO(__VA_ARGS__, CHECK_2, CHECK_1)(__VA_ARGS__)

// Template for checking operations with logger support
template <typename T1, typename T2>
void CheckOp(const char* expr, const char* file, int line, T1 val1, T2 val2, bool result,
             std::shared_ptr<gaemi::base_libs::Logger> logger = nullptr) {
  if (!result) {
    std::ostringstream os;
    os << "Check failed: " << expr << " (" << val1 << " vs. " << val2 << ")";
    LogFatal(file, line, os.str(), logger);
  }
}

// Comparison check macros with optional logger support
#define CHECK_OP_2(opname, op, val1, val2) \
  CheckOp(#val1 " " #op " " #val2, __FILE__, __LINE__, val1, val2, (val1)op(val2), nullptr)
#define CHECK_OP_3(opname, op, val1, val2, logger) \
  CheckOp(#val1 " " #op " " #val2, __FILE__, __LINE__, val1, val2, (val1)op(val2), logger)

#define GET_CHECK_OP_MACRO(_1, _2, _3, _4, NAME, ...) NAME
#define CHECK_OP(...) GET_CHECK_OP_MACRO(__VA_ARGS__, CHECK_OP_3, CHECK_OP_2)(__VA_ARGS__)

#define CHECK_EQ_2(val1, val2) CHECK_OP_2(_EQ, ==, val1, val2)
#define CHECK_EQ_3(val1, val2, logger) CHECK_OP_3(_EQ, ==, val1, val2, logger)
#define GET_CHECK_EQ_MACRO(_1, _2, _3, NAME, ...) NAME
#define CHECK_EQ(...) GET_CHECK_EQ_MACRO(__VA_ARGS__, CHECK_EQ_3, CHECK_EQ_2)(__VA_ARGS__)

#define CHECK_NE_2(val1, val2) CHECK_OP_2(_NE, !=, val1, val2)
#define CHECK_NE_3(val1, val2, logger) CHECK_OP_3(_NE, !=, val1, val2, logger)
#define GET_CHECK_NE_MACRO(_1, _2, _3, NAME, ...) NAME
#define CHECK_NE(...) GET_CHECK_NE_MACRO(__VA_ARGS__, CHECK_NE_3, CHECK_NE_2)(__VA_ARGS__)

#define CHECK_LE_2(val1, val2) CHECK_OP_2(_LE, <=, val1, val2)
#define CHECK_LE_3(val1, val2, logger) CHECK_OP_3(_LE, <=, val1, val2, logger)
#define GET_CHECK_LE_MACRO(_1, _2, _3, NAME, ...) NAME
#define CHECK_LE(...) GET_CHECK_LE_MACRO(__VA_ARGS__, CHECK_LE_3, CHECK_LE_2)(__VA_ARGS__)

#define CHECK_LT_2(val1, val2) CHECK_OP_2(_LT, <, val1, val2)
#define CHECK_LT_3(val1, val2, logger) CHECK_OP_3(_LT, <, val1, val2, logger)
#define GET_CHECK_LT_MACRO(_1, _2, _3, NAME, ...) NAME
#define CHECK_LT(...) GET_CHECK_LT_MACRO(__VA_ARGS__, CHECK_LT_3, CHECK_LT_2)(__VA_ARGS__)

#define CHECK_GE_2(val1, val2) CHECK_OP_2(_GE, >=, val1, val2)
#define CHECK_GE_3(val1, val2, logger) CHECK_OP_3(_GE, >=, val1, val2, logger)
#define GET_CHECK_GE_MACRO(_1, _2, _3, NAME, ...) NAME
#define CHECK_GE(...) GET_CHECK_GE_MACRO(__VA_ARGS__, CHECK_GE_3, CHECK_GE_2)(__VA_ARGS__)

#define CHECK_GT_2(val1, val2) CHECK_OP_2(_GT, >, val1, val2)
#define CHECK_GT_3(val1, val2, logger) CHECK_OP_3(_GT, >, val1, val2, logger)
#define GET_CHECK_GT_MACRO(_1, _2, _3, NAME, ...) NAME
#define CHECK_GT(...) GET_CHECK_GT_MACRO(__VA_ARGS__, CHECK_GT_3, CHECK_GT_2)(__VA_ARGS__)

// Null pointer check for raw pointers with logger support
template <typename T>
T* CheckNotNull(T* ptr, const char* file, int line,
                std::shared_ptr<gaemi::base_libs::Logger> logger = nullptr) {
  if (ptr == nullptr) {
    std::ostringstream os;
    os << "Null pointer passed to CheckNotNull at " << file << ":" << line;
    LogFatal(file, line, os.str(), logger);
  }
  return ptr;
}

// Null pointer check for smart pointers with logger support
template <typename T>
T& CheckNotNull(T& ptr, const char* file, int line,
                std::shared_ptr<gaemi::base_libs::Logger> logger = nullptr) {
  if (ptr == nullptr) {
    std::ostringstream os;
    os << "Null pointer passed to CheckNotNull at " << file << ":" << line;
    LogFatal(file, line, os.str(), logger);
  }
  return ptr;
}

// Null pointer check macro with optional logger support
#if defined(CHECK_NOTNULL)
#undef CHECK_NOTNULL
#endif

// Overloaded versions to support both 1 and 2 parameters
#define CHECK_NOTNULL_1(val) CheckNotNull(val, __FILE__, __LINE__, nullptr)
#define CHECK_NOTNULL_2(val, logger) CheckNotNull(val, __FILE__, __LINE__, logger)

// Macro dispatcher based on number of arguments
#define GET_CHECK_NOTNULL_MACRO(_1, _2, NAME, ...) NAME
#define CHECK_NOTNULL(...) GET_CHECK_NOTNULL_MACRO(__VA_ARGS__, CHECK_NOTNULL_2, CHECK_NOTNULL_1)(__VA_ARGS__)


namespace orbbec_camera {

// Logger-based fatal error function

// Utility function declarations
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

cv::Mat undistortImage(const cv::Mat& image, const OBCameraIntrinsic& intrinsic,
                       const OBCameraDistortion& distortion);

}  // namespace orbbec_camera
