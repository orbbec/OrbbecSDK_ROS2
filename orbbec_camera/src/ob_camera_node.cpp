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

#include "orbbec_camera/ob_camera_node.h"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "orbbec_camera/utils.h"
#include <filesystem>
#include <fstream>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "libobsensor/hpp/Utils.hpp"

#if defined(USE_RK_HW_DECODER)
#include "orbbec_camera/rk_mpp_decoder.h"
#elif defined(USE_NV_HW_DECODER)
#include "orbbec_camera/jetson_nv_decoder.h"
#endif

namespace orbbec_camera {
using namespace std::chrono_literals;

OBCameraNode::OBCameraNode(rclcpp::Node *node, std::shared_ptr<ob::Device> device,
                           std::shared_ptr<Parameters> parameters, bool use_intra_process)
    : node_(node),
      device_(std::move(device)),
      parameters_(std::move(parameters)),
      logger_(node->get_logger()),
      use_intra_process_(use_intra_process) {
  RCLCPP_INFO_STREAM(logger_,
                     "OBCameraNode: use_intra_process: " << (use_intra_process ? "ON" : "OFF"));
  is_running_.store(true);
  stream_name_[COLOR] = "color";
  stream_name_[DEPTH] = "depth";
  stream_name_[INFRA0] = "ir";
  stream_name_[INFRA1] = "left_ir";
  stream_name_[INFRA2] = "right_ir";
  stream_name_[ACCEL] = "accel";
  stream_name_[GYRO] = "gyro";

  default_config_ = DefaultConfig();

  compression_params_.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params_.push_back(0);
  compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY);
  compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
  setupDefaultImageFormat();
  setupTopics();

#if defined(USE_NV_HW_DECODER)
  jpeg_decoder_ = std::make_unique<JetsonNvJPEGDecoder>(width_[COLOR], height_[COLOR]);
#endif
  if (default_config_.enable_d2c_viewer_) {
    auto rgb_qos = getRMWQosProfileFromString(image_qos_[COLOR]);
    auto depth_qos = getRMWQosProfileFromString(image_qos_[DEPTH]);
    d2c_viewer_ = std::make_unique<D2CViewer>(node_, rgb_qos, depth_qos);
  }
  if (enable_stream_[COLOR]) {
    rgb_buffer_ = new uint8_t[width_[COLOR] * height_[COLOR] * 3];
  }
  if (enable_colored_point_cloud_ && enable_stream_[DEPTH] && enable_stream_[COLOR]) {
    rgb_point_cloud_buffer_size_ = width_[COLOR] * height_[COLOR] * sizeof(OBColorPoint);
    rgb_point_cloud_buffer_ = new uint8_t[rgb_point_cloud_buffer_size_];
    xy_table_data_size_ = width_[DEPTH] * height_[DEPTH] * 2;
    xy_table_data_ = new float[xy_table_data_size_];
  }
  is_camera_node_initialized_ = true;
}

// OBCameraNode::OBCameraNode(rclcpp::Node *node, std::shared_ptr<ob::Device> device,
//                            GeminiConfig gemini_config, bool use_intra_process)
//     : node_(node),
//       device_(std::move(device)),
//       gemini_config_(std::move(gemini_config)),
//       logger_(node->get_logger()),
//       use_intra_process_(use_intra_process) {
//   RCLCPP_INFO_STREAM(logger_,
//                      "OBCameraNode: use_intra_process: " << (use_intra_process ? "ON" : "OFF"));
//   is_running_.store(true);
//   stream_name_[COLOR] = "color";
//   stream_name_[DEPTH] = "depth";
//   stream_name_[INFRA0] = "ir";
//   stream_name_[INFRA1] = "left_ir";
//   stream_name_[INFRA2] = "right_ir";
//   stream_name_[ACCEL] = "accel";
//   stream_name_[GYRO] = "gyro";

//   default_config_ = CameraConfig();

//   compression_params_.push_back(cv::IMWRITE_PNG_COMPRESSION);
//   compression_params_.push_back(0);
//   compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY);
//   compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
//   setupDefaultImageFormat();
//   setupTopics();

// #if defined(USE_NV_HW_DECODER)
//   jpeg_decoder_ = std::make_unique<JetsonNvJPEGDecoder>(width_[COLOR], height_[COLOR]);
// #endif
//   if (default_config_.enable_d2c_viewer_) {
//     auto rgb_qos = getRMWQosProfileFromString(image_qos_[COLOR]);
//     auto depth_qos = getRMWQosProfileFromString(image_qos_[DEPTH]);
//     d2c_viewer_ = std::make_unique<D2CViewer>(node_, rgb_qos, depth_qos);
//   }
//   if (enable_stream_[COLOR]) {
//     rgb_buffer_ = new uint8_t[width_[COLOR] * height_[COLOR] * 3];
//   }
//   if (enable_colored_point_cloud_ && enable_stream_[DEPTH] && enable_stream_[COLOR]) {
//     rgb_point_cloud_buffer_size_ = width_[COLOR] * height_[COLOR] * sizeof(OBColorPoint);
//     rgb_point_cloud_buffer_ = new uint8_t[rgb_point_cloud_buffer_size_];
//     xy_table_data_size_ = width_[DEPTH] * height_[DEPTH] * 2;
//     xy_table_data_ = new float[xy_table_data_size_];
//   }
//   is_camera_node_initialized_ = true;
// }

template <class T>
void OBCameraNode::setAndGetNodeParameter(
    T &param, const std::string &param_name, const T &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor) {
  try {
    param = parameters_
                ->setParam(param_name, rclcpp::ParameterValue(default_value),
                           std::function<void(const rclcpp::Parameter &)>(), parameter_descriptor)
                .get<T>();
  } catch (const rclcpp::ParameterTypeException &ex) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to set parameter: " << param_name << ". " << ex.what());
    throw;
  }
}

OBCameraNode::~OBCameraNode() noexcept { clean(); }

void OBCameraNode::rebootDevice() {
  RCLCPP_WARN_STREAM(logger_, "Reboot device");
  clean();
  if (device_) {
    device_->reboot();
  }
  RCLCPP_WARN_STREAM(logger_, "Reboot device DONE");
}

void OBCameraNode::clean() noexcept {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  RCLCPP_WARN_STREAM(logger_, "Do destroy ~OBCameraNode");
  is_running_.store(false);
  RCLCPP_WARN_STREAM(logger_, "Stop tf thread");
  if (tf_thread_ && tf_thread_->joinable()) {
    tf_thread_->join();
  }
  RCLCPP_WARN_STREAM(logger_, "Stop color frame thread");
  if (colorFrameThread_ && colorFrameThread_->joinable()) {
    color_frame_queue_cv_.notify_all();
    colorFrameThread_->join();
  }

  RCLCPP_WARN_STREAM(logger_, "stop streams");
  stopStreams();
  stopIMU();
  if (rgb_buffer_) {
    delete[] rgb_buffer_;
    rgb_buffer_ = nullptr;
  }
  if (rgb_point_cloud_buffer_) {
    delete[] rgb_point_cloud_buffer_;
    rgb_point_cloud_buffer_ = nullptr;
  }
  if (xy_table_data_) {
    delete[] xy_table_data_;
    xy_table_data_ = nullptr;
  }
  RCLCPP_WARN_STREAM(logger_, "Destroy ~OBCameraNode DONE");
}
// check
void OBCameraNode::setupDevices() {
  auto sensor_list = device_->getSensorList();
  for (size_t i = 0; i < sensor_list->count(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profiles = sensor->getStreamProfileList();
    for (size_t j = 0; j < profiles->count(); j++) {
      auto profile = profiles->getProfile(j);
      stream_index_pair sip{profile->type(), 0};
      if (sensors_.find(sip) != sensors_.end()) {
        continue;
      }
      sensors_[sip] = sensor;
    }
  }

  for (const auto &[stream_index, enable] : enable_stream_) {
    if (enable && sensors_.find(stream_index) == sensors_.end()) {
      RCLCPP_INFO_STREAM(logger_,
                         magic_enum::enum_name(stream_index.first)
                             << "sensor isn't supported by current device! -- Skipping...");
      enable_stream_[stream_index] = false;
    }
  }
  auto info = device_->getDeviceInfo();
  if (default_config_.retry_on_usb3_detection_failure_ &&
      device_->isPropertySupported(OB_PROP_DEVICE_USB3_REPEAT_IDENTIFY_BOOL,
                                   OB_PERMISSION_READ_WRITE)) {
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_DEVICE_USB3_REPEAT_IDENTIFY_BOOL,
                        default_config_.retry_on_usb3_detection_failure_);
  }
  if (device_->isPropertySupported(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL,
                                   OB_PERMISSION_READ_WRITE)) {
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL,
                        default_config_.enable_noise_removal_filter_);
  }
  if (device_->isPropertySupported(OB_PROP_HEARTBEAT_BOOL, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting heartbeat to " << (default_config_.enable_heartbeat_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_HEARTBEAT_BOOL, default_config_.enable_heartbeat_);
  }
  if (!default_config_.industry_mode_.empty() &&
      device_->isPropertySupported(OB_PROP_DEPTH_INDUSTRY_MODE_INT, OB_PERMISSION_READ_WRITE)) {
    if (default_config_.industry_mode_ == "default") {
      OBDepthIndustryMode mode = OB_INDUSTRY_DEFAULT;
      device_->setIntProperty(OB_PROP_DEPTH_INDUSTRY_MODE_INT, (int32_t)mode);
    } else if (default_config_.industry_mode_ == "mode1") {
      OBDepthIndustryMode mode = OB_INDUSTRY_MODE1;
      device_->setIntProperty(OB_PROP_DEPTH_INDUSTRY_MODE_INT, (int32_t)mode);
    } else if (default_config_.industry_mode_ == "mode2") {
      OBDepthIndustryMode mode = OB_INDUSTRY_MODE2;
      device_->setIntProperty(OB_PROP_DEPTH_INDUSTRY_MODE_INT, (int32_t)mode);
    } else if (default_config_.industry_mode_ == "mode3") {
      OBDepthIndustryMode mode = OB_INDUSTRY_MODE3;
      device_->setIntProperty(OB_PROP_DEPTH_INDUSTRY_MODE_INT, (int32_t)mode);
    } else if (default_config_.industry_mode_ == "mode4") {
      OBDepthIndustryMode mode = OB_INDUSTRY_MODE4;
      device_->setIntProperty(OB_PROP_DEPTH_INDUSTRY_MODE_INT, (int32_t)mode);
    } else if (default_config_.industry_mode_ == "mode5") {
      OBDepthIndustryMode mode = OB_INDUSTRY_MODE5;
      device_->setIntProperty(OB_PROP_DEPTH_INDUSTRY_MODE_INT, (int32_t)mode);
    }
    RCLCPP_INFO_STREAM(logger_, "Setting industry mode to "
                                    << device_->getIntProperty(OB_PROP_DEPTH_INDUSTRY_MODE_INT));
  }
  if (default_config_.max_depth_limit_ > 0 &&
      device_->isPropertySupported(OB_PROP_MAX_DEPTH_INT, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting max depth limit to " << default_config_.max_depth_limit_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_MAX_DEPTH_INT, default_config_.max_depth_limit_);
  }
  if (default_config_.min_depth_limit_ > 0 &&
      device_->isPropertySupported(OB_PROP_MIN_DEPTH_INT, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting min depth limit to " << default_config_.min_depth_limit_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_MIN_DEPTH_INT, default_config_.min_depth_limit_);
  }
  if (default_config_.laser_energy_level_ != -1 &&
      device_->isPropertySupported(OB_PROP_LASER_ENERGY_LEVEL_INT, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting laser energy level to " << default_config_.laser_energy_level_);
    auto range = device_->getIntPropertyRange(OB_PROP_LASER_ENERGY_LEVEL_INT);
    if (default_config_.laser_energy_level_ < range.min || default_config_.laser_energy_level_ > range.max) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Laser energy level is out of range " << range.min << " - " << range.max);
    } else {
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_LASER_ENERGY_LEVEL_INT, default_config_.laser_energy_level_);
      auto new_laser_energy_level = device_->getIntProperty(OB_PROP_LASER_ENERGY_LEVEL_INT);
      RCLCPP_INFO_STREAM(logger_,
                         "Laser energy level set to " << new_laser_energy_level << " (new value)");
    }
  }
  if (depth_registration_) {
    RCLCPP_INFO_STREAM(logger_, "Create align filter");
    align_filter_ = std::make_unique<ob::Align>(align_target_stream_);
  }
  if (device_->isPropertySupported(OB_PROP_DISPARITY_TO_DEPTH_BOOL, OB_PERMISSION_READ_WRITE)) {
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_DISPARITY_TO_DEPTH_BOOL, default_config_.enable_hardware_d2d_);
    bool is_hardware_d2d = device_->getBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL);
    std::string d2d_mode = is_hardware_d2d ? "HW D2D" : "SW D2D";
    RCLCPP_INFO_STREAM(logger_, "Depth process is " << d2d_mode);
  }
  if (device_->isPropertySupported(OB_PROP_LDP_BOOL, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting LDP to " << (default_config_.enable_ldp_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_LDP_BOOL, default_config_.enable_ldp_);
  }
  if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting laser control to " << default_config_.enable_laser_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_LASER_CONTROL_INT, default_config_.enable_laser_);
  }
  if (device_->isPropertySupported(OB_PROP_LASER_ON_OFF_MODE_INT, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting laser on off mode to " << default_config_.laser_on_off_mode_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_LASER_ON_OFF_MODE_INT, default_config_.laser_on_off_mode_);
  }
  if (!default_config_.device_preset_.empty()) {
    try {
      RCLCPP_INFO_STREAM(logger_, "Available presets:");
      auto preset_list = device_->getAvailablePresetList();
      for (uint32_t i = 0; i < preset_list->count(); i++) {
        RCLCPP_INFO_STREAM(logger_, "Preset " << i << ": " << preset_list->getName(i));
      }
      RCLCPP_INFO_STREAM(logger_, "Load device preset: " << default_config_.device_preset_);
      TRY_EXECUTE_BLOCK(device_->loadPreset(default_config_.device_preset_.c_str()));
      RCLCPP_INFO_STREAM(logger_, "Device preset " << device_->getCurrentPresetName() << " loaded");
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to load device preset: " << e.getMessage());
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to load device preset: " << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to load device preset");
    }
  }
  if (!default_config_.depth_work_mode_.empty()) {
    RCLCPP_INFO_STREAM(logger_, "Set depth work mode: " << default_config_.depth_work_mode_);
    TRY_EXECUTE_BLOCK(device_->switchDepthWorkMode(default_config_.depth_work_mode_.c_str()));
  }

  if (device_->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_READ_WRITE) &&
      !default_config_.depth_precision_str_.empty()) {
    auto default_precision_level = device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
    if (default_precision_level != depth_precision_) {
      device_->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, depth_precision_);
      RCLCPP_INFO_STREAM(logger_, "set depth precision to " << default_config_.depth_precision_str_);
    }
  } else if (device_->isPropertySupported(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT,
                                          OB_PERMISSION_READ_WRITE) &&
             !default_config_.depth_precision_str_.empty()) {
    auto depth_unit_flexible_adjustment = depthPrecisionFromString(default_config_.depth_precision_str_);
    auto range = device_->getFloatPropertyRange(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT);
    RCLCPP_INFO_STREAM(logger_,
                       "Depth unit flexible adjustment range: " << range.min << " - " << range.max);
    if (depth_unit_flexible_adjustment < range.min || depth_unit_flexible_adjustment > range.max) {
      RCLCPP_ERROR_STREAM(
          logger_, "depth unit flexible adjustment value is out of range, please check the value");
    } else {
      RCLCPP_INFO_STREAM(logger_, "set depth unit to " << depth_unit_flexible_adjustment << "mm");
      TRY_TO_SET_PROPERTY(setFloatProperty, OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT,
                          depth_unit_flexible_adjustment);
    }
  }

  if (!depth_filter_config_.empty() && enable_depth_filter_) {
    RCLCPP_INFO_STREAM(logger_, "Load depth filter config: " << depth_filter_config_);
    TRY_EXECUTE_BLOCK(device_->loadDepthFilterConfig(depth_filter_config_.c_str()));
  } else {
    if (device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE)) {
      RCLCPP_INFO_STREAM(logger_,
                         "Setting depth soft filter to " << (enable_soft_filter_ ? "ON" : "OFF"));
      TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_DEPTH_SOFT_FILTER_BOOL, enable_soft_filter_);
    }
  }

  if (device_->isPropertySupported(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color auto white balance to "
                                    << (default_config_.enable_color_auto_white_balance_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL,
                        default_config_.enable_color_auto_white_balance_);
  }
  if (default_config_.color_exposure_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
    auto range = device_->getIntPropertyRange(OB_PROP_COLOR_EXPOSURE_INT);
    if (default_config_.color_exposure_ < range.min || default_config_.color_exposure_ > range.max) {
      RCLCPP_ERROR(logger_, "color exposure value is out of range[%d,%d], please check the value",
                   range.min, range.max);
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting color exposure to " << default_config_.color_exposure_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_EXPOSURE_INT, default_config_.color_exposure_);
    }
  }
  if (default_config_.color_gain_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_GAIN_INT, OB_PERMISSION_WRITE)) {
    auto range = device_->getIntPropertyRange(OB_PROP_COLOR_GAIN_INT);
    if (default_config_.color_gain_ < range.min || default_config_.color_gain_ > range.max) {
      RCLCPP_ERROR(logger_, "color gain value is out of range[%d,%d], please check the value",
                   range.min, range.max);
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting color gain to " << default_config_.color_gain_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_GAIN_INT, default_config_.color_gain_);
    }
  }
  if (default_config_.color_white_balance_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_WHITE_BALANCE_INT, OB_PERMISSION_WRITE)) {
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, false);
    auto range = device_->getIntPropertyRange(OB_PROP_COLOR_WHITE_BALANCE_INT);
    if (default_config_.color_white_balance_ < range.min || default_config_.color_white_balance_ > range.max) {
      RCLCPP_ERROR(logger_,
                   "color white balance value is out of range[%d,%d], please check the value",
                   range.min, range.max);
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting color white balance to " << default_config_.color_white_balance_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_WHITE_BALANCE_INT, default_config_.color_white_balance_);
    }
  }
  if (device_->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(
        logger_, "Setting color auto exposure to " << (default_config_.enable_color_auto_exposure_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL,
                        default_config_.enable_color_auto_exposure_);
  }
  if (default_config_.color_ae_max_exposure_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_AE_MAX_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color AE max exposure to " << default_config_.color_ae_max_exposure_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_AE_MAX_EXPOSURE_INT, default_config_.color_ae_max_exposure_);
  }
  if (default_config_.color_brightness_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color brightness to " << default_config_.color_brightness_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_BRIGHTNESS_INT, default_config_.color_brightness_);
  }
  if (default_config_.color_sharpness_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_SHARPNESS_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color sharpness to " << default_config_.color_sharpness_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_SHARPNESS_INT, default_config_.color_sharpness_);
  }
  if (default_config_.color_saturation_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_SATURATION_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color saturation to " << default_config_.color_saturation_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_SATURATION_INT, default_config_.color_saturation_);
  }
  if (default_config_.color_contrast_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_CONTRAST_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color contrast to " << default_config_.color_contrast_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_CONTRAST_INT, default_config_.color_contrast_);
  }
  if (default_config_.color_gamma_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_GAMMA_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color gamma to " << default_config_.color_gamma_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_GAMMA_INT, default_config_.color_gamma_);
  }
  if (default_config_.color_hue_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_HUE_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color hue to " << default_config_.color_hue_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_HUE_INT, default_config_.color_hue_);
  }

  // ir ae max
  if (default_config_.ir_ae_max_exposure_ != -1 &&
      device_->isPropertySupported(OB_PROP_IR_AE_MAX_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting IR AE max exposure to " << default_config_.ir_ae_max_exposure_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_IR_AE_MAX_EXPOSURE_INT, default_config_.ir_ae_max_exposure_);
  }
  // ir brightness
  if (default_config_.ir_brightness_ != -1 &&
      device_->isPropertySupported(OB_PROP_IR_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting IR brightness to " << default_config_.ir_brightness_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_IR_BRIGHTNESS_INT, default_config_.ir_brightness_);
  }
  if (default_config_.ir_exposure_ != -1 &&
      device_->isPropertySupported(OB_PROP_IR_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
    auto range = device_->getIntPropertyRange(OB_PROP_IR_EXPOSURE_INT);
    if (default_config_.ir_exposure_ < range.min || default_config_.ir_exposure_ > range.max) {
      RCLCPP_ERROR(logger_, "ir exposure value is out of range[%d,%d], please check the value",
                   range.min, range.max);
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting IR exposure to " << default_config_.ir_exposure_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_IR_EXPOSURE_INT, default_config_.ir_exposure_);
    }
  }
  if (default_config_.ir_gain_ != -1 && device_->isPropertySupported(OB_PROP_IR_GAIN_INT, OB_PERMISSION_WRITE)) {
    auto range = device_->getIntPropertyRange(OB_PROP_IR_GAIN_INT);
    if (default_config_.ir_gain_ < range.min || default_config_.ir_gain_ > range.max) {
      RCLCPP_ERROR(logger_, "ir gain value is out of range[%d,%d], please check the value",
                   range.min, range.max);
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting IR gain to " << default_config_.ir_gain_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_IR_GAIN_INT, default_config_.ir_gain_);
    }
  }
  // ir auto exposure
  if (device_->isPropertySupported(OB_PROP_IR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_,
                       "Setting IR auto exposure to " << (default_config_.enable_ir_auto_exposure_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_IR_AUTO_EXPOSURE_BOOL, default_config_.enable_ir_auto_exposure_);
  }
  if (device_->isPropertySupported(OB_PROP_IR_LONG_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_,
                       "Setting IR long exposure to " << (default_config_.enable_ir_long_exposure_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_IR_LONG_EXPOSURE_BOOL, default_config_.enable_ir_long_exposure_);
  }

  if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE)) {
    auto default_soft_filter_max_diff = device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
    RCLCPP_INFO_STREAM(logger_, "default_soft_filter_max_diff: " << default_soft_filter_max_diff);
    if (default_config_.soft_filter_max_diff_ != -1 && default_soft_filter_max_diff != default_config_.soft_filter_max_diff_) {
      device_->setIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT, default_config_.soft_filter_max_diff_);
      auto new_soft_filter_max_diff = device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
      RCLCPP_INFO_STREAM(logger_, "after set soft_filter_max_diff: " << new_soft_filter_max_diff);
    }
  }

  if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, OB_PERMISSION_WRITE)) {
    auto default_soft_filter_speckle_size =
        device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
    RCLCPP_INFO_STREAM(logger_,
                       "default_soft_filter_speckle_size: " << default_soft_filter_speckle_size);
    if (default_config_.soft_filter_speckle_size_ != -1 &&
        default_soft_filter_speckle_size != default_config_.soft_filter_speckle_size_) {
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT,
                          default_config_.soft_filter_speckle_size_);
      auto new_soft_filter_speckle_size =
          device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
      RCLCPP_INFO_STREAM(logger_,
                         "after set soft_filter_speckle_size: " << new_soft_filter_speckle_size);
    }
  }
}
// need more check
void OBCameraNode::setupDepthPostProcessFilter() {
  auto depth_sensor = device_->getSensor(OB_SENSOR_DEPTH);
  // set depth sensor to filter
  auto filter_list = depth_sensor->getRecommendedFilters();
  if (!filter_list) {
    RCLCPP_ERROR(logger_, "Failed to get depth sensor filter list");
    return;
  }
  for (size_t i = 0; i < filter_list->count(); i++) {
    auto filter = filter_list->getFilter(i);
    std::map<std::string, bool> filter_params = {
        {"DecimationFilter", default_config_.enable_decimation_filter_},
        {"HDRMerge", default_config_.enable_hdr_merge_},
        {"SequencedFilter", default_config_.enable_sequence_id_filter_},
        {"ThresholdFilter", default_config_.enable_threshold_filter_},
        {"NoiseRemovalFilter", default_config_.enable_noise_removal_filter_},
        {"SpatialAdvancedFilter", default_config_.enable_spatial_filter_},
        {"TemporalFilter", default_config_.enable_temporal_filter_},
        {"HoleFillingFilter", default_config_.enable_hole_filling_filter_},

    };
    std::string filter_name = filter->type();
    RCLCPP_INFO_STREAM(logger_, "Setting " << filter_name << "......");
    if (filter_params.find(filter_name) != filter_params.end()) {
      std::string value = filter_params[filter_name] ? "true" : "false";
      RCLCPP_INFO_STREAM(logger_, "set " << filter_name << " to " << value);
      filter->enable(filter_params[filter_name]);
      filter_status_[filter_name] = filter_params[filter_name];
    }

    if (filter_name == "NoiseRemovalFilter" && default_config_.enable_noise_removal_filter_) {
      auto noise_removal_filter = filter->as<ob::NoiseRemovalFilter>();
      OBNoiseRemovalFilterParams params = noise_removal_filter->getFilterParams();
      RCLCPP_INFO_STREAM(
          logger_, "Default noise removal filter params: " << "disp_diff: " << params.disp_diff
                                                           << ", max_size: " << params.max_size);
      params.disp_diff = default_config_.noise_removal_filter_min_diff_;
      params.max_size = default_config_.noise_removal_filter_max_size_;
      RCLCPP_INFO_STREAM(logger_,
                         "Set noise removal filter params: " << "disp_diff: " << params.disp_diff
                                                             << ", max_size: " << params.max_size);
      if (default_config_.noise_removal_filter_min_diff_ != -1 && default_config_.noise_removal_filter_max_size_ != -1) {
        noise_removal_filter->setFilterParams(params);
      }
    } else if (filter_name == "HDRMerge" && default_config_.enable_hdr_merge_) {
      if (default_config_.hdr_merge_exposure_1_ != -1 && default_config_.hdr_merge_gain_1_ != -1 && default_config_.hdr_merge_exposure_2_ != -1 &&
          default_config_.hdr_merge_gain_2_ != -1) {
        auto hdr_merge_filter = filter->as<ob::HdrMerge>();
        hdr_merge_filter->enable(true);
        RCLCPP_INFO_STREAM(
            logger_, "Set HDR merge filter params: " << "exposure_1: " << default_config_.hdr_merge_exposure_1_
                                                     << ", gain_1: " << default_config_.hdr_merge_gain_1_
                                                     << ", exposure_2: " << default_config_.hdr_merge_exposure_2_
                                                     << ", gain_2: " << default_config_.hdr_merge_gain_2_);
        auto config = OBHdrConfig();
        config.enable = true;
        config.exposure_1 = default_config_.hdr_merge_exposure_1_;
        config.gain_1 = default_config_.hdr_merge_gain_1_;
        config.exposure_2 = default_config_.hdr_merge_exposure_2_;
        config.gain_2 = default_config_.hdr_merge_gain_2_;
        device_->setStructuredData(OB_STRUCT_DEPTH_HDR_CONFIG, &config, sizeof(config));
      }
    } else {
      RCLCPP_INFO_STREAM(logger_, "Skip setting filter: " << filter_name);
    }
  }
}
// check
void OBCameraNode::selectBaseStream() {
  if (enable_stream_[DEPTH]) {
    base_stream_ = DEPTH;
  } else if (enable_stream_[INFRA0]) {
    base_stream_ = INFRA0;
  } else if (enable_stream_[INFRA1]) {
    base_stream_ = INFRA1;
  } else if (enable_stream_[INFRA2]) {
    base_stream_ = INFRA2;
  } else if (enable_stream_[COLOR]) {
    base_stream_ = COLOR;
  }
}

//check
void OBCameraNode::printSensorProfiles(const std::shared_ptr<ob::Sensor> &sensor) {
  auto profiles = sensor->getStreamProfileList();
  for (size_t i = 0; i < profiles->count(); i++) {
    auto origin_profile = profiles->getProfile(i);
    if (sensor->type() == OB_SENSOR_COLOR) {
      auto profile = origin_profile->as<ob::VideoStreamProfile>();
      RCLCPP_INFO_STREAM(logger_, "color profile: " << profile->width() << "x" << profile->height()
                                                    << " " << profile->fps() << "fps "
                                                    << profile->format());
    } else if (sensor->type() == OB_SENSOR_DEPTH) {
      auto profile = origin_profile->as<ob::VideoStreamProfile>();
      RCLCPP_INFO_STREAM(logger_, "depth profile: " << profile->width() << "x" << profile->height()
                                                    << " " << profile->fps() << "fps "
                                                    << profile->format());
    } else if (sensor->type() == OB_SENSOR_IR) {
      auto profile = origin_profile->as<ob::VideoStreamProfile>();
      RCLCPP_INFO_STREAM(logger_, "ir profile: " << profile->width() << "x" << profile->height()
                                                 << " " << profile->fps() << "fps "
                                                 << profile->format());
    } else if (sensor->type() == OB_SENSOR_ACCEL) {
      auto profile = origin_profile->as<ob::AccelStreamProfile>();
      RCLCPP_INFO_STREAM(logger_, "accel profile: sampleRate " << profile->sampleRate()
                                                               << "  full scale_range "
                                                               << profile->fullScaleRange());
    } else if (sensor->type() == OB_SENSOR_GYRO) {
      auto profile = origin_profile->as<ob::GyroStreamProfile>();
      RCLCPP_INFO_STREAM(logger_, "gyro profile: sampleRate " << profile->sampleRate()
                                                              << "  full scale_range "
                                                              << profile->fullScaleRange());
    } else {
      RCLCPP_INFO_STREAM(logger_, "unknown profile: " << magic_enum::enum_name(sensor->type()));
    }
  }
}

// check
void OBCameraNode::setupProfiles() {
  // Image stream
  for (const auto &elem : IMAGE_STREAMS) {
    if (enable_stream_[elem]) {
      const auto &sensor = sensors_[elem];
      CHECK_NOTNULL(sensor.get());
      auto profiles = sensor->getStreamProfileList();
      CHECK_NOTNULL(profiles.get());
      CHECK(profiles->count() > 0);
      for (size_t i = 0; i < profiles->count(); i++) {
        auto base_profile = profiles->getProfile(i);
        if (base_profile == nullptr) {
          throw std::runtime_error("Failed to get profile " + std::to_string(i));
        }
        auto profile = base_profile->as<ob::VideoStreamProfile>();
        if (profile == nullptr) {
          throw std::runtime_error("Failed cast profile to VideoStreamProfile");
        }
        RCLCPP_DEBUG_STREAM(
            logger_,
            "Sensor profile: " << "stream_type: " << magic_enum::enum_name(profile->type())
                               << "Format: " << profile->format() << ", Width: " << profile->width()
                               << ", Height: " << profile->height() << ", FPS: " << profile->fps());
        supported_profiles_[elem].emplace_back(profile);
      }
      std::shared_ptr<ob::VideoStreamProfile> selected_profile;
      std::shared_ptr<ob::VideoStreamProfile> default_profile;
      try {
        if (width_[elem] == 0 && height_[elem] == 0 && fps_[elem] == 0 &&
            format_[elem] == OB_FORMAT_UNKNOWN) {
          selected_profile = profiles->getProfile(0)->as<ob::VideoStreamProfile>();
        } else {
          selected_profile = profiles->getVideoStreamProfile(width_[elem], height_[elem],
                                                             format_[elem], fps_[elem]);
        }

      } catch (const ob::Error &ex) {
        RCLCPP_ERROR_STREAM(
            logger_, "Failed to get " << stream_name_[elem] << "  profile: " << ex.getMessage());
        RCLCPP_ERROR_STREAM(
            logger_, "Stream: " << magic_enum::enum_name(elem.first)
                                << ", Stream Index: " << elem.second << ", Width: " << width_[elem]
                                << ", Height: " << height_[elem] << ", FPS: " << fps_[elem]
                                << ", Format: " << magic_enum::enum_name(format_[elem]));
        RCLCPP_ERROR(logger_,
                     "Error: The device might be connected via USB 2.0. Please verify your "
                     "configuration and try again. The current process will now exit.");
        RCLCPP_INFO_STREAM(logger_, "Available profiles:");
        printSensorProfiles(sensor);
        RCLCPP_ERROR(logger_, "Because can not set this stream, so exit.");
        exit(-1);
      }

      if (!selected_profile) {
        RCLCPP_WARN_STREAM(logger_, "Given stream configuration is not supported by the device! "
                                        << " Stream: " << magic_enum::enum_name(elem.first)
                                        << ", Stream Index: " << elem.second
                                        << ", Width: " << width_[elem]
                                        << ", Height: " << height_[elem] << ", FPS: " << fps_[elem]
                                        << ", Format: " << magic_enum::enum_name(format_[elem]));
        if (default_profile) {
          RCLCPP_WARN_STREAM(logger_, "Using default profile instead.");
          RCLCPP_WARN_STREAM(logger_, "default FPS " << default_profile->fps());
          selected_profile = default_profile;
        } else {
          RCLCPP_ERROR_STREAM(
              logger_, " NO default_profile found , Stream: " << magic_enum::enum_name(elem.first)
                                                              << " will be disable");
          enable_stream_[elem] = false;
          continue;
        }
      }
      CHECK_NOTNULL(selected_profile);
      stream_profile_[elem] = selected_profile;
      height_[elem] = static_cast<int>(selected_profile->height());
      width_[elem] = static_cast<int>(selected_profile->width());
      fps_[elem] = static_cast<int>(selected_profile->fps());
      format_[elem] = selected_profile->format();
      updateImageConfig(elem);
      images_[elem] =
          cv::Mat(height_[elem], width_[elem], image_format_[elem], cv::Scalar(0, 0, 0));
      RCLCPP_INFO_STREAM(
          logger_, " stream " << stream_name_[elem]
                              << " is enabled - width: " << selected_profile->width()
                              << ", height: " << selected_profile->height()
                              << ", fps: " << selected_profile->fps() << ", "
                              << "Format: " << magic_enum::enum_name(selected_profile->format()));
    }
  }
  // IMU
  for (const auto &stream_index : HID_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    try {
      auto profile_list = sensors_[stream_index]->getStreamProfileList();
      if (stream_index == ACCEL) {
        auto full_scale_range = fullAccelScaleRangeFromString(imu_range_[stream_index]);
        auto sample_rate = sampleRateFromString(imu_rate_[stream_index]);
        auto profile = profile_list->getAccelStreamProfile(full_scale_range, sample_rate);
        stream_profile_[stream_index] = profile;
      } else if (stream_index == GYRO) {
        auto full_scale_range = fullGyroScaleRangeFromString(imu_range_[stream_index]);
        auto sample_rate = sampleRateFromString(imu_rate_[stream_index]);
        auto profile = profile_list->getGyroStreamProfile(full_scale_range, sample_rate);
        stream_profile_[stream_index] = profile;
      }
      RCLCPP_INFO_STREAM(logger_, "stream " << stream_name_[stream_index] << " full scale range "
                                            << imu_range_[stream_index] << " sample rate "
                                            << imu_rate_[stream_index]);
    } catch (const ob::Error &e) {
      RCLCPP_INFO_STREAM(logger_, "Failed to setup << " << stream_name_[stream_index]
                                                        << " profile: " << e.getMessage());
      enable_stream_[stream_index] = false;
      stream_profile_[stream_index] = nullptr;
    }
  }
}

// check
void OBCameraNode::updateImageConfig(const stream_index_pair &stream_index) {
  if (format_[stream_index] == OB_FORMAT_Y8) {
    image_format_[stream_index] = CV_8UC1;
    encoding_[stream_index] = stream_index.first == OB_STREAM_DEPTH
                                  ? sensor_msgs::image_encodings::TYPE_8UC1
                                  : sensor_msgs::image_encodings::MONO8;
    unit_step_size_[stream_index] = sizeof(uint8_t);
  }
  if (format_[stream_index] == OB_FORMAT_MJPG) {
    if (stream_index.first == OB_STREAM_IR || stream_index.first == OB_STREAM_IR_LEFT ||
        stream_index.first == OB_STREAM_IR_RIGHT) {
      image_format_[stream_index] = CV_8UC1;
      encoding_[stream_index] = sensor_msgs::image_encodings::MONO8;
      unit_step_size_[stream_index] = sizeof(uint8_t);
    }
  }
  if (format_[stream_index] == OB_FORMAT_Y16 && stream_index == COLOR) {
    image_format_[stream_index] = CV_16UC1;
    encoding_[stream_index] = sensor_msgs::image_encodings::MONO16;
    unit_step_size_[stream_index] = sizeof(uint16_t);
  }
}

// check
void OBCameraNode::startStreams() {
  if (pipeline_ != nullptr) {
    pipeline_.reset();
  }
  pipeline_ = std::make_unique<ob::Pipeline>(device_);
  try {
    setupPipelineConfig();
    pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet> &frame_set) {
      onNewFrameSetCallback(frame_set);
    });
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to start pipeline: " << e.getMessage());
    RCLCPP_INFO_STREAM(logger_, "try to disable ir stream and try again");
    enable_stream_[INFRA0] = false;
    setupPipelineConfig();
    pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet> &frame_set) {
      onNewFrameSetCallback(frame_set);
    });
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to start pipeline");
    throw std::runtime_error("Failed to start pipeline");
  }
  if (enable_stream_[COLOR] && !colorFrameThread_) {
    colorFrameThread_ = std::make_shared<std::thread>([this]() { onNewColorFrameCallback(); });
  }
  if (enable_frame_sync_) {
    RCLCPP_INFO_STREAM(logger_, "Enable frame sync");
    TRY_EXECUTE_BLOCK(pipeline_->enableFrameSync());
  } else {
    RCLCPP_INFO_STREAM(logger_, "Disable frame sync");
    TRY_EXECUTE_BLOCK(pipeline_->disableFrameSync());
  }
  if (device_->isPropertySupported(OB_PROP_LDP_BOOL, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting LDP to " << (default_config_.enable_ldp_ ? "ON" : "OFF"));
    if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
      auto laser_enable = device_->getIntProperty(OB_PROP_LASER_CONTROL_INT);
      TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_LDP_BOOL, default_config_.enable_ldp_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_LASER_CONTROL_INT, laser_enable);
    } else if (device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
      if (!default_config_.enable_ldp_) {
        auto laser_enable = device_->getIntProperty(OB_PROP_LASER_BOOL);
        TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_LDP_BOOL, default_config_.enable_ldp_);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_LASER_BOOL, laser_enable);
      } else {
        TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_LDP_BOOL, default_config_.enable_ldp_);
      }
    }
  }
  pipeline_started_.store(true);
}

// check
void OBCameraNode::startIMUSyncStream() {
  if (imuPipeline_ != nullptr) {
    imuPipeline_.reset();
  }

  imuPipeline_ = std::make_unique<ob::Pipeline>(device_);
  if (imu_sync_output_start_) {
    return;
  }

  // ACCEL
  auto accelProfiles = imuPipeline_->getStreamProfileList(OB_SENSOR_ACCEL);
  auto accel_range = fullAccelScaleRangeFromString(imu_range_[ACCEL]);
  auto accel_rate = sampleRateFromString(imu_rate_[ACCEL]);
  auto accelProfile = accelProfiles->getAccelStreamProfile(accel_range, accel_rate);
  // GYRO
  auto gyroProfiles = imuPipeline_->getStreamProfileList(OB_SENSOR_GYRO);
  auto gyro_range = fullGyroScaleRangeFromString(imu_range_[GYRO]);
  auto gyro_rate = sampleRateFromString(imu_rate_[GYRO]);
  auto gyroProfile = gyroProfiles->getGyroStreamProfile(gyro_range, gyro_rate);
  std::shared_ptr<ob::Config> imuConfig = std::make_shared<ob::Config>();
  imuConfig->enableStream(accelProfile);
  imuConfig->enableStream(gyroProfile);
  TRY_EXECUTE_BLOCK(imuPipeline_->enableFrameSync());
  imuPipeline_->start(imuConfig, [&](std::shared_ptr<ob::Frame> frame) {
    auto frameSet = frame->as<ob::FrameSet>();
    auto aFrame = frameSet->getFrame(OB_FRAME_ACCEL);
    auto gFrame = frameSet->getFrame(OB_FRAME_GYRO);
    if (aFrame && gFrame) {
      onNewIMUFrameSyncOutputCallback(aFrame, gFrame);
    }
  });

  imu_sync_output_start_ = true;
  if (!imu_sync_output_start_) {
    RCLCPP_ERROR_STREAM(
        logger_, "Failed to start IMU stream, please check the imu_rate and imu_range parameters.");
  } else {
    RCLCPP_INFO_STREAM(
        logger_, "start accel stream with range: " << fullAccelScaleRangeToString(accel_range)
                                                   << ",rate:" << sampleRateToString(accel_rate)
                                                   << ", and start gyro stream with range:"
                                                   << fullGyroScaleRangeToString(gyro_range)
                                                   << ",rate:" << sampleRateToString(gyro_rate));
  }
}

// check
void OBCameraNode::startIMU() {
  if (enable_sync_output_accel_gyro_) {
    startIMUSyncStream();
  } else {
    for (const auto &stream_index : HID_STREAMS) {
      if (enable_stream_[stream_index] && !imu_started_[stream_index]) {
        auto imu_profile = stream_profile_[stream_index];
        CHECK_NOTNULL(imu_profile);
        RCLCPP_INFO_STREAM(logger_, "start " << stream_name_[stream_index] << " stream");
        CHECK_NOTNULL(sensors_[stream_index]);
        sensors_[stream_index]->start(
            imu_profile, [this, stream_index](const std::shared_ptr<ob::Frame> &frame) {
              onNewIMUFrameCallback(frame, stream_index);
            });
      }
    }
  }
}

// check
void OBCameraNode::stopStreams() {
  if (!pipeline_started_ || !pipeline_) {
    RCLCPP_INFO_STREAM(logger_, "pipeline not started or not exist, skip stop pipeline");
    return;
  }
  try {
    pipeline_->stop();
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to stop pipeline: " << e.getMessage());
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to stop pipeline");
  }
}

// check
void OBCameraNode::stopIMU() {
  if (enable_sync_output_accel_gyro_) {
    if (!imu_sync_output_start_ || !imuPipeline_) {
      RCLCPP_INFO_STREAM(logger_, "imu pipeline not started or not exist, skip stop imu pipeline");
      return;
    }
    try {
      imuPipeline_->stop();
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to stop imu pipeline: " << e.getMessage());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to stop imu pipeline");
    }
  } else {
    for (const auto &stream_index : HID_STREAMS) {
      if (imu_started_[stream_index]) {
        CHECK(sensors_.count(stream_index));
        RCLCPP_INFO_STREAM(logger_, "stop " << stream_name_[stream_index] << " stream");
        try {
          sensors_[stream_index]->stop();
        } catch (const ob::Error &e) {
          RCLCPP_ERROR_STREAM(logger_, "Failed to stop " << stream_name_[stream_index]
                                                         << " stream: " << e.getMessage());
        }
        imu_started_[stream_index] = false;
      }
    }
  }
}

// check
void OBCameraNode::setupDefaultImageFormat() {
  format_[DEPTH] = OB_FORMAT_Y16;
  format_str_[DEPTH] = "Y16";
  image_format_[DEPTH] = CV_16UC1;
  encoding_[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
  unit_step_size_[DEPTH] = sizeof(uint16_t);
  format_[INFRA0] = OB_FORMAT_Y16;
  format_str_[INFRA0] = "Y16";
  image_format_[INFRA0] = CV_16UC1;
  encoding_[INFRA0] = sensor_msgs::image_encodings::MONO16;
  unit_step_size_[INFRA0] = sizeof(uint16_t);

  format_[INFRA1] = OB_FORMAT_Y16;
  format_str_[INFRA1] = "Y16";
  image_format_[INFRA1] = CV_16UC1;
  encoding_[INFRA1] = sensor_msgs::image_encodings::MONO16;
  unit_step_size_[INFRA1] = sizeof(uint16_t);

  format_[INFRA2] = OB_FORMAT_Y16;
  format_str_[INFRA2] = "Y16";
  image_format_[INFRA2] = CV_16UC1;
  encoding_[INFRA2] = sensor_msgs::image_encodings::MONO16;
  unit_step_size_[INFRA2] = sizeof(uint16_t);

  image_format_[COLOR] = CV_8UC3;
  encoding_[COLOR] = sensor_msgs::image_encodings::RGB8;
  unit_step_size_[COLOR] = 3 * sizeof(uint8_t);
}

// check
void OBCameraNode::getParameters() {
  setAndGetNodeParameter<std::string>(camera_name_, "camera_name", "camera");
  camera_link_frame_id_ = camera_name_ + "_link";
  for (auto stream_index : IMAGE_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_width";
    setAndGetNodeParameter(width_[stream_index], param_name, 0);
    param_name = stream_name_[stream_index] + "_height";
    setAndGetNodeParameter(height_[stream_index], param_name, 0);
    param_name = stream_name_[stream_index] + "_fps";
    setAndGetNodeParameter(fps_[stream_index], param_name, 0);
    param_name = "enable_" + stream_name_[stream_index];
    if (stream_index == DEPTH) {
      setAndGetNodeParameter(enable_stream_[stream_index], param_name, true);
    } else {
      setAndGetNodeParameter(enable_stream_[stream_index], param_name, false);
    }
    param_name = "flip_" + stream_name_[stream_index];
    setAndGetNodeParameter(flip_stream_[stream_index], param_name, false);
    param_name = camera_name_ + "_" + stream_name_[stream_index] + "_frame_id";
    std::string default_frame_id = camera_name_ + "_" + stream_name_[stream_index] + "_frame";
    setAndGetNodeParameter(frame_id_[stream_index], param_name, default_frame_id);
    std::string default_optical_frame_id =
        camera_name_ + "_" + stream_name_[stream_index] + "_optical_frame";
    param_name = stream_name_[stream_index] + "_optical_frame_id";
    setAndGetNodeParameter(optical_frame_id_[stream_index], param_name, default_optical_frame_id);
    param_name = stream_name_[stream_index] + "_format";
    setAndGetNodeParameter(format_str_[stream_index], param_name, format_str_[stream_index]);
    format_[stream_index] = OBFormatFromString(format_str_[stream_index]);
    updateImageConfig(stream_index);
    // param_name = stream_name_[stream_index] + "_qos";
    // setAndGetNodeParameter<std::string>(image_qos_[stream_index], param_name, "default");
    // param_name = stream_name_[stream_index] + "_camera_info_qos";
    // setAndGetNodeParameter<std::string>(camera_info_qos_[stream_index], param_name, "default");
  }

  for (auto stream_index : IMAGE_STREAMS) {
    depth_aligned_frame_id_[stream_index] = optical_frame_id_[COLOR];
  }

  // Check qos etc...
  setAndGetNodeParameter(enable_sync_output_accel_gyro_, "enable_sync_output_accel_gyro", false);
  for (const auto &stream_index : HID_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_qos";
    setAndGetNodeParameter<std::string>(imu_qos_[stream_index], param_name, "default");
    param_name = "enable_" + stream_name_[stream_index];
    setAndGetNodeParameter(enable_stream_[stream_index], param_name, false);
    if (enable_sync_output_accel_gyro_) {
      enable_stream_[stream_index] = true;
    }
    param_name = stream_name_[stream_index] + "_rate";
    setAndGetNodeParameter<std::string>(imu_rate_[stream_index], param_name, "");
    param_name = stream_name_[stream_index] + "_range";
    setAndGetNodeParameter<std::string>(imu_range_[stream_index], param_name, "");
    param_name = camera_name_ + "_" + stream_name_[stream_index] + "_frame_id";
    std::string default_frame_id = camera_name_ + "_" + stream_name_[stream_index] + "_frame";
    setAndGetNodeParameter(frame_id_[stream_index], param_name, default_frame_id);
    std::string default_optical_frame_id =
        camera_name_ + "_" + stream_name_[stream_index] + "_optical_frame";
    param_name = stream_name_[stream_index] + "_optical_frame_id";
    setAndGetNodeParameter(optical_frame_id_[stream_index], param_name, default_optical_frame_id);
    depth_aligned_frame_id_[stream_index] =
        camera_name_ + "_" + stream_name_[COLOR] + "_optical_frame";
  }

  accel_gyro_frame_id_ = camera_name_ + "_accel_gyro_optical_frame";

  setAndGetNodeParameter(depth_registration_, "depth_registration", false);
  setAndGetNodeParameter(enable_point_cloud_, "enable_point_cloud", false);
  setAndGetNodeParameter(enable_colored_point_cloud_, "enable_colored_point_cloud", false);
  setAndGetNodeParameter(enable_point_cloud_, "enable_point_cloud", false);
  if (!default_config_.depth_filter_config_.empty()) {
    enable_depth_filter_ = true;
  }
  setAndGetNodeParameter(enable_frame_sync_, "enable_frame_sync", false);

  if (!default_config_.depth_precision_str_.empty()) {
    depth_precision_ = depthPrecisionLevelFromString(default_config_.depth_precision_str_);
  }
  if (enable_colored_point_cloud_ || default_config_.enable_d2c_viewer_) {
    depth_registration_ = true;
  }
  if (!enable_stream_[COLOR]) {
    enable_colored_point_cloud_ = false;
    depth_registration_ = false;
  }

  setAndGetNodeParameter<std::string>(align_mode_, "align_mode", "HW");
  align_target_stream_ = obStreamTypeFromString(default_config_.align_target_stream_str_);

  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info.get());
  auto pid = device_info->pid();
  if (isOpenNIDevice(pid)) {
    time_domain_ = "system";
  }
  RCLCPP_INFO_STREAM(logger_, "current time domain: " << time_domain_);
}

// check
void OBCameraNode::setupTopics() {
  try {
    getParameters();
    setupDevices();
    setupDepthPostProcessFilter();
    setupProfiles();
    setupCameraInfo();
    selectBaseStream();
    // setupCameraCtrlServices();
    setupPublishers();
    // setupDiagnosticUpdater();
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to setup topics: " << e.getMessage());
    throw std::runtime_error(e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to setup topics: " << e.what());
    throw std::runtime_error(e.what());
  } catch (...) {
    RCLCPP_ERROR(logger_, "Failed to setup topics");
    throw std::runtime_error("Failed to setup topics");
  }
}

// check
void OBCameraNode::setupPipelineConfig() {
  if (pipeline_config_) {
    pipeline_config_.reset();
  }
  pipeline_config_ = std::make_shared<ob::Config>();
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info.get());
  auto pid = device_info->pid();
  if (depth_registration_ && enable_stream_[COLOR] && enable_stream_[DEPTH] &&
      !isGemini335PID(pid)) {
    OBAlignMode align_mode = align_mode_ == "HW" ? ALIGN_D2C_HW_MODE : ALIGN_D2C_SW_MODE;
    RCLCPP_INFO_STREAM(logger_, "set align mode to " << magic_enum::enum_name(align_mode));
    pipeline_config_->setAlignMode(align_mode);
    RCLCPP_INFO_STREAM(logger_, "enable depth scale " << (default_config_.enable_depth_scale_ ? "ON" : "OFF"));
    pipeline_config_->setDepthScaleRequire(default_config_.enable_depth_scale_);
  }
  for (const auto &stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      RCLCPP_INFO_STREAM(logger_, "Enable " << stream_name_[stream_index] << " stream");
      auto profile = stream_profile_[stream_index]->as<ob::VideoStreamProfile>();
      RCLCPP_INFO_STREAM(logger_,
                         "Stream " << stream_name_[stream_index] << " width: " << profile->width()
                                   << " height: " << profile->height() << " fps: " << profile->fps()
                                   << " format: " << profile->format());
      pipeline_config_->enableStream(stream_profile_[stream_index]);
    }
  }

  if (frame_aggregate_mode_ == "full_frame") {
    pipeline_config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE);
  } else if (frame_aggregate_mode_ == "color_frame") {
    pipeline_config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_COLOR_FRAME_REQUIRE);
  } else if (frame_aggregate_mode_ == "disable") {
    pipeline_config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_DISABLE);
  } else {
    pipeline_config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
  }
}

// check
void OBCameraNode::setupCameraInfo() {
  std::string color_camera_name = camera_name_ + "_color";
  if (!default_config_.color_info_url_.empty()) {
    color_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        node_, color_camera_name, default_config_.color_info_url_);
  }
  // std::string ir_camera_name = camera_name_ + "_ir";
  // if (!default_config_.ir_info_url_.empty()) {
  //   ir_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
  //       node_, ir_camera_name, default_config_.ir_info_url_);
  // }
}

// need to delete without imu
void OBCameraNode::setupPublishers() {
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  // need to change
  auto point_cloud_qos_profile = getRMWQosProfileFromString(default_config_.point_cloud_qos_);
  if (use_intra_process_) {
    point_cloud_qos_profile = rmw_qos_profile_default;
  }
  if (enable_colored_point_cloud_) {
    depth_registration_cloud_pub_ = node_->create_publisher<PointCloud2>(
        "depth_registered/points",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                    point_cloud_qos_profile));
  }
  if (enable_point_cloud_) {
    depth_cloud_pub_ = node_->create_publisher<PointCloud2>(
        "depth/points", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                                    point_cloud_qos_profile));
  }

  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info.get());
  auto pid = device_info->pid();
  for (const auto &stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    std::string name = stream_name_[stream_index];
    std::string topic = name + "/image_raw";
    auto image_qos = image_qos_[stream_index];
    auto image_qos_profile = getRMWQosProfileFromString(image_qos);
    if (use_intra_process_) {
      image_qos_profile = rmw_qos_profile_default;
    }
    if (use_intra_process_) {
      image_publishers_[stream_index] =
          std::make_shared<image_rcl_publisher>(*node_, topic, image_qos_profile);
    } else {
      image_publishers_[stream_index] =
          std::make_shared<image_transport_publisher>(*node_, topic, image_qos_profile);
    }

    topic = name + "/camera_info";
    auto camera_info_qos = camera_info_qos_[stream_index];
    auto camera_info_qos_profile = getRMWQosProfileFromString(camera_info_qos);
    if (use_intra_process_) {
      camera_info_qos_profile = rmw_qos_profile_default;
    }
    camera_info_publishers_[stream_index] = node_->create_publisher<CameraInfo>(
        topic, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(camera_info_qos_profile),
                           camera_info_qos_profile));
    if (isGemini335PID(pid)) {
      metadata_publishers_[stream_index] =
          node_->create_publisher<orbbec_camera_msgs::msg::Metadata>(
              name + "/metadata",
              rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(camera_info_qos_profile),
                          camera_info_qos_profile));
    }
    if (stream_index == COLOR && default_config_.enable_color_undistortion_) {
      if (use_intra_process_) {
        color_undistortion_publisher_ = std::make_shared<image_rcl_publisher>(
            *node_, "color/image_undistorted", image_qos_profile);
      } else {
        color_undistortion_publisher_ = std::make_shared<image_transport_publisher>(
            *node_, "color/image_undistorted", image_qos_profile);
      }
    }
  }

  if (enable_sync_output_accel_gyro_) {
    std::string topic_name = stream_name_[GYRO] + "_" + stream_name_[ACCEL] + "/sample";
    auto data_qos = getRMWQosProfileFromString(imu_qos_[GYRO]);
    if (use_intra_process_) {
      data_qos = rmw_qos_profile_default;
    }
    imu_gyro_accel_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>(
        topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos));
    topic_name = stream_name_[GYRO] + "/imu_info";
    imu_info_publishers_[GYRO] = node_->create_publisher<orbbec_camera_msgs::msg::IMUInfo>(
        topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos));
    topic_name = stream_name_[ACCEL] + "/imu_info";
    imu_info_publishers_[ACCEL] = node_->create_publisher<orbbec_camera_msgs::msg::IMUInfo>(
        topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos));
  } else {
    for (const auto &stream_index : HID_STREAMS) {
      if (!enable_stream_[stream_index]) {
        continue;
      }
      std::string data_topic_name = stream_name_[stream_index] + "/sample";
      auto data_qos = getRMWQosProfileFromString(imu_qos_[stream_index]);
      if (use_intra_process_) {
        data_qos = rmw_qos_profile_default;
      }
      imu_publishers_[stream_index] = node_->create_publisher<sensor_msgs::msg::Imu>(
          data_topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos));
      data_topic_name = stream_name_[stream_index] + "/imu_info";
      imu_info_publishers_[stream_index] =
          node_->create_publisher<orbbec_camera_msgs::msg::IMUInfo>(
              data_topic_name,
              rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos));
    }
  }

  auto extrinsics_qos = rclcpp::QoS(1).transient_local();
  if (use_intra_process_) {
    extrinsics_qos = rclcpp::QoS(1);
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA0]) {
    depth_to_other_extrinsics_publishers_[INFRA0] =
        node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/depth_to_ir", extrinsics_qos);
  }
  if (enable_stream_[DEPTH] && enable_stream_[COLOR]) {
    depth_to_other_extrinsics_publishers_[COLOR] =
        node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/depth_to_color", extrinsics_qos);
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA1]) {
    depth_to_other_extrinsics_publishers_[INFRA1] =
        node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/depth_to_left_ir", extrinsics_qos);
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA2]) {
    depth_to_other_extrinsics_publishers_[INFRA2] =
        node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/depth_to_right_ir", extrinsics_qos);
  }
  if (enable_stream_[DEPTH] && enable_stream_[ACCEL]) {
    depth_to_other_extrinsics_publishers_[ACCEL] =
        node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/depth_to_accel", extrinsics_qos);
  }
  if (enable_stream_[DEPTH] && enable_stream_[GYRO]) {
    depth_to_other_extrinsics_publishers_[GYRO] =
        node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/depth_to_gyro", extrinsics_qos);
  }
  filter_status_pub_ =
      node_->create_publisher<std_msgs::msg::String>("depth_filter_status", extrinsics_qos);
  std_msgs::msg::String msg;
  msg.data = filter_status_.dump(2);
  filter_status_pub_->publish(msg);
}

// check need to delete to publish
void OBCameraNode::publishPointCloud(const std::shared_ptr<ob::FrameSet> &frame_set) {
  try {
    if (depth_registration_ || enable_colored_point_cloud_) {
      if (frame_set->depthFrame() != nullptr && frame_set->colorFrame() != nullptr) {
        publishColoredPointCloud(frame_set);
      }
    }

    if (enable_point_cloud_ && frame_set->depthFrame() != nullptr) {
      publishDepthPointCloud(frame_set);
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "publishPointCloud with unknown error");
  }
}

// need to delete publish
void OBCameraNode::publishDepthPointCloud(const std::shared_ptr<ob::FrameSet> &frame_set) {
  (void)frame_set;
  if (!depth_cloud_pub_ || depth_cloud_pub_->get_subscription_count() == 0 ||
      !enable_point_cloud_ || !depth_frame_) {
    return;
  }
  std::lock_guard<decltype(point_cloud_mutex_)> point_cloud_msg_lock(point_cloud_mutex_);
  if (!depth_frame_) {
    RCLCPP_ERROR_STREAM(logger_, "depth frame is null");
    return;
  }
  auto depth_frame = depth_frame_->as<ob::DepthFrame>();
  if (!depth_frame) {
    RCLCPP_ERROR_STREAM(logger_, "depth frame is null");
    return;
  }
  CHECK_NOTNULL(pipeline_);
  auto camera_params = pipeline_->getCameraParam();
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info.get());
  auto pid = device_info->pid();
  if (depth_registration_ || pid == DABAI_MAX_PID) {
    camera_params.depthIntrinsic = camera_params.rgbIntrinsic;
  }
  depth_point_cloud_filter_.setCameraParam(camera_params);
  float depth_scale = depth_frame->getValueScale();
  depth_point_cloud_filter_.setPositionDataScaled(depth_scale);
  depth_point_cloud_filter_.setCreatePointFormat(OB_FORMAT_POINT);
  auto result_frame = depth_point_cloud_filter_.process(depth_frame);
  if (!result_frame) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to process depth frame");
    return;
  }
  auto point_size = result_frame->dataSize() / sizeof(OBPoint);
  auto *points = static_cast<OBPoint *>(result_frame->data());
  auto width = depth_frame->width();
  auto height = depth_frame->height();
  auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(width * height);
  point_cloud_msg->width = depth_frame->width();
  point_cloud_msg->height = depth_frame->height();
  point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
  point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
  const static float MIN_DISTANCE = 20.0;     // 2cm
  const static float MAX_DISTANCE = 10000.0;  // 10m
  const static float min_depth = MIN_DISTANCE / depth_scale;
  const static float max_depth = MAX_DISTANCE / depth_scale;
  size_t valid_count = 0;
  for (size_t i = 0; i < point_size; i++) {
    bool valid_point = points[i].z >= min_depth && points[i].z <= max_depth;
    if (valid_point || default_config_.ordered_pc_) {
      *iter_x = static_cast<float>(points[i].x / 1000.0);
      *iter_y = static_cast<float>(points[i].y / 1000.0);
      *iter_z = static_cast<float>(points[i].z / 1000.0);
      ++iter_x, ++iter_y, ++iter_z;
      valid_count++;
    }
  }
  if (valid_count == 0) {
    RCLCPP_WARN(logger_, "No valid point in point cloud");
    return;
  }
  if (!default_config_.ordered_pc_) {
    point_cloud_msg->is_dense = true;
    point_cloud_msg->width = valid_count;
    point_cloud_msg->height = 1;
    modifier.resize(valid_count);
  }

  // depth_cloud_pub_->publish(std::move(point_cloud_msg));
}

// need to delete publish
void OBCameraNode::publishColoredPointCloud(const std::shared_ptr<ob::FrameSet> &frame_set) {
  if (!depth_registration_cloud_pub_ ||
      depth_registration_cloud_pub_->get_subscription_count() == 0 ||
      !enable_colored_point_cloud_ || !depth_frame_) {
    return;
  }

  CHECK_NOTNULL(depth_frame_.get());
  std::lock_guard<decltype(point_cloud_mutex_)> point_cloud_msg_lock(point_cloud_mutex_);
  if (!depth_frame_) {
    RCLCPP_ERROR_STREAM(logger_, "depth frame is null");
    return;
  }
  auto depth_frame = depth_frame_->as<ob::DepthFrame>();
  auto color_frame = frame_set->colorFrame();
  if (!depth_frame || !color_frame) {
    return;
  }
  auto depth_width = depth_frame->width();
  auto depth_height = depth_frame->height();
  auto color_width = color_frame->width();
  auto color_height = color_frame->height();
  if (depth_width != color_width || depth_height != color_height) {
    RCLCPP_DEBUG(logger_, "Depth (%d x %d) and color (%d x %d) frame size mismatch", depth_width,
                 depth_height, color_width, color_height);
    return;
  }
  if (!xy_tables_.has_value()) {
    calibration_param_ = pipeline_->getCalibrationParam(pipeline_config_);

    uint32_t table_size =
        color_width * color_height * 2;  // one for x-coordinate and one for y-coordinate LUT
    if (xy_table_data_size_ != table_size) {
      RCLCPP_INFO_STREAM(logger_, "Init xy tables with size " << table_size);
      xy_table_data_size_ = table_size;
      delete[] xy_table_data_;
      xy_table_data_ = new float[table_size];
    }

    xy_tables_ = OBXYTables();
    CHECK_NOTNULL(xy_table_data_);
    if (!ob::CoordinateTransformHelper::transformationInitXYTables(
            *calibration_param_, OB_SENSOR_COLOR, xy_table_data_, &table_size, &(*xy_tables_))) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to init xy tables");
      return;
    }
  }

  const auto *depth_data = (uint8_t *)depth_frame->data();
  const auto *color_data = (uint8_t *)(rgb_buffer_);
  CHECK_NOTNULL(rgb_point_cloud_buffer_);
  uint32_t point_cloud_buffer_size = color_width * color_height * sizeof(OBColorPoint);
  if (point_cloud_buffer_size > rgb_point_cloud_buffer_size_) {
    delete[] rgb_point_cloud_buffer_;
    rgb_point_cloud_buffer_ = new uint8_t[point_cloud_buffer_size];
    rgb_point_cloud_buffer_size_ = point_cloud_buffer_size;
  }
  memset(rgb_point_cloud_buffer_, 0, rgb_point_cloud_buffer_size_);
  auto *point_cloud = (OBColorPoint *)rgb_point_cloud_buffer_;
  ob::CoordinateTransformHelper::transformationDepthToRGBDPointCloud(&(*xy_tables_), depth_data,
                                                                     color_data, point_cloud);
  auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  point_cloud_msg->width = color_frame->width();
  point_cloud_msg->height = color_frame->height();
  std::string format_str = "rgb";
  point_cloud_msg->point_step =
      addPointField(*point_cloud_msg, format_str, 1, sensor_msgs::msg::PointField::FLOAT32,
                    static_cast<int>(point_cloud_msg->point_step));
  point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
  point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*point_cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*point_cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*point_cloud_msg, "b");
  size_t valid_count = 0;
  static const float MIN_DISTANCE = 20.0;
  static const float MAX_DISTANCE = 10000.0;
  double depth_scale = depth_frame->getValueScale();
  static float min_depth = MIN_DISTANCE / depth_scale;
  static float max_depth = MAX_DISTANCE / depth_scale;
  for (size_t i = 0; i < color_width * color_height; i++) {
    bool valid_point = point_cloud[i].z >= min_depth && point_cloud[i].z <= max_depth;
    if (valid_point || default_config_.ordered_pc_) {
      *iter_x = static_cast<float>(point_cloud[i].x / 1000.0);
      *iter_y = static_cast<float>(point_cloud[i].y / 1000.0);
      *iter_z = static_cast<float>(point_cloud[i].z / 1000.0);
      *iter_r = static_cast<uint8_t>(point_cloud[i].r);
      *iter_g = static_cast<uint8_t>(point_cloud[i].g);
      *iter_b = static_cast<uint8_t>(point_cloud[i].b);
      ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b;
      ++valid_count;
    }
  }
  if (valid_count == 0) {
    RCLCPP_WARN(logger_, "No valid points in point cloud");
    return;
  }
  if (!default_config_.ordered_pc_) {
    point_cloud_msg->is_dense = true;
    point_cloud_msg->width = valid_count;
    point_cloud_msg->height = 1;
    modifier.resize(valid_count);
  }

  // depth_registration_cloud_pub_->publish(std::move(point_cloud_msg));
}

// check
std::shared_ptr<ob::Frame> OBCameraNode::processDepthFrameFilter(
    std::shared_ptr<ob::Frame> &frame) {
  if (frame == nullptr || frame->type() != OB_FRAME_DEPTH) {
    return nullptr;
  }
  auto sensor = device_->getSensor(OB_SENSOR_DEPTH);
  CHECK_NOTNULL(sensor.get());
  auto filter_list = sensor->getRecommendedFilters();
  for (size_t i = 0; i < filter_list->count(); i++) {
    auto filter = filter_list->getFilter(i);
    CHECK_NOTNULL(filter.get());
    if (filter->isEnabled() && frame != nullptr) {
      frame = filter->process(frame);
      if (frame == nullptr) {
        RCLCPP_ERROR_STREAM(logger_, "Depth filter process failed");
        break;
      }
    }
  }
  return frame;
}

// check
uint64_t OBCameraNode::getFrameTimestampUs(const std::shared_ptr<ob::Frame> &frame) {
  if (frame == nullptr) {
    RCLCPP_WARN(logger_, "getFrameTimestampUs: frame is nullptr, return 0");
    return 0;
  }
  if (time_domain_ == "device") {
    return frame->timeStampUs();
  } else if (time_domain_ == "global") {
    return frame->globalTimeStampUs();
  } else {
    return frame->systemTimeStampUs();
  }
}

// check
void OBCameraNode::onNewFrameSetCallback(std::shared_ptr<ob::FrameSet> frame_set) {
  if (!is_running_.load()) {
    return;
  }
  if (!is_camera_node_initialized_.load()) {
    return;
  }
  if (frame_set == nullptr) {
    return;
  }
  try {
    depth_frame_ = frame_set->getFrame(OB_FRAME_DEPTH);
    auto device_info = device_->getDeviceInfo();
    CHECK_NOTNULL(device_info.get());
    auto pid = device_info->pid();
    auto color_frame = frame_set->getFrame(OB_FRAME_COLOR);
    has_first_color_frame_ = has_first_color_frame_ || color_frame;
    if (isGemini335PID(pid) && depth_frame_) {
      depth_frame_ = processDepthFrameFilter(depth_frame_);
      if (depth_registration_ && align_filter_ && depth_frame_ && has_first_color_frame_) {
        auto new_frame = align_filter_->process(frame_set);
        if (new_frame) {
          auto new_frame_set = new_frame->as<ob::FrameSet>();
          CHECK_NOTNULL(new_frame_set.get());
          depth_frame_ = new_frame_set->getFrame(OB_FRAME_DEPTH);
        } else {
          RCLCPP_ERROR(logger_, "Failed to align depth frame to color frame");
        }
      } else {
        RCLCPP_DEBUG(logger_,
                     "Depth registration is disabled or align filter is null or depth frame is "
                     "null or color frame is null");
      }
    }
    if (enable_stream_[COLOR] && color_frame) {
      std::unique_lock<std::mutex> lock(color_frame_queue_lock_);
      // if (color_frame_queue_.size() > 2) {
      //   color_frame_queue_.pop();
      // }
      color_frame_queue_.push(frame_set);
      color_frame_queue_cv_.notify_all();
    } else {
      publishPointCloud(frame_set);
    }

    for (const auto &stream_index : IMAGE_STREAMS) {
      if (enable_stream_[stream_index]) {
        auto frame_type = STREAM_TYPE_TO_FRAME_TYPE.at(stream_index.first);
        if (frame_type == OB_FRAME_COLOR) {
          continue;
        }

        auto frame = frame_set->getFrame(frame_type);
        if (frame == nullptr) {
          continue;
        }
        if (stream_index == DEPTH) {
          frame = depth_frame_;
        }
        onNewFrameCallback(frame, stream_index);
      }
    }

  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: unknown error");
  }
}

// check
void OBCameraNode::onNewColorFrameCallback() {
  while (enable_stream_[COLOR] && rclcpp::ok() && is_running_.load()) {
    std::unique_lock<std::mutex> lock(color_frame_queue_lock_);
    color_frame_queue_cv_.wait(
        lock, [this]() { return !color_frame_queue_.empty() || !(is_running_.load()); });

    if (!rclcpp::ok() || !is_running_.load()) {
      break;
    }

    std::shared_ptr<ob::FrameSet> frameSet = color_frame_queue_.front();
    is_color_frame_decoded_ = decodeColorFrameToBuffer(frameSet->colorFrame(), rgb_buffer_);
    publishPointCloud(frameSet);
    onNewFrameCallback(frameSet->colorFrame(), COLOR);
    color_frame_queue_.pop();
  }

  RCLCPP_INFO_STREAM(logger_, "Color frame thread exit!");
}

// check
std::shared_ptr<ob::Frame> OBCameraNode::softwareDecodeColorFrame(
    const std::shared_ptr<ob::Frame> &frame) {
  if (frame == nullptr) {
    return nullptr;
  }
  if (frame->format() == OB_FORMAT_RGB || frame->format() == OB_FORMAT_BGR) {
    return frame;
  }
  if (frame->format() == OB_FORMAT_RGB || frame->format() == OB_FORMAT_BGR) {
    return frame;
  }
  if (frame->format() == OB_FORMAT_Y16 || frame->format() == OB_FORMAT_Y8) {
    return frame;
  }
  if (!setupFormatConvertType(frame->format())) {
    RCLCPP_ERROR(logger_, "Unsupported color format: %d", frame->format());
    return nullptr;
  }
  auto color_frame = format_convert_filter_.process(frame);
  if (color_frame == nullptr) {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(logger_, *(node_->get_clock()), 1000,
                                    "Failed to convert frame to RGB format");
    return nullptr;
  }
  return color_frame;
}

// check
bool OBCameraNode::decodeColorFrameToBuffer(const std::shared_ptr<ob::Frame> &frame,
                                            uint8_t *buffer) {
  if (frame == nullptr) {
    return false;
  }
  if (!rgb_buffer_) {
    return false;
  }
  CHECK_NOTNULL(image_publishers_[COLOR]);
  bool has_subscriber = image_publishers_[COLOR]->get_subscription_count() > 0;
  if (enable_colored_point_cloud_ && depth_registration_cloud_pub_->get_subscription_count() > 0) {
    has_subscriber = true;
  }
  if (!has_subscriber) {
    return false;
  }
  if (metadata_publishers_.count(COLOR) &&
      metadata_publishers_[COLOR]->get_subscription_count() > 0) {
    has_subscriber = true;
  }
  if (camera_info_publishers_.count(COLOR) &&
      camera_info_publishers_[COLOR]->get_subscription_count() > 0) {
    has_subscriber = true;
  }
  bool is_decoded = false;
  if (!frame) {
    return false;
  }

#if defined(USE_RK_HW_DECODER) || defined(USE_NV_HW_DECODER)
  if (frame && frame->format() != OB_FORMAT_RGB888) {
    if (frame->format() == OB_FORMAT_MJPG && jpeg_decoder_) {
      CHECK_NOTNULL(jpeg_decoder_.get());
      CHECK_NOTNULL(rgb_buffer_);
      auto video_frame = frame->as<ob::ColorFrame>();
      bool ret = jpeg_decoder_->decode(video_frame, rgb_buffer_);
      if (!ret) {
        RCLCPP_ERROR_STREAM(logger_, "Decode frame failed");
        is_decoded = false;

      } else {
        is_decoded = true;
      }
    }
  }
#endif
  if (!is_decoded) {
    auto video_frame = softwareDecodeColorFrame(frame);
    if (!video_frame) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to convert frame to video frame");
      return false;
    }
    CHECK_NOTNULL(buffer);
    memcpy(buffer, video_frame->data(), video_frame->dataSize());
    return true;
  }
  return true;
}


// check
void OBCameraNode::onNewFrameCallback(const std::shared_ptr<ob::Frame> &frame,
                                      const stream_index_pair &stream_index) {
  if (frame == nullptr) {
    return;
  }

  std::shared_ptr<ob::VideoFrame> video_frame;
  if (frame->type() == OB_FRAME_COLOR) {
    video_frame = frame->as<ob::ColorFrame>();
  } else if (frame->type() == OB_FRAME_DEPTH) {
    video_frame = frame->as<ob::DepthFrame>();
  } else if (frame->type() == OB_FRAME_IR || frame->type() == OB_FRAME_IR_LEFT ||
             frame->type() == OB_FRAME_IR_RIGHT) {
    video_frame = frame->as<ob::IRFrame>();
  } else {
    RCLCPP_ERROR(logger_, "Unsupported frame type: %d", frame->type());
    return;
  }
  if (!video_frame) {
    RCLCPP_ERROR(logger_, "Failed to convert frame to video frame");
    return;
  }
  int width = static_cast<int>(video_frame->width());
  int height = static_cast<int>(video_frame->height());
  auto frame_timestamp = getFrameTimestampUs(frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  auto pid = device_info->pid();
  OBCameraIntrinsic intrinsic;
  OBCameraDistortion distortion;
  if (isGemini335PID(pid)) {
    auto stream_profile = frame->getStreamProfile();
    CHECK_NOTNULL(stream_profile);
    auto video_stream_profile = stream_profile->as<ob::VideoStreamProfile>();
    CHECK_NOTNULL(video_stream_profile);
    intrinsic = video_stream_profile->getIntrinsic();
    distortion = video_stream_profile->getDistortion();
  } else {
    auto camera_params = pipeline_->getCameraParam();
    intrinsic = stream_index.first == OB_STREAM_COLOR ? camera_params.rgbIntrinsic
                                                      : camera_params.depthIntrinsic;
    distortion = stream_index.first == OB_STREAM_COLOR ? camera_params.rgbDistortion
                                                       : camera_params.depthDistortion;
    if (pid == DABAI_MAX_PID) {
      // use color param
      intrinsic = camera_params.rgbIntrinsic;
      distortion = camera_params.rgbDistortion;
    }
  }
  std::string frame_id = optical_frame_id_[stream_index];
  if (depth_registration_ && stream_index == DEPTH) {
    frame_id = depth_aligned_frame_id_[stream_index];
  }
  if (stream_index == COLOR && default_config_.enable_color_undistortion_) {
    memset(&distortion, 0, sizeof(distortion));
  }
  sensor_msgs::msg::CameraInfo camera_info{};

  if (color_info_manager_ && color_info_manager_->isCalibrated() && stream_index == COLOR) {
    camera_info = color_info_manager_->getCameraInfo();
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info.width = width;
    camera_info.height = height;
  } else if (ir_info_manager_ && ir_info_manager_->isCalibrated() &&
             (stream_index == INFRA1 || stream_index == INFRA2 || stream_index == DEPTH)) {
    camera_info = ir_info_manager_->getCameraInfo();
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info.width = width;
    camera_info.height = height;
  } else {
    camera_info = convertToCameraInfo(intrinsic, distortion, width);
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info.width = width;
    camera_info.height = height;
  }
  // if (frame->type() == OB_FRAME_IR_RIGHT && enable_stream_[INFRA1]) {
  //   auto stream_profile = frame->getStreamProfile();
  //   CHECK_NOTNULL(stream_profile);
  //   auto video_stream_profile = stream_profile->as<ob::VideoStreamProfile>();
  //   CHECK_NOTNULL(video_stream_profile);
  //   auto left_video_profile = stream_profile_[INFRA1]->as<ob::VideoStreamProfile>();
  //   CHECK_NOTNULL(left_video_profile);
  //   auto ex = video_stream_profile->getExtrinsicTo(left_video_profile);
  //   float fx = camera_info.k.at(0);
  //   float fy = camera_info.k.at(4);
  //   camera_info.p.at(3) = -fx * ex.trans[0] / 1000.0 + 0.0;
  //   camera_info.p.at(7) = -fy * ex.trans[1] / 1000.0 + 0.0;
  // }
  CHECK(camera_info_publishers_.count(stream_index) > 0);
  if (flip_stream_[stream_index]) {
    // We are performing a horizontal flip (left-right mirror) of the image.
    //
    // After flipping the image, the camera's principal point (cx) in the
    // camera_info must be adjusted so that it still refers to the correct point
    // in the flipped image.

    // Intrinsic matrix K:
    // K = [ fx  0  cx
    //       0   fy cy
    //       0   0   1 ]
    double &cx = camera_info.k[2];  // K[0,2]

    // Store the original principal point cx
    double old_cx = cx;

    // For a horizontal flip, the new cx = (width - 1) - old_cx
    // This effectively mirrors the cx value about the center of the image.
    cx = (width - 1) - old_cx;

    // Update the projection matrix P:
    // P = [ fx  0   cx  Tx
    //       0   fy  cy  Ty
    //       0   0   1   0 ]
    double &p_cx = camera_info.p[2];

    double old_p_cx = p_cx;
    p_cx = (width - 1) - old_p_cx;

    // fx, fy, cy remain the same for a simple horizontal flip.
    // The only changes are cx and p_cx.
  }

  // camera_info_publishers_[stream_index]->publish(camera_info);
  if (isGemini335PID(pid)) {
    publishMetadata(frame, stream_index, camera_info.header);
  }
  // CHECK_NOTNULL(image_publishers_[stream_index]);
  // if (image_publishers_[stream_index]->get_subscription_count() == 0) {
  //   return;
  // }
  auto &image = images_[stream_index];
  if (image.empty() || image.cols != width || image.rows != height) {
    image.create(height, width, image_format_[stream_index]);
  }
  if (frame->type() == OB_FRAME_COLOR && !is_color_frame_decoded_) {
    RCLCPP_ERROR(logger_, "color frame is not decoded. Frame type: %d, decoded status: %s",
                 OB_FRAME_COLOR, is_color_frame_decoded_ ? "true" : "false");
    return;
  }
  if (frame->type() == OB_FRAME_COLOR) {
    memcpy(image.data, rgb_buffer_, video_frame->width() * video_frame->height() * 3);
  } else {
    memcpy(image.data, video_frame->data(), video_frame->dataSize());
  }
  if (stream_index == DEPTH) {
    auto depth_scale = video_frame->as<ob::DepthFrame>()->getValueScale();
    image = image * depth_scale;
  }
  if (flip_stream_[stream_index]) {
    // flip image
    cv::flip(image, image, 1);
  }
  sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

  cv_bridge::CvImage(std_msgs::msg::Header(), encoding_[stream_index], image)
      .toImageMsg(*image_msg);
  CHECK_NOTNULL(image_msg.get());
  image_msg->header.stamp = timestamp;
  image_msg->is_bigendian = false;
  image_msg->step = width * unit_step_size_[stream_index];
  image_msg->header.frame_id = frame_id;
  CHECK(image_publishers_.count(stream_index) > 0);
  // saveImageToFile(stream_index, image, *image_msg);
  image_publishers_[stream_index]->publish(std::move(image_msg));
  if (stream_index == COLOR && default_config_.enable_color_undistortion_ &&
      color_undistortion_publisher_->get_subscription_count() > 0) {
    auto undistorted_image = undistortImage(image, intrinsic, distortion);
    sensor_msgs::msg::Image::UniquePtr undistorted_image_msg(new sensor_msgs::msg::Image());
    cv_bridge::CvImage(std_msgs::msg::Header(), encoding_[stream_index], undistorted_image)
        .toImageMsg(*undistorted_image_msg);
    CHECK_NOTNULL(undistorted_image_msg.get());
    undistorted_image_msg->header.stamp = timestamp;
    undistorted_image_msg->is_bigendian = false;
    undistorted_image_msg->step = width * unit_step_size_[stream_index];
    undistorted_image_msg->header.frame_id = frame_id;
    color_undistortion_publisher_->publish(std::move(undistorted_image_msg));
  }
}

// check
void OBCameraNode::publishMetadata(const std::shared_ptr<ob::Frame> &frame,
                                   const stream_index_pair &stream_index,
                                   const std_msgs::msg::Header &header) {
  if (metadata_publishers_.count(stream_index) == 0) {
    return;
  }
  auto metadata_publisher = metadata_publishers_[stream_index];
  if (metadata_publisher->get_subscription_count() == 0) {
    return;
  }
  orbbec_camera_msgs::msg::Metadata metadata_msg;
  metadata_msg.header = header;
  nlohmann::json json_data;

  for (int i = 0; i < OB_FRAME_METADATA_TYPE_COUNT; i++) {
    auto meta_data_type = static_cast<OBFrameMetadataType>(i);
    std::string field_name = metaDataTypeToString(meta_data_type);
    if (!frame->hasMetadata(meta_data_type)) {
      continue;
    }
    int64_t value = frame->getMetadataValue(meta_data_type);
    json_data[field_name] = value;
  }
  metadata_msg.json_data = json_data.dump(2);
  // metadata_publisher->publish(metadata_msg);
}

// check
void OBCameraNode::onNewIMUFrameSyncOutputCallback(const std::shared_ptr<ob::Frame> &accelframe,
                                                   const std::shared_ptr<ob::Frame> &gryoframe) {
  if (!is_camera_node_initialized_) {
    return;
  }
  if (!imu_gyro_accel_publisher_) {
    RCLCPP_ERROR_STREAM(logger_, "stream Accel Gryo publisher not initialized");
    return;
  }
  bool has_subscriber = imu_gyro_accel_publisher_->get_subscription_count() > 0;
  has_subscriber = has_subscriber || imu_info_publishers_[GYRO]->get_subscription_count() > 0;
  has_subscriber = has_subscriber || imu_info_publishers_[ACCEL]->get_subscription_count() > 0;
  if (!has_subscriber) {
    return;
  }
  auto imu_msg = sensor_msgs::msg::Imu();
  setDefaultIMUMessage(imu_msg);

  imu_msg.header.frame_id = optical_frame_id_[GYRO];
  auto frame_timestamp = getFrameTimestampUs(accelframe);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  imu_msg.header.stamp = timestamp;

  auto gyro_info = createIMUInfo(GYRO);
  gyro_info.header = imu_msg.header;
  imu_info_publishers_[GYRO]->publish(gyro_info);

  auto accel_info = createIMUInfo(ACCEL);
  imu_msg.header.frame_id = optical_frame_id_[ACCEL];
  accel_info.header = imu_msg.header;
  imu_info_publishers_[ACCEL]->publish(accel_info);

  imu_msg.header.frame_id = accel_gyro_frame_id_;

  auto gyro_frame = gryoframe->as<ob::GyroFrame>();
  auto gyroData = gyro_frame->value();
  imu_msg.angular_velocity.x = gyroData.x - gyro_info.bias[0];
  imu_msg.angular_velocity.y = gyroData.y - gyro_info.bias[1];
  imu_msg.angular_velocity.z = gyroData.z - gyro_info.bias[2];

  auto accel_frame = accelframe->as<ob::AccelFrame>();
  auto accelData = accel_frame->value();

  imu_msg.linear_acceleration.x = accelData.x - accel_info.bias[0];
  imu_msg.linear_acceleration.y = accelData.y - accel_info.bias[1];
  imu_msg.linear_acceleration.z = accelData.z - accel_info.bias[2];

  imu_gyro_accel_publisher_->publish(imu_msg);
}

// check
void OBCameraNode::onNewIMUFrameCallback(const std::shared_ptr<ob::Frame> &frame,
                                         const stream_index_pair &stream_index) {
  if (!is_camera_node_initialized_) {
    return;
  }
  if (!imu_publishers_.count(stream_index)) {
    RCLCPP_ERROR_STREAM(logger_,
                        "stream " << stream_name_[stream_index] << " publisher not initialized");
    return;
  }
  bool has_subscriber = imu_publishers_[stream_index]->get_subscription_count() > 0;
  has_subscriber =
      has_subscriber || imu_info_publishers_[stream_index]->get_subscription_count() > 0;
  if (!has_subscriber) {
    return;
  }

  auto imu_msg = sensor_msgs::msg::Imu();
  setDefaultIMUMessage(imu_msg);

  imu_msg.header.frame_id = optical_frame_id_[stream_index];
  auto timestamp = fromUsToROSTime(frame->timeStampUs());
  imu_msg.header.stamp = timestamp;

  auto imu_info = createIMUInfo(stream_index);
  imu_info.header = imu_msg.header;
  imu_info_publishers_[stream_index]->publish(imu_info);

  if (frame->type() == OB_FRAME_GYRO) {
    auto gyro_frame = frame->as<ob::GyroFrame>();
    auto data = gyro_frame->value();
    imu_msg.angular_velocity.x = data.x - imu_info.bias[0];
    imu_msg.angular_velocity.y = data.y - imu_info.bias[1];
    imu_msg.angular_velocity.z = data.z - imu_info.bias[2];
  } else if (frame->type() == OB_FRAME_ACCEL) {
    auto accel_frame = frame->as<ob::AccelFrame>();
    auto data = accel_frame->value();
    imu_msg.linear_acceleration.x = data.x - imu_info.bias[0];
    imu_msg.linear_acceleration.y = data.y - imu_info.bias[1];
    imu_msg.linear_acceleration.z = data.z - imu_info.bias[2];
  } else {
    RCLCPP_ERROR(logger_, "Unsupported IMU frame type");
    return;
  }
  imu_publishers_[stream_index]->publish(imu_msg);
}

// check
void OBCameraNode::setDefaultIMUMessage(sensor_msgs::msg::Imu &imu_msg) {
  imu_msg.header.frame_id = "imu_link";
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  imu_msg.linear_acceleration_covariance = {
      default_config_.linear_accel_cov_, 0.0, 0.0, 0.0, default_config_.linear_accel_cov_, 0.0, 0.0, 0.0, default_config_.linear_accel_cov_};
  imu_msg.angular_velocity_covariance = {
      default_config_.angular_vel_cov_, 0.0, 0.0, 0.0, default_config_.angular_vel_cov_, 0.0, 0.0, 0.0, default_config_.angular_vel_cov_};
}

// check
sensor_msgs::msg::Imu OBCameraNode::createUnitIMUMessage(const IMUData &accel_data,
                                                         const IMUData &gyro_data) {
  sensor_msgs::msg::Imu imu_msg;
  rclcpp::Time timestamp(gyro_data.timestamp_);
  imu_msg.header.stamp = timestamp;
  imu_msg.angular_velocity.x = gyro_data.data_.x();
  imu_msg.angular_velocity.y = gyro_data.data_.y();
  imu_msg.angular_velocity.z = gyro_data.data_.z();

  imu_msg.linear_acceleration.x = accel_data.data_.x();
  imu_msg.linear_acceleration.y = accel_data.data_.y();
  imu_msg.linear_acceleration.z = accel_data.data_.z();
  return imu_msg;
}

// check
template <typename T>
T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

// check
void OBCameraNode::FillImuDataLinearInterpolation(const IMUData &imu_data,
                                                  std::deque<sensor_msgs::msg::Imu> &imu_msgs) {
  imu_history_.push_back(imu_data);
  stream_index_pair steam_index(imu_data.stream_);
  imu_msgs.clear();
  std::deque<IMUData> gyros_data;
  IMUData accel0, accel1, current_imu;
  while (!imu_history_.empty()) {
    current_imu = imu_history_.front();
    imu_history_.pop_front();
    if (accel0.isSet() && current_imu.stream_ == ACCEL) {
      accel0 = current_imu;
    } else if (accel0.isSet() && current_imu.stream_ == ACCEL) {
      accel1 = current_imu;
      const double dt = accel1.timestamp_ - accel0.timestamp_;
      while (!gyros_data.empty()) {
        auto current_gyro = gyros_data.front();
        gyros_data.pop_front();
        const double alpha = (current_gyro.timestamp_ - accel0.timestamp_) / dt;
        IMUData current_accel(ACCEL, lerp(accel0.data_, accel1.data_, alpha),
                              current_gyro.timestamp_);
        imu_msgs.push_back((createUnitIMUMessage(current_accel, current_gyro)));
      }
      accel0 = accel1;
    } else if (accel0.isSet() && current_imu.timestamp_ >= accel0.timestamp_ &&
               current_imu.stream_ == GYRO) {
      gyros_data.push_back(current_imu);
    }
  }
  imu_history_.push_back(current_imu);
}

// check
void OBCameraNode::FillImuDataCopy(const IMUData &imu_data,
                                   std::deque<sensor_msgs::msg::Imu> &imu_msgs) {
  stream_index_pair steam_index(imu_data.stream_);
  if (steam_index == ACCEL) {
    accel_data_ = imu_data;
    return;
  }
  if (accel_data_.isSet()) {
    return;
  }
  imu_msgs.push_back(createUnitIMUMessage(accel_data_, imu_data));
}

// check
bool OBCameraNode::setupFormatConvertType(OBFormat format) {
  switch (format) {
    case OB_FORMAT_RGB888:
      return true;
    case OB_FORMAT_I420:
      format_convert_filter_.setFormatConvertType(FORMAT_I420_TO_RGB888);
      break;
    case OB_FORMAT_MJPG:
      format_convert_filter_.setFormatConvertType(FORMAT_MJPEG_TO_RGB888);
      break;
    case OB_FORMAT_YUYV:
      format_convert_filter_.setFormatConvertType(FORMAT_YUYV_TO_RGB888);
      break;
    case OB_FORMAT_NV21:
      format_convert_filter_.setFormatConvertType(FORMAT_NV21_TO_RGB888);
      break;
    case OB_FORMAT_NV12:
      format_convert_filter_.setFormatConvertType(FORMAT_NV12_TO_RGB888);
      break;
    case OB_FORMAT_UYVY:
      format_convert_filter_.setFormatConvertType(FORMAT_UYVY_TO_RGB888);
      break;
    default:
      return false;
  }
  return true;
}

// check
bool OBCameraNode::isGemini335PID(uint32_t pid) {
  return pid == GEMINI_335_PID || pid == GEMINI_330_PID || pid == GEMINI_336_PID ||
         pid == GEMINI_335L_PID || pid == GEMINI_330L_PID || pid == GEMINI_336L_PID ||
         pid == GEMINI_335LG_PID || pid == GEMINI_336LG_PID || pid == GEMINI_335LE_PID ||
         pid == GEMINI_336LE_PID;
}

// check
orbbec_camera_msgs::msg::IMUInfo OBCameraNode::createIMUInfo(
    const stream_index_pair &stream_index) {
  orbbec_camera_msgs::msg::IMUInfo imu_info;
  imu_info.header.frame_id = optical_frame_id_[stream_index];
  imu_info.header.stamp = node_->now();
  auto imu_profile = stream_profile_[stream_index];
  if (stream_index == GYRO) {
    auto gyro_profile = stream_profile_[stream_index]->as<ob::GyroStreamProfile>();
    auto gyro_intrinsics = gyro_profile->getIntrinsic();
    imu_info.noise_density = gyro_intrinsics.noiseDensity;
    imu_info.random_walk = gyro_intrinsics.randomWalk;
    imu_info.reference_temperature = gyro_intrinsics.referenceTemp;
    imu_info.bias = {gyro_intrinsics.bias[0], gyro_intrinsics.bias[1], gyro_intrinsics.bias[2]};
    imu_info.scale_misalignment = {
        gyro_intrinsics.scaleMisalignment[0], gyro_intrinsics.scaleMisalignment[1],
        gyro_intrinsics.scaleMisalignment[2], gyro_intrinsics.scaleMisalignment[3],
        gyro_intrinsics.scaleMisalignment[4], gyro_intrinsics.scaleMisalignment[5],
        gyro_intrinsics.scaleMisalignment[6], gyro_intrinsics.scaleMisalignment[7],
        gyro_intrinsics.scaleMisalignment[8]};
    imu_info.temperature_slope = {
        gyro_intrinsics.tempSlope[0], gyro_intrinsics.tempSlope[1], gyro_intrinsics.tempSlope[2],
        gyro_intrinsics.tempSlope[3], gyro_intrinsics.tempSlope[4], gyro_intrinsics.tempSlope[5],
        gyro_intrinsics.tempSlope[6], gyro_intrinsics.tempSlope[7], gyro_intrinsics.tempSlope[8]};
  } else if (stream_index == ACCEL) {
    auto accel_profile = stream_profile_[stream_index]->as<ob::AccelStreamProfile>();
    auto accel_intrinsics = accel_profile->getIntrinsic();
    imu_info.noise_density = accel_intrinsics.noiseDensity;
    imu_info.random_walk = accel_intrinsics.randomWalk;
    imu_info.reference_temperature = accel_intrinsics.referenceTemp;
    imu_info.bias = {accel_intrinsics.bias[0], accel_intrinsics.bias[1], accel_intrinsics.bias[2]};
    imu_info.gravity = {accel_intrinsics.gravity[0], accel_intrinsics.gravity[1],
                        accel_intrinsics.gravity[2]};
    imu_info.scale_misalignment = {
        accel_intrinsics.scaleMisalignment[0], accel_intrinsics.scaleMisalignment[1],
        accel_intrinsics.scaleMisalignment[2], accel_intrinsics.scaleMisalignment[3],
        accel_intrinsics.scaleMisalignment[4], accel_intrinsics.scaleMisalignment[5],
        accel_intrinsics.scaleMisalignment[6], accel_intrinsics.scaleMisalignment[7],
        accel_intrinsics.scaleMisalignment[8]};
    imu_info.temperature_slope = {accel_intrinsics.tempSlope[0], accel_intrinsics.tempSlope[1],
                                  accel_intrinsics.tempSlope[2], accel_intrinsics.tempSlope[3],
                                  accel_intrinsics.tempSlope[4], accel_intrinsics.tempSlope[5],
                                  accel_intrinsics.tempSlope[6], accel_intrinsics.tempSlope[7],
                                  accel_intrinsics.tempSlope[8]};
  }

  return imu_info;
}

}  // namespace orbbec_camera
