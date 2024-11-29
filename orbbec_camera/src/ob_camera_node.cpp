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
  compression_params_.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params_.push_back(0);
  compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY);
  compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
  setupDefaultImageFormat();
  setupTopics();
#if defined(USE_RK_HW_DECODER)
  jpeg_decoder_ = std::make_unique<RKJPEGDecoder>(width_[COLOR], height_[COLOR]);
#elif defined(USE_NV_HW_DECODER)
  jpeg_decoder_ = std::make_unique<JetsonNvJPEGDecoder>(width_[COLOR], height_[COLOR]);
#endif
  if (enable_d2c_viewer_) {
    auto rgb_qos = getRMWQosProfileFromString(image_qos_[COLOR]);
    auto depth_qos = getRMWQosProfileFromString(image_qos_[DEPTH]);
    d2c_viewer_ = std::make_unique<D2CViewer>(node_, rgb_qos, depth_qos);
  }
  if (enable_stream_[COLOR]) {
    rgb_buffer_ = new uint8_t[width_[COLOR] * height_[COLOR] * 4];
  }
  if (enable_colored_point_cloud_ && enable_stream_[DEPTH] && enable_stream_[COLOR]) {
    rgb_point_cloud_buffer_size_ = width_[COLOR] * height_[COLOR] * sizeof(OBColorPoint);
    rgb_point_cloud_buffer_ = new uint8_t[rgb_point_cloud_buffer_size_];
    xy_table_data_size_ = width_[DEPTH] * height_[DEPTH] * 2;
    xy_table_data_ = new float[xy_table_data_size_];
  }
  is_camera_node_initialized_ = true;
}

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
    RCLCPP_WARN_STREAM(logger_, "Reboot device DONE");
  }
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
  delete[] rgb_buffer_;
  rgb_buffer_ = nullptr;

  delete[] rgb_point_cloud_buffer_;
  rgb_point_cloud_buffer_ = nullptr;

  delete[] xy_table_data_;
  xy_table_data_ = nullptr;

  delete[] depth_xy_table_data_;
  depth_xy_table_data_ = nullptr;

  delete[] depth_point_cloud_buffer_;
  depth_point_cloud_buffer_ = nullptr;

  RCLCPP_WARN_STREAM(logger_, "Destroy ~OBCameraNode DONE");
}

void OBCameraNode::setupDevices() {
  auto sensor_list = device_->getSensorList();
  for (size_t i = 0; i < sensor_list->getCount(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profiles = sensor->getStreamProfileList();
    for (size_t j = 0; j < profiles->getCount(); j++) {
      auto profile = profiles->getProfile(j);
      stream_index_pair sip{profile->getType(), 0};
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
  if (retry_on_usb3_detection_failure_ &&
      device_->isPropertySupported(OB_PROP_DEVICE_USB3_REPEAT_IDENTIFY_BOOL,
                                   OB_PERMISSION_READ_WRITE)) {
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_DEVICE_USB3_REPEAT_IDENTIFY_BOOL,
                        retry_on_usb3_detection_failure_);
  }
  // if (device_->isPropertySupported(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL,
  //                                  OB_PERMISSION_READ_WRITE)) {
  //   TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL,
  //                       enable_noise_removal_filter_);
  // }
  if (device_->isPropertySupported(OB_PROP_HEARTBEAT_BOOL, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting heartbeat to " << (enable_heartbeat_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_HEARTBEAT_BOOL, enable_heartbeat_);
  }
  if (max_depth_limit_ > 0 &&
      device_->isPropertySupported(OB_PROP_MAX_DEPTH_INT, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting max depth limit to " << max_depth_limit_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_MAX_DEPTH_INT, max_depth_limit_);
  }
  if (min_depth_limit_ > 0 &&
      device_->isPropertySupported(OB_PROP_MIN_DEPTH_INT, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting min depth limit to " << min_depth_limit_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_MIN_DEPTH_INT, min_depth_limit_);
  }
  if (laser_energy_level_ != -1 &&
      device_->isPropertySupported(OB_PROP_LASER_ENERGY_LEVEL_INT, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting laser energy level to " << laser_energy_level_);
    auto range = device_->getIntPropertyRange(OB_PROP_LASER_ENERGY_LEVEL_INT);
    if (laser_energy_level_ < range.min || laser_energy_level_ > range.max) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Laser energy level is out of range " << range.min << " - " << range.max);
    } else {
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_LASER_ENERGY_LEVEL_INT, laser_energy_level_);
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
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_DISPARITY_TO_DEPTH_BOOL, enable_hardware_d2d_);
    bool is_hardware_d2d = device_->getBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL);
    std::string d2d_mode = is_hardware_d2d ? "HW D2D" : "SW D2D";
    RCLCPP_INFO_STREAM(logger_, "Depth process is " << d2d_mode);
  }
  if (device_->isPropertySupported(OB_PROP_LDP_BOOL, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting LDP to " << (enable_ldp_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_LDP_BOOL, enable_ldp_);
  }
  if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting laser control to " << enable_laser_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_LASER_CONTROL_INT, enable_laser_);
  }
  if (device_->isPropertySupported(OB_PROP_LASER_ON_OFF_MODE_INT, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting laser on off mode to " << laser_on_off_mode_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_LASER_ON_OFF_MODE_INT, laser_on_off_mode_);
  }
  if (!device_preset_.empty()) {
    try {
      RCLCPP_INFO_STREAM(logger_, "Available presets:");
      auto preset_list = device_->getAvailablePresetList();
      for (uint32_t i = 0; i < preset_list->getCount(); i++) {
        RCLCPP_INFO_STREAM(logger_, "Preset " << i << ": " << preset_list->getName(i));
      }
      RCLCPP_INFO_STREAM(logger_, "Load device preset: " << device_preset_);
      TRY_EXECUTE_BLOCK(device_->loadPreset(device_preset_.c_str()));
      RCLCPP_INFO_STREAM(logger_, "Device preset " << device_->getCurrentPresetName() << " loaded");
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to load device preset: " << e.getMessage());
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to load device preset: " << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to load device preset");
    }
  }
  if (!depth_work_mode_.empty()) {
    RCLCPP_INFO_STREAM(logger_, "Set depth work mode: " << depth_work_mode_);
    TRY_EXECUTE_BLOCK(device_->switchDepthWorkMode(depth_work_mode_.c_str()));
  }
  if (!sync_mode_str_.empty() && device_->isPropertySupported(OB_PROP_SYNC_SIGNAL_TRIGGER_OUT_BOOL,
                                                              OB_PERMISSION_READ_WRITE)) {
    auto sync_config = device_->getMultiDeviceSyncConfig();
    RCLCPP_INFO_STREAM(logger_,
                       "Current sync mode: " << magic_enum::enum_name(sync_config.syncMode));
    std::transform(sync_mode_str_.begin(), sync_mode_str_.end(), sync_mode_str_.begin(), ::toupper);
    sync_mode_ = OBSyncModeFromString(sync_mode_str_);
    sync_config.syncMode = sync_mode_;
    sync_config.depthDelayUs = depth_delay_us_;
    sync_config.colorDelayUs = color_delay_us_;
    sync_config.trigger2ImageDelayUs = trigger2image_delay_us_;
    sync_config.triggerOutDelayUs = trigger_out_delay_us_;
    sync_config.triggerOutEnable = trigger_out_enabled_;
    sync_config.framesPerTrigger = frames_per_trigger_;
    TRY_EXECUTE_BLOCK(device_->setMultiDeviceSyncConfig(sync_config));
    sync_config = device_->getMultiDeviceSyncConfig();
    RCLCPP_INFO_STREAM(logger_, "Set sync mode: " << magic_enum::enum_name(sync_config.syncMode));
    if (sync_mode_ == OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING) {
      RCLCPP_INFO_STREAM(logger_, "Frames per trigger: " << sync_config.framesPerTrigger);
      RCLCPP_INFO_STREAM(logger_,
                         "Software trigger period " << software_trigger_period_.count() << " ms");
      software_trigger_timer_ = node_->create_wall_timer(
          software_trigger_period_, [this]() { TRY_EXECUTE_BLOCK(device_->triggerCapture()); });
    }
  }

  if (device_->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_READ_WRITE) &&
      !depth_precision_str_.empty()) {
    auto default_precision_level = device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
    if (default_precision_level != depth_precision_) {
      device_->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, depth_precision_);
      RCLCPP_INFO_STREAM(logger_, "set depth precision to " << depth_precision_str_);
    }
  } else if (device_->isPropertySupported(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT,
                                          OB_PERMISSION_READ_WRITE) &&
             !depth_precision_str_.empty()) {
    auto depth_unit_flexible_adjustment = depthPrecisionFromString(depth_precision_str_);
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

  for (const auto &stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      OBPropertyID mirrorPropertyID = OB_PROP_DEPTH_MIRROR_BOOL;
      if (stream_index == COLOR) {
        mirrorPropertyID = OB_PROP_COLOR_MIRROR_BOOL;
      } else if (stream_index == DEPTH) {
        mirrorPropertyID = OB_PROP_DEPTH_MIRROR_BOOL;
      } else if (stream_index == INFRA0) {
        mirrorPropertyID = OB_PROP_IR_MIRROR_BOOL;

      } else if (stream_index == INFRA1) {
        mirrorPropertyID = OB_PROP_IR_MIRROR_BOOL;
      } else if (stream_index == INFRA2) {
        mirrorPropertyID = OB_PROP_IR_RIGHT_MIRROR_BOOL;
      }

      if (device_->isPropertySupported(mirrorPropertyID, OB_PERMISSION_WRITE)) {
        RCLCPP_INFO_STREAM(logger_, "Setting " << stream_name_[stream_index] << " mirror to "
                                               << (flip_stream_[stream_index] ? "ON" : "OFF"));
        TRY_TO_SET_PROPERTY(setBoolProperty, mirrorPropertyID, flip_stream_[stream_index]);
      }
    }
  }

  if (device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE)) {
    device_->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL, enable_noise_removal_filter_);
  }

  if (device_->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(
        logger_, "Setting color auto exposure to " << (enable_color_auto_exposure_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL,
                        enable_color_auto_exposure_);
  }
  if (device_->isPropertySupported(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color auto white balance to "
                                    << (enable_color_auto_white_balance_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL,
                        enable_color_auto_white_balance_);
  }
  if (color_exposure_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, false);
    auto range = device_->getIntPropertyRange(OB_PROP_COLOR_EXPOSURE_INT);
    if (color_exposure_ < range.min || color_exposure_ > range.max) {
      RCLCPP_ERROR(logger_, "color exposure value is out of range[%d,%d], please check the value",
                   range.min, range.max);
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting color exposure to " << color_exposure_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_EXPOSURE_INT, color_exposure_);
    }
  }
  if (color_gain_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_GAIN_INT, OB_PERMISSION_WRITE)) {
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, false);
    auto range = device_->getIntPropertyRange(OB_PROP_COLOR_GAIN_INT);
    if (color_gain_ < range.min || color_gain_ > range.max) {
      RCLCPP_ERROR(logger_, "color gain value is out of range[%d,%d], please check the value",
                   range.min, range.max);
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting color gain to " << color_gain_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_GAIN_INT, color_gain_);
    }
  }
  if (color_white_balance_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_WHITE_BALANCE_INT, OB_PERMISSION_WRITE)) {
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, false);
    auto range = device_->getIntPropertyRange(OB_PROP_COLOR_WHITE_BALANCE_INT);
    if (color_white_balance_ < range.min || color_white_balance_ > range.max) {
      RCLCPP_ERROR(logger_,
                   "color white balance value is out of range[%d,%d], please check the value",
                   range.min, range.max);
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting color white balance to " << color_white_balance_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_WHITE_BALANCE_INT, color_white_balance_);
    }
  }

  if (color_ae_max_exposure_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_AE_MAX_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color AE max exposure to " << color_ae_max_exposure_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_AE_MAX_EXPOSURE_INT, color_ae_max_exposure_);
  }
  if (color_brightness_ != -1 &&
      device_->isPropertySupported(OB_PROP_COLOR_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting color brightness to " << color_brightness_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_COLOR_BRIGHTNESS_INT, color_brightness_);
  }
  // ir ae max
  if (ir_ae_max_exposure_ != -1 &&
      device_->isPropertySupported(OB_PROP_IR_AE_MAX_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting IR AE max exposure to " << ir_ae_max_exposure_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_IR_AE_MAX_EXPOSURE_INT, ir_ae_max_exposure_);
  }
  // ir brightness
  if (ir_brightness_ != -1 &&
      device_->isPropertySupported(OB_PROP_IR_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting IR brightness to " << ir_brightness_);
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_IR_BRIGHTNESS_INT, ir_brightness_);
  }

  if (device_->isPropertySupported(OB_PROP_IR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_,
                       "Setting IR auto exposure to " << (enable_ir_auto_exposure_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_IR_AUTO_EXPOSURE_BOOL, enable_ir_auto_exposure_);
  }
  if (ir_exposure_ != -1 &&
      device_->isPropertySupported(OB_PROP_IR_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_IR_AUTO_EXPOSURE_BOOL, false);
    auto range = device_->getIntPropertyRange(OB_PROP_IR_EXPOSURE_INT);
    if (ir_exposure_ < range.min || ir_exposure_ > range.max) {
      RCLCPP_ERROR(logger_, "ir exposure value is out of range[%d,%d], please check the value",
                   range.min, range.max);
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting IR exposure to " << ir_exposure_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_IR_EXPOSURE_INT, ir_exposure_);
    }
  }
  if (ir_gain_ != -1 && device_->isPropertySupported(OB_PROP_IR_GAIN_INT, OB_PERMISSION_WRITE)) {
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_IR_AUTO_EXPOSURE_BOOL, false);
    auto range = device_->getIntPropertyRange(OB_PROP_IR_GAIN_INT);
    if (ir_gain_ < range.min || ir_gain_ > range.max) {
      RCLCPP_ERROR(logger_, "ir gain value is out of range[%d,%d], please check the value",
                   range.min, range.max);
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting IR gain to " << ir_gain_);
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_IR_GAIN_INT, ir_gain_);
    }
  }

  if (device_->isPropertySupported(OB_PROP_IR_LONG_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
    RCLCPP_INFO_STREAM(logger_,
                       "Setting IR long exposure to " << (enable_ir_long_exposure_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_IR_LONG_EXPOSURE_BOOL, enable_ir_long_exposure_);
  }

  if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE)) {
    auto default_soft_filter_max_diff = device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
    RCLCPP_INFO_STREAM(logger_, "default_soft_filter_max_diff: " << default_soft_filter_max_diff);
    if (soft_filter_max_diff_ != -1 && default_soft_filter_max_diff != soft_filter_max_diff_) {
      device_->setIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT, soft_filter_max_diff_);
      auto new_soft_filter_max_diff = device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
      RCLCPP_INFO_STREAM(logger_, "after set soft_filter_max_diff: " << new_soft_filter_max_diff);
    }
  }

  if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, OB_PERMISSION_WRITE)) {
    auto default_soft_filter_speckle_size =
        device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
    RCLCPP_INFO_STREAM(logger_,
                       "default_soft_filter_speckle_size: " << default_soft_filter_speckle_size);
    if (soft_filter_speckle_size_ != -1 &&
        default_soft_filter_speckle_size != soft_filter_speckle_size_) {
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT,
                          soft_filter_speckle_size_);
      auto new_soft_filter_speckle_size =
          device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
      RCLCPP_INFO_STREAM(logger_,
                         "after set soft_filter_speckle_size: " << new_soft_filter_speckle_size);
    }
  }
}

void OBCameraNode::setupDepthPostProcessFilter() {
  auto depth_sensor = device_->getSensor(OB_SENSOR_DEPTH);
  // set depth sensor to filter
  filter_list_ = depth_sensor->createRecommendedFilters();
  if (!filter_list_.empty()) {
    // RCLCPP_ERROR(logger_, "Failed to get depth sensor filter list");
    return;
  }
  for (size_t i = 0; i < filter_list_.size(); i++) {
    auto filter = filter_list_[i];
    std::map<std::string, bool> filter_params = {
        {"DecimationFilter", enable_decimation_filter_},
        {"HDRMerge", enable_hdr_merge_},
        {"SequencedFilter", enable_sequence_id_filter_},
        {"ThresholdFilter", enable_threshold_filter_},
        {"NoiseRemovalFilter", enable_noise_removal_filter_},
        {"SpatialAdvancedFilter", enable_spatial_filter_},
        {"TemporalFilter", enable_temporal_filter_},
        {"HoleFillingFilter", enable_hole_filling_filter_},

    };
    std::string filter_name = filter->type();
    RCLCPP_INFO_STREAM(logger_, "Setting " << filter_name << "......");
    if (filter_params.find(filter_name) != filter_params.end()) {
      std::string value = filter_params[filter_name] ? "true" : "false";
      RCLCPP_INFO_STREAM(logger_, "set " << filter_name << " to " << value);
      filter->enable(filter_params[filter_name]);
      filter_status_[filter_name] = filter_params[filter_name];
    }
    if (filter_name == "DecimationFilter" && enable_decimation_filter_) {
      auto decimation_filter = filter->as<ob::DecimationFilter>();
      auto range = decimation_filter->getScaleRange();
      if (decimation_filter_scale_ != -1 && decimation_filter_scale_ < range.max &&
          decimation_filter_scale_ > range.min) {
        RCLCPP_INFO_STREAM(logger_,
                           "Set decimation filter scale value to " << decimation_filter_scale_);
        decimation_filter->setScaleValue(decimation_filter_scale_);
      }
      if (decimation_filter_scale_ != -1 &&
          (decimation_filter_scale_ < range.min || decimation_filter_scale_ > range.max)) {
        RCLCPP_ERROR_STREAM(logger_, "Decimation filter scale value is out of range "
                                         << range.min << " - " << range.max);
      }
    } else if (filter_name == "ThresholdFilter" && enable_threshold_filter_) {
      auto threshold_filter = filter->as<ob::ThresholdFilter>();
      if (threshold_filter_min_ != -1 && threshold_filter_max_ != -1) {
        RCLCPP_INFO_STREAM(logger_, "Set threshold filter value range to "
                                        << threshold_filter_min_ << " - " << threshold_filter_max_);
        threshold_filter->setValueRange(threshold_filter_min_, threshold_filter_max_);
      }
    } else if (filter_name == "SpatialAdvancedFilter" && enable_spatial_filter_) {
      auto spatial_filter = filter->as<ob::SpatialAdvancedFilter>();
      if (spatial_filter_alpha_ != -1.0 && spatial_filter_magnitude_ != -1 &&
          spatial_filter_radius_ != -1 && spatial_filter_diff_threshold_ != -1) {
        OBSpatialAdvancedFilterParams params{};
        params.alpha = spatial_filter_alpha_;
        params.magnitude = spatial_filter_magnitude_;
        params.radius = spatial_filter_radius_;
        params.disp_diff = spatial_filter_diff_threshold_;
        spatial_filter->setFilterParams(params);
      }
    } else if (filter_name == "TemporalFilter" && enable_temporal_filter_) {
      auto temporal_filter = filter->as<ob::TemporalFilter>();
      if (temporal_filter_diff_threshold_ != -1.0 && temporal_filter_weight_ != -1.0) {
        RCLCPP_INFO_STREAM(logger_, "Set temporal filter value to "
                                        << temporal_filter_diff_threshold_ << " - "
                                        << temporal_filter_weight_);
        temporal_filter->setDiffScale(temporal_filter_diff_threshold_);
        temporal_filter->setWeight(temporal_filter_weight_);
      }
    } else if (filter_name == "HoleFillingFilter" && enable_hole_filling_filter_ &&
               !hole_filling_filter_mode_.empty()) {
      auto hole_filling_filter = filter->as<ob::HoleFillingFilter>();
      RCLCPP_INFO_STREAM(logger_,
                         "Default hole filling filter mode: " << hole_filling_filter_mode_);
      OBHoleFillingMode hole_filling_mode = holeFillingModeFromString(hole_filling_filter_mode_);
      hole_filling_filter->setFilterMode(hole_filling_mode);
    } else if (filter_name == "SequenceIdFilter" && enable_sequence_id_filter_) {
      auto sequenced_filter = filter->as<ob::SequenceIdFilter>();
      if (sequence_id_filter_id_ != -1) {
        sequenced_filter->selectSequenceId(sequence_id_filter_id_);
      }
    } else if (filter_name == "NoiseRemovalFilter" && enable_noise_removal_filter_) {
      auto noise_removal_filter = filter->as<ob::NoiseRemovalFilter>();
      OBNoiseRemovalFilterParams params = noise_removal_filter->getFilterParams();
      RCLCPP_INFO_STREAM(logger_, "Default noise removal filter params: "
                                      << "disp_diff: " << params.disp_diff
                                      << ", max_size: " << params.max_size);
      params.disp_diff = noise_removal_filter_min_diff_;
      params.max_size = noise_removal_filter_max_size_;
      RCLCPP_INFO_STREAM(logger_, "Set noise removal filter params: "
                                      << "disp_diff: " << params.disp_diff
                                      << ", max_size: " << params.max_size);
      if (noise_removal_filter_min_diff_ != -1 && noise_removal_filter_max_size_ != -1) {
        noise_removal_filter->setFilterParams(params);
      }
    } else if (filter_name == "HDRMerge" && enable_hdr_merge_) {
      if (hdr_merge_exposure_1_ != -1 && hdr_merge_gain_1_ != -1 && hdr_merge_exposure_2_ != -1 &&
          hdr_merge_gain_2_ != -1) {
        auto hdr_merge_filter = filter->as<ob::HdrMerge>();
        hdr_merge_filter->enable(true);
        RCLCPP_INFO_STREAM(logger_, "Set HDR merge filter params: "
                                        << "exposure_1: " << hdr_merge_exposure_1_
                                        << ", gain_1: " << hdr_merge_gain_1_
                                        << ", exposure_2: " << hdr_merge_exposure_2_
                                        << ", gain_2: " << hdr_merge_gain_2_);
        auto config = OBHdrConfig();
        config.enable = true;
        config.exposure_1 = hdr_merge_exposure_1_;
        config.gain_1 = hdr_merge_gain_1_;
        config.exposure_2 = hdr_merge_exposure_2_;
        config.gain_2 = hdr_merge_gain_2_;
        device_->setStructuredData(OB_STRUCT_DEPTH_HDR_CONFIG,
                                   reinterpret_cast<const uint8_t *>(&config), sizeof(config));
      }
    } else {
      RCLCPP_INFO_STREAM(logger_, "Skip setting filter: " << filter_name);
    }
  }
}

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

void OBCameraNode::printSensorProfiles(const std::shared_ptr<ob::Sensor> &sensor) {
  auto profiles = sensor->getStreamProfileList();
  for (size_t i = 0; i < profiles->getCount(); i++) {
    auto origin_profile = profiles->getProfile(i);
    if (sensor->getType() == OB_SENSOR_COLOR) {
      auto profile = origin_profile->as<ob::VideoStreamProfile>();
      RCLCPP_INFO_STREAM(
          logger_, "color profile: " << profile->getWidth() << "x" << profile->getHeight() << " "
                                     << profile->getFps() << "fps " << profile->getFormat());
    } else if (sensor->getType() == OB_SENSOR_DEPTH) {
      auto profile = origin_profile->as<ob::VideoStreamProfile>();
      RCLCPP_INFO_STREAM(
          logger_, "depth profile: " << profile->getWidth() << "x" << profile->getHeight() << " "
                                     << profile->getFps() << "fps " << profile->getFormat());
    } else if (sensor->getType() == OB_SENSOR_IR) {
      auto profile = origin_profile->as<ob::VideoStreamProfile>();
      RCLCPP_INFO_STREAM(logger_, "ir profile: " << profile->getWidth() << "x"
                                                 << profile->getHeight() << " " << profile->getFps()
                                                 << "fps " << profile->getFormat());
    } else if (sensor->getType() == OB_SENSOR_ACCEL) {
      auto profile = origin_profile->as<ob::AccelStreamProfile>();
      RCLCPP_INFO_STREAM(logger_, "accel profile: sampleRate " << profile->getSampleRate()
                                                               << "  full scale_range "
                                                               << profile->getFullScaleRange());
    } else if (sensor->getType() == OB_SENSOR_GYRO) {
      auto profile = origin_profile->as<ob::GyroStreamProfile>();
      RCLCPP_INFO_STREAM(logger_, "gyro profile: sampleRate " << profile->getSampleRate()
                                                              << "  full scale_range "
                                                              << profile->getFullScaleRange());
    } else {
      RCLCPP_INFO_STREAM(logger_, "unknown profile: " << magic_enum::enum_name(sensor->getType()));
    }
  }
}

void OBCameraNode::setupProfiles() {
  // Image stream
  for (const auto &elem : IMAGE_STREAMS) {
    if (enable_stream_[elem]) {
      const auto &sensor = sensors_[elem];
      CHECK_NOTNULL(sensor.get());
      auto profiles = sensor->getStreamProfileList();
      CHECK_NOTNULL(profiles.get());
      CHECK(profiles->getCount() > 0);
      for (size_t i = 0; i < profiles->getCount(); i++) {
        auto base_profile = profiles->getProfile(i)->as<ob::VideoStreamProfile>();
        if (base_profile == nullptr) {
          throw std::runtime_error("Failed to get profile " + std::to_string(i));
        }
        auto profile = base_profile->as<ob::VideoStreamProfile>();
        if (profile == nullptr) {
          throw std::runtime_error("Failed cast profile to VideoStreamProfile");
        }
        RCLCPP_DEBUG_STREAM(
            logger_, "Sensor profile: "
                         << "stream_type: " << magic_enum::enum_name(profile->getType())
                         << "Format: " << profile->getFormat() << ", Width: " << profile->getWidth()
                         << ", Height: " << profile->getHeight() << ", FPS: " << profile->getFps());
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
          RCLCPP_WARN_STREAM(logger_, "default FPS " << default_profile->getFps());
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
      height_[elem] = static_cast<int>(selected_profile->getHeight());
      width_[elem] = static_cast<int>(selected_profile->getWidth());
      fps_[elem] = static_cast<int>(selected_profile->getFps());
      format_[elem] = selected_profile->getFormat();
      updateImageConfig(elem);
      if (selected_profile->format() == OB_FORMAT_BGRA) {
        images_[elem] = cv::Mat(height_[elem], width_[elem], CV_8UC4, cv::Scalar(0, 0, 0, 0));
        encoding_[elem] = sensor_msgs::image_encodings::BGRA8;
        unit_step_size_[COLOR] = 4 * sizeof(uint8_t);
      } else if (selected_profile->format() == OB_FORMAT_RGBA) {
        images_[elem] = cv::Mat(height_[elem], width_[elem], CV_8UC4, cv::Scalar(0, 0, 0, 0));
        encoding_[elem] = sensor_msgs::image_encodings::RGBA8;
        unit_step_size_[COLOR] = 4 * sizeof(uint8_t);
      } else {
        images_[elem] =
            cv::Mat(height_[elem], width_[elem], image_format_[elem], cv::Scalar(0, 0, 0));
      }
      RCLCPP_INFO_STREAM(logger_,
                         " stream "
                             << stream_name_[elem]
                             << " is enabled - width: " << selected_profile->getWidth()
                             << ", height: " << selected_profile->getHeight()
                             << ", fps: " << selected_profile->getFps() << ", "
                             << "Format: " << magic_enum::enum_name(selected_profile->getFormat()));
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

int OBCameraNode::init_interleave_hdr_param() {
  device_->setIntProperty(OB_PROP_FRAME_INTERLEAVE_CONFIG_INDEX_INT, 1);
  device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, 1);
  device_->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, 1);
  device_->setIntProperty(OB_PROP_DEPTH_GAIN_INT, 16);
  device_->setIntProperty(OB_PROP_IR_BRIGHTNESS_INT, 20);
  device_->setIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT, 2000);

  // set interleaveae
  device_->setIntProperty(OB_PROP_FRAME_INTERLEAVE_CONFIG_INDEX_INT, 0);
  device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, 1);
  device_->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, 7500);
  device_->setIntProperty(OB_PROP_DEPTH_GAIN_INT, 16);
  device_->setIntProperty(OB_PROP_IR_BRIGHTNESS_INT, 60);
  device_->setIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT, 10000);
  return 0;
}

int OBCameraNode::init_interleave_laser_param() {
  device_->setIntProperty(OB_PROP_FRAME_INTERLEAVE_CONFIG_INDEX_INT, 1);
  device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, 0);
  device_->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, 3000);
  device_->setIntProperty(OB_PROP_DEPTH_GAIN_INT, 16);
  device_->setIntProperty(OB_PROP_IR_BRIGHTNESS_INT, 60);
  // device_->setIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT, 30000);
  device_->setIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT, 17000);

  // set interleaveae
  device_->setIntProperty(OB_PROP_FRAME_INTERLEAVE_CONFIG_INDEX_INT, 0);
  device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, 1);
  device_->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, 3000);
  device_->setIntProperty(OB_PROP_DEPTH_GAIN_INT, 16);
  device_->setIntProperty(OB_PROP_IR_BRIGHTNESS_INT, 60);
  device_->setIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT, 30000);
  return 0;
}

void OBCameraNode::startStreams() {
  if (pipeline_ != nullptr) {
    pipeline_.reset();
  }
  pipeline_ = std::make_unique<ob::Pipeline>(device_);

  try {
    setupPipelineConfig();

    // set interleave mode
    if (interleave_ae_mode_ == "hdr") {
      RCLCPP_INFO_STREAM(logger_, "Setting interleave mode to hdr");
      device_->loadFrameInterleave("hdr interleave");
      init_interleave_hdr_param();
    } else if (interleave_ae_mode_ == "laser") {
      RCLCPP_INFO_STREAM(logger_, "Setting interleave mode to laser");
      device_->loadFrameInterleave("laser interleave");
      init_interleave_laser_param();
    } else {
      RCLCPP_INFO_STREAM(logger_, "Setting interleave mode to nothing");
    }

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
  // enable interleave frame
  if ((interleave_ae_mode_ == "hdr") || (interleave_ae_mode_ == "laser")) {
    RCLCPP_INFO_STREAM(logger_, "current interleave_ae_mode_: " << interleave_ae_mode_);
    if (device_->isPropertySupported(OB_PROP_FRAME_INTERLEAVE_ENABLE_BOOL, OB_PERMISSION_WRITE)) {
      TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_FRAME_INTERLEAVE_ENABLE_BOOL,
                          interleave_frame_enable_);
      RCLCPP_INFO_STREAM(logger_, "Enable enable_interleave_depth_frame to "
                                      << (interleave_frame_enable_ ? "true" : "false"));
    }
  }
  // set interleave larse PATTERN_SYNC_DELAY
  if ((interleave_ae_mode_ == "laser") && interleave_frame_enable_ &&
      device_->isPropertySupported(OB_PROP_FRAME_INTERLEAVE_LASER_PATTERN_SYNC_DELAY_INT,
                                   OB_PERMISSION_READ_WRITE) &&
      (sync_mode_str_ == "PRIMARY" || sync_mode_str_ == "SOFTWARE_TRIGGERING")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_FRAME_INTERLEAVE_LASER_PATTERN_SYNC_DELAY_INT, 0);
    RCLCPP_INFO_STREAM(logger_, "Setting OB_PROP_FRAME_INTERLEAVE_LASER_PATTERN_SYNC_DELAY_INT 0 ");
  }
  pipeline_started_.store(true);
}

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

void OBCameraNode::stopStreams() {
  if (!pipeline_started_ || !pipeline_) {
    RCLCPP_INFO_STREAM(logger_, "pipeline not started or not exist, skip stop pipeline");
    return;
  }
  try {
    pipeline_->stop();

    // disable interleave frame
    if ((interleave_ae_mode_ == "hdr") || (interleave_ae_mode_ == "laser")) {
      RCLCPP_INFO_STREAM(logger_, "current interleave_ae_mode_: " << interleave_ae_mode_);
      if (device_->isPropertySupported(OB_PROP_FRAME_INTERLEAVE_ENABLE_BOOL, OB_PERMISSION_WRITE)) {
        interleave_frame_enable_ = false;
        RCLCPP_INFO_STREAM(logger_, "Enable enable_interleave_depth_frame to "
                                        << (interleave_frame_enable_ ? "true" : "false"));
        TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_FRAME_INTERLEAVE_ENABLE_BOOL,
                            interleave_frame_enable_);
      }
    }

  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to stop pipeline: " << e.getMessage());
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to stop pipeline");
  }
}

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

// cs_param_t rd_par = {0, 0}, param = {1, 3000}; //30
int OBCameraNode::openSocSyncPwmTrigger(uint16_t fps) {
  const char *devicePath = DEVICE_PATH;
  const int TRIGGER_MODE_ENABLE = 1;
  const int TRIGGER_MODE_DISABLE = 0;

  int ret = -1;
  cs_param_t param = {TRIGGER_MODE_ENABLE, fps};
  cs_param_t rd_par = {TRIGGER_MODE_DISABLE, 0};

  if (access(devicePath, F_OK) != 0) {
    std::cerr << "Device node " << devicePath << " does not exist." << std::endl;
    return ret;
  }
  gmsl_trigger_fd_ = open(DEVICE_PATH, O_RDWR);
  if (gmsl_trigger_fd_ < 0) {
    perror("open device failed\n");
    return gmsl_trigger_fd_;
  }

  std::cout << "Written param mode=" << param.mode << ", fps=" << param.fps << std::endl;
  ret = write(gmsl_trigger_fd_, &param, sizeof(param));
  if (ret < 0) {
    perror("write device failed\n");
    close(gmsl_trigger_fd_);
    return ret;
  }

  ret = read(gmsl_trigger_fd_, &rd_par, sizeof(rd_par));
  if (ret < 0) {
    perror("read device failed\n");
    close(gmsl_trigger_fd_);
    return ret;
  }
  std::cout << "Read param mode=" << rd_par.mode << ", fps=" << rd_par.fps << std::endl;

  std::cout << "Start hardware triggering..." << std::endl;

  return 0;
}
int OBCameraNode::closeSocSyncPwmTrigger() {
  if (gmsl_trigger_fd_ >= 0) {
    close(gmsl_trigger_fd_);
    gmsl_trigger_fd_ = -1;  // Reset file descriptors
    std::cout << "close camSync success" << std::endl;
    return 0;
  }
  return -1;
}

void OBCameraNode::startGmslTrigger() {
  if (gmsl_trigger_fps_ > 0 && enable_gmsl_trigger_) {
    RCLCPP_WARN_STREAM(logger_, "Start HardwareTrigger by soc-trigger-source. gmsl_trigger_fps_: "
                                    << gmsl_trigger_fps_);
    openSocSyncPwmTrigger(gmsl_trigger_fps_);
  } else {
    RCLCPP_WARN_STREAM(logger_,
                       "Start HardwareTrigger by soc-trigger-source. gmsl_trigger_fps_ illegal: "
                           << gmsl_trigger_fps_);
  }
}
void OBCameraNode::stopGmslTrigger() { closeSocSyncPwmTrigger(); }

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
    param_name = stream_name_[stream_index] + "_qos";
    setAndGetNodeParameter<std::string>(image_qos_[stream_index], param_name, "default");
    param_name = stream_name_[stream_index] + "_camera_info_qos";
    setAndGetNodeParameter<std::string>(camera_info_qos_[stream_index], param_name, "default");
  }

  for (auto stream_index : IMAGE_STREAMS) {
    depth_aligned_frame_id_[stream_index] = optical_frame_id_[COLOR];
  }

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

  setAndGetNodeParameter(publish_tf_, "publish_tf", true);
  setAndGetNodeParameter(tf_publish_rate_, "tf_publish_rate", 0.0);
  setAndGetNodeParameter(depth_registration_, "depth_registration", false);
  setAndGetNodeParameter(enable_point_cloud_, "enable_point_cloud", false);
  setAndGetNodeParameter<std::string>(ir_info_url_, "ir_info_url", "");
  setAndGetNodeParameter<std::string>(color_info_url_, "color_info_url", "");
  setAndGetNodeParameter(enable_colored_point_cloud_, "enable_colored_point_cloud", false);
  setAndGetNodeParameter(enable_point_cloud_, "enable_point_cloud", false);
  setAndGetNodeParameter<std::string>(point_cloud_qos_, "point_cloud_qos", "default");
  setAndGetNodeParameter(enable_d2c_viewer_, "enable_d2c_viewer", false);
  setAndGetNodeParameter(enable_hardware_d2d_, "enable_hardware_d2d", true);
  setAndGetNodeParameter(enable_soft_filter_, "enable_soft_filter", false);
  setAndGetNodeParameter<std::string>(depth_filter_config_, "depth_filter_config", "");
  if (!depth_filter_config_.empty()) {
    enable_depth_filter_ = true;
  }
  setAndGetNodeParameter(enable_frame_sync_, "enable_frame_sync", false);
  setAndGetNodeParameter(enable_color_auto_exposure_, "enable_color_auto_exposure", true);
  setAndGetNodeParameter(enable_color_auto_white_balance_, "enable_color_auto_white_balance", true);
  setAndGetNodeParameter<int>(color_exposure_, "color_exposure", -1);
  setAndGetNodeParameter<int>(color_gain_, "color_gain", -1);
  setAndGetNodeParameter<int>(color_white_balance_, "color_white_balance", -1);
  setAndGetNodeParameter<int>(color_ae_max_exposure_, "color_ae_max_exposure", -1);
  setAndGetNodeParameter<int>(color_brightness_, "color_brightness", -1);
  setAndGetNodeParameter(enable_ir_auto_exposure_, "enable_ir_auto_exposure", true);
  setAndGetNodeParameter<int>(ir_exposure_, "ir_exposure", -1);
  setAndGetNodeParameter<int>(ir_gain_, "ir_gain", -1);
  setAndGetNodeParameter<int>(ir_ae_max_exposure_, "ir_ae_max_exposure", -1);
  setAndGetNodeParameter<int>(ir_brightness_, "ir_brightness", -1);
  setAndGetNodeParameter(enable_ir_long_exposure_, "enable_ir_long_exposure", true);
  setAndGetNodeParameter<std::string>(depth_work_mode_, "depth_work_mode", "");
  setAndGetNodeParameter<std::string>(sync_mode_str_, "sync_mode", "");
  setAndGetNodeParameter(depth_delay_us_, "depth_delay_us", 0);
  setAndGetNodeParameter(color_delay_us_, "color_delay_us", 0);
  setAndGetNodeParameter(trigger2image_delay_us_, "trigger2image_delay_us", 0);
  setAndGetNodeParameter(trigger_out_delay_us_, "trigger_out_delay_us", 0);
  setAndGetNodeParameter(trigger_out_enabled_, "trigger_out_enabled", false);
  setAndGetNodeParameter<std::string>(depth_precision_str_, "depth_precision", "");
  setAndGetNodeParameter<std::string>(cloud_frame_id_, "cloud_frame_id", "");
  if (!depth_precision_str_.empty()) {
    depth_precision_ = depthPrecisionLevelFromString(depth_precision_str_);
  }
  if (enable_colored_point_cloud_ || enable_d2c_viewer_) {
    depth_registration_ = true;
  }
  if (!enable_stream_[COLOR]) {
    enable_colored_point_cloud_ = false;
    depth_registration_ = false;
  }
  setAndGetNodeParameter<bool>(enable_ldp_, "enable_ldp", true);
  setAndGetNodeParameter<int>(soft_filter_max_diff_, "soft_filter_max_diff", -1);
  setAndGetNodeParameter<int>(soft_filter_speckle_size_, "soft_filter_speckle_size", -1);
  setAndGetNodeParameter<double>(liner_accel_cov_, "linear_accel_cov", 0.0003);
  setAndGetNodeParameter<double>(angular_vel_cov_, "angular_vel_cov", 0.02);
  setAndGetNodeParameter<bool>(ordered_pc_, "ordered_pc", false);
  setAndGetNodeParameter<int>(max_save_images_count_, "max_save_images_count", 10);
  setAndGetNodeParameter<bool>(enable_depth_scale_, "enable_depth_scale", true);
  setAndGetNodeParameter<std::string>(device_preset_, "device_preset", "");
  setAndGetNodeParameter<bool>(enable_decimation_filter_, "enable_decimation_filter", false);
  setAndGetNodeParameter<bool>(enable_hdr_merge_, "enable_hdr_merge", false);
  setAndGetNodeParameter<bool>(enable_sequence_id_filter_, "enable_sequence_id_filter", false);
  setAndGetNodeParameter<bool>(enable_threshold_filter_, "enable_threshold_filter", false);
  setAndGetNodeParameter<bool>(enable_noise_removal_filter_, "enable_noise_removal_filter", true);
  setAndGetNodeParameter<bool>(enable_spatial_filter_, "enable_spatial_filter", false);
  setAndGetNodeParameter<bool>(enable_temporal_filter_, "enable_temporal_filter", false);
  setAndGetNodeParameter<bool>(enable_hole_filling_filter_, "enable_hole_filling_filter", false);
  setAndGetNodeParameter<int>(decimation_filter_scale_, "decimation_filter_scale_", -1);
  setAndGetNodeParameter<int>(sequence_id_filter_id_, "sequence_id_filter_id", -1);
  setAndGetNodeParameter<int>(threshold_filter_max_, "threshold_filter_max", -1);
  setAndGetNodeParameter<int>(threshold_filter_min_, "threshold_filter_min", -1);
  setAndGetNodeParameter<int>(noise_removal_filter_min_diff_, "noise_removal_filter_min_diff", 256);
  setAndGetNodeParameter<int>(noise_removal_filter_max_size_, "noise_removal_filter_max_size", 80);
  setAndGetNodeParameter<float>(spatial_filter_alpha_, "spatial_filter_alpha", -1.0);
  setAndGetNodeParameter<int>(spatial_filter_diff_threshold_, "spatial_filter_diff_threshold", -1);
  setAndGetNodeParameter<int>(spatial_filter_magnitude_, "spatial_filter_magnitude", -1);
  setAndGetNodeParameter<int>(spatial_filter_radius_, "spatial_filter_radius", -1);
  setAndGetNodeParameter<float>(temporal_filter_diff_threshold_, "temporal_filter_diff_threshold",
                                -1.0);
  setAndGetNodeParameter<float>(temporal_filter_weight_, "temporal_filter_weight", -1.0);
  setAndGetNodeParameter<std::string>(hole_filling_filter_mode_, "hole_filling_filter_mode", "");
  setAndGetNodeParameter<int>(hdr_merge_exposure_1_, "hdr_merge_exposure_1", -1);
  setAndGetNodeParameter<int>(hdr_merge_gain_1_, "hdr_merge_gain_1", -1);
  setAndGetNodeParameter<int>(hdr_merge_exposure_2_, "hdr_merge_exposure_2", -1);
  setAndGetNodeParameter<int>(hdr_merge_gain_2_, "hdr_merge_gain_2", -1);
  setAndGetNodeParameter<std::string>(align_mode_, "align_mode", "HW");
  setAndGetNodeParameter<double>(diagnostic_period_, "diagnostic_period", 1.0);
  setAndGetNodeParameter<bool>(enable_laser_, "enable_laser", true);
  setAndGetNodeParameter<int>(laser_on_off_mode_, "laser_on_off_mode", 0);
  std::string align_target_stream_str_;
  setAndGetNodeParameter<std::string>(align_target_stream_str_, "align_target_stream", "COLOR");
  align_target_stream_ = obStreamTypeFromString(align_target_stream_str_);
  setAndGetNodeParameter<bool>(retry_on_usb3_detection_failure_, "retry_on_usb3_detection_failure",
                               false);
  setAndGetNodeParameter<int>(laser_energy_level_, "laser_energy_level", -1);
  setAndGetNodeParameter<bool>(enable_3d_reconstruction_mode_, "enable_3d_reconstruction_mode",
                               false);
  setAndGetNodeParameter<int>(min_depth_limit_, "min_depth_limit", 0);
  setAndGetNodeParameter<int>(max_depth_limit_, "max_depth_limit", 0);
  setAndGetNodeParameter<bool>(enable_heartbeat_, "enable_heartbeat", false);
  setAndGetNodeParameter<bool>(enable_color_undistortion_, "enable_color_undistortion", false);
  if (enable_3d_reconstruction_mode_) {
    laser_on_off_mode_ = 1;  // 0 off, 1 on-off, 1 off-on
  }
  setAndGetNodeParameter<std::string>(time_domain_, "time_domain", "device");
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info.get());
  auto pid = device_info->getPid();
  if (isOpenNIDevice(pid)) {
    time_domain_ = "system";
  }
  RCLCPP_INFO_STREAM(logger_, "current time domain: " << time_domain_);
  setAndGetNodeParameter<int>(frames_per_trigger_, "frames_per_trigger", 2);
  long software_trigger_period = 33;
  setAndGetNodeParameter<long>(software_trigger_period, "software_trigger_period", 33);
  software_trigger_period_ = std::chrono::milliseconds(software_trigger_period);
  setAndGetNodeParameter<int>(gmsl_trigger_fps_, "gmsl_trigger_fps", 3000);
  setAndGetNodeParameter<bool>(enable_gmsl_trigger_, "enable_gmsl_trigger", false);
  setAndGetNodeParameter<std::string>(interleave_ae_mode_, "interleave_ae_mode", "hdr");
  setAndGetNodeParameter<bool>(interleave_frame_enable_, "interleave_frame_enable", false);
  setAndGetNodeParameter<bool>(interleave_skip_enable_, "interleave_skip_enable", false);
  setAndGetNodeParameter<int>(interleave_skip_index_, "interleave_skip_index", 1);

  setAndGetNodeParameter<double>(delta_duration_, "delta_duration", 5000.0);
  setAndGetNodeParameter<int>(delta_fps_, "delta_fps", 2);
}

void OBCameraNode::setupTopics() {
  try {
    getParameters();
    setupDevices();
    setupDepthPostProcessFilter();
    setupProfiles();
    selectBaseStream();
    setupCameraCtrlServices();
    setupPublishers();
    setupDiagnosticUpdater();
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

void OBCameraNode::onTemperatureUpdate(diagnostic_updater::DiagnosticStatusWrapper &status) {
  try {
    OBDeviceTemperature temperature;
    uint32_t data_size = sizeof(OBDeviceTemperature);
    device_->getStructuredData(OB_STRUCT_DEVICE_TEMPERATURE,
                               reinterpret_cast<uint8_t *>(&temperature), &data_size);
    status.add("CPU Temperature", temperature.cpuTemp);
    status.add("IR Temperature", temperature.irTemp);
    status.add("LDM Temperature", temperature.ldmTemp);
    status.add("MainBoard Temperature", temperature.mainBoardTemp);
    status.add("TEC Temperature", temperature.tecTemp);
    status.add("IMU Temperature", temperature.imuTemp);
    status.add("RGB Temperature", temperature.rgbTemp);
    status.add("Left IR Temperature", temperature.irLeftTemp);
    status.add("Right IR Temperature", temperature.irRightTemp);
    status.add("Chip Top Temperature", temperature.chipTopTemp);
    status.add("Chip Bottom Temperature", temperature.chipBottomTemp);
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Temperature is normal");
  } catch (const ob::Error &e) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, e.getMessage());
  }
}

void OBCameraNode::setupDiagnosticUpdater() {
  if (diagnostic_period_ <= 0.0) {
    return;
  }
  try {
    RCLCPP_INFO_STREAM(logger_, "Publish diagnostics every " << diagnostic_period_ << " seconds");
    auto info = device_->getDeviceInfo();
    std::string serial_number = info->getSerialNumber();
    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(node_, diagnostic_period_);
    diagnostic_updater_->setHardwareID(serial_number);
    diagnostic_updater_->add("Temperatures", this, &OBCameraNode::onTemperatureUpdate);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to setup diagnostic updater: " << e.what());
  }
}

void OBCameraNode::setupPipelineConfig() {
  if (pipeline_config_) {
    pipeline_config_.reset();
  }
  pipeline_config_ = std::make_shared<ob::Config>();
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info.get());
  auto pid = device_info->getPid();
  if (depth_registration_ && enable_stream_[COLOR] && enable_stream_[DEPTH] &&
      !isGemini335PID(pid)) {
    OBAlignMode align_mode = align_mode_ == "HW" ? ALIGN_D2C_HW_MODE : ALIGN_D2C_SW_MODE;
    RCLCPP_INFO_STREAM(logger_, "set align mode to " << magic_enum::enum_name(align_mode));
    calibration_param_ = pipeline_->getCalibrationParam(pipeline_config_);
    pipeline_config_->setAlignMode(align_mode);
    RCLCPP_INFO_STREAM(logger_, "enable depth scale " << (enable_depth_scale_ ? "ON" : "OFF"));
    pipeline_config_->setDepthScaleRequire(enable_depth_scale_);
  }
  for (const auto &stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      RCLCPP_INFO_STREAM(logger_, "Enable " << stream_name_[stream_index] << " stream");
      auto profile = stream_profile_[stream_index]->as<ob::VideoStreamProfile>();
      RCLCPP_INFO_STREAM(
          logger_, "Stream " << stream_name_[stream_index] << " width: " << profile->getWidth()
                             << " height: " << profile->getHeight() << " fps: " << profile->getFps()
                             << " format: " << profile->getFormat());
      pipeline_config_->enableStream(stream_profile_[stream_index]);
    }
  }
}

void OBCameraNode::setupPublishers() {
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  auto point_cloud_qos_profile = getRMWQosProfileFromString(point_cloud_qos_);
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
  auto pid = device_info->getPid();
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
    if (stream_index == COLOR && enable_color_undistortion_) {
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

void OBCameraNode::publishDepthPointCloud(const std::shared_ptr<ob::FrameSet> &frame_set) {
  (void)frame_set;
  if (!depth_cloud_pub_ || depth_cloud_pub_->get_subscription_count() == 0 ||
      !enable_point_cloud_) {
    return;
  }
  std::lock_guard<decltype(point_cloud_mutex_)> point_cloud_msg_lock(point_cloud_mutex_);
  auto depth_frame = frame_set->depthFrame();
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
    if (valid_point || ordered_pc_) {
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
  if (!ordered_pc_) {
    point_cloud_msg->is_dense = true;
    point_cloud_msg->width = valid_count;
    point_cloud_msg->height = 1;
    modifier.resize(valid_count);
  }
  auto frame_timestamp = getFrameTimestampUs(depth_frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  std::string frame_id = depth_registration_ ? optical_frame_id_[COLOR] : optical_frame_id_[DEPTH];
  if (!cloud_frame_id_.empty()) {
    frame_id = cloud_frame_id_;
  }
  point_cloud_msg->header.stamp = timestamp;
  point_cloud_msg->header.frame_id = frame_id;
  if (save_point_cloud_) {
    save_point_cloud_ = false;
    auto now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = std::filesystem::current_path().string();
    std::string filename = current_path + "/point_cloud/points_" + ss.str() + ".ply";
    if (!std::filesystem::exists(current_path + "/point_cloud")) {
      std::filesystem::create_directory(current_path + "/point_cloud");
    }
    RCLCPP_INFO_STREAM(logger_, "Saving point cloud to " << filename);
    try {
      saveDepthPointsToPly(point_cloud_msg, filename);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to save point cloud: " << e.what());
    }
  }
  depth_cloud_pub_->publish(std::move(point_cloud_msg));
}

void OBCameraNode::publishColoredPointCloud(const std::shared_ptr<ob::FrameSet> &frame_set) {
  if (!depth_registration_cloud_pub_ ||
      depth_registration_cloud_pub_->get_subscription_count() == 0 ||
      !enable_colored_point_cloud_ || !frame_set) {
    return;
  }
  std::lock_guard<decltype(point_cloud_mutex_)> point_cloud_msg_lock(point_cloud_mutex_);

  auto depth_frame = frame_set->depthFrame();
  auto color_frame = frame_set->colorFrame();

  if (!depth_frame || !color_frame) {
    return;
  }

  auto depth_width = depth_frame->getWidth();
  auto depth_height = depth_frame->getHeight();
  auto color_width = color_frame->getWidth();
  auto color_height = color_frame->getHeight();
  if (depth_width != color_width || depth_height != color_height) {
    RCLCPP_DEBUG(logger_, "Depth (%d x %d) and color (%d x %d) frame size mismatch", depth_width,
                 depth_height, color_width, color_height);
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

  color_point_cloud_filter_.setCameraParam(camera_params);
  auto depth_scale = depth_frame->getValueScale();
  color_point_cloud_filter_.setPositionDataScaled(depth_scale);
  color_point_cloud_filter_.setCreatePointFormat(OB_FORMAT_RGB_POINT);
  auto result_frame = color_point_cloud_filter_.process(frame_set);
  if (!result_frame) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to process depth frame");
    return;
  }
  auto point_size = result_frame->dataSize() / sizeof(OBColorPoint);
  auto *point_cloud = static_cast<OBColorPoint *>(result_frame->data());

  auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  point_cloud_msg->width = color_frame->getWidth();
  point_cloud_msg->height = color_frame->getHeight();
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
  static float min_depth = MIN_DISTANCE / depth_scale;
  static float max_depth = MAX_DISTANCE / depth_scale;
  for (size_t i = 0; i < point_size; i++) {
    bool valid_point = point_cloud[i].z >= min_depth && point_cloud[i].z <= max_depth;
    if (valid_point || ordered_pc_) {
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
  if (!ordered_pc_) {
    point_cloud_msg->is_dense = true;
    point_cloud_msg->width = valid_count;
    point_cloud_msg->height = 1;
    modifier.resize(valid_count);
  }
  auto frame_timestamp = getFrameTimestampUs(depth_frame);
  std::string frame_id = optical_frame_id_[COLOR];
  if (!cloud_frame_id_.empty()) {
    frame_id = cloud_frame_id_;
  }
  auto timestamp = fromUsToROSTime(frame_timestamp);
  point_cloud_msg->header.stamp = timestamp;
  point_cloud_msg->header.frame_id = frame_id;
  if (save_colored_point_cloud_) {
    save_colored_point_cloud_ = false;
    auto now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = std::filesystem::current_path().string();
    std::string filename = current_path + "/point_cloud/colored_points_" + ss.str() + ".ply";
    if (!std::filesystem::exists(current_path + "/point_cloud")) {
      std::filesystem::create_directory(current_path + "/point_cloud");
    }
    RCLCPP_INFO_STREAM(logger_, "Saving point cloud to " << filename);
    try {
      saveRGBPointCloudMsgToPly(point_cloud_msg, filename);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to save point cloud: " << e.what());
    } catch (...) {
      RCLCPP_ERROR(logger_, "Failed to save point cloud");
    }
  }

  depth_registration_cloud_pub_->publish(std::move(point_cloud_msg));
}

std::shared_ptr<ob::Frame> OBCameraNode::processDepthFrameFilter(
    std::shared_ptr<ob::Frame> &frame) {
  if (frame == nullptr || frame->getType() != OB_FRAME_DEPTH) {
    return nullptr;
  }
  for (size_t i = 0; i < filter_list_.size(); i++) {
    auto filter = filter_list_[i];
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

uint64_t OBCameraNode::getFrameTimestampUs(const std::shared_ptr<ob::Frame> &frame) {
  if (frame == nullptr) {
    RCLCPP_WARN(logger_, "getFrameTimestampUs: frame is nullptr, return 0");
    return 0;
  }
  if (time_domain_ == "device") {
    return frame->getTimeStampUs();
  } else if (time_domain_ == "global") {
    return frame->getGlobalTimeStampUs();
  } else {
    return frame->getSystemTimeStampUs();
  }
}
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
    if (!tf_published_) {
      publishStaticTransforms();
      tf_published_ = true;
    }
    auto depth_frame = frame_set->getFrame(OB_FRAME_DEPTH);
    bool depth_laser_status = false;
    if (depth_frame && depth_frame->hasMetadata(OB_FRAME_METADATA_TYPE_LASER_STATUS)) {
      depth_laser_status = depth_frame->getMetadataValue(OB_FRAME_METADATA_TYPE_LASER_STATUS) == 1;
    }
    auto color_frame = frame_set->getFrame(OB_FRAME_COLOR);
    depth_frame = processDepthFrameFilter(depth_frame);
    bool depth_aligned = false;
    if (depth_frame) {
      frame_set->pushFrame(depth_frame);
    }
    if (depth_registration_ && align_filter_ && depth_frame && color_frame) {
      if (auto new_frame = align_filter_->process(frame_set)) {
        auto new_frame_set = new_frame->as<ob::FrameSet>();
        CHECK_NOTNULL(new_frame_set.get());
        frame_set = new_frame_set;
        depth_aligned = true;
      } else {
        RCLCPP_ERROR(logger_, "Failed to align depth frame to color frame");
        return;
      }
    } else {
      RCLCPP_DEBUG(logger_,
                   "Depth registration is disabled or align filter is null or depth frame is "
                   "null or color frame is null");
    }
    if (depth_registration_ && align_filter_ && !depth_aligned) {
      return;
    }

    if (enable_stream_[COLOR] && color_frame) {
      std::unique_lock<std::mutex> lock(color_frame_queue_lock_);
      if (!enable_3d_reconstruction_mode_ || depth_laser_status) {
        color_frame_queue_.push(frame_set);
        color_frame_queue_cv_.notify_all();
      }
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
          frame = (enable_3d_reconstruction_mode_ && !depth_laser_status) ? nullptr : frame;
        }
        auto is_ir_frame = frame_type == OB_FRAME_IR_LEFT || frame_type == OB_FRAME_IR_RIGHT ||
                           frame_type == OB_FRAME_IR;
        if (is_ir_frame) {
          std::shared_ptr<ob::Frame> ir_frame =
              frame->getFormat() == OB_FORMAT_MJPG ? decodeIRMJPGFrame(frame) : frame;
          bool ir_laser_status = false;
          if (ir_frame && ir_frame->hasMetadata(OB_FRAME_METADATA_TYPE_LASER_STATUS)) {
            ir_laser_status = ir_frame->getMetadataValue(OB_FRAME_METADATA_TYPE_LASER_STATUS) == 1;
          }
          if (ir_frame && (!enable_3d_reconstruction_mode_ || !ir_laser_status)) {
            onNewFrameCallback(ir_frame, stream_index);
          }
        } else if (frame_type == OB_FRAME_DEPTH) {
          onNewFrameCallback(frame, stream_index);
        }
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

std::shared_ptr<ob::Frame> OBCameraNode::softwareDecodeColorFrame(
    const std::shared_ptr<ob::Frame> &frame) {
  if (frame == nullptr) {
    return nullptr;
  }
  if (frame->getFormat() == OB_FORMAT_RGB || frame->getFormat() == OB_FORMAT_BGR) {
    return frame;
  }
  if (frame->getFormat() == OB_FORMAT_RGBA || frame->getFormat() == OB_FORMAT_BGRA) {
    return frame;
  }
  if (frame->getFormat() == OB_FORMAT_Y16 || frame->getFormat() == OB_FORMAT_Y8) {
    return frame;
  }
  if (!setupFormatConvertType(frame->getFormat())) {
    RCLCPP_ERROR(logger_, "Unsupported color format: %d", frame->getFormat());
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
  if (frame && frame->getFormat() != OB_FORMAT_RGB888) {
    if (frame->getFormat() == OB_FORMAT_MJPG && jpeg_decoder_) {
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
    memcpy(rgb_buffer_, video_frame->getData(), video_frame->getDataSize());
    return true;
  }
  return true;
}

std::shared_ptr<ob::Frame> OBCameraNode::decodeIRMJPGFrame(
    const std::shared_ptr<ob::Frame> &frame) {
  if (frame == nullptr) {
    return nullptr;
  }
  if (frame->getFormat() == OB_FORMAT_MJPEG &&
      (frame->getType() == OB_FRAME_IR || frame->getType() == OB_FRAME_IR_LEFT ||
       frame->getType() == OB_FRAME_IR_RIGHT)) {
    auto video_frame = frame->as<ob::IRFrame>();

    cv::Mat mjpgMat(1, video_frame->getDataSize(), CV_8UC1, video_frame->getData());
    cv::Mat irRawMat = cv::imdecode(mjpgMat, cv::IMREAD_GRAYSCALE);

    std::shared_ptr<ob::Frame> irFrame =
        ob::FrameFactory::createVideoFrame(video_frame->getType(), video_frame->getFormat(),
                                           video_frame->getWidth(), video_frame->getHeight(), 0);

    uint32_t buffer_size = irRawMat.rows * irRawMat.cols * irRawMat.channels();

    if (buffer_size > irFrame->getDataSize()) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Insufficient buffer size allocation,failed to decode ir mjpg frame!");
      return nullptr;
    }

    memcpy(irFrame->getData(), irRawMat.data, buffer_size);
    ob::FrameHelper::setFrameDeviceTimestamp(irFrame, video_frame->getTimeStampUs());
    ob::FrameHelper::setFrameDeviceTimestampUs(irFrame, video_frame->getTimeStampUs());
    ob::FrameHelper::setFrameSystemTimestamp(irFrame, video_frame->getSystemTimeStampUs());
    return irFrame;
  }

  return frame;
}

void OBCameraNode::updateStreamInfo(const std::shared_ptr<ob::Frame> &frame,
                                    VideoStreamInfo &stream_info) {
  uint64_t current_timestamp = getFrameTimestampUs(frame);
  uint64_t duration = current_timestamp - stream_info.last_frame_time;
  double dst_duration = 0.0;
  int dst_fps = 0;
  stream_index_pair dst_frame_type;
  double delta_duration = delta_duration_;
  int delta_fps = delta_fps_;
  if (current_timestamp <= stream_info.last_frame_time) {
    RCLCPP_INFO_STREAM(logger_, "[WARNING] current_timestamp "
                                    << current_timestamp << " stream_info.last_frame_time "
                                    << stream_info.last_frame_time << "\n");
  }

  switch (stream_info.frame_type) {
    case OB_FRAME_COLOR:
      dst_frame_type = stream_index_pair{OB_STREAM_COLOR, 0};
      break;
    case OB_FRAME_DEPTH:
      dst_frame_type = stream_index_pair{OB_STREAM_DEPTH, 0};
      break;
    case OB_FRAME_IR_LEFT:
      dst_frame_type = stream_index_pair{OB_STREAM_IR_LEFT, 0};
      break;
    case OB_FRAME_IR_RIGHT:
      dst_frame_type = stream_index_pair{OB_STREAM_IR_RIGHT, 0};
      break;
    default:
      RCLCPP_INFO_STREAM(logger_,
                         "[WARNING] Unknown frame_type " << stream_info.frame_type << "\n");
      return;
  }

  if (interleave_skip_enable_) {
    dst_duration = (1000000.0 / fps_[dst_frame_type]) * 2 + delta_duration;
    dst_fps = fps_[dst_frame_type] / 2;
  } else {
    dst_duration = 1000000.0 / fps_[dst_frame_type] + delta_duration;
    dst_fps = fps_[dst_frame_type];
  }

  if (duration > dst_duration) {
    RCLCPP_INFO_STREAM(logger_, "[WARNING] Frame interval for frame_type "
                                    << stream_info.frame_type << " exceeded "
                                    << dst_duration / 1000.0
                                    << " ms. Interval: " << duration / 1000.0 << " ms\n");
  }

  stream_info.frame_count++;

  if (stream_info.frame_count % dst_fps == 0) {
    double elapsed_seconds = (current_timestamp - stream_info.last_fps_calculation_time) / 1e6;

    if (elapsed_seconds > 0) {
      int frames_in_interval = stream_info.frame_count - stream_info.last_fps_frame_count;
      stream_info.frame_rate = frames_in_interval / elapsed_seconds;

      if (stream_info.frame_rate < (dst_fps - delta_fps)) {
        RCLCPP_INFO_STREAM(logger_, "[INFO] Frame rate for frame_type "
                                        << stream_info.frame_type << ": " << stream_info.frame_rate
                                        << " FPS"
                                        << " frames_in_interval " << frames_in_interval
                                        << " elapsed_seconds " << elapsed_seconds << "\n");
      }

      stream_info.last_fps_calculation_time = current_timestamp;
      stream_info.last_fps_frame_count = stream_info.frame_count;
    }
  }

  stream_info.last_frame_time = current_timestamp;
}

void OBCameraNode::onNewFrameCallback(const std::shared_ptr<ob::Frame> &frame,
                                      const stream_index_pair &stream_index) {
  if (frame == nullptr) {
    return;
  }
  CHECK_NOTNULL(image_publishers_[stream_index]);
  bool has_subscriber = image_publishers_[stream_index]->get_subscription_count() > 0;
  has_subscriber =
      has_subscriber || camera_info_publishers_[stream_index]->get_subscription_count() > 0;
  has_subscriber =
      has_subscriber || (metadata_publishers_.count(stream_index) &&
                         metadata_publishers_[stream_index]->get_subscription_count() > 0);
  if (!has_subscriber) {
    return;
  }
  std::shared_ptr<ob::VideoFrame> video_frame;
  if (frame->getType() == OB_FRAME_COLOR) {
    updateStreamInfo(frame, color_stream_info_);
    video_frame = frame->as<ob::ColorFrame>();
  } else if (frame->getType() == OB_FRAME_DEPTH) {
    // interleave filter depth
    if (interleave_skip_enable_) {
      interleave_skip_depth_index_++;
      RCLCPP_DEBUG(logger_, "interleave filter skip interleave_skip_index_: %d",
                   interleave_skip_depth_index_);
      if (interleave_skip_depth_index_ % 2 == 0) {
        RCLCPP_DEBUG(logger_, "interleave filter skip frame type: %d", frame->getType());
        return;
      }
      updateStreamInfo(frame, depth_stream_info_);
    }
    video_frame = frame->as<ob::DepthFrame>();
  } else if (frame->getType() == OB_FRAME_IR || frame->getType() == OB_FRAME_IR_LEFT ||
             frame->getType() == OB_FRAME_IR_RIGHT) {
    video_frame = frame->as<ob::IRFrame>();

    // interleave filter speckle or flood ir
    if (interleave_skip_enable_) {
      RCLCPP_DEBUG(logger_, "interleave filter skip interleave_skip_index_: %d",
                   interleave_skip_index_);
      if (video_frame->getMetadataValue(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX) ==
          interleave_skip_index_) {
        RCLCPP_DEBUG(logger_, "interleave filter skip frame type: %d", frame->getType());
        return;
      }
      if (frame->getType() == OB_FRAME_IR_LEFT) {
        updateStreamInfo(frame, left_ir_stream_info_);
      }
      if (frame->getType() == OB_FRAME_IR_RIGHT) {
        updateStreamInfo(frame, right_ir_stream_info_);
      }
    }

  } else {
    RCLCPP_ERROR(logger_, "Unsupported frame type: %d", frame->getType());
    return;
  }
  if (!video_frame) {
    RCLCPP_ERROR(logger_, "Failed to convert frame to video frame");
    return;
  }
  int width = static_cast<int>(video_frame->getWidth());
  int height = static_cast<int>(video_frame->getHeight());
  auto frame_timestamp = getFrameTimestampUs(frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  auto pid = device_info->getPid();
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
  if (stream_index == COLOR && enable_color_undistortion_) {
    memset(&distortion, 0, sizeof(distortion));
  }
  auto camera_info = convertToCameraInfo(intrinsic, distortion, width);
  camera_info.header.stamp = timestamp;
  camera_info.header.frame_id = frame_id;
  camera_info.width = width;
  camera_info.height = height;
  if (frame->getType() == OB_FRAME_IR_RIGHT && enable_stream_[INFRA1]) {
    auto stream_profile = frame->getStreamProfile();
    CHECK_NOTNULL(stream_profile);
    auto video_stream_profile = stream_profile->as<ob::VideoStreamProfile>();
    CHECK_NOTNULL(video_stream_profile);
    auto left_video_profile = stream_profile_[INFRA1]->as<ob::VideoStreamProfile>();
    CHECK_NOTNULL(left_video_profile);
    auto ex = video_stream_profile->getExtrinsicTo(left_video_profile);
    float fx = camera_info.k.at(0);
    float fy = camera_info.k.at(4);
    camera_info.p.at(3) = -fx * ex.trans[0] / 1000.0 + 0.0;
    camera_info.p.at(7) = -fy * ex.trans[1] / 1000.0 + 0.0;
  }
  CHECK(camera_info_publishers_.count(stream_index) > 0);
  camera_info_publishers_[stream_index]->publish(camera_info);
  if (isGemini335PID(pid)) {
    publishMetadata(frame, stream_index, camera_info.header);
  }
  CHECK_NOTNULL(image_publishers_[stream_index]);
  if (image_publishers_[stream_index]->get_subscription_count() == 0) {
    return;
  }
  auto &image = images_[stream_index];
  if (image.empty() || image.cols != width || image.rows != height) {
    image.create(height, width, image_format_[stream_index]);
  }
  if (frame->getType() == OB_FRAME_COLOR && !is_color_frame_decoded_) {
    RCLCPP_ERROR(logger_, "color frame is not decoded");
    return;
  }
  if (frame->getType() == OB_FRAME_COLOR && frame->format() != OB_FORMAT_Y8 &&
      frame->format() != OB_FORMAT_Y16 && frame->format() != OB_FORMAT_BGRA &&
      frame->format() != OB_FORMAT_RGBA && image_publishers_[COLOR]->get_subscription_count() > 0) {
    memcpy(image.data, rgb_buffer_, video_frame->getWidth() * video_frame->getHeight() * 3);
  } else {
    memcpy(image.data, video_frame->getData(), video_frame->getDataSize());
  }
  if (stream_index == DEPTH) {
    auto depth_scale = video_frame->as<ob::DepthFrame>()->getValueScale();
    image = image * depth_scale;
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
  saveImageToFile(stream_index, image, *image_msg);
  image_publishers_[stream_index]->publish(std::move(image_msg));
  if (stream_index == COLOR && enable_color_undistortion_ &&
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
  metadata_publisher->publish(metadata_msg);
}

void OBCameraNode::saveImageToFile(const stream_index_pair &stream_index, const cv::Mat &image,
                                   const sensor_msgs::msg::Image &image_msg) {
  if (save_images_[stream_index]) {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto us =
        std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    ss << "_" << std::setw(6) << std::setfill('0') << us.count();
    auto current_path = std::filesystem::current_path().string();
    auto fps = fps_[stream_index];
    int index = save_images_count_[stream_index];
    std::string file_suffix = stream_index == COLOR ? ".png" : ".raw";
    std::string filename = current_path + "/image/" + stream_name_[stream_index] + "_" +
                           std::to_string(image_msg.width) + "x" +
                           std::to_string(image_msg.height) + "_" + std::to_string(fps) + "hz_" +
                           ss.str() + "_" + std::to_string(index) + file_suffix;
    if (!std::filesystem::exists(current_path + "/image")) {
      std::filesystem::create_directory(current_path + "/image");
    }
    RCLCPP_INFO_STREAM(logger_, "Saving image to " << filename);
    if (stream_index.first == OB_STREAM_COLOR) {
      auto image_to_save =
          cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::imwrite(filename, image_to_save);
    } else if (stream_index.first == OB_STREAM_IR || stream_index.first == OB_STREAM_IR_LEFT ||
               stream_index.first == OB_STREAM_IR_RIGHT || stream_index.first == OB_STREAM_DEPTH) {
      std::ofstream ofs(filename, std::ios::out | std::ios::binary);
      if (!ofs.is_open()) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to open file: " << filename);
        return;
      }
      if (image.isContinuous()) {
        ofs.write(reinterpret_cast<const char *>(image.data), image.total() * image.elemSize());
      } else {
        int rows = image.rows;
        int cols = image.cols * image.channels();
        for (int r = 0; r < rows; ++r) {
          ofs.write(reinterpret_cast<const char *>(image.ptr<uchar>(r)), cols);
        }
      }
      ofs.close();
    } else {
      RCLCPP_ERROR_STREAM(logger_, "Unsupported stream type: " << stream_index.first);
    }
    if (++save_images_count_[stream_index] >= max_save_images_count_) {
      save_images_[stream_index] = false;
    }
  }
}

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

  imu_msg.header.frame_id = imu_optical_frame_id_;
  auto frame_timestamp = getFrameTimestampUs(accelframe);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  imu_msg.header.stamp = timestamp;
  auto gyro_frame = gryoframe->as<ob::GyroFrame>();
  auto gyro_info = createIMUInfo(GYRO);
  gyro_info.header = imu_msg.header;
  gyro_info.header.frame_id = imu_optical_frame_id_;
  imu_info_publishers_[GYRO]->publish(gyro_info);
  auto gyroData = gyro_frame->getValue();
  imu_msg.angular_velocity.x = gyroData.x - gyro_info.bias[0];
  imu_msg.angular_velocity.y = gyroData.y - gyro_info.bias[1];
  imu_msg.angular_velocity.z = gyroData.z - gyro_info.bias[2];
  auto accel_frame = accelframe->as<ob::AccelFrame>();
  auto accelData = accel_frame->getValue();
  auto accel_info = createIMUInfo(ACCEL);
  imu_msg.linear_acceleration.x = accelData.x - accel_info.bias[0];
  imu_msg.linear_acceleration.y = accelData.y - accel_info.bias[1];
  imu_msg.linear_acceleration.z = accelData.z - accel_info.bias[2];
  imu_info_publishers_[ACCEL]->publish(accel_info);
  imu_gyro_accel_publisher_->publish(imu_msg);
}

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
  auto timestamp = fromUsToROSTime(frame->getTimeStampUs());

  imu_msg.header.stamp = timestamp;
  auto imu_info = createIMUInfo(stream_index);
  imu_info.header = imu_msg.header;
  imu_info.header.frame_id = imu_optical_frame_id_;
  imu_info_publishers_[stream_index]->publish(imu_info);
  if (frame->getType() == OB_FRAME_GYRO) {
    auto gyro_frame = frame->as<ob::GyroFrame>();
    auto data = gyro_frame->getValue();
    imu_msg.angular_velocity.x = data.x - imu_info.bias[0];
    imu_msg.angular_velocity.y = data.y - imu_info.bias[1];
    imu_msg.angular_velocity.z = data.z - imu_info.bias[2];
  } else if (frame->getType() == OB_FRAME_ACCEL) {
    auto accel_frame = frame->as<ob::AccelFrame>();
    auto data = accel_frame->getValue();
    imu_msg.linear_acceleration.x = data.x - imu_info.bias[0];
    imu_msg.linear_acceleration.y = data.y - imu_info.bias[1];
    imu_msg.linear_acceleration.z = data.z - imu_info.bias[2];
  } else {
    RCLCPP_ERROR(logger_, "Unsupported IMU frame type");
    return;
  }
  imu_publishers_[stream_index]->publish(imu_msg);
}

void OBCameraNode::setDefaultIMUMessage(sensor_msgs::msg::Imu &imu_msg) {
  imu_msg.header.frame_id = "imu_link";
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;

  imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  imu_msg.linear_acceleration_covariance = {
      liner_accel_cov_, 0.0, 0.0, 0.0, liner_accel_cov_, 0.0, 0.0, 0.0, liner_accel_cov_};
  imu_msg.angular_velocity_covariance = {
      angular_vel_cov_, 0.0, 0.0, 0.0, angular_vel_cov_, 0.0, 0.0, 0.0, angular_vel_cov_};
}

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

std::optional<OBCameraParam> OBCameraNode::findDefaultCameraParam() {
  auto camera_params = device_->getCalibrationCameraParamList();
  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int depth_w = param.depthIntrinsic.width;
    int depth_h = param.depthIntrinsic.height;
    int color_w = param.rgbIntrinsic.width;
    int color_h = param.rgbIntrinsic.height;
    if ((depth_w * height_[DEPTH] == depth_h * width_[DEPTH]) &&
        (color_w * height_[COLOR] == color_h * width_[COLOR])) {
      return param;
    }
  }
  return {};
}

std::optional<OBCameraParam> OBCameraNode::getDepthCameraParam() {
  auto camera_params = device_->getCalibrationCameraParamList();
  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int depth_w = param.depthIntrinsic.width;
    int depth_h = param.depthIntrinsic.height;
    if (depth_w == width_[DEPTH] && depth_h == height_[DEPTH]) {
      RCLCPP_INFO_STREAM(logger_, "getCameraDepthParam w: " << depth_w << ",h:" << depth_h);
      return param;
    }
  }

  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int depth_w = param.depthIntrinsic.width;
    int depth_h = param.depthIntrinsic.height;
    if (depth_w * height_[DEPTH] == depth_h * width_[DEPTH]) {
      RCLCPP_INFO_STREAM(logger_, "getCameraDepthParam w: " << depth_w << ",h:" << depth_h);
      return param;
    }
  }
  return {};
}

std::optional<OBCameraParam> OBCameraNode::getColorCameraParam() {
  auto camera_params = device_->getCalibrationCameraParamList();
  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int color_w = param.rgbIntrinsic.width;
    int color_h = param.rgbIntrinsic.height;
    if (color_w == width_[COLOR] && color_h == height_[COLOR]) {
      RCLCPP_INFO_STREAM(logger_, "getColorCameraParam w: " << color_w << ",h:" << color_h);
      return param;
    }
  }

  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int color_w = param.rgbIntrinsic.width;
    int color_h = param.rgbIntrinsic.height;
    if (color_w * height_[COLOR] == color_h * width_[COLOR]) {
      RCLCPP_INFO_STREAM(logger_, "getColorCameraParam w: " << color_w << ",h:" << color_h);
      return param;
    }
  }
  return {};
}

void OBCameraNode::publishStaticTF(const rclcpp::Time &t, const tf2::Vector3 &trans,
                                   const tf2::Quaternion &q, const std::string &from,
                                   const std::string &to) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = t;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
  msg.transform.translation.x = trans[2] / 1000.0;
  msg.transform.translation.y = -trans[0] / 1000.0;
  msg.transform.translation.z = -trans[1] / 1000.0;
  msg.transform.rotation.x = q.getX();
  msg.transform.rotation.y = q.getY();
  msg.transform.rotation.z = q.getZ();
  msg.transform.rotation.w = q.getW();
  static_tf_msgs_.push_back(msg);
}

void OBCameraNode::calcAndPublishStaticTransform() {
  tf2::Quaternion quaternion_optical, zero_rot;
  zero_rot.setRPY(0.0, 0.0, 0.0);
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  tf2::Vector3 zero_trans(0, 0, 0);
  auto base_stream_profile = stream_profile_[base_stream_];
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  auto pid = device_info->getPid();
  if (!base_stream_profile) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to get base stream profile");
    return;
  }
  CHECK_NOTNULL(base_stream_profile.get());
  for (const auto &item : stream_profile_) {
    auto stream_index = item.first;

    auto stream_profile = item.second;
    if (!stream_profile) {
      continue;
    }
    OBExtrinsic ex;
    try {
      ex = stream_profile->getExtrinsicTo(base_stream_profile);
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get " << stream_name_[stream_index]
                                                    << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }

    auto Q = rotationMatrixToQuaternion(ex.rot);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    tf2::Vector3 trans(ex.trans[0], ex.trans[1], ex.trans[2]);
    auto timestamp = node_->now();
    if (stream_index.first != base_stream_.first) {
      if (stream_index.first == OB_STREAM_IR_RIGHT && base_stream_.first == OB_STREAM_DEPTH) {
        trans[0] = std::abs(trans[0]);  // because left and right ir calibration is error
      }
      publishStaticTF(timestamp, trans, Q, frame_id_[base_stream_], frame_id_[stream_index]);
    }
    publishStaticTF(timestamp, zero_trans, quaternion_optical, frame_id_[stream_index],
                    optical_frame_id_[stream_index]);
    RCLCPP_INFO_STREAM(logger_, "Publishing static transform from " << stream_name_[stream_index]
                                                                    << " to "
                                                                    << stream_name_[base_stream_]);
    RCLCPP_INFO_STREAM(logger_, "Translation " << trans[0] << ", " << trans[1] << ", " << trans[2]);
    RCLCPP_INFO_STREAM(logger_, "Rotation " << Q.getX() << ", " << Q.getY() << ", " << Q.getZ()
                                            << ", " << Q.getW());
  }

  if ((pid == FEMTO_BOLT_PID || pid == FEMTO_MEGA_PID) && enable_stream_[DEPTH] &&
      enable_stream_[COLOR]) {
    // calc depth to color

    CHECK_NOTNULL(stream_profile_[COLOR]);
    auto depth_to_color_extrinsics = base_stream_profile->getExtrinsicTo(stream_profile_[COLOR]);
    auto Q = rotationMatrixToQuaternion(depth_to_color_extrinsics.rot);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    publishStaticTF(node_->now(), zero_trans, Q, camera_link_frame_id_, frame_id_[base_stream_]);
  } else {
    publishStaticTF(node_->now(), zero_trans, zero_rot, camera_link_frame_id_,
                    frame_id_[base_stream_]);
  }

  if (enable_stream_[DEPTH] && enable_stream_[COLOR]) {
    static const char *frame_id = "depth_to_color_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[COLOR]);
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[COLOR] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    CHECK_NOTNULL(depth_to_other_extrinsics_publishers_[COLOR]);
    depth_to_other_extrinsics_publishers_[COLOR]->publish(ex_msg);
  }

  if (enable_stream_[DEPTH] && enable_stream_[INFRA0]) {
    static const char *frame_id = "depth_to_ir_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[INFRA0]);
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[INFRA0] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    CHECK_NOTNULL(depth_to_other_extrinsics_publishers_[INFRA0]);
    depth_to_other_extrinsics_publishers_[INFRA0]->publish(ex_msg);
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA1]) {
    static const char *frame_id = "depth_to_left_ir_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[INFRA1]);
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[INFRA1] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    CHECK_NOTNULL(depth_to_other_extrinsics_publishers_[INFRA1]);
    depth_to_other_extrinsics_publishers_[INFRA1]->publish(ex_msg);
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA2]) {
    static const char *frame_id = "depth_to_right_ir_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[INFRA2]);
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    ex.trans[0] = -std::abs(ex.trans[0]);
    depth_to_other_extrinsics_[INFRA2] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    CHECK_NOTNULL(depth_to_other_extrinsics_publishers_[INFRA2]);
    depth_to_other_extrinsics_publishers_[INFRA2]->publish(ex_msg);
  }
  if (enable_stream_[DEPTH] && enable_stream_[ACCEL]) {
    static const char *frame_id = "depth_to_accel_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[ACCEL]);
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[ACCEL] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    CHECK_NOTNULL(depth_to_other_extrinsics_publishers_[ACCEL]);
    depth_to_other_extrinsics_publishers_[ACCEL]->publish(ex_msg);
  }
  if (enable_stream_[DEPTH] && enable_stream_[GYRO]) {
    static const char *frame_id = "depth_to_gyro_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[GYRO]);
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[GYRO] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    CHECK_NOTNULL(depth_to_other_extrinsics_publishers_[GYRO]);
    depth_to_other_extrinsics_publishers_[GYRO]->publish(ex_msg);
  }
}

void OBCameraNode::publishStaticTransforms() {
  if (!publish_tf_) {
    return;
  }
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  calcAndPublishStaticTransform();
  if (tf_publish_rate_ > 0) {
    tf_thread_ = std::make_shared<std::thread>([this]() { publishDynamicTransforms(); });
  } else {
    static_tf_broadcaster_->sendTransform(static_tf_msgs_);
  }
}

void OBCameraNode::publishDynamicTransforms() {
  RCLCPP_WARN(logger_, "Publishing dynamic camera transforms (/tf) at %g Hz", tf_publish_rate_);
  std::mutex mu;
  std::unique_lock<std::mutex> lock(mu);
  while (rclcpp::ok() && is_running_) {
    tf_cv_.wait_for(lock, std::chrono::milliseconds((int)(1000.0 / tf_publish_rate_)),
                    [this] { return (!(is_running_)); });
    {
      rclcpp::Time t = node_->now();
      for (auto &msg : static_tf_msgs_) {
        msg.header.stamp = t;
      }
      dynamic_tf_broadcaster_->sendTransform(static_tf_msgs_);
    }
  }
}

template <typename T>
T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

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

bool OBCameraNode::isGemini335PID(uint32_t pid) {
  return pid == GEMINI_335_PID || pid == GEMINI_330_PID || pid == GEMINI_336_PID ||
         pid == GEMINI_335L_PID || pid == GEMINI_330L_PID || pid == GEMINI_336L_PID ||
         pid == GEMINI_335LG_PID || pid == GEMINI_336LG_PID || pid == GEMINI_335LE_PID ||
         pid == GEMINI_336LE_PID;
}

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
