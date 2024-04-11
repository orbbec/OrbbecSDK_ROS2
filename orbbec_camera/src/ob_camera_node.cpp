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

#if defined(USE_RK_HW_DECODER)
#include "orbbec_camera/rk_mpp_decoder.h"
#elif defined(USE_NV_HW_DECODER)
#include "orbbec_camera/jetson_nv_decoder.h"
#endif

namespace orbbec_camera {
using namespace std::chrono_literals;

OBCameraNode::OBCameraNode(rclcpp::Node *node, std::shared_ptr<ob::Device> device,
                           std::shared_ptr<Parameters> parameters)
    : node_(node),
      device_(std::move(device)),
      parameters_(std::move(parameters)),
      logger_(node->get_logger()) {
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
    rgb_buffer_ = new uint8_t[width_[COLOR] * height_[COLOR] * 3];
  }
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

OBCameraNode::~OBCameraNode() { clean(); }

void OBCameraNode::clean() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  RCLCPP_WARN_STREAM(logger_, "Do destroy ~OBCameraNode");
  is_running_.store(false);
  if (tf_thread_ && tf_thread_->joinable()) {
    tf_thread_->join();
  }

  if (colorFrameThread_ && colorFrameThread_->joinable()) {
    colorFrameCV_.notify_all();
    colorFrameThread_->join();
  }

  RCLCPP_WARN_STREAM(logger_, "stop streams");
  stopStreams();
  stopIMU();
  RCLCPP_WARN_STREAM(logger_, "Destroy ~OBCameraNode DONE");
  if (rgb_buffer_) {
    delete[] rgb_buffer_;
    rgb_buffer_ = nullptr;
  }
}

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
  if (enable_hardware_d2d_ && info->pid() == GEMINI2_PID) {
    device_->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, true);
    bool isHWD2D = device_->getBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL);
    if (isHWD2D == false) {
      RCLCPP_INFO_STREAM(logger_, "Depth process is soft D2D.");
    } else {
      RCLCPP_INFO_STREAM(logger_, "Depth process is HW D2D.");
    }
  }
  try {
    device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, enable_laser_);
    device_->loadPreset(device_preset_.c_str());
    auto depth_sensor = device_->getSensor(OB_SENSOR_DEPTH);
    // set depth sensor to filter
    auto filter_list = depth_sensor->getRecommendedFilters();
    for (size_t i = 0; i < filter_list->count(); i++) {
      auto filter = filter_list->getFilter(i);
      std::map<std::string, bool> filter_params = {
          {"DecimationFilter", enable_decimation_filter_},
          {"HdrMerge", enable_hdr_merge_},
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
      if (filter_name == "DecimationFilter") {
        auto decimation_filter = filter->as<ob::DecimationFilter>();
        decimation_filter->setScaleValue(decimation_filter_scale_range_);
      } else if (filter_name == "ThresholdFilter") {
        auto threshold_filter = filter->as<ob::ThresholdFilter>();
        threshold_filter->setValueRange(threshold_filter_min_, threshold_filter_max_);
      } else if (filter_name == "SpatialAdvancedFilter") {
        auto spatial_filter = filter->as<ob::SpatialAdvancedFilter>();
        OBSpatialAdvancedFilterParams params{};
        params.alpha = spatial_filter_alpha_;
        params.magnitude = spatial_filter_magnitude_;
        params.radius = spatial_filter_radius_;
        params.disp_diff = spatial_filter_diff_threshold_;
        spatial_filter->setFilterParams(params);
      } else if (filter_name == "TemporalFilter") {
        auto temporal_filter = filter->as<ob::TemporalFilter>();
        temporal_filter->setDiffScale(temporal_filter_diff_threshold_);
        temporal_filter->setWeight(temporal_filter_weight_);
      } else if (filter_name == "HoleFillingFilter") {
        auto hole_filling_filter = filter->as<ob::HoleFillingFilter>();
        OBHoleFillingMode hole_filling_mode = holeFillingModeFromString(hole_filling_filter_mode_);
        hole_filling_filter->setFilterMode(hole_filling_mode);
      } else if (filter_name == "SequenceIdFilter") {
        auto sequenced_filter = filter->as<ob::SequenceIdFilter>();
        sequenced_filter->selectSequenceId(sequence_id_filter_id_);
      } else if (filter_name == "NoiseRemovalFilter") {
        auto noise_removal_filter = filter->as<ob::NoiseRemovalFilter>();
        OBNoiseRemovalFilterParams params{};
        params.disp_diff = noise_removal_filter_min_diff_;
        params.max_size = noise_removal_filter_max_size_;
        noise_removal_filter->setFilterParams(params);
      } else if (filter_name == "HdrMerge") {
        // do nothing
      } else {
        RCLCPP_ERROR_STREAM(logger_, "Unsupported filter: " << filter_name);
      }
    }
    if (!depth_work_mode_.empty()) {
      device_->switchDepthWorkMode(depth_work_mode_.c_str());
    }
    if (sync_mode_ != OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN) {
      auto sync_config = device_->getMultiDeviceSyncConfig();
      sync_config.syncMode = sync_mode_;
      sync_config.depthDelayUs = depth_delay_us_;
      sync_config.colorDelayUs = color_delay_us_;
      sync_config.trigger2ImageDelayUs = trigger2image_delay_us_;
      sync_config.triggerOutDelayUs = trigger_out_delay_us_;
      sync_config.triggerOutEnable = trigger_out_enabled_;
      device_->setMultiDeviceSyncConfig(sync_config);
    }
    if (info->pid() == GEMINI2_PID) {
      auto default_precision_level = device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
      if (default_precision_level != depth_precision_) {
        device_->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, depth_precision_);
      }

      int32_t precisionLevel = device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
      RCLCPP_INFO_STREAM(logger_, "Depth precision level:" << precisionLevel);
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
          device_->setBoolProperty(mirrorPropertyID, flip_stream_[stream_index]);
        }
      }
    }

    if (!depth_filter_config_.empty() && enable_depth_filter_) {
      RCLCPP_INFO_STREAM(logger_, "Load depth filter config: " << depth_filter_config_);
      device_->loadDepthFilterConfig(depth_filter_config_.c_str());
    } else {
      if (device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE)) {
        device_->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL, enable_soft_filter_);
      }
    }

    if (device_->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, enable_color_auto_exposure_);
    }

    if (device_->isPropertySupported(OB_PROP_IR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL, enable_ir_auto_exposure_);
    }

    if (device_->isPropertySupported(OB_PROP_IR_LONG_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_IR_LONG_EXPOSURE_BOOL, enable_ir_long_exposure_);
    }

    if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE)) {
      auto default_soft_filter_max_diff = device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
      if (soft_filter_max_diff_ != -1 && default_soft_filter_max_diff != soft_filter_max_diff_) {
        device_->setIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT, soft_filter_max_diff_);
      }
    }

    if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, OB_PERMISSION_WRITE)) {
      auto default_soft_filter_speckle_size =
          device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
      if (soft_filter_speckle_size_ != -1 &&
          default_soft_filter_speckle_size != soft_filter_speckle_size_) {
        device_->setIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, soft_filter_speckle_size_);
      }
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to setup devices: " << e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to setup devices: " << e.what());
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
      CHECK(profiles->count() > 0);
      for (size_t i = 0; i < profiles->count(); i++) {
        auto profile = profiles->getProfile(i)->as<ob::VideoStreamProfile>();
        RCLCPP_DEBUG_STREAM(
            logger_, "Sensor profile: "
                         << "stream_type: " << magic_enum::enum_name(profile->type())
                         << "Format: " << profile->format() << ", Width: " << profile->width()
                         << ", Height: " << profile->height() << ", FPS: " << profile->fps());
        supported_profiles_[elem].emplace_back(profile);
      }
      std::shared_ptr<ob::VideoStreamProfile> selected_profile;
      std::shared_ptr<ob::VideoStreamProfile> default_profile;
      try {
        selected_profile =
            profiles->getVideoStreamProfile(width_[elem], height_[elem], format_[elem], fps_[elem]);
        default_profile =
            profiles->getVideoStreamProfile(width_[elem], height_[elem], format_[elem]);
      } catch (const ob::Error &ex) {
        RCLCPP_ERROR_STREAM(
            logger_, "Failed to get " << stream_name_[elem] << " << profile: " << ex.getMessage());
        RCLCPP_ERROR_STREAM(
            logger_, "Stream: " << magic_enum::enum_name(elem.first)
                                << ", Stream Index: " << elem.second << ", Width: " << width_[elem]
                                << ", Height: " << height_[elem] << ", FPS: " << fps_[elem]
                                << ", Format: " << magic_enum::enum_name(format_[elem]));
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
      images_[elem] =
          cv::Mat(height_[elem], width_[elem], image_format_[elem], cv::Scalar(0, 0, 0));
      RCLCPP_INFO_STREAM(
          logger_, " stream " << stream_name_[elem] << " is enabled - width: " << width_[elem]
                              << ", height: " << height_[elem] << ", fps: " << fps_[elem] << ", "
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
    pipeline_->enableFrameSync();
  } else {
    pipeline_->disableFrameSync();
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
  imuPipeline_->enableFrameSync();
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
        CHECK(imu_profile);
        RCLCPP_INFO_STREAM(logger_, "start " << stream_name_[stream_index] << " stream");
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
    return;
  }
  try {
    pipeline_->stop();
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to stop pipeline: " << e.getMessage());
  }
}

void OBCameraNode::stopIMU() {
  if (enable_sync_output_accel_gyro_) {
    if (!imu_sync_output_start_ || !imuPipeline_) {
      return;
    }
    try {
      imuPipeline_->stop();
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to stop imu pipeline: " << e.getMessage());
    }
  } else {
    for (const auto &stream_index : HID_STREAMS) {
      if (imu_started_[stream_index]) {
        CHECK(sensors_.count(stream_index));
        RCLCPP_INFO_STREAM(logger_, "stop " << stream_name_[stream_index] << " stream");
        sensors_[stream_index]->stop();
        imu_started_[stream_index] = false;
      }
    }
  }
}

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
    setAndGetNodeParameter(width_[stream_index], param_name, IMAGE_WIDTH);
    param_name = stream_name_[stream_index] + "_height";
    setAndGetNodeParameter(height_[stream_index], param_name, IMAGE_HEIGHT);
    param_name = stream_name_[stream_index] + "_fps";
    setAndGetNodeParameter(fps_[stream_index], param_name, IMAGE_FPS);
    param_name = "enable_" + stream_name_[stream_index];
    setAndGetNodeParameter(enable_stream_[stream_index], param_name, false);
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
  setAndGetNodeParameter(tf_publish_rate_, "tf_publish_rate", 10.0);
  setAndGetNodeParameter(depth_registration_, "depth_registration", false);
  setAndGetNodeParameter(enable_point_cloud_, "enable_point_cloud", true);
  setAndGetNodeParameter<std::string>(ir_info_url_, "ir_info_url", "");
  setAndGetNodeParameter<std::string>(color_info_url_, "color_info_url", "");
  setAndGetNodeParameter(enable_colored_point_cloud_, "enable_colored_point_cloud", false);
  setAndGetNodeParameter(enable_point_cloud_, "enable_point_cloud", true);
  setAndGetNodeParameter<std::string>(point_cloud_qos_, "point_cloud_qos", "default");
  setAndGetNodeParameter(enable_d2c_viewer_, "enable_d2c_viewer", false);
  setAndGetNodeParameter(enable_hardware_d2d_, "enable_hardware_d2d", true);
  setAndGetNodeParameter(enable_soft_filter_, "enable_soft_filter", true);
  setAndGetNodeParameter<std::string>(depth_filter_config_, "depth_filter_config", "");
  if (!depth_filter_config_.empty()) {
    enable_depth_filter_ = true;
  }
  setAndGetNodeParameter(enable_frame_sync_, "enable_frame_sync", false);
  setAndGetNodeParameter(enable_color_auto_exposure_, "enable_color_auto_exposure", true);
  setAndGetNodeParameter(enable_ir_auto_exposure_, "enable_ir_auto_exposure", true);
  setAndGetNodeParameter(enable_ir_long_exposure_, "enable_ir_long_exposure", true);
  setAndGetNodeParameter<std::string>(depth_work_mode_, "depth_work_mode", "");
  setAndGetNodeParameter<std::string>(sync_mode_str_, "sync_mode", "close");
  setAndGetNodeParameter(depth_delay_us_, "depth_delay_us", 0);
  setAndGetNodeParameter(color_delay_us_, "color_delay_us", 0);
  setAndGetNodeParameter(trigger2image_delay_us_, "trigger2image_delay_us", 0);
  setAndGetNodeParameter(trigger_out_delay_us_, "trigger_out_delay_us", 0);
  setAndGetNodeParameter(trigger_out_enabled_, "trigger_out_enabled", false);
  setAndGetNodeParameter<std::string>(depth_precision_str_, "depth_precision", "1mm");
  std::transform(sync_mode_str_.begin(), sync_mode_str_.end(), sync_mode_str_.begin(), ::toupper);
  sync_mode_ = OBSyncModeFromString(sync_mode_str_);
  depth_precision_ = depthPrecisionLevelFromString(depth_precision_str_);
  if (enable_colored_point_cloud_) {
    depth_registration_ = true;
  }
  setAndGetNodeParameter<bool>(enable_ldp_, "enable_ldp", true);
  setAndGetNodeParameter<int>(soft_filter_max_diff_, "soft_filter_max_diff", -1);
  setAndGetNodeParameter<int>(soft_filter_speckle_size_, "soft_filter_speckle_size", -1);
  setAndGetNodeParameter<double>(liner_accel_cov_, "linear_accel_cov", 0.0003);
  setAndGetNodeParameter<double>(angular_vel_cov_, "angular_vel_cov", 0.02);
  setAndGetNodeParameter<bool>(ordered_pc_, "ordered_pc", false);
  setAndGetNodeParameter<int>(max_save_images_count_, "max_save_images_count", 10);
  setAndGetNodeParameter<bool>(use_hardware_time_, "use_hardware_time", true);
  setAndGetNodeParameter<bool>(enable_depth_scale_, "enable_depth_scale", true);
  setAndGetNodeParameter<std::string>(device_preset_, "device_preset", "Default");
  setAndGetNodeParameter<bool>(enable_decimation_filter_, "enable_decimation_filter", false);
  setAndGetNodeParameter<bool>(enable_hdr_merge_, "enable_hdr_merge", false);
  setAndGetNodeParameter<bool>(enable_sequence_id_filter_, "enable_sequence_id_filter", false);
  setAndGetNodeParameter<bool>(enable_threshold_filter_, "enable_threshold_filter", false);
  setAndGetNodeParameter<bool>(enable_noise_removal_filter_, "enable_noise_removal_filter", true);
  setAndGetNodeParameter<bool>(enable_spatial_filter_, "enable_spatial_filter", true);
  setAndGetNodeParameter<bool>(enable_temporal_filter_, "enable_temporal_filter", false);
  setAndGetNodeParameter<bool>(enable_hole_filling_filter_, "enable_hole_filling_filter", false);
  setAndGetNodeParameter<int>(decimation_filter_scale_range_, "decimation_filter_scale_range", 2);
  setAndGetNodeParameter<int>(sequence_id_filter_id_, "sequence_id_filter_id", 1);
  setAndGetNodeParameter<int>(threshold_filter_max_, "threshold_filter_max", 16000);
  setAndGetNodeParameter<int>(threshold_filter_min_, "threshold_filter_min", 0);
  setAndGetNodeParameter<int>(noise_removal_filter_min_diff_, "noise_removal_filter_min_diff", 8);
  setAndGetNodeParameter<int>(noise_removal_filter_max_size_, "noise_removal_filter_max_size", 80);
  setAndGetNodeParameter<float>(spatial_filter_alpha_, "spatial_filter_alpha", 0.5);
  setAndGetNodeParameter<int>(spatial_filter_diff_threshold_, "spatial_filter_diff_threshold", 8);
  setAndGetNodeParameter<int>(spatial_filter_magnitude_, "spatial_filter_magnitude", 1);
  setAndGetNodeParameter<int>(spatial_filter_radius_, "spatial_filter_radius", 1);
  setAndGetNodeParameter<float>(temporal_filter_diff_threshold_, "temporal_filter_diff_threshold",
                                0.1);
  setAndGetNodeParameter<float>(temporal_filter_weight_, "temporal_filter_weight", 0.4);
  setAndGetNodeParameter<std::string>(hole_filling_filter_mode_, "hole_filling_filter_mode",
                                      "FILL_TOP");
  setAndGetNodeParameter<std::string>(align_mode_, "align_mode", "HW");
  setAndGetNodeParameter<double>(diagnostic_period_, "diagnostic_period", 1.0);
  setAndGetNodeParameter<bool>(enable_laser_, "enable_laser", false);
}

void OBCameraNode::setupTopics() {
  getParameters();
  setupDevices();
  setupProfiles();
  setupCameraCtrlServices();
  setupPublishers();
  setupDiagnosticUpdater();
}

void OBCameraNode::onTemperatureUpdate(diagnostic_updater::DiagnosticStatusWrapper &status) {
  try {
    OBDeviceTemperature temperature;
    uint32_t data_size = sizeof(OBDeviceTemperature);
    device_->getStructuredData(OB_STRUCT_DEVICE_TEMPERATURE, &temperature, &data_size);
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
  RCLCPP_INFO_STREAM(logger_, "Publish diagnostics every " << diagnostic_period_ << " seconds");
  auto info = device_->getDeviceInfo();
  std::string serial_number = info->serialNumber();
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(node_, diagnostic_period_);
  diagnostic_updater_->setHardwareID(serial_number);
  diagnostic_updater_->add("Temperatures", this, &OBCameraNode::onTemperatureUpdate);
}

void OBCameraNode::setupPipelineConfig() {
  if (pipeline_config_) {
    pipeline_config_.reset();
  }
  pipeline_config_ = std::make_shared<ob::Config>();
  pipeline_config_->setDepthScaleRequire(enable_depth_scale_);
  if (depth_registration_ && enable_stream_[COLOR] && enable_stream_[DEPTH]) {
    auto info = device_->getDeviceInfo();
    auto pid = info->pid();
    if (pid == FEMTO_BOLT_PID || pid == GEMINI2R_PID || pid == GEMINI2RL_PID) {
      RCLCPP_INFO_STREAM(logger_, "set align mode ALIGN_D2C_SW_MODE.");
      pipeline_config_->setAlignMode(ALIGN_D2C_SW_MODE);
    } else {
      RCLCPP_INFO_STREAM(logger_, "set align mode ALIGN_D2C_HW_MODE.");
      OBAlignMode align_mode = align_mode_ == "HW" ? ALIGN_D2C_HW_MODE : ALIGN_D2C_SW_MODE;
      pipeline_config_->setAlignMode(align_mode);
    }
  }
  for (const auto &stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      RCLCPP_INFO_STREAM(logger_, "Enable " << stream_name_[stream_index] << " stream");
      RCLCPP_INFO_STREAM(
          logger_, "Stream " << stream_name_[stream_index] << " width: " << width_[stream_index]
                             << " height: " << height_[stream_index] << " fps: "
                             << fps_[stream_index] << " format: " << format_str_[stream_index]);
      pipeline_config_->enableStream(stream_profile_[stream_index]);
    }
  }
}

void OBCameraNode::setupPublishers() {
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  auto point_cloud_qos_profile = getRMWQosProfileFromString(point_cloud_qos_);
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
  for (const auto &stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    std::string name = stream_name_[stream_index];
    std::string topic = name + "/image_raw";
    auto image_qos = image_qos_[stream_index];
    auto image_qos_profile = getRMWQosProfileFromString(image_qos);
    image_publishers_[stream_index] =
        image_transport::create_publisher(node_, topic, image_qos_profile);
    topic = name + "/camera_info";
    auto camera_info_qos = camera_info_qos_[stream_index];
    auto camera_info_qos_profile = getRMWQosProfileFromString(camera_info_qos);
    camera_info_publishers_[stream_index] = node_->create_publisher<CameraInfo>(
        topic, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(camera_info_qos_profile),
                           camera_info_qos_profile));
    metadata_publishers_[stream_index] = node_->create_publisher<orbbec_camera_msgs::msg::Metadata>(
        name + "/metadata",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(camera_info_qos_profile),
                    camera_info_qos_profile));
  }

  if (enable_sync_output_accel_gyro_) {
    std::string topic_name = stream_name_[GYRO] + "_" + stream_name_[ACCEL] + "/sample";
    auto data_qos = getRMWQosProfileFromString(imu_qos_[GYRO]);
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
      imu_publishers_[stream_index] = node_->create_publisher<sensor_msgs::msg::Imu>(
          data_topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos));
      data_topic_name = stream_name_[stream_index] + "/imu_info";
      imu_info_publishers_[stream_index] =
          node_->create_publisher<orbbec_camera_msgs::msg::IMUInfo>(
              data_topic_name,
              rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos));
    }
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA0]) {
    depth_to_other_extrinsics_publishers_[INFRA0] =
        node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/depth_to_ir", rclcpp::QoS(1).transient_local());
  }
  if (enable_stream_[DEPTH] && enable_stream_[COLOR]) {
    depth_to_other_extrinsics_publishers_[COLOR] =
        node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/depth_to_color", rclcpp::QoS(1).transient_local());
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA1]) {
    depth_to_other_extrinsics_publishers_[INFRA1] =
        node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/depth_to_left_ir", rclcpp::QoS(1).transient_local());
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA2]) {
    depth_to_other_extrinsics_publishers_[INFRA2] =
        node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/depth_to_right_ir", rclcpp::QoS(1).transient_local());
  }
  filter_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "depth_filter_status", rclcpp::QoS(1).transient_local());
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
  if (depth_cloud_pub_->get_subscription_count() == 0 || !enable_point_cloud_) {
    return;
  }
  if (!depth_frame_) {
    return;
  }
  auto depth_frame = depth_frame_->as<ob::DepthFrame>();
  if (!depth_frame) {
    RCLCPP_ERROR_STREAM(logger_, "depth frame is null");
    return;
  }
  auto width = depth_frame->width();
  auto height = depth_frame->height();
  auto depth_profile = stream_profile_[DEPTH]->as<ob::VideoStreamProfile>();
  CHECK_NOTNULL(depth_profile.get());
  auto depth_intrinsics = depth_profile->getIntrinsic();
  float fdx = depth_intrinsics.fx * ((float)(width) / depth_intrinsics.width);
  float fdy = depth_intrinsics.fy * ((float)(height) / depth_intrinsics.height);
  fdx = 1 / fdx;
  fdy = 1 / fdy;
  float u0 = depth_intrinsics.cx * ((float)(width) / depth_intrinsics.width);
  float v0 = depth_intrinsics.cy * ((float)(height) / depth_intrinsics.height);

  const auto *depth_data = (uint16_t *)depth_frame->data();
  sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg_);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(width * height);
  point_cloud_msg_.width = depth_frame->width();
  point_cloud_msg_.height = depth_frame->height();
  point_cloud_msg_.row_step = point_cloud_msg_.width * point_cloud_msg_.point_step;
  point_cloud_msg_.data.resize(point_cloud_msg_.height * point_cloud_msg_.row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  size_t valid_count = 0;
  const static float MIN_DISTANCE = 20.0;
  const static float MAX_DISTANCE = 10000.0;
  double depth_scale = depth_frame->getValueScale();
  const static float min_depth = MIN_DISTANCE / depth_scale;
  const static float max_depth = MAX_DISTANCE / depth_scale;
  for (uint32_t y = 0; y < height; y++) {
    for (uint32_t x = 0; x < width; x++) {
      bool valid_point = true;
      if (depth_data[y * width + x] < min_depth || depth_data[y * width + x] > max_depth) {
        valid_point = false;
      }
      if (valid_point || ordered_pc_) {
        float xf = (x - u0) * fdx;
        float yf = (y - v0) * fdy;
        float zf = depth_data[y * width + x] * depth_scale;
        *iter_x = zf * xf / 1000.0;
        *iter_y = zf * yf / 1000.0;
        *iter_z = zf / 1000.0;
        ++iter_x, ++iter_y, ++iter_z;
        valid_count++;
      }
    }
  }
  if (!ordered_pc_) {
    point_cloud_msg_.is_dense = true;
    point_cloud_msg_.width = valid_count;
    point_cloud_msg_.height = 1;
    modifier.resize(valid_count);
  }
  auto timestamp = fromMsToROSTime(depth_frame->timeStamp());
  std::string frame_id = depth_registration_ ? optical_frame_id_[COLOR] : optical_frame_id_[DEPTH];
  point_cloud_msg_.header.stamp = timestamp;
  point_cloud_msg_.header.frame_id = frame_id;
  depth_cloud_pub_->publish(point_cloud_msg_);

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
    saveDepthPointsToPly(point_cloud_msg_, filename);
  }
}

void OBCameraNode::publishColoredPointCloud(const std::shared_ptr<ob::FrameSet> &frame_set) {
  if (depth_registration_cloud_pub_->get_subscription_count() == 0 ||
      !enable_colored_point_cloud_) {
    return;
  }
  if (!depth_frame_) {
    return;
  }
  CHECK_NOTNULL(depth_frame_.get());
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
    RCLCPP_ERROR(logger_, "Depth (%d x %d) and color (%d x %d) frame size mismatch", depth_width,
                 depth_height, color_width, color_height);
    return;
  }
  auto color_profile = stream_profile_[COLOR]->as<ob::VideoStreamProfile>();
  CHECK_NOTNULL(color_profile.get());
  auto intrinsics = color_profile->getIntrinsic();
  float fdx = intrinsics.fx * ((float)(color_width) / intrinsics.width);
  float fdy = intrinsics.fy * ((float)(color_height) / intrinsics.height);
  fdx = 1 / fdx;
  fdy = 1 / fdy;
  float u0 = intrinsics.cx * ((float)(color_width) / intrinsics.width);
  float v0 = intrinsics.cy * ((float)(color_height) / intrinsics.height);
  const auto *depth_data = (uint16_t *)depth_frame->data();
  const auto *color_data = (uint8_t *)(rgb_buffer_);
  sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg_);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  point_cloud_msg_.width = color_frame->width();
  point_cloud_msg_.height = color_frame->height();
  std::string format_str = "rgb";
  point_cloud_msg_.point_step =
      addPointField(point_cloud_msg_, format_str, 1, sensor_msgs::msg::PointField::FLOAT32,
                    static_cast<int>(point_cloud_msg_.point_step));
  point_cloud_msg_.row_step = point_cloud_msg_.width * point_cloud_msg_.point_step;
  point_cloud_msg_.data.resize(point_cloud_msg_.height * point_cloud_msg_.row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud_msg_, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud_msg_, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud_msg_, "b");
  size_t valid_count = 0;
  static const float MIN_DISTANCE = 20.0;
  static const float MAX_DISTANCE = 10000.0;
  double depth_scale = depth_frame->getValueScale();
  static float min_depth = MIN_DISTANCE / depth_scale;
  static float max_depth = MAX_DISTANCE / depth_scale;
  for (uint32_t y = 0; y < color_height; y++) {
    for (uint32_t x = 0; x < color_width; x++) {
      bool valid_point = true;
      float depth = depth_data[y * depth_width + x];
      if (depth < min_depth || depth > max_depth) {
        valid_point = false;
      }
      if (valid_point || ordered_pc_) {
        float xf = (x - u0) * fdx;
        float yf = (y - v0) * fdy;
        float zf = depth * depth_scale;
        *iter_x = zf * xf / 1000.0;
        *iter_y = zf * yf / 1000.0;
        *iter_z = zf / 1000.0;
        *iter_r = color_data[(y * color_width + x) * 3];
        *iter_g = color_data[(y * color_width + x) * 3 + 1];
        *iter_b = color_data[(y * color_width + x) * 3 + 2];
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
        ++valid_count;
      }
    }
  }
  if (!ordered_pc_) {
    point_cloud_msg_.is_dense = true;
    point_cloud_msg_.width = valid_count;
    point_cloud_msg_.height = 1;
    modifier.resize(valid_count);
  }
  auto timestamp = fromUsToROSTime(depth_frame->timeStampUs());
  point_cloud_msg_.header.stamp = timestamp;
  point_cloud_msg_.header.frame_id = optical_frame_id_[COLOR];
  depth_registration_cloud_pub_->publish(point_cloud_msg_);
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
    saveRGBPointCloudMsgToPly(point_cloud_msg_, filename);
  }
}

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
    if (filter->isEnabled()) {
      frame = filter->process(frame);
      if (frame == nullptr) {
        RCLCPP_ERROR_STREAM(logger_, "Depth filter process failed");
        break;
      }
    }
  }
  return frame;
}

void OBCameraNode::onNewFrameSetCallback(const std::shared_ptr<ob::FrameSet> &frame_set) {
  if (!is_running_.load()) {
    return;
  }
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (frame_set == nullptr) {
    return;
  }
  try {
    if (!tf_published_) {
      publishStaticTransforms();
      tf_published_ = true;
    }

    // is_color_frame_decoded_ = decodeColorFrameToBuffer(frame_set->colorFrame(), rgb_buffer_);
    std::shared_ptr<ob::ColorFrame> colorFrame = frame_set->colorFrame();
    depth_frame_ = frame_set->getFrame(OB_FRAME_DEPTH);
    depth_frame_ = processDepthFrameFilter(depth_frame_);
    if (enable_stream_[COLOR] && colorFrame) {
      std::lock_guard<std::mutex> colorLock(colorFrameMtx_);
      colorFrameQueue_.push(frame_set);
      colorFrameCV_.notify_all();
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
        if (frame_type == OB_FRAME_DEPTH) {
          frame = depth_frame_;
        }

        std::shared_ptr<ob::Frame> irFrame = decodeIRMJPGFrame(frame);
        if (irFrame) {
          onNewFrameCallback(irFrame, stream_index);
        } else {
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
    std::unique_lock<std::mutex> lock(colorFrameMtx_);
    colorFrameCV_.wait(lock,
                       [this]() { return !colorFrameQueue_.empty() || !(is_running_.load()); });

    if (!rclcpp::ok() || !is_running_.load()) {
      break;
    }

    std::shared_ptr<ob::FrameSet> frameSet = colorFrameQueue_.front();
    is_color_frame_decoded_ = decodeColorFrameToBuffer(frameSet->colorFrame(), rgb_buffer_);
    publishPointCloud(frameSet);
    onNewFrameCallback(frameSet->colorFrame(), IMAGE_STREAMS.at(2));
    colorFrameQueue_.pop();
  }

  RCLCPP_INFO_STREAM(logger_, "Color frame thread exit!");
}

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

bool OBCameraNode::decodeColorFrameToBuffer(const std::shared_ptr<ob::Frame> &frame,
                                            uint8_t *buffer) {
  if (frame == nullptr) {
    return false;
  }
  if (!rgb_buffer_) {
    return false;
  }
  bool has_subscriber = image_publishers_[COLOR].getNumSubscribers() > 0;
  if (enable_colored_point_cloud_ && depth_registration_cloud_pub_->get_subscription_count() > 0) {
    has_subscriber = true;
  }
  if (!has_subscriber) {
    return false;
  }
  if (metadata_publishers_[COLOR]->get_subscription_count() > 0) {
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

std::shared_ptr<ob::Frame> OBCameraNode::decodeIRMJPGFrame(
    const std::shared_ptr<ob::Frame> &frame) {
  if (frame->format() == OB_FORMAT_MJPEG &&
      (frame->type() == OB_FRAME_IR || frame->type() == OB_FRAME_IR_LEFT ||
       frame->type() == OB_FRAME_IR_RIGHT)) {
    auto video_frame = frame->as<ob::IRFrame>();

    cv::Mat mjpgMat(1, video_frame->dataSize(), CV_8UC1, video_frame->data());
    cv::Mat irRawMat = cv::imdecode(mjpgMat, cv::IMREAD_GRAYSCALE);

    std::shared_ptr<ob::Frame> irFrame = ob::FrameHelper::createFrame(
        video_frame->type(), video_frame->format(), video_frame->width(), video_frame->height(), 0);

    uint32_t buffer_size = irRawMat.rows * irRawMat.cols * irRawMat.channels();

    if (buffer_size > irFrame->dataSize()) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Insufficient buffer size allocation,failed to decode ir mjpg frame!");
      return nullptr;
    }

    memcpy(irFrame->data(), irRawMat.data, buffer_size);
    ob::FrameHelper::setFrameDeviceTimestamp(irFrame, video_frame->timeStamp());
    ob::FrameHelper::setFrameDeviceTimestampUs(irFrame, video_frame->timeStampUs());
    ob::FrameHelper::setFrameSystemTimestamp(irFrame, video_frame->systemTimeStamp());
    return irFrame;
  }

  return nullptr;
}

void OBCameraNode::onNewFrameCallback(const std::shared_ptr<ob::Frame> &frame,
                                      const stream_index_pair &stream_index) {
  if (frame == nullptr) {
    return;
  }
  bool has_subscriber = image_publishers_[stream_index].getNumSubscribers() > 0;
  has_subscriber =
      has_subscriber || camera_info_publishers_[stream_index]->get_subscription_count() > 0;
  has_subscriber =
      has_subscriber || metadata_publishers_[stream_index]->get_subscription_count() > 0;
  if (!has_subscriber) {
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
  auto timestamp = use_hardware_time_ ? fromUsToROSTime(video_frame->timeStampUs())
                                      : fromMsToROSTime(video_frame->systemTimeStamp());
  auto stream_profile = frame->getStreamProfile();
  CHECK_NOTNULL(stream_profile);
  auto video_stream_profile = stream_profile->as<ob::VideoStreamProfile>();
  CHECK_NOTNULL(video_stream_profile);
  const auto &intrinsic = video_stream_profile->getIntrinsic();
  const auto &distortion = video_stream_profile->getDistortion();
  std::string frame_id =
      depth_registration_ ? depth_aligned_frame_id_[stream_index] : optical_frame_id_[stream_index];
  auto camera_info = convertToCameraInfo(intrinsic, distortion, width);
  camera_info.header.stamp = timestamp;
  camera_info.header.frame_id = frame_id;
  camera_info.width = width;
  camera_info.height = height;
  if (frame->type() == OB_FRAME_IR_RIGHT) {
    auto left_video_profile = stream_profile_[INFRA1]->as<ob::VideoStreamProfile>();
    auto ex = video_stream_profile->getExtrinsicTo(left_video_profile);
    float fx = camera_info.k.at(0);
    float fy = camera_info.k.at(4);
    camera_info.p.at(3) = -fx * ex.trans[0] + 0.0;
    camera_info.p.at(7) = -fy * ex.trans[1] + 0.0;
  }
  CHECK(camera_info_publishers_.count(stream_index) > 0);
  camera_info_publishers_[stream_index]->publish(camera_info);
  auto &image = images_[stream_index];
  if (image.empty() || image.cols != width || image.rows != height) {
    image.create(height, width, image_format_[stream_index]);
  }
  if (frame->type() == OB_FRAME_COLOR && !is_color_frame_decoded_) {
    RCLCPP_ERROR(logger_, "color frame is not decoded");
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
  auto image_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), encoding_[stream_index], image).toImageMsg();
  image_msg->header.stamp = timestamp;
  image_msg->is_bigendian = false;
  image_msg->step = width * unit_step_size_[stream_index];
  image_msg->header.frame_id = frame_id;
  publishMetadata(frame, stream_index, image_msg->header);

  CHECK(image_publishers_.count(stream_index) > 0);
  image_publishers_[stream_index].publish(image_msg);
  saveImageToFile(stream_index, image, image_msg);
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
                                   const sensor_msgs::msg::Image::SharedPtr &image_msg) {
  if (save_images_[stream_index]) {
    auto now = time(nullptr);
    std::stringstream ss;
    ss << std::put_time(localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = std::filesystem::current_path().string();
    auto fps = fps_[stream_index];
    int index = save_images_count_[stream_index];
    std::string file_suffix = stream_index == COLOR ? ".png" : ".raw";
    std::string filename = current_path + "/image/" + stream_name_[stream_index] + "_" +
                           std::to_string(image_msg->width) + "x" +
                           std::to_string(image_msg->height) + "_" + std::to_string(fps) + "hz_" +
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
  auto timestamp = fromUsToROSTime(accelframe->globalTimeStampUs());
  imu_msg.header.stamp = timestamp;
  auto gyro_frame = gryoframe->as<ob::GyroFrame>();
  auto gyroData = gyro_frame->value();
  imu_msg.angular_velocity.x = gyroData.x;
  imu_msg.angular_velocity.y = gyroData.y;
  imu_msg.angular_velocity.z = gyroData.z;
  auto accel_frame = accelframe->as<ob::AccelFrame>();
  auto accelData = accel_frame->value();
  imu_msg.linear_acceleration.x = accelData.x;
  imu_msg.linear_acceleration.y = accelData.y;
  imu_msg.linear_acceleration.z = accelData.z;
  imu_gyro_accel_publisher_->publish(imu_msg);
  for (const auto &stream_index : {GYRO, ACCEL}) {
    auto imu_info = createIMUInfo(stream_index);
    imu_info.header = imu_msg.header;
    imu_info.header.frame_id = imu_optical_frame_id_;
    imu_info_publishers_[stream_index]->publish(imu_info);
  }
}

void OBCameraNode::onNewIMUFrameCallback(const std::shared_ptr<ob::Frame> &frame,
                                         const stream_index_pair &stream_index) {
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
  if (frame->type() == OB_FRAME_GYRO) {
    auto gyro_frame = frame->as<ob::GyroFrame>();
    auto data = gyro_frame->value();
    imu_msg.angular_velocity.x = data.x;
    imu_msg.angular_velocity.y = data.y;
    imu_msg.angular_velocity.z = data.z;
  } else if (frame->type() == OB_FRAME_ACCEL) {
    auto accel_frame = frame->as<ob::AccelFrame>();
    auto data = accel_frame->value();
    imu_msg.linear_acceleration.x = data.x;
    imu_msg.linear_acceleration.y = data.y;
    imu_msg.linear_acceleration.z = data.z;
  } else {
    RCLCPP_ERROR(logger_, "Unsupported IMU frame type");
    return;
  }
  imu_publishers_[stream_index]->publish(imu_msg);
  auto imu_info = createIMUInfo(stream_index);
  imu_info.header = imu_msg.header;
  imu_info.header.frame_id = imu_optical_frame_id_;
  imu_info_publishers_[stream_index]->publish(imu_info);
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
    RCLCPP_INFO_STREAM(logger_, "Publishing static transform from " << camera_link_frame_id_
                                                                    << " to "
                                                                    << stream_name_[stream_index]);
    RCLCPP_INFO_STREAM(logger_, "Translation " << trans[0] << ", " << trans[1] << ", " << trans[2]);
    RCLCPP_INFO_STREAM(logger_, "Rotation " << Q.getX() << ", " << Q.getY() << ", " << Q.getZ()
                                            << ", " << Q.getW());
    auto timestamp = node_->now();
    publishStaticTF(timestamp, trans, Q, camera_link_frame_id_, frame_id_[stream_index]);
    publishStaticTF(timestamp, zero_trans, quaternion_optical, frame_id_[stream_index],
                    optical_frame_id_[stream_index]);
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
    depth_to_other_extrinsics_[INFRA2] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    depth_to_other_extrinsics_publishers_[INFRA2]->publish(ex_msg);
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
