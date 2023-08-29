/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2022 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#include "orbbec_camera/ob_camera_node.h"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "orbbec_camera/utils.h"
#include <filesystem>

#if defined(USE_RK_HW_DECODER)
#include "orbbec_camera/rk_mpp_decoder.h"
#elif defined(USE_GST_HW_DECODER)
#include "orbbec_camera/gst_decoder.h"
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
  stream_name_[INFRA1] = "ir2";
  stream_name_[ACCEL] = "accel";
  stream_name_[GYRO] = "gyro";

  compression_params_.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params_.push_back(0);
  compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY);
  compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
  setupDefaultImageFormat();
  setupTopics();
#if defined(USE_RK_HW_DECODER)
  mjpeg_decoder_ = std::make_unique<RKMjpegDecoder>(width_[COLOR], height_[COLOR]);
#elif defined(USE_GST_HW_DECODER)
  mjpeg_decoder_ = std::make_unique<GstreamerMjpegDecoder>(
      width_[COLOR], height_[COLOR], jpeg_decoder_, video_convert_, jpeg_parse_);
#endif
  startStreams();
  if (enable_d2c_viewer_) {
    auto rgb_qos = getRMWQosProfileFromString(image_qos_[COLOR]);
    auto depth_qos = getRMWQosProfileFromString(image_qos_[DEPTH]);
    d2c_viewer_ = std::make_unique<D2CViewer>(node_, rgb_qos, depth_qos);
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
  RCLCPP_WARN_STREAM(logger_, "Do destroy ~OBCameraNode");
  is_running_.store(false);
  if (tf_thread_->joinable()) {
    tf_thread_->join();
  }
  RCLCPP_WARN_STREAM(logger_, "stop streams");
  stopStreams();
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
  }
  try {
    if (!depth_work_mode_.empty()) {
      device_->switchDepthWorkMode(depth_work_mode_.c_str());
    }
    if (sync_mode_ != OB_SYNC_MODE_CLOSE) {
      OBDeviceSyncConfig sync_config;
      sync_config.syncMode = sync_mode_;
      sync_config.irTriggerSignalInDelay = ir_trigger_signal_in_delay_;
      sync_config.rgbTriggerSignalInDelay = rgb_trigger_signal_in_delay_;
      sync_config.deviceTriggerSignalOutDelay = device_trigger_signal_out_delay_;
      device_->setSyncConfig(sync_config);
      if (device_->isPropertySupported(OB_PROP_SYNC_SIGNAL_TRIGGER_OUT_BOOL,
                                       OB_PERMISSION_READ_WRITE)) {
        device_->setBoolProperty(OB_PROP_SYNC_SIGNAL_TRIGGER_OUT_BOOL, sync_signal_trigger_out_);
      }
    }
    if (info->pid() == GEMINI2_PID) {
      auto default_precision_level = device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
      if (default_precision_level != depth_precision_) {
        device_->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, depth_precision_);
      }
    }

    device_->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL, enable_soft_filter_);
    device_->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, enable_color_auto_exposure_);
    device_->setBoolProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL, enable_ir_auto_exposure_);
    auto default_soft_filter_max_diff = device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
    if (soft_filter_max_diff_ != -1 && default_soft_filter_max_diff != soft_filter_max_diff_) {
      device_->setIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT, soft_filter_max_diff_);
    }
    auto default_soft_filter_speckle_size =
        device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
    if (soft_filter_speckle_size_ != -1 &&
        default_soft_filter_speckle_size != soft_filter_speckle_size_) {
      device_->setIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, soft_filter_speckle_size_);
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to setup devices: " << e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to setup devices: " << e.what());
  }
}

void OBCameraNode::setupProfiles() {
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
        RCLCPP_ERROR_STREAM(logger_, "Failed to get profile: " << ex.getMessage());
        RCLCPP_ERROR_STREAM(
            logger_, "Stream: " << magic_enum::enum_name(elem.first)
                                << ", Stream Index: " << elem.second << ", Width: " << width_[elem]
                                << ", Height: " << height_[elem] << ", FPS: " << fps_[elem]
                                << ", Format: " << magic_enum::enum_name(format_[elem]));
        throw;
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
  }
  pipeline_started_.store(true);
  startIMU();
}

void OBCameraNode::startIMU() {
  for (const auto &stream_index : HID_STREAMS) {
    if (enable_stream_[stream_index]) {
      CHECK(sensors_.count(stream_index));
      auto profile_list = sensors_[stream_index]->getStreamProfileList();
      for (size_t i = 0; i < profile_list->count(); i++) {
        auto item = profile_list->getProfile(i);
        if (stream_index == ACCEL) {
          auto profile = item->as<ob::AccelStreamProfile>();
          auto accel_rate = sampleRateFromString(imu_rate_[stream_index]);
          auto accel_range = fullAccelScaleRangeFromString(imu_range_[stream_index]);
          if (profile->fullScaleRange() == accel_range && profile->sampleRate() == accel_rate) {
            sensors_[stream_index]->start(
                profile, [this, stream_index](const std::shared_ptr<ob::Frame> &frame) {
                  onNewIMUFrameCallback(frame, stream_index);
                });
            imu_started_[stream_index] = true;
            RCLCPP_INFO_STREAM(logger_, "start accel stream with "
                                            << magic_enum::enum_name(accel_range) << " range and "
                                            << magic_enum::enum_name(accel_rate) << " rate");
          }
        } else if (stream_index == GYRO) {
          auto profile = item->as<ob::GyroStreamProfile>();
          auto gyro_rate = sampleRateFromString(imu_rate_[stream_index]);
          auto gyro_range = fullGyroScaleRangeFromString(imu_range_[stream_index]);
          if (profile->fullScaleRange() == gyro_range && profile->sampleRate() == gyro_rate) {
            sensors_[stream_index]->start(
                profile, [this, stream_index](const std::shared_ptr<ob::Frame> &frame) {
                  onNewIMUFrameCallback(frame, stream_index);
                });
            RCLCPP_INFO_STREAM(logger_, "start gyro stream with "
                                            << magic_enum::enum_name(gyro_range) << " range and "
                                            << magic_enum::enum_name(gyro_rate) << " rate");
            imu_started_[stream_index] = true;
          }
        }
      }
    }
  }
  for (const auto &stream_index : HID_STREAMS) {
    if (enable_stream_[stream_index] && !imu_started_[stream_index]) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to start IMU stream: "
                                       << magic_enum::enum_name(stream_index.first)
                                       << ", please check the imu_rate and imu_range parameters");
    }
  }
}

void OBCameraNode::stopStreams() {
  if (!pipeline_started_ || !pipeline_) {
    return;
  }
  try {
    pipeline_->stop();
    stopIMU();
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to stop pipeline: " << e.getMessage());
  }
}

void OBCameraNode::stopIMU() {
  for (const auto &stream_index : HID_STREAMS) {
    if (imu_started_[stream_index]) {
      CHECK(sensors_.count(stream_index));
      RCLCPP_INFO_STREAM(logger_, "stop " << stream_name_[stream_index] << " stream");
      sensors_[stream_index]->stop();
      imu_started_[stream_index] = false;
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
    param_name = camera_name_ + "_" + stream_name_[stream_index] + "_frame_id";
    std::string default_frame_id = camera_name_ + "_" + stream_name_[stream_index] + "_frame";
    setAndGetNodeParameter(frame_id_[stream_index], param_name, default_frame_id);
    std::string default_optical_frame_id =
        camera_name_ + "_" + stream_name_[stream_index] + "_optical_frame";
    param_name = stream_name_[stream_index] + "_optical_frame_id";
    setAndGetNodeParameter(optical_frame_id_[stream_index], param_name, default_optical_frame_id);
    depth_aligned_frame_id_[stream_index] = stream_name_[COLOR] + "_optical_frame";
    param_name = stream_name_[stream_index] + "_format";
    setAndGetNodeParameter(format_str_[stream_index], param_name, format_str_[stream_index]);
    format_[stream_index] = OBFormatFromString(format_str_[stream_index]);
    if (format_[stream_index] == OB_FORMAT_Y8) {
      CHECK(stream_index.first != OB_STREAM_COLOR);
      image_format_[stream_index] = CV_8UC1;
      encoding_[stream_index] = stream_index.first == OB_STREAM_DEPTH
                                    ? sensor_msgs::image_encodings::TYPE_8UC1
                                    : sensor_msgs::image_encodings::MONO8;
      unit_step_size_[stream_index] = sizeof(uint8_t);
    }
    param_name = stream_name_[stream_index] + "_qos";
    setAndGetNodeParameter<std::string>(image_qos_[stream_index], param_name, "default");
    param_name = stream_name_[stream_index] + "_camera_info_qos";
    setAndGetNodeParameter<std::string>(camera_info_qos_[stream_index], param_name, "default");
  }

  for (const auto &stream_index : HID_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_qos";
    setAndGetNodeParameter<std::string>(imu_qos_[stream_index], param_name, "default");
    param_name = "enable_" + stream_name_[stream_index];
    setAndGetNodeParameter(enable_stream_[stream_index], param_name, false);
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
    depth_aligned_frame_id_[stream_index] = stream_name_[COLOR] + "_optical_frame";
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
  setAndGetNodeParameter(enable_publish_extrinsic_, "enable_publish_extrinsic", false);
  setAndGetNodeParameter(enable_d2c_viewer_, "enable_d2c_viewer", false);
  setAndGetNodeParameter(enable_hardware_d2d_, "enable_hardware_d2d", true);
  setAndGetNodeParameter(enable_soft_filter_, "enable_soft_filter", true);
  setAndGetNodeParameter(enable_color_auto_exposure_, "enable_color_auto_exposure", true);
  setAndGetNodeParameter(enable_ir_auto_exposure_, "enable_ir_auto_exposure", true);
  setAndGetNodeParameter<std::string>(depth_work_mode_, "depth_work_mode", "");
  setAndGetNodeParameter<std::string>(sync_mode_str_, "sync_mode", "close");
  setAndGetNodeParameter(ir_trigger_signal_in_delay_, "ir_trigger_signal_in_delay", 0);
  setAndGetNodeParameter(rgb_trigger_signal_in_delay_, "rgb_trigger_signal_in_delay", 0);
  setAndGetNodeParameter(device_trigger_signal_out_delay_, "device_trigger_signal_out_delay", 0);
  setAndGetNodeParameter(sync_signal_trigger_out_, "sync_signal_trigger_out", false);
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
  setAndGetNodeParameter<std::string>(jpeg_decoder_, "jpeg_decoder", "avdec_mjpeg");
  setAndGetNodeParameter<std::string>(video_convert_, "video_convert", "videoconvert");
  setAndGetNodeParameter<std::string>(jpeg_parse_, "jpeg_parse", "jpegparse");
}

void OBCameraNode::setupTopics() {
  getParameters();
  setupDevices();
  setupProfiles();
  setupCameraCtrlServices();
  setupPublishers();
}

void OBCameraNode::setupPipelineConfig() {
  if (pipeline_config_) {
    pipeline_config_.reset();
  }
  pipeline_config_ = std::make_shared<ob::Config>();
  if (depth_registration_ && enable_stream_[COLOR] && enable_stream_[DEPTH]) {
    pipeline_config_->setAlignMode(ALIGN_D2C_HW_MODE);
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
        "depth/color/points",
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
  }
  for (const auto &stream_index : HID_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    std::string data_topic_name = stream_name_[stream_index] + "/sample";
    auto data_qos = getRMWQosProfileFromString(imu_qos_[stream_index]);
    imu_publishers_[stream_index] = node_->create_publisher<sensor_msgs::msg::Imu>(
        data_topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos));
  }
  if (enable_publish_extrinsic_) {
    extrinsics_publisher_ = node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
        "extrinsic/depth_to_color", rclcpp::QoS{1}.transient_local());
  }
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
  if (!enable_point_cloud_ || !depth_cloud_pub_ ||
      depth_cloud_pub_->get_subscription_count() == 0) {
    return;
  }
  if (!camera_param_ && depth_registration_) {
    camera_param_ = pipeline_->getCameraParam();
  } else if (!camera_param_) {
    camera_param_ = getDepthCameraParam();
  }
  point_cloud_filter_.setCameraParam(*camera_param_);
  point_cloud_filter_.setCreatePointFormat(OB_FORMAT_POINT);
  auto depth_frame = frame_set->depthFrame();
  if (!depth_frame) {
    return;
  }
  auto frame = point_cloud_filter_.process(frame_set);
  if (!frame) {
    RCLCPP_ERROR_STREAM(logger_, "point cloud filter process failed");
    return;
  }
  size_t point_size = frame->dataSize() / sizeof(OBPoint);
  auto *points = (OBPoint *)frame->data();
  if (!points) {
    RCLCPP_ERROR_STREAM(logger_, "point cloud data is null");
    return;
  }
  CHECK_NOTNULL(points);
  sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg_);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(point_size);
  point_cloud_msg_.width = depth_frame->width();
  point_cloud_msg_.height = depth_frame->height();
  point_cloud_msg_.row_step = point_cloud_msg_.width * point_cloud_msg_.point_step;
  point_cloud_msg_.data.resize(point_cloud_msg_.height * point_cloud_msg_.row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  size_t valid_count = 0;
  auto scale = depth_frame->getValueScale();
  for (size_t point_idx = 0; point_idx < point_size; point_idx++, points++) {
    bool valid_pixel(points->z > 0);
    if (valid_pixel) {
      *iter_x = static_cast<float>((points->x * scale) / 1000.0);
      *iter_y = -static_cast<float>((points->y * scale) / 1000.0);
      *iter_z = static_cast<float>((points->z * scale) / 1000.0);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++valid_count;
    }
  }
  auto timestamp = frameTimeStampToROSTime(depth_frame->systemTimeStamp());
  point_cloud_msg_.header.stamp = timestamp;
  point_cloud_msg_.header.frame_id = optical_frame_id_[DEPTH];
  point_cloud_msg_.is_dense = true;
  point_cloud_msg_.width = valid_count;
  point_cloud_msg_.height = 1;
  modifier.resize(valid_count);
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
    savePointsToPly(frame, filename);
  }
}

void OBCameraNode::publishColoredPointCloud(const std::shared_ptr<ob::FrameSet> &frame_set) {
  if (!enable_colored_point_cloud_ || !depth_registration_cloud_pub_ ||
      depth_registration_cloud_pub_->get_subscription_count() == 0) {
    return;
  }
  auto depth_frame = frame_set->depthFrame();
  auto color_frame = frame_set->colorFrame();
  if (!depth_frame || !color_frame) {
    return;
  }
  if (!camera_param_) {
    camera_param_ = pipeline_->getCameraParam();
  }
  point_cloud_filter_.setCameraParam(*camera_param_);
  point_cloud_filter_.setCreatePointFormat(OB_FORMAT_POINT);
  point_cloud_filter_.setCreatePointFormat(OB_FORMAT_RGB_POINT);
  auto frame = point_cloud_filter_.process(frame_set);
  if (!frame) {
    RCLCPP_ERROR_STREAM(logger_, "point cloud filter process failed");
    return;
  }
  size_t point_size = frame->dataSize() / sizeof(OBColorPoint);
  auto *points = (OBColorPoint *)frame->data();
  if (!points) {
    RCLCPP_ERROR_STREAM(logger_, "point cloud data is null");
    return;
  }
  CHECK_NOTNULL(points);
  sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg_);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(point_size);
  point_cloud_msg_.width = color_frame->width();
  point_cloud_msg_.height = color_frame->height();
  std::string format_str = "rgb";
  point_cloud_msg_.point_step =
      addPointField(point_cloud_msg_, format_str, 1, sensor_msgs::msg::PointField::FLOAT32,
                    point_cloud_msg_.point_step);
  point_cloud_msg_.row_step = point_cloud_msg_.width * point_cloud_msg_.point_step;
  point_cloud_msg_.data.resize(point_cloud_msg_.height * point_cloud_msg_.row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud_msg_, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud_msg_, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud_msg_, "b");
  size_t valid_count = 0;
  auto scale = depth_frame->getValueScale();
  for (size_t point_idx = 0; point_idx < point_size; point_idx += 1) {
    bool valid_pixel((points + point_idx)->z > 0);
    if (valid_pixel) {
      *iter_x = static_cast<float>(((points + point_idx)->x * scale) / 1000.0);
      *iter_y = -static_cast<float>(((points + point_idx)->y * scale) / 1000.0);
      *iter_z = static_cast<float>(((points + point_idx)->z * scale) / 1000.0);
      *iter_r = static_cast<uint8_t>((points + point_idx)->r);
      *iter_g = static_cast<uint8_t>((points + point_idx)->g);
      *iter_b = static_cast<uint8_t>((points + point_idx)->b);

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
      ++valid_count;
    }
  }
  auto timestamp = frameTimeStampToROSTime(depth_frame->systemTimeStamp());
  point_cloud_msg_.header.stamp = timestamp;
  point_cloud_msg_.header.frame_id = optical_frame_id_[COLOR];
  point_cloud_msg_.is_dense = true;
  point_cloud_msg_.width = valid_count;
  point_cloud_msg_.height = 1;
  modifier.resize(valid_count);
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
    saveRGBPointsToPly(frame, filename);
  }
}

void OBCameraNode::onNewFrameSetCallback(const std::shared_ptr<ob::FrameSet> &frame_set) {
  if (frame_set == nullptr) {
    return;
  }
  try {
    if (!tf_published_) {
      publishStaticTransforms();
      tf_published_ = true;
    }
    publishPointCloud(frame_set);
    auto color_frame = std::dynamic_pointer_cast<ob::Frame>(frame_set->colorFrame());
    auto depth_frame = std::dynamic_pointer_cast<ob::Frame>(frame_set->depthFrame());
    auto ir_frame = std::dynamic_pointer_cast<ob::Frame>(frame_set->irFrame());
    onNewFrameCallback(color_frame, COLOR);
    onNewFrameCallback(depth_frame, DEPTH);
    onNewFrameCallback(ir_frame, INFRA0);
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: unknown error");
  }
}

std::shared_ptr<ob::Frame> OBCameraNode::softwareDecodeColorFrame(
    const std::shared_ptr<ob::Frame> &frame) {
  if (frame == nullptr) {
    return nullptr;
  }
  if (frame->format() == OB_FORMAT_RGB888) {
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

void OBCameraNode::onNewFrameCallback(const std::shared_ptr<ob::Frame> &frame,
                                      const stream_index_pair &stream_index) {
  if (frame == nullptr) {
    return;
  }
  std::shared_ptr<ob::VideoFrame> video_frame;
  bool hw_decode = false;
  auto frame_format = frame->format();
  if (frame->type() == OB_FRAME_COLOR && frame_format != OB_FORMAT_RGB888) {
    if (frame_format == OB_FORMAT_MJPG || frame_format == OB_FORMAT_MJPEG) {
#if defined(USE_RK_HW_DECODER) || defined(USE_GST_HW_DECODER)
      CHECK_NOTNULL(mjpeg_decoder_.get());
      video_frame = frame->as<ob::ColorFrame>();
      const auto &color_frame = frame->as<ob::ColorFrame>();
      if (rgb_buffer_ == nullptr) {
        rgb_buffer_ = new uint8_t[video_frame->width() * video_frame->height() * 3];
      }
      bool ret = mjpeg_decoder_->decode(color_frame, rgb_buffer_);
      if (!ret) {
        RCLCPP_ERROR(logger_, "Decode frame failed");
        return;
      }
      hw_decode = true;
#else
      auto covert_frame = softwareDecodeColorFrame(frame);
      video_frame = covert_frame->as<ob::ColorFrame>();
#endif
    } else {
      auto covert_frame = softwareDecodeColorFrame(frame);
      video_frame = covert_frame->as<ob::ColorFrame>();
    }
  } else if (frame->type() == OB_FRAME_COLOR) {
    video_frame = frame->as<ob::ColorFrame>();
  } else if (frame->type() == OB_FRAME_DEPTH) {
    video_frame = frame->as<ob::DepthFrame>();
  } else if (frame->type() == OB_FRAME_IR) {
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
  auto &image = images_[stream_index];
  if (image.empty() || image.cols != width || image.rows != height) {
    image.create(height, width, image_format_[stream_index]);
  }
  if (hw_decode) {
    memcpy(image.data, rgb_buffer_, video_frame->width() * video_frame->height() * 3);
  } else {
    memcpy(image.data, video_frame->data(), video_frame->dataSize());
  }
  if (stream_index == DEPTH) {
    auto depth_scale = video_frame->as<ob::DepthFrame>()->getValueScale();
    image = image * depth_scale;
  }

  auto timestamp = frameTimeStampToROSTime(video_frame->systemTimeStamp());
  if (!camera_param_ && depth_registration_) {
    camera_param_ = pipeline_->getCameraParam();
  } else if (!camera_param_ && stream_index == COLOR) {
    camera_param_ = getColorCameraParam();
  } else if (!camera_param_ && (stream_index == DEPTH || stream_index == INFRA0)) {
    camera_param_ = getDepthCameraParam();
  }
  auto &intrinsic =
      stream_index == COLOR ? camera_param_->rgbIntrinsic : camera_param_->depthIntrinsic;
  auto &distortion =
      stream_index == COLOR ? camera_param_->rgbDistortion : camera_param_->depthDistortion;
  auto camera_info = convertToCameraInfo(intrinsic, distortion, width);
  CHECK(camera_info_publishers_.count(stream_index) > 0);
  camera_info_publishers_[stream_index]->publish(camera_info);
  auto image_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), encoding_[stream_index], image).toImageMsg();
  image_msg->header.stamp = timestamp;
  image_msg->is_bigendian = false;
  image_msg->step = width * unit_step_size_[stream_index];
  image_msg->header.frame_id =
      depth_registration_ ? depth_aligned_frame_id_[stream_index] : optical_frame_id_[stream_index];
  CHECK(image_publishers_.count(stream_index) > 0);
  image_publishers_[stream_index].publish(image_msg);
  saveImageToFile(stream_index, image, image_msg);
}

void OBCameraNode::saveImageToFile(const stream_index_pair &stream_index, const cv::Mat &image,
                                   const sensor_msgs::msg::Image::SharedPtr &image_msg) {
  if (save_images_[stream_index]) {
    auto now = time(nullptr);
    std::stringstream ss;
    ss << std::put_time(localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = std::filesystem::current_path().string();
    auto fps = fps_[stream_index];
    std::string filename = current_path + "/image/" + stream_name_[stream_index] + "_" +
                           std::to_string(image_msg->width) + "x" +
                           std::to_string(image_msg->height) + "_" + std::to_string(fps) + "hz_" +
                           ss.str() + ".png";
    if (!std::filesystem::exists(current_path + "/image")) {
      std::filesystem::create_directory(current_path + "/image");
    }
    RCLCPP_INFO_STREAM(logger_, "Saving image to " << filename);
    if (stream_index.first == OB_STREAM_DEPTH) {
      auto image_to_save = cv_bridge::toCvCopy(image_msg, encoding_[stream_index])->image;
      cv::imwrite(filename, image_to_save);
    } else if (stream_index.first == OB_STREAM_COLOR) {
      auto image_to_save =
          cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::imwrite(filename, image_to_save);
    } else if (stream_index.first == OB_STREAM_IR) {
      cv::imwrite(filename, image);
    } else {
      RCLCPP_ERROR_STREAM(logger_, "Unsupported stream type: " << stream_index.first);
    }
    save_images_[stream_index] = false;
  }
}

void OBCameraNode::onNewIMUFrameCallback(const std::shared_ptr<ob::Frame> &frame,
                                         const stream_index_pair &stream_index) {
  if (!imu_publishers_.count(stream_index)) {
    RCLCPP_ERROR_STREAM(logger_,
                        "stream " << stream_name_[stream_index] << " publisher not initialized");
    return;
  }
  auto subscriber_count = imu_publishers_[stream_index]->get_subscription_count();
  if (subscriber_count == 0) {
    return;
  }
  auto imu_msg = sensor_msgs::msg::Imu();
  setDefaultIMUMessage(imu_msg);
  imu_msg.header.frame_id = optical_frame_id_[stream_index];
  auto timestamp = frameTimeStampToROSTime(frame->systemTimeStamp());
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
    if (depth_w * height_[DEPTH] == depth_h * width_[DEPTH]) {
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
    if (color_w * height_[COLOR] == color_h * width_[COLOR]) {
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
  tf2::Quaternion quaternion_optical, zero_rot, Q;
  zero_rot.setRPY(0.0, 0.0, 0.0);
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  tf2::Vector3 zero_trans(0, 0, 0);
  tf2::Vector3 trans(0, 0, 0);
  auto camera_param = pipeline_->getCameraParam();
  auto ex = camera_param.transform;
  RCLCPP_INFO_STREAM(logger_,
                     "transform x " << ex.trans[0] << " y " << ex.trans[1] << " z " << trans[2]);
  Q = rotationMatrixToQuaternion(ex.rot);
  Q = quaternion_optical * Q * quaternion_optical.inverse();
  trans[0] = ex.trans[0];
  trans[1] = ex.trans[1];
  trans[2] = ex.trans[2];
  tf2::Transform transform(Q, trans);
  transform = transform.inverse();
  Q = transform.getRotation();
  trans = transform.getOrigin();
  rclcpp::Time tf_timestamp = node_->now();

  publishStaticTF(tf_timestamp, trans, Q, frame_id_[DEPTH], frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[COLOR],
                  optical_frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[DEPTH],
                  optical_frame_id_[DEPTH]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[ACCEL],
                  optical_frame_id_[ACCEL]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[GYRO],
                  optical_frame_id_[GYRO]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[INFRA0],
                  optical_frame_id_[INFRA0]);
  publishStaticTF(tf_timestamp, zero_trans, zero_rot, camera_link_frame_id_, frame_id_[DEPTH]);
  publishStaticTF(tf_timestamp, zero_trans, zero_rot, camera_link_frame_id_, frame_id_[INFRA0]);
  publishStaticTF(tf_timestamp, trans, Q, camera_link_frame_id_, frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, zero_rot, camera_link_frame_id_, frame_id_[ACCEL]);
  publishStaticTF(tf_timestamp, zero_trans, zero_rot, camera_link_frame_id_, frame_id_[GYRO]);
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

}  // namespace orbbec_camera
