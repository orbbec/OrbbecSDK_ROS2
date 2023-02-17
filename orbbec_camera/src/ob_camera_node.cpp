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
namespace orbbec_camera {
using namespace std::chrono_literals;

OBCameraNode::OBCameraNode(rclcpp::Node* node, std::shared_ptr<ob::Device> device,
                           std::shared_ptr<Parameters> parameters)
    : node_(node),
      device_(std::move(device)),
      parameters_(std::move(parameters)),
      logger_(node->get_logger()) {
  is_running_.store(true);
  stream_name_[COLOR] = "color";
  stream_name_[DEPTH] = "depth";
  stream_name_[INFRA0] = "ir";

  compression_params_.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params_.push_back(0);
  compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY);
  compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
  setupDefaultImageFormat();
  setupTopics();
  startStreams();
}

template <class T>
void OBCameraNode::setAndGetNodeParameter(
    T& param, const std::string& param_name, const T& default_value,
    const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor) {
  try {
    param = parameters_
                ->setParam(param_name, rclcpp::ParameterValue(default_value),
                           std::function<void(const rclcpp::Parameter&)>(), parameter_descriptor)
                .get<T>();
  } catch (const rclcpp::ParameterTypeException& ex) {
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

  for (const auto& [stream_index, enable] : enable_stream_) {
    if (enable && sensors_.find(stream_index) == sensors_.end()) {
      RCLCPP_INFO_STREAM(logger_,
                         magic_enum::enum_name(stream_index.first)
                             << "sensor isn't supported by current device! -- Skipping...");
      enable_stream_[stream_index] = false;
    }
  }
}

void OBCameraNode::setupProfiles() {
  for (const auto& elem : IMAGE_STREAMS) {
    if (enable_stream_[elem]) {
      const auto& sensor = sensors_[elem];
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
        soupported_profiles_[elem].emplace_back(profile);
      }
      std::shared_ptr<ob::VideoStreamProfile> selected_profile;
      std::shared_ptr<ob::VideoStreamProfile> default_profile;
      try {
        selected_profile =
            profiles->getVideoStreamProfile(width_[elem], height_[elem], format_[elem], fps_[elem]);
        default_profile =
            profiles->getVideoStreamProfile(width_[elem], height_[elem], format_[elem]);
      } catch (const ob::Error& ex) {
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
    pipeline_->start(pipeline_config_, [this](std::shared_ptr<ob::FrameSet> frame_set) {
      onNewFrameSetCallback(frame_set);
    });
  } catch (const ob::Error& e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to start pipeline: " << e.getMessage());
    RCLCPP_INFO_STREAM(logger_, "try to disable ir stream and try again");
    enable_stream_[INFRA0] = false;
    setupPipelineConfig();
    pipeline_->start(pipeline_config_, [this](std::shared_ptr<ob::FrameSet> frame_set) {
      onNewFrameSetCallback(frame_set);
    });
  }
  pipeline_started_.store(true);
}

void OBCameraNode::stopStreams() {
  if (!pipeline_started_ || !pipeline_) {
    return;
  }
  try {
    pipeline_->stop();
  } catch (const ob::Error& e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to stop pipeline: " << e.getMessage());
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
  unit_step_size_[INFRA0] = sizeof(uint8_t);

  image_format_[COLOR] = CV_8UC3;
  encoding_[COLOR] = sensor_msgs::image_encodings::RGB8;
  unit_step_size_[COLOR] = 3 * sizeof(uint8_t);
}

void OBCameraNode::getParameters() {
  for (auto stream_index : IMAGE_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_width";
    setAndGetNodeParameter(width_[stream_index], param_name, IMAGE_WIDTH);
    param_name = stream_name_[stream_index] + "_height";
    setAndGetNodeParameter(height_[stream_index], param_name, IMAGE_HEIGHT);
    param_name = stream_name_[stream_index] + "_fps";
    setAndGetNodeParameter(fps_[stream_index], param_name, IMAGE_FPS);
    param_name = "enable_" + stream_name_[stream_index];
    setAndGetNodeParameter(enable_stream_[stream_index], param_name, false);
    param_name = stream_name_[stream_index] + "_frame_id";
    std::string default_frame_id = "camera_" + stream_name_[stream_index] + "_frame";
    setAndGetNodeParameter(frame_id_[stream_index], param_name, default_frame_id);
    std::string default_optical_frame_id =
        "camera_" + stream_name_[stream_index] + "_optical_frame";
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
  setAndGetNodeParameter(publish_tf_, "publish_tf", true);
  setAndGetNodeParameter(tf_publish_rate_, "tf_publish_rate", 10.0);
  setAndGetNodeParameter(depth_registration_, "depth_registration", false);
  setAndGetNodeParameter(enable_point_cloud_, "enable_point_cloud", true);
  setAndGetNodeParameter<std::string>(ir_info_url_, "ir_info_url", "");
  setAndGetNodeParameter<std::string>(color_info_url_, "color_info_url", "");
  setAndGetNodeParameter(enable_colored_point_cloud_, "enable_colored_point_cloud", false);
  setAndGetNodeParameter(camera_link_frame_id_, "camera_link_frame_id", DEFAULT_BASE_FRAME_ID);
  setAndGetNodeParameter(enable_point_cloud_, "enable_point_cloud", true);
  setAndGetNodeParameter<std::string>(point_cloud_qos_, "point_cloud_qos", "default");
  setAndGetNodeParameter(enable_publish_extrinsic_, "enable_publish_extrinsic", false);
  if (enable_colored_point_cloud_) {
    depth_registration_ = true;
  }
}

void OBCameraNode::setupTopics() {
  getParameters();
  setupDevices();
  setupProfiles();
  setupCameraCtrlServices();
  setupPublishers();
  publishStaticTransforms();
}

void OBCameraNode::setupPipelineConfig() {
  if (pipeline_config_) {
    pipeline_config_.reset();
  }
  pipeline_config_ = std::make_shared<ob::Config>();
  if (depth_registration_ && enable_stream_[COLOR] && enable_stream_[DEPTH]) {
    pipeline_config_->setAlignMode(ALIGN_D2C_HW_MODE);
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
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
    colored_point_cloud_publisher_ = node_->create_publisher<PointCloud2>(
        "depth/color/points",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                    point_cloud_qos_profile));
  }
  if (enable_point_cloud_) {
    point_cloud_publisher_ = node_->create_publisher<PointCloud2>(
        "depth/points", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                                    point_cloud_qos_profile));
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
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
  if (enable_publish_extrinsic_) {
    extrinsics_publisher_ = node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
        "extrinsic/depth_to_color", rclcpp::QoS{1}.transient_local());
  }
}

void OBCameraNode::publishPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set) {
  try {
    if (depth_registration_ || enable_colored_point_cloud_) {
      if (frame_set->depthFrame() != nullptr && frame_set->colorFrame() != nullptr) {
        publishColoredPointCloud(frame_set);
      }
    }
    if (enable_point_cloud_ && frame_set->depthFrame() != nullptr) {
      publishDepthPointCloud(frame_set);
    }
  } catch (const ob::Error& e) {
    RCLCPP_ERROR_STREAM(logger_, e.getMessage());
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(logger_, e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "publishPointCloud with unknown error");
  }
}

void OBCameraNode::publishDepthPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!enable_point_cloud_ || !point_cloud_publisher_ ||
      point_cloud_publisher_->get_subscription_count() == 0) {
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
  auto frame = point_cloud_filter_.process(frame_set);
  size_t point_size = frame->dataSize() / sizeof(OBPoint);
  auto* points = (OBPoint*)frame->data();
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
  point_cloud_publisher_->publish(point_cloud_msg_);
}

void OBCameraNode::publishColoredPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!enable_colored_point_cloud_ || !colored_point_cloud_publisher_ ||
      colored_point_cloud_publisher_->get_subscription_count() == 0) {
    return;
  }
  auto depth_frame = frame_set->depthFrame();
  auto color_frame = frame_set->colorFrame();
  if (!camera_param_) {
    camera_param_ = pipeline_->getCameraParam();
  }
  point_cloud_filter_.setCameraParam(*camera_param_);
  point_cloud_filter_.setCreatePointFormat(OB_FORMAT_POINT);
  point_cloud_filter_.setCreatePointFormat(OB_FORMAT_RGB_POINT);
  auto frame = point_cloud_filter_.process(frame_set);
  size_t point_size = frame->dataSize() / sizeof(OBColorPoint);
  auto* points = (OBColorPoint*)frame->data();
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
  colored_point_cloud_publisher_->publish(point_cloud_msg_);
}

void OBCameraNode::onNewFrameSetCallback(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (frame_set == nullptr) {
    return;
  }
  try {
    auto color_frame = std::dynamic_pointer_cast<ob::Frame>(frame_set->colorFrame());
    auto depth_frame = std::dynamic_pointer_cast<ob::Frame>(frame_set->depthFrame());
    auto ir_frame = std::dynamic_pointer_cast<ob::Frame>(frame_set->irFrame());
    onNewFrameCallback(color_frame, COLOR);
    onNewFrameCallback(depth_frame, DEPTH);
    onNewFrameCallback(ir_frame, INFRA0);
    publishPointCloud(frame_set);
  } catch (const ob::Error& e) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.getMessage());
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: unknown error");
  }
}

void OBCameraNode::onNewFrameCallback(const std::shared_ptr<ob::Frame>& frame,
                                      const stream_index_pair& stream_index) {
  if (frame == nullptr) {
    return;
  }
  std::shared_ptr<ob::VideoFrame> video_frame;
  if (frame->type() == OB_FRAME_COLOR && frame->format() != OB_FORMAT_RGB888) {
    if (!setupFormatConvertType(frame->format())) {
      RCLCPP_ERROR(logger_, "Unsupported color format: %d", frame->format());
      return;
    }
    auto color_frame = format_convert_filter_.process(frame);
    if (color_frame == nullptr) {
      RCLCPP_ERROR(logger_, "Failed to convert color frame");
      return;
    }
    video_frame = color_frame->as<ob::ColorFrame>();
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
  auto& image = images_[stream_index];
  if (image.empty() || image.cols != width || image.rows != height) {
    image.create(height, width, image_format_[stream_index]);
  }
  image.data = (uchar*)video_frame->data();
  auto timestamp = frameTimeStampToROSTime(video_frame->systemTimeStamp());
  if (!camera_param_ && depth_registration_) {
    camera_param_ = pipeline_->getCameraParam();
  } else if (!camera_param_ && stream_index == COLOR) {
    camera_param_ = getColorCameraParam();
  } else if (!camera_param_) {
    camera_param_ = getDepthCameraParam();
  }
  auto& intrinsic =
      stream_index == COLOR ? camera_param_->rgbIntrinsic : camera_param_->depthIntrinsic;
  auto& distortion =
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

void OBCameraNode::publishStaticTF(const rclcpp::Time& t, const std::vector<float>& trans,
                                   const tf2::Quaternion& q, const std::string& from,
                                   const std::string& to) {
  CHECK_EQ(trans.size(), 3u);
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = t;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
  msg.transform.translation.x = trans.at(2) / 1000.0;
  msg.transform.translation.y = -trans.at(0) / 1000.0;
  msg.transform.translation.z = -trans.at(1) / 1000.0;
  msg.transform.rotation.x = q.getX();
  msg.transform.rotation.y = q.getY();
  msg.transform.rotation.z = q.getZ();
  msg.transform.rotation.w = q.getW();
  static_tf_msgs_.push_back(msg);
}

void OBCameraNode::calcAndPublishStaticTransform() {
  tf2::Quaternion quaternion_optical, zero_rot, Q;
  std::vector<float> trans(3, 0);
  zero_rot.setRPY(0.0, 0.0, 0.0);
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  std::vector<float> zero_trans = {0, 0, 0};
  auto camera_param = findDefaultCameraParam();
  if (enable_publish_extrinsic_ && extrinsics_publisher_ && camera_param.has_value()) {
    auto ex = camera_param->transform;
    Q = rotationMatrixToQuaternion(ex.rot);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    extrinsics_publisher_->publish(obExtrinsicsToMsg(ex, "depth_to_color_extrinsics"));
  } else {
    Q.setRPY(0, 0, 0);
  }
  rclcpp::Time tf_timestamp = node_->now();

  publishStaticTF(tf_timestamp, trans, Q, frame_id_[DEPTH], frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, trans, Q, camera_link_frame_id_, frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[COLOR],
                  optical_frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[DEPTH],
                  optical_frame_id_[DEPTH]);
  publishStaticTF(tf_timestamp, zero_trans, zero_rot, camera_link_frame_id_, frame_id_[DEPTH]);
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
      for (auto& msg : static_tf_msgs_) {
        msg.header.stamp = t;
      }
      dynamic_tf_broadcaster_->sendTransform(static_tf_msgs_);
    }
  }
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
    default:
      return false;
  }
  return true;
}

}  // namespace orbbec_camera
