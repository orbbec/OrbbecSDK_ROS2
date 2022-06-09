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
    : node_(node), device_(device), parameters_(parameters), logger_(node->get_logger()) {
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // FIXME:
  is_running_.store(true);
  format_[DEPTH] = OB_FORMAT_Y16;
  image_format_[OB_STREAM_DEPTH] = CV_16UC1;
  encoding_[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
  stream_name_[OB_STREAM_DEPTH] = "depth";
  unit_step_size_[DEPTH] = sizeof(uint16_t);
  format_[IR0] = OB_FORMAT_Y16;
  image_format_[OB_STREAM_IR] = CV_16UC1;
  encoding_[IR0] = sensor_msgs::image_encodings::MONO16;
  stream_name_[OB_STREAM_IR] = "ir";
  unit_step_size_[IR0] = sizeof(uint8_t);

  format_[COLOR] = OB_FORMAT_I420;
  image_format_[OB_STREAM_COLOR] = CV_8UC3;
  encoding_[COLOR] = sensor_msgs::image_encodings::BGR8;
  stream_name_[OB_STREAM_COLOR] = "color";
  unit_step_size_[COLOR] = 3;

  compression_params_.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params_.push_back(0);
  compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY);
  compression_params_.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
  setupTopics();
  startPipeline();
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
  RCLCPP_WARN_STREAM(logger_, "stop pipeline");
  pipeline_->stop();
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

  for (const auto& [stream_index, enable] : enable_) {
    if (enable && sensors_.find(stream_index) == sensors_.end()) {
      RCLCPP_INFO_STREAM(logger_,
                         magic_enum::enum_name(stream_index.first)
                             << "sensor isn't supported by current device! -- Skipping...");
      enable_[stream_index] = false;
    }
  }
}

void OBCameraNode::setupProfiles() {
  config_ = std::make_shared<ob::Config>();
  for (const auto& elem : IMAGE_STREAMS) {
    if (enable_[elem]) {
      const auto& sensor = sensors_[elem];
      auto profiles = sensor->getStreamProfileList();
      for (size_t i = 0; i < profiles->count(); i++) {
        auto profile = profiles->getProfile(i)->as<ob::VideoStreamProfile>();
        RCLCPP_DEBUG_STREAM(
            logger_, "Sensor profile: "
                         << "stream_type: " << magic_enum::enum_name(profile->type())
                         << "Format: " << profile->format() << ", Width: " << profile->width()
                         << ", Height: " << profile->height() << ", FPS: " << profile->fps());
        enabled_profiles_[elem].emplace_back(profile);
      }

      auto selected_profile =
          profiles->getVideoStreamProfile(width_[elem], height_[elem], format_[elem], fps_[elem]);
      auto default_profile = profiles->getVideoStreamProfile();
      if (!selected_profile) {
        RCLCPP_WARN_STREAM(logger_, "Given stream configuration is not supported by the device! "
                                        << " Stream: " << magic_enum::enum_name(elem.first)
                                        << ", Stream Index: " << elem.second
                                        << ", Width: " << width_[elem]
                                        << ", Height: " << height_[elem] << ", FPS: " << fps_[elem]
                                        << ", Format: " << format_[elem]);
        if (default_profile) {
          RCLCPP_WARN_STREAM(logger_, "Using default profile instead.");
          selected_profile = default_profile;
        } else {
          RCLCPP_ERROR_STREAM(
              logger_, " NO default_profile found , Stream: " << magic_enum::enum_name(elem.first)
                                                              << " will be disable");
          enable_[elem] = false;
          continue;
        }
      }
      CHECK_NOTNULL(selected_profile);
      config_->enableStream(selected_profile);
      images_[elem] =
          cv::Mat(height_[elem], width_[elem], image_format_[elem.first], cv::Scalar(0, 0, 0));
      RCLCPP_INFO_STREAM(
          logger_, " stream is enabled - width: " << width_[elem] << ", height: " << height_[elem]
                                                  << ", fps: " << fps_[elem] << ", "
                                                  << "Format: " << selected_profile->format());
    }
  }
}

void OBCameraNode::startPipeline() {
  if (d2c_mode_ == "sw") {
    config_->setAlignMode(ALIGN_D2C_SW_MODE);
  } else if (d2c_mode_ == "hw") {
    config_->setAlignMode(ALIGN_D2C_HW_MODE);
  } else {
    config_->setAlignMode(ALIGN_DISABLE);
  }
  pipeline_ = std::make_unique<ob::Pipeline>(device_);
  pipeline_->start(config_, [this](std::shared_ptr<ob::FrameSet> frame_set) {
    frameSetCallback(std::move(frame_set));
  });
}

void OBCameraNode::getParameters() {
  for (auto stream_index : IMAGE_STREAMS) {
    std::string param_name = stream_name_[stream_index.first] + "_width";
    setAndGetNodeParameter(width_[stream_index], param_name, IMAGE_WIDTH);
    param_name = stream_name_[stream_index.first] + "_height";
    setAndGetNodeParameter(height_[stream_index], param_name, IMAGE_HEIGHT);
    param_name = stream_name_[stream_index.first] + "_fps";
    setAndGetNodeParameter(fps_[stream_index], param_name, IMAGE_FPS);
    param_name = "enable_" + stream_name_[stream_index.first];
    setAndGetNodeParameter(enable_[stream_index], param_name, true);
    param_name = stream_name_[stream_index.first] + "_frame_id";
    std::string default_frame_id = "camera_" + stream_name_[stream_index.first] + "_frame";
    setAndGetNodeParameter(frame_id_[stream_index], param_name, default_frame_id);
    std::string default_optical_frame_id =
        "camera_" + stream_name_[stream_index.first] + "_optical_frame";
    param_name = stream_name_[stream_index.first] + "_optical_frame_id";
    setAndGetNodeParameter(optical_frame_id_[stream_index], param_name, default_optical_frame_id);
    depth_aligned_frame_id_[stream_index] = stream_name_[OB_STREAM_COLOR] + "_optical_frame";
  }
  setAndGetNodeParameter(publish_tf_, "publish_tf", true);
  setAndGetNodeParameter(align_depth_, "align_depth", true);
  setAndGetNodeParameter(tf_publish_rate_, "tf_publish_rate", 10.0);
  setAndGetNodeParameter(publish_rgb_point_cloud_, "publish_rgb_point_cloud", false);
  setAndGetNodeParameter(d2c_mode_, "d2c_mode_", DEFAULT_D2C_MODE);
}

void OBCameraNode::setupTopics() {
  getParameters();
  setupDevices();
  updateStreamCalibData();
  setupProfiles();
  setupCameraCtrlServices();
  setupPublishers();
  publishStaticTransforms();
}

void OBCameraNode::setupPublishers() {
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  point_cloud_publisher_ =
      node_->create_publisher<PointCloud2>("depth/points", rclcpp::QoS{1}.best_effort());
  for (const auto& stream_index : IMAGE_STREAMS) {
    std::string name = stream_name_[stream_index.first];
    std::string topic = name + "/image_raw";
    image_publishers_[stream_index] = image_transport::create_publisher(node_, topic);
    topic = name + "/camera_info";
    camera_info_publishers_[stream_index] =
        node_->create_publisher<CameraInfo>(topic, rclcpp::QoS{1}.best_effort());
  }
  extrinsics_publisher_ = node_->create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
      "extrinsic/depth_to_color", rclcpp::QoS{1}.transient_local());
}

void OBCameraNode::publishPointCloud(std::shared_ptr<ob::FrameSet> frame_set) {
  if (publish_rgb_point_cloud_ && frame_set->depthFrame() != nullptr &&
      frame_set->colorFrame() != nullptr) {
    publishColorPointCloud(frame_set);
  } else if (frame_set->depthFrame() != nullptr) {
    publishDepthPointCloud(frame_set);
  }
}

void OBCameraNode::publishDepthPointCloud(std::shared_ptr<ob::FrameSet> frame_set) {
  auto camera_param = pipeline_->getCameraParam();
  point_cloud_filter_.setCameraParam(camera_param);
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
  for (size_t point_idx = 0; point_idx < point_size; point_idx++, points++) {
    bool valid_pixel(points->z > 0);
    if (valid_pixel) {
      *iter_x = static_cast<float>(points->x / 1000.0);
      *iter_y = static_cast<float>(points->y / 1000.0);
      *iter_z = static_cast<float>(points->z / 1000.0);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++valid_count;
    }
  }
  auto timestamp = frameTimeStampToROSTime(depth_frame->systemTimeStamp());
  point_cloud_msg_.header.stamp = timestamp;
  point_cloud_msg_.header.frame_id = optical_frame_id_[DEPTH];
  point_cloud_publisher_->publish(point_cloud_msg_);
}

void OBCameraNode::publishColorPointCloud(std::shared_ptr<ob::FrameSet> frame_set) {
  auto depth_frame = frame_set->depthFrame();
  auto color_frame = frame_set->colorFrame();
  auto camera_param = findCameraParam(color_frame->width(), color_frame->height(),
                                      depth_frame->width(), depth_frame->height());
  point_cloud_filter_.setCameraParam(*camera_param);
  point_cloud_filter_.setCreatePointFormat(OB_FORMAT_RGB_POINT);
  auto frame = point_cloud_filter_.process(frame_set);
  size_t point_size = frame->dataSize() / sizeof(OBColorPoint);
  auto* points = (OBColorPoint*)frame->data();
  CHECK_NOTNULL(points);
  sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg_);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(point_size);
  point_cloud_msg_.width = color_frame->width();
  point_cloud_msg_.height = color_frame->height();
  std::string format_str = "rgb";

  point_cloud_msg_.point_step =
      addPointField(point_cloud_msg_, format_str.c_str(), 1, sensor_msgs::msg::PointField::FLOAT32,
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
  for (size_t point_idx = 0; point_idx < point_size; point_idx++, points++) {
    bool valid_pixel(points->z > 0);
    if (valid_pixel) {
      *iter_x = static_cast<float>(points->x / 1000.0);
      *iter_y = static_cast<float>(points->y / 1000.0);
      *iter_z = static_cast<float>(points->z / 1000.0);
      *iter_r = static_cast<uint8_t>(points->r);
      *iter_g = static_cast<uint8_t>(points->g);
      *iter_b = static_cast<uint8_t>(points->b);

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
  point_cloud_publisher_->publish(point_cloud_msg_);
}
void OBCameraNode::frameSetCallback(std::shared_ptr<ob::FrameSet> frame_set) {
  auto color_frame = frame_set->colorFrame();
  auto depth_frame = frame_set->depthFrame();
  auto ir_frame = frame_set->irFrame();
  if (color_frame && enable_[COLOR]) {
    publishColorFrame(color_frame);
  }
  if (depth_frame && enable_[DEPTH]) {
    publishDepthFrame(depth_frame);
  }
  if (ir_frame && enable_[IR0]) {
    publishIRFrame(ir_frame);
  }
  publishPointCloud(frame_set);
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

std::optional<OBCameraParam> OBCameraNode::findStreamDefaultCameraParam(
    const stream_index_pair& stream) {
  uint32_t width = width_[stream];
  uint32_t height = height_[stream];
  auto camera_params = device_->getCalibrationCameraParamList();
  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    OBCameraIntrinsic intrinsic;
    if (stream.first == OB_STREAM_COLOR) {
      intrinsic = param.rgbIntrinsic;
    } else if (stream.first == OB_STREAM_DEPTH) {
      intrinsic = param.depthIntrinsic;
    } else {
      return {};
    }
    if (width * intrinsic.height == height * intrinsic.width) {
      return param;
    }
  }
  return {};
}

std::optional<OBCameraParam> OBCameraNode::findStreamCameraParam(const stream_index_pair& stream,
                                                                 uint32_t width, uint32_t height) {
  auto camera_params = device_->getCalibrationCameraParamList();
  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    OBCameraIntrinsic intrinsic;
    if (stream.first == OB_STREAM_COLOR) {
      intrinsic = param.rgbIntrinsic;
    } else if (stream.first == OB_STREAM_DEPTH) {
      intrinsic = param.depthIntrinsic;
    } else {
      return {};
    }
    if (width * intrinsic.height == height * intrinsic.width) {
      return param;
    }
  }
  return {};
}

std::optional<OBCameraParam> OBCameraNode::findCameraParam(uint32_t color_width,
                                                           uint32_t color_height,
                                                           uint32_t depth_width,
                                                           uint32_t depth_height) {
  auto camera_params = device_->getCalibrationCameraParamList();
  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int depth_w = param.depthIntrinsic.width;
    int depth_h = param.depthIntrinsic.height;
    int color_w = param.rgbIntrinsic.width;
    int color_h = param.rgbIntrinsic.height;
    if ((depth_w * depth_height == depth_h * depth_width) &&
        (color_w * color_height == color_h * color_width)) {
      return param;
    }
  }
  return {};
}

void OBCameraNode::updateStreamCalibData() {
  auto param = findDefaultCameraParam();
  CHECK(param.has_value());
  camera_infos_[DEPTH] = convertToCameraInfo(param->depthIntrinsic, param->depthDistortion);
  camera_infos_[COLOR] = convertToCameraInfo(param->rgbIntrinsic, param->rgbDistortion);
  camera_infos_[IR0] = camera_infos_[DEPTH];
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
  tf2::Quaternion quaternion_optical, zero_rot;
  zero_rot.setRPY(0.0, 0.0, 0.0);
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  std::vector<float> zero_trans = {0, 0, 0};
  auto camera_param = findDefaultCameraParam();
  CHECK(camera_param.has_value());
  auto ex = camera_param->transform;
  auto Q = rotationMatrixToQuaternion(ex.rot);
  Q = quaternion_optical * Q * quaternion_optical.inverse();
  std::vector<float> trans = {ex.trans[0], ex.trans[1], ex.trans[2]};
  rclcpp::Time tf_timestamp = node_->now();

  publishStaticTF(tf_timestamp, trans, Q, frame_id_[DEPTH], frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, trans, Q, "camera_link", frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[COLOR],
                  optical_frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[DEPTH],
                  optical_frame_id_[DEPTH]);
  publishStaticTF(tf_timestamp, zero_trans, zero_rot, "camera_link", frame_id_[DEPTH]);
  extrinsics_publisher_->publish(obExtrinsicsToMsg(ex, "depth_to_color_extrinsics"));
}

void OBCameraNode::publishStaticTransforms() {
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

void OBCameraNode::publishColorFrame(std::shared_ptr<ob::ColorFrame> frame) {
  format_convert_filter.setFormatConvertType(FORMAT_I420_TO_RGB888);
  frame = format_convert_filter.process(frame)->as<ob::ColorFrame>();
  format_convert_filter.setFormatConvertType(FORMAT_RGB888_TO_BGR);
  frame = format_convert_filter.process(frame)->as<ob::ColorFrame>();
  auto width = frame->width();
  auto height = frame->height();
  auto stream = COLOR;
  auto& image = images_[stream];
  if (image.size() != cv::Size(width, height)) {
    image.create(height, width, image.type());
  }
  image.data = (uint8_t*)frame->data();
  auto& camera_info_publisher = camera_info_publishers_.at(stream);
  auto& image_publisher = image_publishers_.at(stream);
  auto& cam_info = camera_infos_.at(stream);
  if (cam_info.width != width) {
    RCLCPP_ERROR(logger_, "cam info error");
    cam_info.height = height;
    cam_info.width = width;
  }
  auto timestamp = frameTimeStampToROSTime(frame->systemTimeStamp());
  cam_info.header.stamp = timestamp;
  camera_info_publisher->publish(cam_info);
  sensor_msgs::msg::Image::SharedPtr img;
  img = cv_bridge::CvImage(std_msgs::msg::Header(), encoding_.at(stream), image).toImageMsg();

  img->width = width;
  img->height = height;
  img->is_bigendian = false;
  img->step = width * unit_step_size_[stream];
  img->header.frame_id = optical_frame_id_[COLOR];
  img->header.stamp = timestamp;
  image_publisher.publish(img);
}

void OBCameraNode::publishDepthFrame(std::shared_ptr<ob::DepthFrame> frame) {
  auto width = frame->width();
  auto height = frame->height();
  auto stream = DEPTH;
  auto& image = images_[stream];
  if (image.size() != cv::Size(width, height)) {
    image.create(height, width, image.type());
  }
  image.data = (uint8_t*)frame->data();
  auto& camera_info_publisher = camera_info_publishers_.at(stream);
  auto& image_publisher = image_publishers_.at(stream);
  auto& cam_info = camera_infos_.at(stream);
  if (cam_info.width != width) {
    RCLCPP_ERROR(logger_, "cam info error");
    cam_info.height = height;
    cam_info.width = width;
  }
  auto timestamp = frameTimeStampToROSTime(frame->systemTimeStamp());
  cam_info.header.stamp = timestamp;
  camera_info_publisher->publish(cam_info);
  sensor_msgs::msg::Image::SharedPtr img;
  img = cv_bridge::CvImage(std_msgs::msg::Header(), encoding_.at(stream), image).toImageMsg();

  img->width = width;
  img->height = height;
  img->is_bigendian = false;
  img->step = width * unit_step_size_[stream];
  img->header.frame_id = optical_frame_id_[COLOR];
  img->header.stamp = timestamp;
  image_publisher.publish(img);
}

void OBCameraNode::publishIRFrame(std::shared_ptr<ob::IRFrame> frame) {
  auto width = frame->width();
  auto height = frame->height();
  auto stream = IR0;
  auto& image = images_[stream];
  if (image.size() != cv::Size(width, height)) {
    image.create(height, width, image.type());
  }
  image.data = (uint8_t*)frame->data();
  auto& camera_info_publisher = camera_info_publishers_.at(stream);
  auto& image_publisher = image_publishers_.at(stream);
  auto& cam_info = camera_infos_.at(stream);
  if (cam_info.width != width) {
    RCLCPP_ERROR(logger_, "cam info error");
    cam_info.height = height;
    cam_info.width = width;
  }
  auto timestamp = frameTimeStampToROSTime(frame->systemTimeStamp());
  cam_info.header.stamp = timestamp;
  camera_info_publisher->publish(cam_info);
  sensor_msgs::msg::Image::SharedPtr img;
  img = cv_bridge::CvImage(std_msgs::msg::Header(), encoding_.at(stream), image).toImageMsg();

  img->width = width;
  img->height = height;
  img->is_bigendian = false;
  img->step = width * unit_step_size_[stream];
  img->header.frame_id = optical_frame_id_[COLOR];
  img->header.stamp = timestamp;
  image_publisher.publish(img);
}

}  // namespace orbbec_camera
