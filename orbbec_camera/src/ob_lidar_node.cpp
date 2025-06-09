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

#include "orbbec_camera/ob_lidar_node.h"
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
namespace orbbec_lidar {
using namespace std::chrono_literals;

OBLidarNode::OBLidarNode(rclcpp::Node *node, std::shared_ptr<ob::Device> device,
                         std::shared_ptr<Parameters> parameters, bool use_intra_process)
    : node_(node),
      device_(std::move(device)),
      parameters_(std::move(parameters)),
      logger_(node->get_logger()),
      use_intra_process_(use_intra_process) {
  RCLCPP_INFO_STREAM(logger_,
                     "OBLidarNode: use_intra_process: " << (use_intra_process ? "ON" : "OFF"));
  is_running_.store(true);
  stream_name_[LIDAR] = "lidar";
  setupTopics();
#if defined(USE_RK_HW_DECODER)
  jpeg_decoder_ = std::make_unique<RKJPEGDecoder>(width_[COLOR], height_[COLOR]);
#elif defined(USE_NV_HW_DECODER)
  jpeg_decoder_ = std::make_unique<JetsonNvJPEGDecoder>(width_[COLOR], height_[COLOR]);
#endif
  is_camera_node_initialized_ = true;
}

template <class T>
void OBLidarNode::setAndGetNodeParameter(
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

OBLidarNode::~OBLidarNode() noexcept { clean(); }

void OBLidarNode::rebootDevice() {
  RCLCPP_WARN_STREAM(logger_, "Reboot device");
  clean();
  if (device_) {
    device_->reboot();
    RCLCPP_WARN_STREAM(logger_, "Reboot device DONE");
  }
}

void OBLidarNode::clean() noexcept {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  RCLCPP_WARN_STREAM(logger_, "Do destroy ~OBLidarNode");
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
  //   stopIMU();
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

  RCLCPP_WARN_STREAM(logger_, "Destroy ~OBLidarNode DONE");
}

void OBLidarNode::setupTopics() {
  try {
    getParameters();
    setupDevices();
    selectBaseStream();
    setupProfiles();
    setupPublishers();
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

void OBLidarNode::getParameters() {
  setAndGetNodeParameter<std::string>(camera_name_, "camera_name", "lidar");
  for (auto stream_index : IMAGE_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_format";
    setAndGetNodeParameter(format_str_[stream_index], param_name, format_str_[stream_index]);
    format_[stream_index] = OBFormatFromString(format_str_[stream_index]);
    param_name = stream_name_[stream_index] + "_rate";
    setAndGetNodeParameter(rate_int_[stream_index], param_name, 0);
    rate_[stream_index] = OBScanRateFromInt(rate_int_[stream_index]);
    RCLCPP_INFO_STREAM(logger_, "rate_ " << magic_enum::enum_name(rate_[LIDAR]) << "format_"
                                         << magic_enum::enum_name(format_[LIDAR]));
  }

  setAndGetNodeParameter<bool>(publish_tf_, "publish_tf", true);
  setAndGetNodeParameter<double>(tf_publish_rate_, "tf_publish_rate", 0.0);
  setAndGetNodeParameter<std::string>(time_domain_, "time_domain", "global");
  setAndGetNodeParameter<bool>(enable_heartbeat_, "enable_heartbeat", false);
  setAndGetNodeParameter<std::string>(echo_mode_, "echo_mode", "single channel");
  setAndGetNodeParameter<std::string>(point_cloud_qos_, "point_cloud_qos", "default");
  setAndGetNodeParameter<std::string>(frame_id_, "frame_id", "scan");
  setAndGetNodeParameter<float>(min_angle_, "min_angle", -135.0);
  setAndGetNodeParameter<float>(max_angle_, "max_angle", 135.0);
  setAndGetNodeParameter<float>(min_range_, "min_range", 0.05);
  setAndGetNodeParameter<float>(max_range_, "max_range", 30.0);
}

void OBLidarNode::setupDevices() {
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
  if (device_->isPropertySupported(OB_PROP_HEARTBEAT_BOOL, OB_PERMISSION_READ_WRITE)) {
    RCLCPP_INFO_STREAM(logger_, "Setting heartbeat to " << (enable_heartbeat_ ? "ON" : "OFF"));
    TRY_TO_SET_PROPERTY(setBoolProperty, OB_PROP_HEARTBEAT_BOOL, enable_heartbeat_);
  }
  if (device_->isPropertySupported(OB_PROP_LIDAR_ECHO_MODE_INT, OB_PERMISSION_READ_WRITE)) {
    if (echo_mode_ == "single channel") {
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_LIDAR_ECHO_MODE_INT, 0);
    } else if (echo_mode_ == "dual channel") {
      TRY_TO_SET_PROPERTY(setIntProperty, OB_PROP_LIDAR_ECHO_MODE_INT, 1);
    }
    RCLCPP_INFO_STREAM(
        logger_, "Setting echo mode to "
                     << (device_->getIntProperty(OB_PROP_LIDAR_ECHO_MODE_INT) ? "single channel"
                                                                              : "dual channel"));
  }
}

void OBLidarNode::setupProfiles() {
  if (enable_stream_[LIDAR]) {
    const auto &sensor = sensors_[LIDAR];
    CHECK_NOTNULL(sensor.get());
    auto profiles = sensor->getStreamProfileList();
    CHECK_NOTNULL(profiles.get());
    CHECK(profiles->getCount() > 0);
    for (size_t i = 0; i < profiles->getCount(); i++) {
      auto base_profile = profiles->getProfile(i)->as<ob::LiDARStreamProfile>();
      if (base_profile == nullptr) {
        throw std::runtime_error("Failed to get profile " + std::to_string(i));
      }
      auto profile = base_profile->as<ob::LiDARStreamProfile>();
      if (profile == nullptr) {
        throw std::runtime_error("Failed cast profile to LiDARStreamProfile");
      }
      RCLCPP_DEBUG_STREAM(
          logger_,
          "Sensor profile: " << "stream_type: " << magic_enum::enum_name(profile->getType())
                             << "Scan Rate: " << magic_enum::enum_name(profile->getScanRate())
                             << "Format:" << magic_enum::enum_name(profile->getFormat()));
      supported_profiles_[LIDAR].emplace_back(profile);
    }
    std::shared_ptr<ob::LiDARStreamProfile> selected_profile;
    std::shared_ptr<ob::LiDARStreamProfile> default_profile;
    try {
      if (rate_[LIDAR] == OB_LIDAR_SCAN_UNKNOWN && format_[LIDAR] == OB_FORMAT_UNKNOWN) {
        selected_profile = profiles->getProfile(0)->as<ob::LiDARStreamProfile>();
      } else {
        selected_profile = profiles->getLiDARStreamProfile(rate_[LIDAR], format_[LIDAR]);
      }

    } catch (const ob::Error &ex) {
      RCLCPP_ERROR_STREAM(
          logger_, "Failed to get " << stream_name_[LIDAR] << "  profile: " << ex.getMessage());
      RCLCPP_ERROR_STREAM(logger_, "Stream: " << magic_enum::enum_name(LIDAR.first)
                                              << ", Stream Index: " << LIDAR.second
                                              << ", Scan Rate: " << rate_[LIDAR]
                                              << "Format:" << format_[LIDAR]);
      RCLCPP_INFO_STREAM(logger_, "Available profiles:");
      printSensorProfiles(sensor);
      RCLCPP_ERROR(logger_, "Because can not set this stream, so exit.");
      exit(-1);
    }
    if (!selected_profile) {
      RCLCPP_WARN_STREAM(logger_, "Given stream configuration is not supported by the device! "
                                      << " Stream: " << magic_enum::enum_name(LIDAR.first)
                                      << ", Stream Index: " << LIDAR.second
                                      << ", Scan Rate: " << rate_[LIDAR]);
      if (default_profile) {
        RCLCPP_WARN_STREAM(logger_, "Using default profile instead.");
        RCLCPP_WARN_STREAM(logger_, "default scan Rate "
                                        << magic_enum::enum_name(default_profile->getScanRate())
                                        << "default format:"
                                        << magic_enum::enum_name(default_profile->getFormat()));
        selected_profile = default_profile;
      } else {
        RCLCPP_ERROR_STREAM(
            logger_, " NO default_profile found , Stream: " << magic_enum::enum_name(LIDAR.first)
                                                            << " will be disable");
        enable_stream_[LIDAR] = false;
      }
    }
    CHECK_NOTNULL(selected_profile);
    stream_profile_[LIDAR] = selected_profile;
    rate_[LIDAR] = selected_profile->getScanRate();
    format_[LIDAR] = selected_profile->getFormat();
    RCLCPP_INFO_STREAM(
        logger_, " stream " << stream_name_[LIDAR] << " is enabled - scan rate: "
                            << magic_enum::enum_name(selected_profile->getScanRate())
                            << "  format:" << magic_enum::enum_name(selected_profile->getFormat()));
  }
}

void OBLidarNode::selectBaseStream() {
  enable_stream_[LIDAR] = true;
  if (enable_stream_[LIDAR]) {
    base_stream_ = LIDAR;
  }
}

void OBLidarNode::printSensorProfiles(const std::shared_ptr<ob::Sensor> &sensor) {
  auto profiles = sensor->getStreamProfileList();
  for (size_t i = 0; i < profiles->getCount(); i++) {
    auto origin_profile = profiles->getProfile(i);
    if (sensor->getType() == OB_SENSOR_LIDAR) {
      auto profile = origin_profile->as<ob::LiDARStreamProfile>();
      RCLCPP_INFO_STREAM(logger_, "lidar scan rate: "
                                      << profile->getScanRate()
                                      << " format:" << magic_enum::enum_name(profile->getFormat()));
    } else {
      RCLCPP_INFO_STREAM(logger_, "unknown profile: " << magic_enum::enum_name(sensor->getType()));
    }
  }
}
void OBLidarNode::setupPublishers() {
  auto point_cloud_qos_profile = getRMWQosProfileFromString(point_cloud_qos_);
  if (use_intra_process_) {
    point_cloud_qos_profile = rmw_qos_profile_default;
  }
  RCLCPP_INFO_STREAM(logger_, "rate_ " << magic_enum::enum_name(rate_[LIDAR]) << "format_"
                                       << magic_enum::enum_name(format_[LIDAR]));
  if (format_[LIDAR] == OB_FORMAT_LIDAR_SCAN) {
    scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
        "scan/points", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                                   point_cloud_qos_profile));
  } else if (format_[LIDAR] == OB_FORMAT_LIDAR_POINT ||
             format_[LIDAR] == OB_FORMAT_LIDAR_SPHERE_POINT) {
    cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "cloud/points", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                                    point_cloud_qos_profile));
  }
}

void OBLidarNode::startStreams() {
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
    enable_stream_[LIDAR] = false;
    setupPipelineConfig();
    pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet> &frame_set) {
      onNewFrameSetCallback(frame_set);
    });
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to start pipeline");
    throw std::runtime_error("Failed to start pipeline");
  }
  pipeline_started_.store(true);
}

void OBLidarNode::stopStreams() {
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

void OBLidarNode::setupPipelineConfig() {
  if (pipeline_config_) {
    pipeline_config_.reset();
  }
  pipeline_config_ = std::make_shared<ob::Config>();
  if (enable_stream_[LIDAR]) {
    RCLCPP_INFO_STREAM(logger_, "Enable " << stream_name_[LIDAR] << " stream");
    auto profile = stream_profile_[LIDAR]->as<ob::LiDARStreamProfile>();

    if (enable_stream_[LIDAR]) {
      auto video_profile = profile;
      RCLCPP_INFO_STREAM(
          logger_, "lidar profile: " << magic_enum::enum_name(video_profile->getScanRate()) << "  "
                                     << magic_enum::enum_name(video_profile->getFormat()));
    }

    RCLCPP_INFO_STREAM(logger_,
                       "Stream " << stream_name_[LIDAR]
                                 << " scan rate: " << magic_enum::enum_name(profile->getScanRate())
                                 << " format: " << magic_enum::enum_name(profile->getFormat()));
    pipeline_config_->enableStream(stream_profile_[LIDAR]);
  }
}

void OBLidarNode::onNewFrameSetCallback(std::shared_ptr<ob::FrameSet> frame_set) {
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
    RCLCPP_INFO_ONCE(logger_, "New frame received");
    if (format_[LIDAR] == OB_FORMAT_LIDAR_SCAN) {
      publishScan(frame_set);
    } else if (format_[LIDAR] == OB_FORMAT_LIDAR_POINT ||
               format_[LIDAR] == OB_FORMAT_LIDAR_SPHERE_POINT) {
      publishPointCloud(frame_set);
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: unknown error");
  }
}

void OBLidarNode::publishScan(std::shared_ptr<ob::FrameSet> frame_set) {
  (void)frame_set;
  if (frame_set == nullptr) {
    return;
  }
  //   std::shared_ptr<ob::LiDARPointsFrame> lidar_frame;
  auto lidar_frame = frame_set->getFrame(OB_FRAME_LIDAR_POINTS);
  auto *scans_data = reinterpret_cast<OBLiDARScanPoint *>(lidar_frame->getData());
  auto scan_count = lidar_frame->getDataSize() / sizeof(OBLiDARScanPoint);
  // bool valid_point = points[i].z >= min_depth && points[i].z <= max_depth;
  // if (valid_point || ordered_pc_) {
  //   *iter_x = static_cast<float>(points[i].x / 1000.0);
  //   *iter_y = static_cast<float>(points[i].y / 1000.0);
  //   *iter_z = static_cast<float>(points[i].z / 1000.0);
  //   ++iter_x, ++iter_y, ++iter_z;
  //   valid_count++;
  // }
  auto frame_timestamp = getFrameTimestampUs(lidar_frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header.stamp = timestamp;
  scan_msg->header.frame_id = frame_id_;
  scan_msg->angle_min = 0.7853981852531433;
  scan_msg->angle_max = 5.495169162750244;
  scan_msg->angle_increment = 0.0026179938577115536;
  scan_msg->time_increment = 1.0 / rate_int_[LIDAR] / scan_count;
  scan_msg->scan_time = 1.0 / rate_int_[LIDAR];
  scan_msg->range_min = min_range_;
  scan_msg->range_max = max_range_;
  scan_msg->ranges.resize(scan_count);
  scan_msg->intensities.resize(scan_count);
  for (size_t i = 0; i < scan_count; i++) {
    // RCLCPP_INFO_STREAM(logger_, " angle: " << scans_data[i].angle
    //                                        << " distance: " << scans_data[i].distance
    //                                        << " intensity: " << scans_data[i].intensity);
    if (scans_data->distance < min_range_ && scans_data->distance > max_range_) {
      scans_data++;
      continue;
    }
    scan_msg->ranges[i] = scans_data[i].distance / 1000.0;
    scan_msg->intensities[i] = scans_data[i].intensity;
  }
  filterScan(*scan_msg);
  scan_pub_->publish(std::move(scan_msg));
  //   RCLCPP_INFO_STREAM(logger_, "getFormat "<<magic_enum::enum_name(lidar_frame->getFormat()));
  // RCLCPP_INFO_STREAM(logger_, "getDataSize "<<lidar_frame->getDataSize());
  // RCLCPP_INFO_STREAM(logger_, "getType "<<lidar_frame->getType());
  // RCLCPP_INFO_STREAM(logger_, "getSystemTimeStampUs "<<lidar_frame->getSystemTimeStampUs());
  // RCLCPP_INFO_STREAM(logger_, "getGlobalTimeStampUs "<<lidar_frame->getGlobalTimeStampUs());
  // RCLCPP_INFO_STREAM(logger_, "getTimeStampUs "<<lidar_frame->getTimeStampUs());
  // RCLCPP_INFO_STREAM(logger_, "getIndex "<<lidar_frame->getIndex());
  // RCLCPP_INFO_STREAM(logger_, "getMetadataSize "<<lidar_frame->getMetadataSize());
}

void OBLidarNode::publishPointCloud(std::shared_ptr<ob::FrameSet> frame_set) {
  (void)frame_set;
  //   RCLCPP_INFO_STREAM(logger_, "publishPointCloud ");
}

uint64_t OBLidarNode::getFrameTimestampUs(const std::shared_ptr<ob::Frame> &frame) {
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

void OBLidarNode::filterScan(sensor_msgs::msg::LaserScan &scan) {
  double current_angle = scan.angle_min;
  double max_angle = deg2rad(max_angle_);
  double min_angle = deg2rad(min_angle_);
  // map to 0 - 2 * M_PI
  max_angle = std::fmod(max_angle + M_PI, 2 * M_PI);
  if (max_angle < 0) {
    max_angle += 2 * M_PI;
  }

  min_angle = std::fmod(min_angle + M_PI, 2 * M_PI);
  if (min_angle < 0) {
    min_angle += 2 * M_PI;
  }
  if (min_angle > max_angle) {
    std::swap(min_angle, max_angle);
  }
  for (size_t i = 0; i < scan.ranges.size(); ++i, current_angle += scan.angle_increment) {
    bool is_angle_in_range = (current_angle >= min_angle && current_angle <= max_angle);

    bool is_range_in_range = (scan.ranges[i] >= min_range_ && scan.ranges[i] <= max_range_);

    if (!(is_angle_in_range && is_range_in_range)) {
      scan.ranges[i] = 0;
      scan.intensities[i] = 0;
    }
  }
}
}  // namespace orbbec_lidar
}  // namespace orbbec_camera
