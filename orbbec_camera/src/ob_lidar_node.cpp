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
  //   clean();
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
  //   stopStreams();
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
    // setupProfiles();
    // selectBaseStream();
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
  setAndGetNodeParameter<std::string>(camera_name_, "camera_name", "camera");
  camera_link_frame_id_ = camera_name_ + "_link";
  for (auto stream_index : IMAGE_STREAMS) {
    std::string param_name;
    param_name = stream_name_[stream_index] + "_format";
    setAndGetNodeParameter(format_str_[stream_index], param_name, format_str_[stream_index]);
    format_[stream_index] = OBFormatFromString(format_str_[stream_index]);
    RCLCPP_INFO_STREAM(logger_, "lidar format str: " << format_str_[stream_index]);
    RCLCPP_INFO_STREAM(logger_, "lidar format: " << format_[stream_index]);
  }
  setAndGetNodeParameter<bool>(publish_tf_, "publish_tf", true);
  setAndGetNodeParameter<double>(tf_publish_rate_, "tf_publish_rate", 0.0);
  setAndGetNodeParameter<std::string>(time_domain_, "time_domain", "global");
  setAndGetNodeParameter<bool>(enable_heartbeat_, "enable_heartbeat", false);
  setAndGetNodeParameter<std::string>(echo_mode_, "echo_mode", "single channel");
  setAndGetNodeParameter<std::string>(point_cloud_qos_, "point_cloud_qos", "default");
}

void OBLidarNode::setupDevices() {
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

// void OBLidarNode::setupProfiles() {
//   // Image stream
//   for (const auto &elem : IMAGE_STREAMS) {
//     if (enable_stream_[elem]) {
//       const auto &sensor = sensors_[elem];
//       CHECK_NOTNULL(sensor.get());
//       auto profiles = sensor->getStreamProfileList();
//       CHECK_NOTNULL(profiles.get());
//       CHECK(profiles->getCount() > 0);
//       for (size_t i = 0; i < profiles->getCount(); i++) {
//         auto base_profile = profiles->getProfile(i)->as<ob::VideoStreamProfile>();
//         if (base_profile == nullptr) {
//           throw std::runtime_error("Failed to get profile " + std::to_string(i));
//         }
//         auto profile = base_profile->as<ob::VideoStreamProfile>();
//         if (profile == nullptr) {
//           throw std::runtime_error("Failed cast profile to VideoStreamProfile");
//         }
//         RCLCPP_DEBUG_STREAM(
//             logger_, "Sensor profile: "
//                          << "stream_type: " << magic_enum::enum_name(profile->getType())
//                          << "Format: " << profile->getFormat() << ", Width: " <<
//                          profile->getWidth()
//                          << ", Height: " << profile->getHeight() << ", FPS: " <<
//                          profile->getFps());
//         supported_profiles_[elem].emplace_back(profile);
//       }
//       std::shared_ptr<ob::VideoStreamProfile> selected_profile;
//       std::shared_ptr<ob::VideoStreamProfile> default_profile;
//       try {
//         if (width_[elem] == 0 && height_[elem] == 0 && fps_[elem] == 0 &&
//             format_[elem] == OB_FORMAT_UNKNOWN) {
//           selected_profile = profiles->getProfile(0)->as<ob::VideoStreamProfile>();
//         } else {
//           selected_profile = profiles->getVideoStreamProfile(width_[elem], height_[elem],
//                                                              format_[elem], fps_[elem]);
//         }

//       } catch (const ob::Error &ex) {
//         RCLCPP_ERROR_STREAM(
//             logger_, "Failed to get " << stream_name_[elem] << "  profile: " << ex.getMessage());
//         RCLCPP_ERROR_STREAM(
//             logger_, "Stream: " << magic_enum::enum_name(elem.first)
//                                 << ", Stream Index: " << elem.second << ", Width: " <<
//                                 width_[elem]
//                                 << ", Height: " << height_[elem] << ", FPS: " << fps_[elem]
//                                 << ", Format: " << magic_enum::enum_name(format_[elem]));
//         RCLCPP_ERROR(logger_,
//                      "Error: The device might be connected via USB 2.0. Please verify your "
//                      "configuration and try again. The current process will now exit.");
//         RCLCPP_INFO_STREAM(logger_, "Available profiles:");
//         printSensorProfiles(sensor);
//         RCLCPP_ERROR(logger_, "Because can not set this stream, so exit.");
//         exit(-1);
//       }

//       if (!selected_profile) {
//         RCLCPP_WARN_STREAM(logger_, "Given stream configuration is not supported by the device! "
//                                         << " Stream: " << magic_enum::enum_name(elem.first)
//                                         << ", Stream Index: " << elem.second
//                                         << ", Width: " << width_[elem]
//                                         << ", Height: " << height_[elem] << ", FPS: " <<
//                                         fps_[elem]
//                                         << ", Format: " << magic_enum::enum_name(format_[elem]));
//         if (default_profile) {
//           RCLCPP_WARN_STREAM(logger_, "Using default profile instead.");
//           RCLCPP_WARN_STREAM(logger_, "default FPS " << default_profile->getFps());
//           selected_profile = default_profile;
//         } else {
//           RCLCPP_ERROR_STREAM(
//               logger_, " NO default_profile found , Stream: " <<
//               magic_enum::enum_name(elem.first)
//                                                               << " will be disable");
//           enable_stream_[elem] = false;
//           continue;
//         }
//       }
//       CHECK_NOTNULL(selected_profile);
//       stream_profile_[elem] = selected_profile;
//       height_[elem] = static_cast<int>(selected_profile->getHeight());
//       width_[elem] = static_cast<int>(selected_profile->getWidth());
//       fps_[elem] = static_cast<int>(selected_profile->getFps());
//       format_[elem] = selected_profile->getFormat();
//       updateImageConfig(elem);
//       if (selected_profile->format() == OB_FORMAT_BGRA) {
//         images_[elem] = cv::Mat(height_[elem], width_[elem], CV_8UC4, cv::Scalar(0, 0, 0, 0));
//         encoding_[elem] = sensor_msgs::image_encodings::BGRA8;
//         unit_step_size_[COLOR] = 4 * sizeof(uint8_t);
//       } else if (selected_profile->format() == OB_FORMAT_RGBA) {
//         images_[elem] = cv::Mat(height_[elem], width_[elem], CV_8UC4, cv::Scalar(0, 0, 0, 0));
//         encoding_[elem] = sensor_msgs::image_encodings::RGBA8;
//         unit_step_size_[COLOR] = 4 * sizeof(uint8_t);
//       } else {
//         images_[elem] =
//             cv::Mat(height_[elem], width_[elem], image_format_[elem], cv::Scalar(0, 0, 0));
//       }
//       RCLCPP_INFO_STREAM(logger_,
//                          " stream "
//                              << stream_name_[elem]
//                              << " is enabled - width: " << selected_profile->getWidth()
//                              << ", height: " << selected_profile->getHeight()
//                              << ", fps: " << selected_profile->getFps() << ", "
//                              << "Format: " <<
//                              magic_enum::enum_name(selected_profile->getFormat()));
//     }
//   }
//   // IMU
//   for (const auto &stream_index : HID_STREAMS) {
//     if (!enable_stream_[stream_index]) {
//       continue;
//     }
//     try {
//       auto profile_list = sensors_[stream_index]->getStreamProfileList();
//       if (stream_index == ACCEL) {
//         auto full_scale_range = fullAccelScaleRangeFromString(imu_range_[stream_index]);
//         auto sample_rate = sampleRateFromString(imu_rate_[stream_index]);
//         auto profile = profile_list->getAccelStreamProfile(full_scale_range, sample_rate);
//         stream_profile_[stream_index] = profile;
//       } else if (stream_index == GYRO) {
//         auto full_scale_range = fullGyroScaleRangeFromString(imu_range_[stream_index]);
//         auto sample_rate = sampleRateFromString(imu_rate_[stream_index]);
//         auto profile = profile_list->getGyroStreamProfile(full_scale_range, sample_rate);
//         stream_profile_[stream_index] = profile;
//       }
//       RCLCPP_INFO_STREAM(logger_, "stream " << stream_name_[stream_index] << " full scale range "
//                                             << imu_range_[stream_index] << " sample rate "
//                                             << imu_rate_[stream_index]);
//     } catch (const ob::Error &e) {
//       RCLCPP_INFO_STREAM(logger_, "Failed to setup << " << stream_name_[stream_index]
//                                                         << " profile: " << e.getMessage());
//       enable_stream_[stream_index] = false;
//       stream_profile_[stream_index] = nullptr;
//     }
//   }
// }

// void OBLidarNode::selectBaseStream() {
//   if (enable_stream_[DEPTH]) {
//     base_stream_ = DEPTH;
//   } else if (enable_stream_[INFRA0]) {
//     base_stream_ = INFRA0;
//   } else if (enable_stream_[INFRA1]) {
//     base_stream_ = INFRA1;
//   } else if (enable_stream_[INFRA2]) {
//     base_stream_ = INFRA2;
//   } else if (enable_stream_[COLOR]) {
//     base_stream_ = COLOR;
//   }
// }
void OBLidarNode::setupPublishers() {
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  auto point_cloud_qos_profile = getRMWQosProfileFromString(point_cloud_qos_);
  if (use_intra_process_) {
    point_cloud_qos_profile = rmw_qos_profile_default;
  }
  cloud_pub_ = node_->create_publisher<PointCloud2>(
      "cloud/points", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                                  point_cloud_qos_profile));
}

}  // namespace orbbec_camera
