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

#include <nlohmann/json.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <atomic>
// #include "ob_lidar_node.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <image_publisher/image_publisher.hpp>
#include <image_transport/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "libobsensor/ObSensor.hpp"

#include "orbbec_camera_msgs/msg/device_info.hpp"
#include "orbbec_camera_msgs/srv/get_device_info.hpp"
#include "orbbec_camera_msgs/msg/extrinsics.hpp"
#include "orbbec_camera_msgs/msg/metadata.hpp"
#include "orbbec_camera_msgs/msg/imu_info.hpp"
#include "orbbec_camera_msgs/srv/get_int32.hpp"
#include "orbbec_camera_msgs/srv/get_string.hpp"
#include "orbbec_camera_msgs/srv/set_int32.hpp"
#include "orbbec_camera_msgs/srv/get_bool.hpp"
#include "orbbec_camera_msgs/srv/set_string.hpp"
#include "orbbec_camera_msgs/srv/set_filter.hpp"
#include "orbbec_camera_msgs/srv/set_arrays.hpp"
#include "orbbec_camera/constants.h"
#include "orbbec_camera/dynamic_params.h"
#include "orbbec_camera/d2c_viewer.h"
#include "magic_enum/magic_enum.hpp"
#include "orbbec_camera/image_publisher.h"
#include "jpeg_decoder.h"
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <unistd.h>

#if defined(ROS_JAZZY) || defined(ROS_IRON)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#define STREAM_NAME(sip)                                                                       \
  (static_cast<std::ostringstream&&>(std::ostringstream()                                      \
                                     << _stream_name[sip.first]                                \
                                     << ((sip.second > 0) ? std::to_string(sip.second) : ""))) \
      .str()
#define FRAME_ID(sip)                                                                              \
  (static_cast<std::ostringstream&&>(std::ostringstream()                                          \
                                     << getNamespaceStr() << "_" << STREAM_NAME(sip) << "_frame")) \
      .str()
#define OPTICAL_FRAME_ID(sip)                                                                     \
  (static_cast<std::ostringstream&&>(                                                             \
       std::ostringstream() << getNamespaceStr() << "_" << STREAM_NAME(sip) << "_optical_frame")) \
      .str()
#define ALIGNED_DEPTH_TO_FRAME_ID(sip)                                            \
  (static_cast<std::ostringstream&&>(std::ostringstream()                         \
                                     << getNamespaceStr() << "_aligned_depth_to_" \
                                     << STREAM_NAME(sip) << "_frame"))            \
      .str()
#define BASE_FRAME_ID() \
  (static_cast<std::ostringstream&&>(std::ostringstream() << getNamespaceStr() << "_link")).str()
#define ODOM_FRAME_ID()                                                                           \
  (static_cast<std::ostringstream&&>(std::ostringstream() << getNamespaceStr() << "_odom_frame")) \
      .str()

#define DEVICE_PATH "/dev/camsync"

namespace orbbec_camera {
namespace orbbec_lidar {
using GetDeviceInfo = orbbec_camera_msgs::srv::GetDeviceInfo;
using Extrinsics = orbbec_camera_msgs::msg::Extrinsics;
using SetInt32 = orbbec_camera_msgs::srv::SetInt32;
using GetInt32 = orbbec_camera_msgs::srv::GetInt32;
using GetString = orbbec_camera_msgs::srv::GetString;
using SetString = orbbec_camera_msgs::srv::SetString;
using SetBool = std_srvs::srv::SetBool;
using GetBool = orbbec_camera_msgs::srv::GetBool;
using SetFilter = orbbec_camera_msgs::srv::SetFilter;
using SetArrays = orbbec_camera_msgs::srv::SetArrays;

typedef std::pair<ob_stream_type, int> stream_index_pair;

const stream_index_pair LIDAR{OB_STREAM_LIDAR, 0};

const stream_index_pair GYRO{OB_STREAM_GYRO, 0};
const stream_index_pair ACCEL{OB_STREAM_ACCEL, 0};

const std::vector<stream_index_pair> IMAGE_STREAMS = {LIDAR};

const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL};

class OBLidarNode {
 public:
  OBLidarNode(rclcpp::Node* node, std::shared_ptr<ob::Device> device,
              std::shared_ptr<Parameters> parameters, bool use_intra_process = false);

  template <class T>
  void setAndGetNodeParameter(
      T& param, const std::string& param_name, const T& default_value,
      const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor());  // set and get parameter

  ~OBLidarNode() noexcept;

  void clean() noexcept;

  void rebootDevice();

  void startStreams();

 private:
  void setupDevices();

  void selectBaseStream();

  void setupProfiles();

  void getParameters();

  void setupTopics();

  void setupPipelineConfig();

  void printSensorProfiles(const std::shared_ptr<ob::Sensor>& sensor);

  void setupPublishers();

  void stopStreams();

  void onNewFrameSetCallback(std::shared_ptr<ob::FrameSet> frame_set);

  void publishScan(std::shared_ptr<ob::FrameSet> frame_set);

  void publishPointCloud(std::shared_ptr<ob::FrameSet> frame_set);

  void publishSpherePointCloud(std::shared_ptr<ob::FrameSet> frame_set);

  uint64_t getFrameTimestampUs(const std::shared_ptr<ob::Frame>& frame);

  void filterScan(sensor_msgs::msg::LaserScan& scan);

  sensor_msgs::msg::PointCloud2 filterPointCloud(sensor_msgs::msg::PointCloud2& point_cloud) const;

  void publishStaticTransforms();

  void calcAndPublishStaticTransform();

  void publishStaticTF(const rclcpp::Time& t, const tf2::Vector3& trans, const tf2::Quaternion& q,
                       const std::string& from, const std::string& to);

  void publishDynamicTransforms();

 private:
  rclcpp::Node* node_ = nullptr;
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::shared_ptr<Parameters> parameters_ = nullptr;
  std::condition_variable color_frame_queue_cv_;
  rclcpp::Logger logger_;
  std::atomic_bool is_running_{false};
  std::unique_ptr<ob::Pipeline> pipeline_ = nullptr;
  std::atomic_bool pipeline_started_{false};
  std::string camera_name_ = "camera";
  std::shared_ptr<ob::Config> pipeline_config_ = nullptr;
  std::map<stream_index_pair, std::shared_ptr<ob::Sensor>> sensors_;
  std::map<stream_index_pair, std::string> optical_frame_id_;
  std::map<stream_index_pair, ob_format> format_;
  std::map<stream_index_pair, std::string> format_str_;
  std::map<stream_index_pair, std::vector<std::shared_ptr<ob::LiDARStreamProfile>>>
      supported_profiles_;
  std::map<stream_index_pair, std::shared_ptr<ob::StreamProfile>> stream_profile_;
  stream_index_pair base_stream_ = LIDAR;

  std::map<stream_index_pair, bool> enable_stream_;
  std::map<stream_index_pair, std::string> stream_name_;

  std::atomic_bool is_camera_node_initialized_{false};
  std::mutex device_lock_;
  std::shared_ptr<std::thread> colorFrameThread_ = nullptr;
  uint8_t* rgb_buffer_ = nullptr;
  uint8_t* rgb_point_cloud_buffer_ = nullptr;
  float* xy_table_data_ = nullptr;
  float* depth_xy_table_data_ = nullptr;
  uint8_t* depth_point_cloud_buffer_ = nullptr;
  bool publish_tf_ = false;
  bool tf_published_ = false;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_ = nullptr;
  std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  std::string point_cloud_qos_;
  std::vector<geometry_msgs::msg::TransformStamped> static_tf_msgs_;
  std::shared_ptr<std::thread> tf_thread_ = nullptr;
  std::condition_variable tf_cv_;
  double tf_publish_rate_ = 10.0;
  // Only for Gemini2 device

  std::string time_domain_ = "device";  // device, system, global
  bool enable_heartbeat_ = false;
  bool use_intra_process_ = false;

  // lidar
  std::string lidar_format_ = "ANY";
  int lidar_rate_ = 0;
  std::string echo_mode_ = "single channel";
  std::map<stream_index_pair, int> rate_int_;
  std::map<stream_index_pair, OBLiDARScanRate> rate_;
  std::map<stream_index_pair, std::string> frame_id_;
  float min_angle_ = -135.0;
  float max_angle_ = 135.0;
  float min_range_ = 0.05;
  float max_range_ = 30.0;
};

}  // namespace orbbec_lidar
}  // namespace orbbec_camera
