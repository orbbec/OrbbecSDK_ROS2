// Copyright 2023 Intel Corporation. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include "orbbec_camera_msgs/msg/imu_info.hpp"
#include "orbbec_camera_msgs/msg/extrinsics.hpp"
#include "orbbec_camera_msgs/msg/metadata.hpp"
#include "orbbec_camera_msgs/msg/rgbd.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace orbbec_camera {
class FrameLatencyNode : public rclcpp::Node {
 public:
  explicit FrameLatencyNode(const rclcpp::NodeOptions& node_options =
                                rclcpp::NodeOptions().use_intra_process_comms(true));

  FrameLatencyNode(const std::string& node_name, const std::string& ns,
                   const rclcpp::NodeOptions& node_options =
                       rclcpp::NodeOptions().use_intra_process_comms(true));

  template <typename MsgType>
  void createListener(const std::string& topic_name, rmw_qos_profile_t qos_profile);

  void createTFListener(const std::string& topic_name, rmw_qos_profile_t qos_profile);

 private:
  std::shared_ptr<void> sub_ = nullptr;

  rclcpp::Logger logger_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t frame_count_ = 0;
};
}  // namespace orbbec_camera
