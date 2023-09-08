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
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

#include "utils.h"

namespace orbbec_camera {
class D2CViewer {
 public:
  explicit D2CViewer(rclcpp::Node* const node, rmw_qos_profile_t rgb_qos,
                     rmw_qos_profile_t depth_qos);
  ~D2CViewer();

  void messageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

 private:
  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                                       sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr d2c_viewer_pub_;
};
}  // namespace orbbec_camera
