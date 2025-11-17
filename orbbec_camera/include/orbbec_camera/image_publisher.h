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
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <orbbec_camera/utils.h>
#include <fmt/format.h>
#include <image_transport/image_transport.hpp>
#include <shm_bridge/image_helper.h>

namespace orbbec_camera {
class image_publisher {
 public:
  virtual void publish(sensor_msgs::msg::Image::UniquePtr image_ptr) = 0;
  virtual size_t get_subscription_count() const = 0;
  virtual ~image_publisher() = default;
  image_publisher() = default;

};  // namespace image_publisher

// Native RCL implementation of an image publisher (needed for intra-process communication)
class image_rcl_publisher : public image_publisher {
 public:
  image_rcl_publisher(rclcpp::Node& node, const std::string& topic_name,
                      const rmw_qos_profile_t& qos);
  void publish(sensor_msgs::msg::Image::UniquePtr image_ptr) override;
  size_t get_subscription_count() const override;

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_impl;
};

// image_transport implementation of an image publisher (adds a compressed image topic)
class image_transport_publisher : public image_publisher {
 public:
  image_transport_publisher(rclcpp::Node& node, const std::string& topic_name,
                            const rmw_qos_profile_t& qos);
  void publish(sensor_msgs::msg::Image::UniquePtr image_ptr) override;
  size_t get_subscription_count() const override;

 private:
  std::shared_ptr<image_transport::Publisher> image_publisher_impl;
};

class shm_image_publisher : public image_publisher {
 public:
  shm_image_publisher(rclcpp::Node& node,
                      const std::string& topic_name,
                      const rmw_qos_profile_t& qos):
                      image_publisher()
{
  (void) qos;
  shm_publisher_ = std::make_shared<shm_bridge::ShmPublisher>(&node, topic_name);
}

  void publish(sensor_msgs::msg::Image::UniquePtr image_ptr) override;
  size_t get_subscription_count() const override;
 
 private:
  std::shared_ptr<shm_bridge::ShmPublisher> shm_publisher_;
};

}  // namespace orbbec_camera
