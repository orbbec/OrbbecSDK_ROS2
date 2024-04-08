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
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/opencv.hpp>

#include "orbbec_camera/d2c_viewer.h"

namespace orbbec_camera {
D2CViewer::D2CViewer(rclcpp::Node* const node, rmw_qos_profile_t rgb_qos,
                     rmw_qos_profile_t depth_qos)
    : node_(node), logger_(rclcpp::get_logger("d2c_viewer")) {
  rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      node_, "color/image_raw", rgb_qos);
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      node_, "depth/image_raw", depth_qos);
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), *rgb_sub_,
                                                                        *depth_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.10)); // 100ms

  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_->registerCallback(std::bind(&D2CViewer::messageCallback, this, _1, _2));
  d2c_viewer_pub_ =
      node_->create_publisher<sensor_msgs::msg::Image>("depth_to_color/image_raw", rclcpp::QoS(1));
}
D2CViewer::~D2CViewer() = default;

void D2CViewer::messageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                                const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
  if (rgb_msg->width != depth_msg->width || rgb_msg->height != depth_msg->height) {
    RCLCPP_ERROR(logger_, "rgb and depth image size not match(%d, %d) vs (%d, %d)", rgb_msg->width,
                 rgb_msg->height, depth_msg->width, depth_msg->height);
    return;
  }
  auto rgb_img_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8);
  auto depth_img_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cv::Mat gray_depth, depth_img, d2c_img;
  depth_img_ptr->image.convertTo(gray_depth, CV_8UC1);
  cv::cvtColor(gray_depth, depth_img, cv::COLOR_GRAY2RGB);
  depth_img.setTo(cv::Scalar(255, 255, 0), depth_img);
  cv::bitwise_or(rgb_img_ptr->image, depth_img, d2c_img);
  sensor_msgs::msg::Image::SharedPtr d2c_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, d2c_img)
          .toImageMsg();
  d2c_msg->header = rgb_msg->header;
  d2c_viewer_pub_->publish(*d2c_msg);
}
}  // namespace orbbec_camera
