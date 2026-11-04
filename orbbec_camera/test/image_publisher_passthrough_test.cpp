/*******************************************************************************
 * Copyright (c) 2026 Orbbec 3D Technology, Inc
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

#include <gtest/gtest.h>

#include <chrono>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <thread>

#include "orbbec_camera/image_publisher.h"

namespace orbbec_camera {
namespace {

class ImagePublisherPassthroughTest : public testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("publisher_policy_test", "/orbbec_transport_test");
  }

  void TearDown() override {
    clearGlobalImageTransportPublishers(*node_);
    node_.reset();
  }

  template <typename Predicate>
  bool waitFor(Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do {
      rclcpp::spin_some(node_);
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  std::shared_ptr<rclcpp::Node> node_;
  const std::string base_topic_ = "/orbbec_transport_test/color/image_raw";
  const std::string allowlist_parameter_ = "color.image_raw.enable_pub_plugins";
  const std::vector<std::string> exclusions_{"image_transport/compressed"};
};

TEST_F(ImagePublisherPassthroughTest, PreservesAllowlistAndReservesCompressedTopic) {
  const std::vector<std::string> configured{"image_transport/raw", "image_transport/compressed"};
  node_->declare_parameter(allowlist_parameter_, configured);
  const auto decoded =
      getGlobalImageTransportPublisher(*node_, base_topic_, rmw_qos_profile_default, exclusions_);
  const auto passthrough = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
      base_topic_ + "/compressed", rclcpp::QoS(10));

  ASSERT_NE(passthrough, nullptr);
  ASSERT_TRUE(waitFor([&]() {
    return node_->get_publishers_info_by_topic(base_topic_).size() == 1 &&
           node_->get_publishers_info_by_topic(base_topic_ + "/compressed").size() == 1;
  }));
  EXPECT_EQ(node_->get_parameter(allowlist_parameter_).as_string_array(), configured);
  EXPECT_EQ(decoded->get_subscription_count(), 0U);
}

TEST_F(ImagePublisherPassthroughTest, JpegSubscribersDoNotRequestDecodedImages) {
  node_->declare_parameter(allowlist_parameter_,
                           std::vector<std::string>{"image_transport/compressed"});
  const auto decoded =
      getGlobalImageTransportPublisher(*node_, base_topic_, rmw_qos_profile_default, exclusions_);
  const auto passthrough = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
      base_topic_ + "/compressed", rclcpp::QoS(10));
  const auto subscriber = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      base_topic_ + "/compressed", rclcpp::QoS(10),
      [](sensor_msgs::msg::CompressedImage::ConstSharedPtr) {});

  ASSERT_NE(subscriber, nullptr);
  ASSERT_TRUE(waitFor([&]() { return passthrough->get_subscription_count() > 0; }));
  EXPECT_EQ(decoded->get_subscription_count(), 0U);
  EXPECT_TRUE(node_->get_publishers_info_by_topic(base_topic_).empty());
  EXPECT_EQ(node_->get_publishers_info_by_topic(base_topic_ + "/compressed").size(), 1U);
}

TEST_F(ImagePublisherPassthroughTest, CacheSeparatesNativeAndPassthroughPolicies) {
  node_->declare_parameter(allowlist_parameter_, std::vector<std::string>{"image_transport/raw"});
  auto native = getGlobalImageTransportPublisher(*node_, base_topic_, rmw_qos_profile_default);
  auto decoded =
      getGlobalImageTransportPublisher(*node_, base_topic_, rmw_qos_profile_default, exclusions_);
  EXPECT_NE(native.get(), decoded.get());
  EXPECT_EQ(decoded.get(), getGlobalImageTransportPublisher(*node_, base_topic_,
                                                            rmw_qos_profile_default, exclusions_)
                               .get());
  native.reset();
  const auto restored =
      getGlobalImageTransportPublisher(*node_, base_topic_, rmw_qos_profile_default);
  EXPECT_NE(decoded.get(), restored.get());
}

}  // namespace
}  // namespace orbbec_camera
