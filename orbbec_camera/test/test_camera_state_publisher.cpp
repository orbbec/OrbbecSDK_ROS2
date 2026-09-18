#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>

#include "orbbec_camera/camera_state_publisher.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CameraStatusPublisherTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("camera_status_publisher_test_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    subscription_ = node_->create_subscription<CameraStatusMsg>(
        "camera_status", 10, [this](const CameraStatusMsg::SharedPtr msg) {
          last_msg_ = *msg;
          received_ = true;
        });
    publisher_ = std::make_unique<CameraStatusPublisher>(node_.get());
  }

  void TearDown() override { executor_->remove_node(node_); }

  // Spins the executor until predicate() is true or the timeout elapses.
  // Returns the final value of predicate().
  bool spinUntil(const std::function<bool()> &predicate, std::chrono::milliseconds timeout) {
    const rclcpp::Time start = node_->now();
    const rclcpp::Time end = start + rclcpp::Duration(timeout);
    while (!predicate() && node_->now() < end && rclcpp::ok()) {
      executor_->spin_some();
      std::this_thread::sleep_for(10ms);
    }
    return predicate();
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Executor::SharedPtr executor_;
  rclcpp::Subscription<CameraStatusMsg>::SharedPtr subscription_;
  std::unique_ptr<CameraStatusPublisher> publisher_;
  CameraStatusMsg last_msg_;
  bool received_ = false;
};

TEST_F(CameraStatusPublisherTest, ConnectedSetsFlagsAndPublishes) {
  publisher_->connected();

  ASSERT_TRUE(spinUntil([this] { return received_; }, 1000ms));
  EXPECT_TRUE(last_msg_.connected);
  EXPECT_FALSE(last_msg_.color_stream_enabled);
  EXPECT_FALSE(last_msg_.depth_stream_enabled);
  EXPECT_FALSE(last_msg_.imu_stream_enabled);
}

TEST_F(CameraStatusPublisherTest, DisconnectedClearsConnectedAndInitialized) {
  publisher_->connected();
  ASSERT_TRUE(spinUntil([this] { return received_; }, 1000ms));

  received_ = false;
  publisher_->disconnected();
  ASSERT_TRUE(spinUntil([this] { return received_; }, 1000ms));
  EXPECT_FALSE(last_msg_.connected);
  EXPECT_FALSE(last_msg_.initialized);
}

TEST_F(CameraStatusPublisherTest, UpdateColorStreamActiveTogglesFlag) {
  publisher_->updateColorStreamActive(true);
  ASSERT_TRUE(spinUntil([this] { return received_; }, 1000ms));
  EXPECT_TRUE(last_msg_.color_stream_enabled);

  received_ = false;
  publisher_->updateColorStreamActive(false);
  ASSERT_TRUE(spinUntil([this] { return received_; }, 1000ms));
  EXPECT_FALSE(last_msg_.color_stream_enabled);
  EXPECT_FALSE(last_msg_.color_frame_timeout);
}

TEST_F(CameraStatusPublisherTest, ExpectedFrameRateIsReflectedInPublishedMessage) {
  publisher_->setColorExpectedFrameRate(30.0);
  publisher_->updateColorStreamActive(true);  // setColorExpectedFrameRate() alone doesn't publish.

  ASSERT_TRUE(spinUntil([this] { return received_; }, 1000ms));
  EXPECT_EQ(last_msg_.color_expected_frame_rate, 30);
}

TEST_F(CameraStatusPublisherTest, SetMessageAndPublishAppliesMutationAndPublishes) {
  publisher_->setMessageAndPublish(
      [](orbbec_camera_msgs::msg::CameraStatus &msg) { msg.color_stream_enabled = true; });

  ASSERT_TRUE(spinUntil([this] { return received_; }, 1000ms));
  EXPECT_TRUE(last_msg_.color_stream_enabled);
}

TEST_F(CameraStatusPublisherTest, ColorAndDepthFrameCountsAreIndependent) {
  publisher_->updateColorStreamActive(true);
  ASSERT_TRUE(spinUntil([this] { return received_; }, 1000ms));

  publisher_->colorFrameReceived(node_->now());
  publisher_->colorFrameReceived(node_->now());
  publisher_->depthFrameReceived(node_->now());

  // publishStats() runs on a periodic 1 second wall timer; wait for it to fire.
  ASSERT_TRUE(spinUntil([this] { return last_msg_.stats_valid; }, 1500ms));
  EXPECT_EQ(last_msg_.color_frame_rate, 2);
  EXPECT_EQ(last_msg_.depth_frame_rate, 1);
}

TEST_F(CameraStatusPublisherTest, ConnectedResetsAccumulatedStats) {
  publisher_->updateColorStreamActive(true);
  ASSERT_TRUE(spinUntil([this] { return received_; }, 1000ms));

  publisher_->colorFrameReceived(node_->now());
  publisher_->colorFrameReceived(node_->now());

  publisher_->connected();  // Should reset stats before the next periodic publish.

  ASSERT_TRUE(spinUntil([this] { return last_msg_.stats_valid; }, 1500ms));
  EXPECT_EQ(last_msg_.color_frame_rate, 0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
