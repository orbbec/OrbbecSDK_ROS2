#include "orbbec_camera/utils.h"
#include "orbbec_camera/synced_imu_publisher.h"
#include <rclcpp/rclcpp.hpp>

namespace orbbec_camera {
SyncedImuPublisher::SyncedImuPublisher(
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher, size_t queue_size)
    : imu_publisher_(imu_publisher), queue_size_(queue_size) {}

SyncedImuPublisher::~SyncedImuPublisher() { publishPendingMessages(); }

void SyncedImuPublisher::publish(const sensor_msgs::msg::Imu &imu_msg) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto sub_num = imu_publisher_->get_subscription_count();
  if (sub_num == 0 || !is_enabled_) {
    return;
  }
  if (is_paused_) {
    while (queue_.size() >= queue_size_) {
      queue_.pop();
    }
    queue_.push(imu_msg);
  } else {
    imu_publisher_->publish(imu_msg);
  }
}

void SyncedImuPublisher::pause() {
  std::unique_lock<std::mutex> lock(mutex_);
  is_paused_ = true;
}

void SyncedImuPublisher::resume() {
  std::unique_lock<std::mutex> lock(mutex_);
  is_paused_ = false;
  publishPendingMessages();
}

void SyncedImuPublisher::setQueueSize(size_t queue_size) {
  std::unique_lock<std::mutex> lock(mutex_);
  queue_size_ = queue_size;
}

void SyncedImuPublisher::enable(bool enable) { is_enabled_ = enable; }

void SyncedImuPublisher::publishPendingMessages() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (!queue_.empty()) {
    imu_publisher_->publish(queue_.front());
    queue_.pop();
  }
}

}  // namespace orbbec_camera
