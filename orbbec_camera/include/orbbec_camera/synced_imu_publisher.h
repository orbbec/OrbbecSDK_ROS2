#pragma once
#include <rclcpp/rclcpp.hpp>
#include <glog/logging.h>
#include <sensor_msgs/msg/imu.hpp>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace orbbec_camera {
class SyncedImuPublisher {
 public:
  SyncedImuPublisher(rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher,
                     size_t queue_size = 1000);
  ~SyncedImuPublisher();
  void publish(const sensor_msgs::msg::Imu& imu_msg);
  void pause();
  void resume();
  void setQueueSize(size_t queue_size);

  void enable(bool enable);

 private:
  void publishPendingMessages();

 private:
  std::mutex mutex_;
  std::queue<sensor_msgs::msg::Imu> queue_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  bool is_enabled_ = true;
  bool is_paused_ = false;
  size_t queue_size_ = 1000;
};
}  // namespace orbbec_camera
