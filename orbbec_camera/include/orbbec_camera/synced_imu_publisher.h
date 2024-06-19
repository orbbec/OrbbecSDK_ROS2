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
#include <rclcpp/rclcpp.hpp>
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
