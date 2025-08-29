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
#include "orbbec_camera_msgs/msg/device_status.hpp"
#include <chrono>
#include <functional>
#include <mutex>
namespace orbbec_camera {
class FpsDelayStatus {
 public:
  explicit FpsDelayStatus(rclcpp::Logger logger) : log_level_(LogLevel::INFO), logger_(logger) {}

  void tick(u_int64_t stream_timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);

    double dt = (stream_timestamp - last_stream_timestamp_) / 1000000.0;
    double fps = (dt > 0) ? (1.0 / dt) : 0.0;

    // Convert now to milliseconds since steady_clock epoch
    auto now2 = std::chrono::system_clock::now();
    uint64_t ms_since_epoch =
        std::chrono::duration_cast<std::chrono::milliseconds>(now2.time_since_epoch()).count();
    double delay_ms =
        static_cast<double>(ms_since_epoch) - static_cast<double>(stream_timestamp / 1000.0);

    frame_count_++;
    fps_sum_ += fps;
    delay_sum_ += delay_ms;

    last_fps_ = fps;
    last_delay_ms_ = delay_ms;
    last_stream_timestamp_ = stream_timestamp;

    if (fps_max_ <= 0) fps_max_ = fps;
    if (fps_min_ <= 0) fps_min_ = fps;
    fps_max_ = std::max(fps_max_, fps);
    fps_min_ = std::min(fps_min_, fps);
    if (delay_max_ <= 0) delay_max_ = delay_ms;
    if (delay_min_ <= 0) delay_min_ = delay_ms;
    delay_max_ = std::max(delay_max_, delay_ms);
    delay_min_ = std::min(delay_min_, delay_ms);
  }

  void fillColorStatus(orbbec_camera_msgs::msg::DeviceStatus &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.color_frame_rate_cur = last_fps_;
    msg.color_frame_rate_avg = frame_count_ > 0 ? fps_sum_ / frame_count_ : 0;
    msg.color_frame_rate_min = fps_min_;
    msg.color_frame_rate_max = fps_max_;

    msg.color_delay_ms_cur = last_delay_ms_;
    msg.color_delay_ms_avg = frame_count_ > 0 ? delay_sum_ / frame_count_ : 0;
    msg.color_delay_ms_min = delay_min_;
    msg.color_delay_ms_max = delay_max_;

    last_delay_ms_ = 0.0;
    last_fps_ = 0.0;
    frame_count_ = 0;
    fps_sum_ = delay_sum_ = 0.0;
    fps_max_ = delay_max_ = 0.0;
    fps_min_ = delay_min_ = 0.0;
  }

  void fillDepthStatus(orbbec_camera_msgs::msg::DeviceStatus &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.depth_frame_rate_cur = last_fps_;
    msg.depth_frame_rate_avg = frame_count_ > 0 ? fps_sum_ / frame_count_ : 0;
    msg.depth_frame_rate_min = fps_min_;
    msg.depth_frame_rate_max = fps_max_;

    msg.depth_delay_ms_cur = last_delay_ms_;
    msg.depth_delay_ms_avg = frame_count_ > 0 ? delay_sum_ / frame_count_ : 0;
    msg.depth_delay_ms_min = delay_min_;
    msg.depth_delay_ms_max = delay_max_;

    // RCLCPP_ERROR_STREAM(logger_, "Depth status: " << fps_sum_ << "," << frame_count_);

    last_delay_ms_ = 0.0;
    last_fps_ = 0.0;
    frame_count_ = 0;
    fps_sum_ = delay_sum_ = 0.0;
    fps_max_ = delay_max_ = 0.0;
    fps_min_ = delay_min_ = 0.0;
  }

 private:
  mutable std::mutex mutex_;
  u_int64_t last_stream_timestamp_{0};
  double last_delay_ms_{0.0};
  double last_fps_{0.0};

  int frame_count_{0};
  double fps_sum_{0.0};
  double delay_sum_{0.0};
  double fps_max_{std::numeric_limits<double>::lowest()};
  double fps_min_{std::numeric_limits<double>::max()};
  double delay_max_{std::numeric_limits<double>::lowest()};
  double delay_min_{std::numeric_limits<double>::max()};

  LogLevel log_level_;
  rclcpp::Logger logger_;
};

}  // namespace orbbec_camera
