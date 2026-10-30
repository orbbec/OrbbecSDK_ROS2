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

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <utility>

#include "orbbec_camera_msgs/msg/stream_status.hpp"

namespace orbbec_camera {

class StreamStatusTracker {
 public:
  using SubscriberCountFn = std::function<size_t()>;
  using AvailableFn = std::function<bool()>;

  StreamStatusTracker(std::string topic_name, SubscriberCountFn subscriber_count,
                      AvailableFn available = {})
      : topic_name_(std::move(topic_name)),
        subscriber_count_(std::move(subscriber_count)),
        available_(std::move(available)),
        window_start_(std::chrono::steady_clock::now()) {}

  void record(const builtin_interfaces::msg::Time& stamp) {
    const auto now = std::chrono::system_clock::now();
    const double now_ms = std::chrono::duration<double, std::milli>(now.time_since_epoch()).count();
    const double stamp_ms =
        static_cast<double>(stamp.sec) * 1000.0 + static_cast<double>(stamp.nanosec) / 1000000.0;

    std::lock_guard<std::mutex> lock(mutex_);
    published_count_++;
    delay_sum_ms_ += now_ms - stamp_ms;
  }

  bool fill(orbbec_camera_msgs::msg::StreamStatus& status) {
    const bool available = !available_ || available_();
    const bool has_subscribers = subscriber_count_ && subscriber_count_() > 0;
    const auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(mutex_);
    const double window_seconds = std::chrono::duration<double>(now - window_start_).count();
    status.topic_name = topic_name_;
    status.has_subscribers = has_subscribers;
    status.publish_rate_hz =
        window_seconds > 0.0 ? static_cast<double>(published_count_) / window_seconds : 0.0;
    status.delay_ms_avg =
        published_count_ > 0 ? delay_sum_ms_ / static_cast<double>(published_count_) : 0.0;

    published_count_ = 0;
    delay_sum_ms_ = 0.0;
    window_start_ = now;
    return available;
  }

 private:
  std::string topic_name_;
  SubscriberCountFn subscriber_count_;
  AvailableFn available_;
  std::chrono::steady_clock::time_point window_start_;
  std::mutex mutex_;
  uint32_t published_count_{0};
  double delay_sum_ms_{0.0};
};

}  // namespace orbbec_camera
