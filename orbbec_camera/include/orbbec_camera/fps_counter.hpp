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

#include <chrono>
#include <functional>
#include <string>
namespace orbbec_camera {

enum class LogLevel { DEBUG, INFO };
class FpsCounter {
 public:
  explicit FpsCounter(const std::string &name, rclcpp::Logger logger, int print_interval_sec = 1)
      : name_(name),
        print_interval_(std::chrono::seconds(print_interval_sec)),
        last_print_time_(std::chrono::steady_clock::now()),
        frame_count_(0),
        log_level_(LogLevel::INFO),
        logger_(logger)

  {}

  void setLogLevel(LogLevel level) { log_level_ = level; }

  void tick() {
    ++frame_count_;
    auto now = std::chrono::steady_clock::now();
    if (now - last_print_time_ >= print_interval_) {
      double fps =
          static_cast<double>(frame_count_) /
          std::chrono::duration_cast<std::chrono::duration<double>>(now - last_print_time_).count();

      if (log_level_ == LogLevel::INFO) {
        RCLCPP_INFO_STREAM(logger_, name_ << " FPS " << fps);
      } else {
        RCLCPP_DEBUG_STREAM(logger_, name_ << " FPS " << fps);
      }

      last_print_time_ = now;
      frame_count_ = 0;
    }
  }

 private:
  std::string name_;
  std::chrono::seconds print_interval_;
  std::chrono::steady_clock::time_point last_print_time_;
  uint32_t frame_count_;
  LogLevel log_level_;
  rclcpp::Logger logger_;
};

}  // namespace orbbec_camera
