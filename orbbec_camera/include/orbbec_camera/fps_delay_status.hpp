#pragma once

#include <rclcpp/rclcpp.hpp>
#include "orbbec_camera_msgs/msg/device_status.hpp"
#include <chrono>
#include <functional>
#include <mutex>

namespace orbbec_camera {

class FpsDelayStatus {
public:
  explicit FpsDelayStatus():last_frame_stamp_(std::chrono::steady_clock::now()){}
  void tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_frame_stamp_).count();
    double fps = (dt > 0) ? (1.0 / dt) : 0.0;
    double delay_ms = dt * 1000.0;

    last_fps_ = fps;
    last_delay_ms_ = delay_ms;
    last_frame_stamp_ = now;

    frame_count_++;
    fps_sum_ += fps;
    delay_sum_ += delay_ms;

    fps_max_ = std::max(fps_max_, fps);
    fps_min_ = std::min(fps_min_, fps);
    delay_max_ = std::max(delay_max_, delay_ms);
    delay_min_ = std::min(delay_min_, delay_ms);

  }

  void fillColorStatus(orbbec_camera_msgs::msg::DeviceStatus &msg){
    std::lock_guard<std::mutex> lock(mutex_);
    msg.color_frame_rate_cur = last_fps_;
    msg.color_frame_rate_avg = frame_count_ > 0 ? fps_sum_ / frame_count_ : 0.0;
    msg.color_frame_rate_min = fps_min_;
    msg.color_frame_rate_max = fps_max_;

    msg.color_delay_ms_cur = last_delay_ms_;
    msg.color_delay_ms_avg = frame_count_ > 0 ? delay_sum_ / frame_count_ : 0.0;
    msg.color_delay_ms_min = delay_min_;
    msg.color_delay_ms_max = delay_max_;

    frame_count_ = 0;
    fps_sum_ = delay_sum_ = 0.0;
    fps_max_ = delay_max_ = std::numeric_limits<double>::lowest();
    fps_min_ = delay_min_ = std::numeric_limits<double>::max();
  }
  void fillDepthStatus(orbbec_camera_msgs::msg::DeviceStatus &msg){
    std::lock_guard<std::mutex> lock(mutex_);
    msg.depth_frame_rate_cur = last_fps_;
    msg.depth_frame_rate_avg = frame_count_ > 0 ? fps_sum_ / frame_count_ : 0.0;
    msg.depth_frame_rate_min = fps_min_;
    msg.depth_frame_rate_max = fps_max_;

    msg.depth_delay_ms_cur = last_delay_ms_;
    msg.depth_delay_ms_avg = frame_count_ > 0 ? delay_sum_ / frame_count_ : 0.0;
    msg.depth_delay_ms_min = delay_min_;
    msg.depth_delay_ms_max = delay_max_;

    frame_count_ = 0;
    fps_sum_ = delay_sum_ = 0.0;
    fps_max_ = delay_max_ = std::numeric_limits<double>::lowest();
    fps_min_ = delay_min_ = std::numeric_limits<double>::max();
  }


private:
  mutable std::mutex mutex_;
  std::chrono::steady_clock::time_point last_frame_stamp_;
  double last_delay_ms_{0.0};
  double last_fps_{0.0};

  int frame_count_{0};
  double fps_sum_{0.0};
  double delay_sum_{0.0};
  double fps_max_{std::numeric_limits<double>::lowest()};
  double fps_min_{std::numeric_limits<double>::max()};
  double delay_max_{std::numeric_limits<double>::lowest()};
  double delay_min_{std::numeric_limits<double>::max()};

};

}  // namespace orbbec_camera
