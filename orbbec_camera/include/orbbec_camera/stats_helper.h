#pragma once

#include <limits>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "orbbec_camera_msgs/msg/stats_msg.hpp"

using StatsMsg = orbbec_camera_msgs::msg::StatsMsg;

class StatsCollector {
 public:
  StatsCollector();
  virtual ~StatsCollector();

  StatsMsg get_stats();
  void update_stats(float value);
  void reset_stats();

 protected:
  rcl_time_point_value_t time_last_message_received_;
  rcl_time_point_value_t time_last_stats_published_;

 private:
  float min_;
  float max_;
  float mean_;
  float std_dev_;
  float m2_;
  int32_t count_;
};

class PeriodStatsCollector : public StatsCollector {
 public:
    PeriodStatsCollector();
    ~PeriodStatsCollector() = default;
    void update_period(const rcl_time_point_value_t now_nanoseconds);
 private:
    void update_stats(float value) = delete;
    //shadows StatsCollector::update_stats
};


struct Report {
    StatsMsg period;
    StatsMsg latency;
    int32_t count;
};

class FrameStats {
 public:
    FrameStats();
    ~FrameStats();

    void update(const rcl_time_point_value_t now_nanoseconds, float latency);
    Report get_report();
    void reset();

 private:
    PeriodStatsCollector period_;
    StatsCollector latency_;
    int32_t count_;
};

