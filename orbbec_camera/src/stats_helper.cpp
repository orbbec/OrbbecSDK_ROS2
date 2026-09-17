#include "orbbec_camera/stats_helper.h"

StatsCollector::StatsCollector()
    :
      time_last_message_received_(0),
      time_last_stats_published_(0),
      min_(std::numeric_limits<float>::max()),
      max_(std::numeric_limits<float>::min()),
      mean_(0),
      std_dev_(0),
      m2_(0),
      count_(0) {}

PeriodStatsCollector::PeriodStatsCollector()
    : StatsCollector() {}

StatsCollector::~StatsCollector() = default;

StatsMsg StatsCollector::get_stats() {
    StatsMsg msg;
    msg.min = min_;
    msg.max = max_;
    msg.mean = mean_;
    msg.std_dev = std_dev_;
    reset_stats();
    return msg;
}

void StatsCollector::update_stats(float value) {
    if (value < min_) {
        min_ = value;
    }
    if (value > max_) {
        max_ = value;
    }
    count_++;

    // Calculate mean and standard deviation
    // using Welford's algorithm
    // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    float delta = value - mean_;
    mean_ = mean_ + (delta / count_);
    float delta2 = value - mean_;
    m2_ = m2_ + delta * delta2;
    float variance_ = m2_ / count_;
    std_dev_ = sqrt(variance_);
}

void StatsCollector::reset_stats() {
    min_ = std::numeric_limits<float>::max();
    max_ = std::numeric_limits<float>::min();
    mean_ = 0;
    std_dev_ = 0;
    m2_ = 0;
    count_ = 0;
}

void PeriodStatsCollector::update_period(const rcl_time_point_value_t now_nanoseconds) {
    if (time_last_message_received_ == 0) {
        time_last_message_received_ = now_nanoseconds;
    } else {
        const std::chrono::nanoseconds nanos{now_nanoseconds - time_last_message_received_};
        const auto period = std::chrono::duration<double, std::milli>(nanos);
        time_last_message_received_ = now_nanoseconds;
        StatsCollector::update_stats(period.count());
    }
}

FrameStats::FrameStats() = default;
FrameStats::~FrameStats() = default;

void FrameStats::update(const rcl_time_point_value_t now_nanoseconds, float latency) {
    period_.update_period(now_nanoseconds);
    latency_.update_stats(latency);
    count_++;
}

Report FrameStats::get_report() {
    auto old_count_ = count_;
    count_ = 0;
    return Report{period_.get_stats(), latency_.get_stats(), old_count_};
}

void FrameStats::reset() {
    period_.reset_stats();
    latency_.reset_stats();
    count_ = 0;
} 
