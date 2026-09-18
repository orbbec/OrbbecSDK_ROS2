#include <gtest/gtest.h>

#include "orbbec_camera/stats_helper.h"

namespace {
constexpr float kEpsilon = 1e-3f;
}

TEST(StatsCollectorTest, ComputesMinMaxMeanAndStdDev) {
  StatsCollector collector;
  collector.update_stats(1.0f);
  collector.update_stats(2.0f);
  collector.update_stats(3.0f);

  const auto stats = collector.get_stats();
  EXPECT_FLOAT_EQ(stats.min, 1.0f);
  EXPECT_FLOAT_EQ(stats.max, 3.0f);
  EXPECT_NEAR(stats.mean, 2.0f, kEpsilon);
  EXPECT_NEAR(stats.std_dev, std::sqrt(2.0f / 3.0f), kEpsilon);
}

TEST(StatsCollectorTest, GetStatsResetsAccumulator) {
  StatsCollector collector;
  collector.update_stats(1.0f);
  collector.update_stats(3.0f);
  collector.get_stats();  // First read should reset internal accumulators.

  collector.update_stats(100.0f);
  const auto stats = collector.get_stats();
  EXPECT_FLOAT_EQ(stats.min, 100.0f);
  EXPECT_FLOAT_EQ(stats.max, 100.0f);
  EXPECT_FLOAT_EQ(stats.mean, 100.0f);
  EXPECT_FLOAT_EQ(stats.std_dev, 0.0f);
}

TEST(PeriodStatsCollectorTest, FirstSampleOnlyEstablishesBaseline) {
  PeriodStatsCollector collector;
  // The first call has no prior timestamp to diff against, so it must not
  // record a period sample.
  collector.update_period(1'000'000'000);
  const auto stats = collector.get_stats();
  EXPECT_EQ(stats.mean, 0.0f);
  EXPECT_EQ(stats.std_dev, 0.0f);
}

TEST(PeriodStatsCollectorTest, ComputesPeriodBetweenSamplesInMilliseconds) {
  PeriodStatsCollector collector;
  collector.update_period(1'000'000'000);  // baseline, t = 0ms
  collector.update_period(1'010'000'000);  // +10ms
  collector.update_period(1'025'000'000);  // +15ms

  const auto stats = collector.get_stats();
  EXPECT_NEAR(stats.min, 10.0f, kEpsilon);
  EXPECT_NEAR(stats.max, 15.0f, kEpsilon);
  EXPECT_NEAR(stats.mean, 12.5f, kEpsilon);
  EXPECT_NEAR(stats.std_dev, 2.5f, kEpsilon);
}

TEST(FrameStatsTest, ReportAggregatesPeriodLatencyAndCount) {
  FrameStats frame_stats;
  frame_stats.update(1'000'000'000, 5.0f);
  frame_stats.update(1'010'000'000, 6.0f);  // period sample: 10ms
  frame_stats.update(1'025'000'000, 7.0f);  // period sample: 15ms

  const Report report = frame_stats.get_report();
  EXPECT_EQ(report.count, 3);

  EXPECT_NEAR(report.period.mean, 12.5f, kEpsilon);
  EXPECT_NEAR(report.period.min, 10.0f, kEpsilon);
  EXPECT_NEAR(report.period.max, 15.0f, kEpsilon);

  EXPECT_NEAR(report.latency.mean, 6.0f, kEpsilon);
  EXPECT_NEAR(report.latency.min, 5.0f, kEpsilon);
  EXPECT_NEAR(report.latency.max, 7.0f, kEpsilon);
}

TEST(FrameStatsTest, GetReportResetsCountButNotBetweenSeparateReports) {
  FrameStats frame_stats;
  frame_stats.update(1'000'000'000, 1.0f);
  frame_stats.update(1'010'000'000, 2.0f);

  EXPECT_EQ(frame_stats.get_report().count, 2);
  // No updates happened since the previous report, so the count starts over.
  EXPECT_EQ(frame_stats.get_report().count, 0);
}

TEST(FrameStatsTest, ResetClearsCount) {
  FrameStats frame_stats;
  frame_stats.update(1'000'000'000, 1.0f);
  frame_stats.update(1'010'000'000, 2.0f);

  frame_stats.reset();
  EXPECT_EQ(frame_stats.get_report().count, 0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
