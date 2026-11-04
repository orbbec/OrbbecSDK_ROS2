#include <gtest/gtest.h>

#include "orbbec_camera/stream_status.hpp"

namespace orbbec_camera {
namespace {

TEST(StreamStatusTest, HidesUnavailableTopicsAndResetsTheirCounters) {
  bool available = false;
  StreamStatusTracker tracker(
      "/camera/depth/image_raw/compressed", [] { return 1U; }, [&available] { return available; });
  orbbec_camera_msgs::msg::StreamStatus status;
  builtin_interfaces::msg::Time stamp;

  tracker.record(stamp);
  EXPECT_FALSE(tracker.fill(status));

  available = true;
  EXPECT_TRUE(tracker.fill(status));
  EXPECT_TRUE(status.has_subscribers);
  EXPECT_EQ(status.publish_rate_hz, 0.0);

  tracker.record(stamp);
  EXPECT_TRUE(tracker.fill(status));
  EXPECT_GT(status.publish_rate_hz, 0.0);

  available = false;
  EXPECT_FALSE(tracker.fill(status));
}

TEST(StreamStatusTest, TopicsWithoutAvailabilityChecksRemainVisible) {
  StreamStatusTracker tracker("/camera/depth/image_raw", [] { return 0U; });
  orbbec_camera_msgs::msg::StreamStatus status;

  EXPECT_TRUE(tracker.fill(status));
  EXPECT_EQ(status.topic_name, "/camera/depth/image_raw");
  EXPECT_FALSE(status.has_subscribers);
}

}  // namespace
}  // namespace orbbec_camera
