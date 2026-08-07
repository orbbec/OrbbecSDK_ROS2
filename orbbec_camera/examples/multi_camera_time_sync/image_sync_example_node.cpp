#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#if __has_include(<message_filters/subscriber.hpp>)
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#elif __has_include(<message_filters/subscriber.h>)
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#else
#error "No compatible message_filters headers found"
#endif

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

using Image = sensor_msgs::msg::Image;
using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using std::placeholders::_6;
using std::placeholders::_7;
using std::placeholders::_8;

class ImageSyncNode : public rclcpp::Node {
 public:
  explicit ImageSyncNode(const double hz)
      : Node("image_sync_node"),
        diff_sum_(0.0),
        count_(0),
        max_diff_(0.0),
        min_diff_(std::numeric_limits<double>::max()),
        last_time_(0.0),
        frame_interval_(1.0 / hz),
        stop_display_thread_(false) {
    sync_topics_ = this->declare_parameter<std::vector<std::string>>("sync_topics",
                                                                     std::vector<std::string>{});
    queue_size_ = this->declare_parameter<int>("queue_size", 10);
    sync_tolerance_ms_ =
        this->declare_parameter<double>("sync_tolerance_ms", frame_interval_ * 500.0);
    auto_discovery_timeout_ms_ = this->declare_parameter<int>("auto_discovery_timeout_ms", 1000);
    auto_discovery_poll_ms_ = this->declare_parameter<int>("auto_discovery_poll_ms", 100);

    if (sync_topics_.empty()) {
      sync_topics_ = discover_image_topics();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter sync_topics is empty. Auto-discovered %zu color/depth image topics.",
                  sync_topics_.size());
    } else {
      RCLCPP_INFO(this->get_logger(), "Using %zu image topics from parameter sync_topics.",
                  sync_topics_.size());
    }

    validate_topics();
    topic_infos_ = make_topic_infos(sync_topics_);

    RCLCPP_INFO(this->get_logger(),
                "image sync node started with %zu topic(s):", sync_topics_.size());
    for (const auto &topic : sync_topics_) {
      RCLCPP_INFO(this->get_logger(), "  %s", topic.c_str());
    }

    rclcpp::QoS qos{rclcpp::KeepLast(queue_size_)};
    qos.reliable();

#ifdef ORBBEC_MESSAGE_FILTERS_USES_RCLCPP_QOS
    const rclcpp::QoS qos_prof = qos;
#else
    const rmw_qos_profile_t qos_prof = qos.get_rmw_qos_profile();
#endif

    if (sync_topics_.size() == 1) {
      single_sub_ = this->create_subscription<Image>(
          sync_topics_.front(), qos,
          [this](const ImageConstPtr msg) { this->handle_synced_images({msg}); });
    } else {
      for (size_t i = 0; i < sync_topics_.size(); ++i) {
        subscribers_[i].subscribe(this, sync_topics_[i], qos_prof);
      }
      create_synchronizer();
    }

    display_thread_ = std::thread(&ImageSyncNode::display_thread_func, this);
  }

  ~ImageSyncNode() override {
    {
      std::lock_guard<std::mutex> lk(queue_mutex_);
      stop_display_thread_ = true;
    }
    queue_cv_.notify_all();
    if (display_thread_.joinable()) {
      display_thread_.join();
    }
  }

 private:
  struct TopicInfo {
    std::string topic;
    std::string camera_name;
    std::string image_type;
  };

  struct FrameBundle {
    std::vector<cv::Mat> images;
    std::vector<double> timestamps;
    std::vector<TopicInfo> topic_infos;
  };

  double diff_sum_;
  size_t count_;
  double max_diff_;
  double min_diff_;
  double last_time_;
  uint64_t frame_count_ = 0;
  double frame_interval_;
  double fps_cur_ = 0.0;
  double fps_sum_ = 0.0;
  double fps_max_ = 0.0;
  double fps_min_ = std::numeric_limits<double>::max();

  std::vector<std::string> sync_topics_;
  std::vector<TopicInfo> topic_infos_;
  int queue_size_;
  double sync_tolerance_ms_;
  int auto_discovery_timeout_ms_;
  int auto_discovery_poll_ms_;

  rclcpp::Subscription<Image>::SharedPtr single_sub_;
  std::array<message_filters::Subscriber<Image>, 8> subscribers_;
  std::shared_ptr<void> sync_;

  std::thread display_thread_;
  std::deque<FrameBundle> frame_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool stop_display_thread_;

  static bool has_suffix(const std::string &str, const std::string &suffix) {
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  static double stamp_to_seconds(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  std::vector<std::string> discover_image_topics() {
    std::vector<std::string> topics;
    const auto timeout = std::chrono::milliseconds(std::max(0, auto_discovery_timeout_ms_));
    const auto poll_interval = std::chrono::milliseconds(std::max(1, auto_discovery_poll_ms_));
    const auto start = std::chrono::steady_clock::now();

    while (true) {
      const auto names_and_types = this->get_topic_names_and_types();
      for (const auto &entry : names_and_types) {
        const auto &topic = entry.first;
        if (!has_suffix(topic, "/color/image_raw") && !has_suffix(topic, "/depth/image_raw")) {
          continue;
        }

        const auto &types = entry.second;
        const bool is_image =
            std::find(types.begin(), types.end(), "sensor_msgs/msg/Image") != types.end();
        const bool already_found = std::find(topics.begin(), topics.end(), topic) != topics.end();
        if (is_image && !already_found) {
          topics.push_back(topic);
        }
      }

      if (std::chrono::steady_clock::now() - start >= timeout) {
        break;
      }
      rclcpp::sleep_for(poll_interval);
    }

    std::sort(topics.begin(), topics.end());
    topics.erase(std::unique(topics.begin(), topics.end()), topics.end());
    return topics;
  }

  void validate_topics() {
    if (sync_topics_.empty()) {
      throw std::runtime_error(
          "No image topics to synchronize. Set parameter sync_topics or start color/depth cameras "
          "before this node.");
    }

    std::vector<std::string> deduplicated_topics;
    deduplicated_topics.reserve(sync_topics_.size());
    for (const auto &topic : sync_topics_) {
      if (topic.empty()) {
        continue;
      }
      if (std::find(deduplicated_topics.begin(), deduplicated_topics.end(), topic) ==
          deduplicated_topics.end()) {
        deduplicated_topics.push_back(topic);
      }
    }
    sync_topics_ = std::move(deduplicated_topics);

    if (sync_topics_.empty()) {
      throw std::runtime_error("Parameter sync_topics only contains empty entries.");
    }

    if (sync_topics_.size() > 8) {
      throw std::runtime_error(
          "Official ROS message_filters::Synchronizer supports at most 9 inputs, and this example "
          "supports 1-8 image topics. Found " +
          std::to_string(sync_topics_.size()) +
          " color/depth image topics. Please pass <= 8 topics with sync_topics or split the sync "
          "into multiple stages.");
    }
  }

  std::vector<TopicInfo> make_topic_infos(const std::vector<std::string> &topics) {
    std::vector<TopicInfo> infos;
    infos.reserve(topics.size());
    for (const auto &topic : topics) {
      TopicInfo info;
      info.topic = topic;
      info.image_type = "image";
      info.camera_name = topic;

      const auto color_pos = topic.rfind("/color/image_raw");
      const auto depth_pos = topic.rfind("/depth/image_raw");
      if (color_pos != std::string::npos) {
        info.image_type = "color";
        info.camera_name = topic.substr(0, color_pos);
      } else if (depth_pos != std::string::npos) {
        info.image_type = "depth";
        info.camera_name = topic.substr(0, depth_pos);
      }

      const auto slash_pos = info.camera_name.find_last_of('/');
      if (slash_pos != std::string::npos && slash_pos + 1 < info.camera_name.size()) {
        info.camera_name = info.camera_name.substr(slash_pos + 1);
      }
      infos.push_back(info);
    }
    return infos;
  }

  template <typename... MsgPtrs>
  void sync_callback(const MsgPtrs &...msgs) {
    handle_synced_images({msgs...});
  }

  void handle_synced_images(const std::vector<ImageConstPtr> &msgs) {
    std::vector<cv::Mat> images;
    std::vector<double> timestamps;
    images.reserve(msgs.size());
    timestamps.reserve(msgs.size());

    try {
      for (const auto &msg : msgs) {
        auto cv_image = cv_bridge::toCvShare(msg);
        images.push_back(cv_image->image.clone());
        timestamps.push_back(stamp_to_seconds(msg->header.stamp));
      }
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    {
      std::lock_guard<std::mutex> lk(queue_mutex_);
      const size_t max_queue = 3;
      if (frame_queue_.size() >= max_queue) {
        frame_queue_.pop_front();
      }
      frame_queue_.push_back(FrameBundle{std::move(images), timestamps, topic_infos_});
    }
    queue_cv_.notify_one();

    print_stats(timestamps);
  }

  void create_synchronizer() {
    switch (sync_topics_.size()) {
      case 2:
        create_synchronizer_2();
        break;
      case 3:
        create_synchronizer_3();
        break;
      case 4:
        create_synchronizer_4();
        break;
      case 5:
        create_synchronizer_5();
        break;
      case 6:
        create_synchronizer_6();
        break;
      case 7:
        create_synchronizer_7();
        break;
      case 8:
        create_synchronizer_8();
        break;
      default:
        throw std::runtime_error("Unsupported sync topic count: " +
                                 std::to_string(sync_topics_.size()));
    }
  }

  template <typename SyncT>
  void configure_synchronizer(const std::shared_ptr<SyncT> &sync) {
    sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_ms_ / 1000.0));
    sync_ = sync;
  }

  void create_synchronizer_2() {
    using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
    using Sync = message_filters::Synchronizer<Policy>;
    auto sync = std::make_shared<Sync>(Policy(queue_size_), subscribers_[0], subscribers_[1]);
    sync->registerCallback(
        std::bind(&ImageSyncNode::sync_callback<ImageConstPtr, ImageConstPtr>, this, _1, _2));
    configure_synchronizer(sync);
  }

  void create_synchronizer_3() {
    using Policy = message_filters::sync_policies::ApproximateTime<Image, Image, Image>;
    using Sync = message_filters::Synchronizer<Policy>;
    auto sync = std::make_shared<Sync>(Policy(queue_size_), subscribers_[0], subscribers_[1],
                                       subscribers_[2]);
    sync->registerCallback(
        std::bind(&ImageSyncNode::sync_callback<ImageConstPtr, ImageConstPtr, ImageConstPtr>, this,
                  _1, _2, _3));
    configure_synchronizer(sync);
  }

  void create_synchronizer_4() {
    using Policy = message_filters::sync_policies::ApproximateTime<Image, Image, Image, Image>;
    using Sync = message_filters::Synchronizer<Policy>;
    auto sync = std::make_shared<Sync>(Policy(queue_size_), subscribers_[0], subscribers_[1],
                                       subscribers_[2], subscribers_[3]);
    sync->registerCallback(std::bind(
        &ImageSyncNode::sync_callback<ImageConstPtr, ImageConstPtr, ImageConstPtr, ImageConstPtr>,
        this, _1, _2, _3, _4));
    configure_synchronizer(sync);
  }

  void create_synchronizer_5() {
    using Policy =
        message_filters::sync_policies::ApproximateTime<Image, Image, Image, Image, Image>;
    using Sync = message_filters::Synchronizer<Policy>;
    auto sync = std::make_shared<Sync>(Policy(queue_size_), subscribers_[0], subscribers_[1],
                                       subscribers_[2], subscribers_[3], subscribers_[4]);
    sync->registerCallback(
        std::bind(&ImageSyncNode::sync_callback<ImageConstPtr, ImageConstPtr, ImageConstPtr,
                                                ImageConstPtr, ImageConstPtr>,
                  this, _1, _2, _3, _4, _5));
    configure_synchronizer(sync);
  }

  void create_synchronizer_6() {
    using Policy =
        message_filters::sync_policies::ApproximateTime<Image, Image, Image, Image, Image, Image>;
    using Sync = message_filters::Synchronizer<Policy>;
    auto sync =
        std::make_shared<Sync>(Policy(queue_size_), subscribers_[0], subscribers_[1],
                               subscribers_[2], subscribers_[3], subscribers_[4], subscribers_[5]);
    sync->registerCallback(
        std::bind(&ImageSyncNode::sync_callback<ImageConstPtr, ImageConstPtr, ImageConstPtr,
                                                ImageConstPtr, ImageConstPtr, ImageConstPtr>,
                  this, _1, _2, _3, _4, _5, _6));
    configure_synchronizer(sync);
  }

  void create_synchronizer_7() {
    using Policy = message_filters::sync_policies::ApproximateTime<Image, Image, Image, Image,
                                                                   Image, Image, Image>;
    using Sync = message_filters::Synchronizer<Policy>;
    auto sync = std::make_shared<Sync>(Policy(queue_size_), subscribers_[0], subscribers_[1],
                                       subscribers_[2], subscribers_[3], subscribers_[4],
                                       subscribers_[5], subscribers_[6]);
    sync->registerCallback(std::bind(
        &ImageSyncNode::sync_callback<ImageConstPtr, ImageConstPtr, ImageConstPtr, ImageConstPtr,
                                      ImageConstPtr, ImageConstPtr, ImageConstPtr>,
        this, _1, _2, _3, _4, _5, _6, _7));
    configure_synchronizer(sync);
  }

  void create_synchronizer_8() {
    using Policy = message_filters::sync_policies::ApproximateTime<Image, Image, Image, Image,
                                                                   Image, Image, Image, Image>;
    using Sync = message_filters::Synchronizer<Policy>;
    auto sync = std::make_shared<Sync>(Policy(queue_size_), subscribers_[0], subscribers_[1],
                                       subscribers_[2], subscribers_[3], subscribers_[4],
                                       subscribers_[5], subscribers_[6], subscribers_[7]);
    sync->registerCallback(std::bind(
        &ImageSyncNode::sync_callback<ImageConstPtr, ImageConstPtr, ImageConstPtr, ImageConstPtr,
                                      ImageConstPtr, ImageConstPtr, ImageConstPtr, ImageConstPtr>,
        this, _1, _2, _3, _4, _5, _6, _7, _8));
    configure_synchronizer(sync);
  }

  void display_thread_func() {
    while (true) {
      FrameBundle bundle;
      {
        std::unique_lock<std::mutex> lk(queue_mutex_);
        queue_cv_.wait(lk, [this] { return stop_display_thread_ || !frame_queue_.empty(); });
        if (stop_display_thread_ && frame_queue_.empty()) {
          return;
        }
        bundle = std::move(frame_queue_.back());
        frame_queue_.clear();
      }
      image_show(bundle.images, bundle.timestamps, bundle.topic_infos);
    }
  }

  void image_show(std::vector<cv::Mat> &images, const std::vector<double> &timestamps,
                  const std::vector<TopicInfo> &topic_infos) {
    if (images.empty()) {
      return;
    }

    constexpr int margin = 10;
    constexpr int max_columns = 4;
    const int columns = std::min<int>(max_columns, static_cast<int>(images.size()));
    const int rows = static_cast<int>((images.size() + columns - 1) / columns);

    std::vector<cv::Mat> display_images;
    display_images.reserve(images.size());
    for (size_t i = 0; i < images.size(); i++) {
      cv::Mat image;
      if (images[i].channels() == 1) {
        cv::Mat tmp;
        images[i].convertTo(tmp, CV_8U, 255.0 / 10000.0);
        cv::applyColorMap(tmp, image, cv::COLORMAP_JET);
      } else if (images[i].channels() == 3) {
        image = images[i].clone();
      } else {
        cv::cvtColor(images[i], image, cv::COLOR_GRAY2BGR);
      }

      const std::string text = topic_infos[i].camera_name + " " + topic_infos[i].image_type +
                               " stamp: " + std::to_string(timestamps[i]);
      cv::putText(image, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(0, 0, 255), 2);
      display_images.push_back(std::move(image));
    }

    std::vector<int> column_widths(columns, 0);
    std::vector<int> row_heights(rows, 0);
    for (size_t i = 0; i < display_images.size(); ++i) {
      const int row = static_cast<int>(i) / columns;
      const int col = static_cast<int>(i) % columns;
      column_widths[col] = std::max(column_widths[col], display_images[i].cols);
      row_heights[row] = std::max(row_heights[row], display_images[i].rows);
    }

    int canvas_width = margin * (columns - 1);
    for (const auto width : column_widths) {
      canvas_width += width;
    }

    int canvas_height = margin * (rows - 1);
    for (const auto height : row_heights) {
      canvas_height += height;
    }

    cv::Mat canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(30, 30, 30));

    int y_offset = 0;
    for (int row = 0; row < rows; ++row) {
      int x_offset = 0;
      for (int col = 0; col < columns; ++col) {
        const size_t i = static_cast<size_t>(row * columns + col);
        if (i >= display_images.size()) {
          break;
        }
        display_images[i].copyTo(
            canvas(cv::Rect(x_offset, y_offset, display_images[i].cols, display_images[i].rows)));
        x_offset += column_widths[col] + margin;
      }
      y_offset += row_heights[row] + margin;
    }

    constexpr int screen_width = 1800;
    constexpr int screen_height = 800;
    const double scale_w = static_cast<double>(screen_width) / canvas.cols;
    const double scale_h = static_cast<double>(screen_height) / canvas.rows;
    const double scale = std::min(1.0, std::min(scale_w, scale_h));

    cv::Mat display;
    if (scale < 1.0) {
      cv::resize(canvas, display, cv::Size(), scale, scale);
    } else {
      display = canvas;
    }
    cv::imshow("Time Synced Cameras", display);
    cv::waitKey(1);
  }

  void print_stats(const std::vector<double> &timestamps) {
    if (timestamps.empty()) {
      return;
    }

    std::cout << std::fixed;
    std::cout << "===========================================================" << std::endl;

    const double base_t = timestamps[0];
    for (size_t i = 0; i < timestamps.size(); ++i) {
      std::cout << topic_infos_[i].camera_name << " " << topic_infos_[i].image_type
                << " stamp: " << std::setprecision(6) << timestamps[i] << "  Delay relative to "
                << topic_infos_[0].camera_name << " " << topic_infos_[0].image_type << ": "
                << std::setprecision(3) << (timestamps[i] - base_t) * 1000.0 << " ms" << std::endl;
    }

    double cur = 0.0;
    for (const auto timestamp : timestamps) {
      const double diff = std::fabs(timestamp - base_t) * 1000.0;
      cur = std::max(cur, diff);
    }
    diff_sum_ += cur;
    count_++;
    max_diff_ = std::max(max_diff_, cur);
    min_diff_ = std::min(min_diff_, cur);
    const double avg_diff = diff_sum_ / count_;

    std::cout << "\nImage Timestamp Difference Statistics" << std::endl;
    std::cout << "cur: " << cur << " ms"
              << " avg: " << avg_diff << " ms"
              << " max: " << max_diff_ << " ms"
              << " min: " << min_diff_ << " ms" << std::endl;

    if (last_time_ == 0.0) {
      last_time_ = base_t;
    } else {
      const double dt = base_t - last_time_;
      fps_cur_ = dt > 0.0 ? 1.0 / dt : fps_cur_;
      last_time_ = base_t;
      frame_count_++;
      fps_sum_ += fps_cur_;
      fps_max_ = std::max(fps_max_, fps_cur_);
      fps_min_ = std::min(fps_min_, fps_cur_);
      const double fps_avg = fps_sum_ / frame_count_;
      std::cout << "\nFPS Statistics" << std::endl;
      std::cout << "cur: " << std::setprecision(2) << fps_cur_ << " avg: " << std::setprecision(2)
                << fps_avg << " max: " << std::setprecision(2) << fps_max_
                << " min: " << std::setprecision(2) << fps_min_ << std::endl;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ImageSyncNode>(30.0);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
