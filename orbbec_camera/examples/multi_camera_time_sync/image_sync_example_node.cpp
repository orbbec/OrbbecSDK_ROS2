#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include <iomanip>

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
  ImageSyncNode(const double hz)
      : Node("image_sync_node"),
        diff_sum_(0.0),
        count_(0),
        max_diff_(0.0),
        min_diff_(std::numeric_limits<double>::max()),
        last_time_(0.0),
        frame_interval_(1.0 / hz),
        stop_display_thread_(false) {
    auto topics = this->get_topic_names_and_types();
    std::vector<std::string> color_topics;
    std::vector<std::string> depth_topics;

    auto has_suffix = [](const std::string &str, const std::string &suffix) {
      return str.size() >= suffix.size() &&
             str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
    };

    for (const auto &entry : topics) {
      const std::string &topic = entry.first;
      if (has_suffix(topic, "/color/image_raw")) {
        color_topics.push_back(topic);
      } else if (has_suffix(topic, "/depth/image_raw")) {
        depth_topics.push_back(topic);
      }
    }
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();

    camera_01_color_sub_.subscribe(this, "/camera_01/color/image_raw", qos.get_rmw_qos_profile());
    camera_01_depth_sub_.subscribe(this, "/camera_01/depth/image_raw", qos.get_rmw_qos_profile());
    camera_02_color_sub_.subscribe(this, "/camera_02/color/image_raw", qos.get_rmw_qos_profile());
    camera_02_depth_sub_.subscribe(this, "/camera_02/depth/image_raw", qos.get_rmw_qos_profile());
    camera_03_color_sub_.subscribe(this, "/camera_03/color/image_raw", qos.get_rmw_qos_profile());
    camera_03_depth_sub_.subscribe(this, "/camera_03/depth/image_raw", qos.get_rmw_qos_profile());
    camera_04_color_sub_.subscribe(this, "/camera_04/color/image_raw", qos.get_rmw_qos_profile());
    camera_04_depth_sub_.subscribe(this, "/camera_04/depth/image_raw", qos.get_rmw_qos_profile());

    sync_ =
        std::make_shared<Sync>(SyncPolicy(10), camera_01_color_sub_, camera_01_depth_sub_,
                               camera_02_color_sub_, camera_02_depth_sub_, camera_03_color_sub_,
                               camera_03_depth_sub_, camera_04_color_sub_, camera_04_depth_sub_);

    // sync_->setAgePenalty(0.5);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(frame_interval_ / 2));
    sync_->registerCallback(
        std::bind(&ImageSyncNode::sync_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

    RCLCPP_INFO(this->get_logger(), "image sync node started.");

    // start display thread
    display_thread_ = std::thread(&ImageSyncNode::display_thread_func, this);
  }

  ~ImageSyncNode() {
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
  double diff_sum_;
  size_t count_;
  double max_diff_;
  double min_diff_;
  double last_time_;  // FPS
  uint64_t frame_count_ = 0;
  uint64_t drop_count_ = 0;
  double frame_interval_;
  double fps_cur_ = 0.0;
  double fps_sum_ = 0.0;
  double fps_max_ = 0.0;
  double fps_min_ = std::numeric_limits<double>::max();

  std::thread display_thread_;
  struct FrameBundle {
    std::vector<cv::Mat> images;
    std::vector<double> timestamps;
    std::vector<std::string> camera_names;
    std::vector<std::string> image_types;
  };
  std::deque<FrameBundle> frame_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool stop_display_thread_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  message_filters::Subscriber<sensor_msgs::msg::Image> camera_01_color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera_01_depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera_02_color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera_02_depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera_03_color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera_03_depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera_04_color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera_04_depth_sub_;

  std::shared_ptr<Sync> sync_;

  void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img01_c,
                     const sensor_msgs::msg::Image::ConstSharedPtr &img01_d,
                     const sensor_msgs::msg::Image::ConstSharedPtr &img02_c,
                     const sensor_msgs::msg::Image::ConstSharedPtr &img02_d,
                     const sensor_msgs::msg::Image::ConstSharedPtr &img03_c,
                     const sensor_msgs::msg::Image::ConstSharedPtr &img03_d,
                     const sensor_msgs::msg::Image::ConstSharedPtr &img04_c,
                     const sensor_msgs::msg::Image::ConstSharedPtr &img04_d) {
    std::vector<cv::Mat> images;
    std::vector<double> timestamps;
    std::vector<std::string> camera_names = {"camera_01", "camera_01", "camera_02", "camera_02",
                                             "camera_03", "camera_03", "camera_04", "camera_04"};

    std::vector<std::string> image_types = {"color", "depth", "color", "depth",
                                            "color", "depth", "color", "depth"};
    try {
      images.push_back(cv_bridge::toCvShare(img01_c, "bgr8")->image.clone());
      images.push_back(cv_bridge::toCvShare(img01_d, "16UC1")->image.clone());
      images.push_back(cv_bridge::toCvShare(img02_c, "bgr8")->image.clone());
      images.push_back(cv_bridge::toCvShare(img02_d, "16UC1")->image.clone());
      images.push_back(cv_bridge::toCvShare(img03_c, "bgr8")->image.clone());
      images.push_back(cv_bridge::toCvShare(img03_d, "16UC1")->image.clone());
      images.push_back(cv_bridge::toCvShare(img04_c, "bgr8")->image.clone());
      images.push_back(cv_bridge::toCvShare(img04_d, "16UC1")->image.clone());

      timestamps = {img01_c->header.stamp.sec + img01_c->header.stamp.nanosec * 1e-9,
                    img01_d->header.stamp.sec + img01_d->header.stamp.nanosec * 1e-9,
                    img02_c->header.stamp.sec + img02_c->header.stamp.nanosec * 1e-9,
                    img02_d->header.stamp.sec + img02_d->header.stamp.nanosec * 1e-9,
                    img03_c->header.stamp.sec + img03_c->header.stamp.nanosec * 1e-9,
                    img03_d->header.stamp.sec + img03_d->header.stamp.nanosec * 1e-9,
                    img04_c->header.stamp.sec + img04_c->header.stamp.nanosec * 1e-9,
                    img04_d->header.stamp.sec + img04_d->header.stamp.nanosec * 1e-9};
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // push to display queue (avoid blocking sync callback)
    {
      std::lock_guard<std::mutex> lk(queue_mutex_);
      // keep queue bounded to avoid memory blowup, drop oldest if full
      const size_t max_queue = 3;
      if (frame_queue_.size() >= max_queue) {
        frame_queue_.pop_front();
      }
      frame_queue_.push_back(
          FrameBundle{std::move(images), std::move(timestamps), camera_names, image_types});
    }
    queue_cv_.notify_one();

    // retrieve copy of timestamps from back of queue
    {
      std::lock_guard<std::mutex> lk(queue_mutex_);
      if (!frame_queue_.empty()) {
        std::vector<double> ts_copy = frame_queue_.back().timestamps;
        print_stats(ts_copy);
      }
    }
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
        // take the latest frame (drop older if several)
        bundle = std::move(frame_queue_.back());
        frame_queue_.clear();
      }
      // call image_show in this thread
      image_show(bundle.images, bundle.timestamps, bundle.camera_names, bundle.image_types);
    }
  }

  void image_show(std::vector<cv::Mat> &images, std::vector<double> &timestamps,
                  std::vector<std::string> &camera_names, std::vector<std::string> &image_types) {
    for (size_t i = 0; i < images.size(); i++) {
      if (images[i].channels() == 1) {
        cv::Mat tmp;
        images[i].convertTo(tmp, CV_8U, 255.0 / 10000.0);
        cv::applyColorMap(tmp, images[i], cv::COLORMAP_JET);
      }
      // std::string ts_text = "ts: " + std::to_string(timestamps[i]);
      std::string ts_text =
          camera_names[i] + " " + image_types[i] + " stamp: " + std::to_string(timestamps[i]);
      cv::putText(images[i], ts_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                  cv::Scalar(0, 0, 255), 2);
    }

    int margin = 10;
    int row1_height = std::max({images[0].rows, images[1].rows, images[2].rows, images[3].rows});
    int row2_height = std::max({images[4].rows, images[5].rows, images[6].rows, images[7].rows});

    int row1_width = images[0].cols + margin + images[1].cols + margin + images[2].cols + margin +
                     images[3].cols;
    int row2_width = images[4].cols + margin + images[5].cols + margin + images[6].cols + margin +
                     images[7].cols;

    int canvas_width = std::max(row1_width, row2_width);
    int canvas_height = row1_height + margin + row2_height;

    cv::Mat canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(30, 30, 30));

    int x_offset = 0;
    int y_offset = 0;
    for (int i = 0; i < 4; ++i) {
      images[i].copyTo(canvas(cv::Rect(x_offset, y_offset, images[i].cols, images[i].rows)));
      x_offset += images[i].cols + margin;
    }

    x_offset = 0;
    y_offset = row1_height + margin;
    for (int i = 4; i < 8; ++i) {
      images[i].copyTo(canvas(cv::Rect(x_offset, y_offset, images[i].cols, images[i].rows)));
      x_offset += images[i].cols + margin;
    }

    int screen_width = 1800;
    int screen_height = 800;

    double scale_w = static_cast<double>(screen_width) / canvas.cols;
    double scale_h = static_cast<double>(screen_height) / canvas.rows;
    double scale = std::min(1.0, std::min(scale_w, scale_h));

    cv::Mat display;
    if (scale < 1.0) {
      cv::resize(canvas, display, cv::Size(), scale, scale);
    } else {
      display = canvas;
    }
    cv::imshow("Time Synced Cameras", display);
    cv::waitKey(1);
  }

  void print_stats(std::vector<double> &timestamps) {
    std::cout << std::fixed;
    std::cout << "===========================================================" << std::endl;
    for (int i = 0; i < 4; i++) {
      std::cout << "Camera0" << i + 1 << " stamp: color= " << std::setprecision(6)
                << timestamps[i * 2] << "  depth= " << std::setprecision(6) << timestamps[i * 2 + 1]
                << "  Delay relative to camera01(color): color= " << std::setprecision(3)
                << (timestamps[i * 2] - timestamps[0]) * 1000.0 << " ms"
                << "  depth= " << std::setprecision(3)
                << (timestamps[i * 2 + 1] - timestamps[0]) * 1000.0 << " ms" << std::endl;
    }

    double cur = 0.0;
    double base_t = timestamps[0];  // Camera01 color
    for (size_t i = 0; i < timestamps.size(); ++i) {
      double diff = std::fabs(timestamps[i] - base_t) * 1000.0;
      cur = std::max(cur, diff);
    }
    diff_sum_ += cur;
    count_++;
    max_diff_ = std::max(max_diff_, cur);
    min_diff_ = std::min(min_diff_, cur);
    double avg_diff = diff_sum_ / count_;
    std::cout << "\nImage Timestamp Difference Statistics" << std::endl;
    std::cout << "cur: " << cur << " ms"
              << " avg: " << avg_diff << " ms"
              << " max: " << max_diff_ << " ms"
              << " min: " << min_diff_ << " ms" << std::endl;

    // Calculate and display FPS
    if (last_time_ == 0.0) {
      last_time_ = base_t;
    } else {
      double dt = base_t - last_time_;
      // if (dt > frame_interval_ * 1.5) {
      //   std::cout << "base_t: " << base_t << " last_time_: " << last_time_ << std::endl;
      //   std::cout << "Frame drop detected! dt: " << dt << " s" << std::endl;
      //   drop_count_ +=
      //       static_cast<uint64_t>(dt / 0.0333) - 1;  // Assuming 30 FPS, so frame interval
      //       ~33.3ms
      //   std::cout << "Total dropped frames: " << drop_count_ << std::endl;
      // }
      fps_cur_ = dt > 0.0 ? 1.0 / dt : fps_cur_;
      last_time_ = base_t;
      frame_count_++;
      fps_sum_ += fps_cur_;
      fps_max_ = std::max(fps_max_, fps_cur_);
      fps_min_ = std::min(fps_min_, fps_cur_);
      double fps_avg = fps_sum_ / frame_count_;
      std::cout << "\nFPS Statistics" << std::endl;
      std::cout << "cur: " << std::setprecision(2) << fps_cur_ << " avg: " << std::setprecision(2)
                << fps_avg << " max: " << std::setprecision(2) << fps_max_
                << " min: " << std::setprecision(2) << fps_min_ << std::endl;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<ImageSyncNode>(30.0));
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ImageSyncNode>(30.0);  // Modify according to your actual frame rate
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}