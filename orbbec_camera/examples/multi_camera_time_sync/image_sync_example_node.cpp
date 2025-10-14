#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
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
  ImageSyncNode()
      : Node("image_sync_node"),
        diff_sum_(0.0),
        count_(0),
        max_diff_(0.0),
        min_diff_(std::numeric_limits<double>::max()) {
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
    sync_->registerCallback(
        std::bind(&ImageSyncNode::sync_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

    RCLCPP_INFO(this->get_logger(), "image sync node started.");
  }

 private:
  double diff_sum_;
  size_t count_;
  double max_diff_;
  double min_diff_;

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

    std::cout << std::fixed;
    std::cout << "===========================================================" << std::endl;

    // Camera01
    std::cout << "Camera01 stamp: color= " << std::setprecision(6) << timestamps[0]
              << "  depth= " << std::setprecision(6) << timestamps[1]
              << "  Delay relative to camera01(color): color= " << std::setprecision(3)
              << (timestamps[0] - timestamps[0]) * 1000.0 << " ms"
              << "  depth= " << std::setprecision(3) << (timestamps[1] - timestamps[0]) * 1000.0
              << " ms" << std::endl;

    // Camera02
    std::cout << "Camera02 stamp: color= " << std::setprecision(6) << timestamps[2]
              << "  depth= " << std::setprecision(6) << timestamps[3]
              << "  Delay relative to camera01(color): color= " << std::setprecision(3)
              << (timestamps[2] - timestamps[0]) * 1000.0 << " ms"
              << "  depth= " << std::setprecision(3) << (timestamps[3] - timestamps[0]) * 1000.0
              << " ms" << std::endl;

    // Camera03
    std::cout << "Camera03 stamp: color= " << std::setprecision(6) << timestamps[4]
              << "  depth= " << std::setprecision(6) << timestamps[5]
              << "  Delay relative to camera01(color): color= " << std::setprecision(3)
              << (timestamps[4] - timestamps[0]) * 1000.0 << " ms"
              << "  depth= " << std::setprecision(3) << (timestamps[5] - timestamps[0]) * 1000.0
              << " ms" << std::endl;

    // Camera04
    std::cout << "Camera04 stamp: color= " << std::setprecision(6) << timestamps[6]
              << "  depth= " << std::setprecision(6) << timestamps[7]
              << "  Delay relative to camera01(color): color= " << std::setprecision(3)
              << (timestamps[6] - timestamps[0]) * 1000.0 << " ms"
              << "  depth= " << std::setprecision(3) << (timestamps[7] - timestamps[0]) * 1000.0
              << " ms" << std::endl;
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
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSyncNode>());
  rclcpp::shutdown();
  return 0;
}