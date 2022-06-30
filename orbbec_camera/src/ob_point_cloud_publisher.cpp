#include "orbbec_camera/ob_point_cloud_publisher.h"

namespace orbbec_camera {
using namespace std::chrono_literals;
OBPointCloudPublisher::OBPointCloudPublisher(rclcpp::Node* node, size_t max_filter_size)
    : node_(node), logger_(node->get_logger()), max_filter_size_(max_filter_size) {
  is_alive_.store(true);
  using sensor_msgs::msg::PointCloud2;
  color_point_cloud_publisher_ = node_->create_publisher<PointCloud2>(
      "depth/color/points", rclcpp::QoS{1}.best_effort().keep_last(1));
  point_cloud_publisher_ = node_->create_publisher<PointCloud2>(
      "depth/points", rclcpp::QoS{1}.best_effort().keep_last(1));

  point_cloud_thread_ = std::thread([this]() { publishPointCloud(); });
  color_point_cloud_thread_ = std::thread([this]() { publishColorPointCloud(); });
}

OBPointCloudPublisher::~OBPointCloudPublisher() {
  is_alive_.store(false);
  if (point_cloud_thread_.joinable()) {
    point_cloud_thread_.join();
  }
  if (color_point_cloud_thread_.joinable()) {
    color_point_cloud_thread_.join();
  }
}

void OBPointCloudPublisher::pushPointCloud(sensor_msgs::msg::PointCloud2&& point_cloud2) {
  std::lock_guard<decltype(point_cloud_q_lock_)> lock(point_cloud_q_lock_);
  while (point_cloud_q_.size() > max_filter_size_) {
    point_cloud_q_.pop();
  }
  point_cloud_q_.push(point_cloud2);
  point_cloud_cv_.notify_one();
}

void OBPointCloudPublisher::pushColorPointCloud(sensor_msgs::msg::PointCloud2&& point_cloud2) {
  std::lock_guard<decltype(color_point_cloud_q_lock_)> lock(color_point_cloud_q_lock_);
  while (color_point_cloud_q_.size() > max_filter_size_) {
    color_point_cloud_q_.pop();
  }
  color_point_cloud_q_.push(point_cloud2);
  color_point_cloud_cv_.notify_one();
}

void OBPointCloudPublisher::publishPointCloud() {
  while (is_alive_) {
    std::unique_lock<decltype(point_cloud_q_lock_)> lock(point_cloud_q_lock_);
    point_cloud_cv_.wait_for(lock, 100ms, [this]() { return !point_cloud_q_.empty(); });
    if (!point_cloud_q_.empty()) {
      auto msg = point_cloud_q_.front();
      point_cloud_q_.pop();
      point_cloud_publisher_->publish(msg);
    }
    lock.unlock();
  }
}

void OBPointCloudPublisher::publishColorPointCloud() {
  while (is_alive_) {
    std::unique_lock<decltype(color_point_cloud_q_lock_)> lock(color_point_cloud_q_lock_);
    color_point_cloud_cv_.wait_for(lock, 100ms, [this]() { return !color_point_cloud_q_.empty(); });
    if (!color_point_cloud_q_.empty()) {
      auto msg = color_point_cloud_q_.front();
      color_point_cloud_q_.pop();
      color_point_cloud_publisher_->publish(msg);
    }
    lock.unlock();
  }
}
}  // namespace orbbec_camera
