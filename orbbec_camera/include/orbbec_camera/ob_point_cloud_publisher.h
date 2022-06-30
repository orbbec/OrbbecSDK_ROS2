#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <thread>
#include <condition_variable>
#include <queue>
#include <thread>
#include <atomic>

namespace orbbec_camera {
class OBPointCloudPublisher {
 public:
  explicit OBPointCloudPublisher(rclcpp::Node* node, size_t max_filter_size = 2);

  ~OBPointCloudPublisher();

  void pushPointCloud(sensor_msgs::msg::PointCloud2&& point_cloud2);

  void pushColorPointCloud(sensor_msgs::msg::PointCloud2&& point_cloud2);

  void publishPointCloud();

  void publishColorPointCloud();

 private:
  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  std::atomic_bool is_alive_{false};
  size_t max_filter_size_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
  std::queue<sensor_msgs::msg::PointCloud2> point_cloud_q_;
  std::mutex point_cloud_q_lock_;
  std::condition_variable point_cloud_cv_;
  std::thread point_cloud_thread_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr color_point_cloud_publisher_;
  std::queue<sensor_msgs::msg::PointCloud2> color_point_cloud_q_;
  std::mutex color_point_cloud_q_lock_;
  std::condition_variable color_point_cloud_cv_;
  std::thread color_point_cloud_thread_;
};
}  // namespace orbbec_camera
