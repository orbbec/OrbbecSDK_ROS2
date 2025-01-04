#pragma once
#include <rclcpp/rclcpp.hpp>

#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/ob_camera_node.h>
#include <orbbec_camera/utils.h>
#include "orbbec_camera_msgs/msg/metadata.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <std_msgs/msg/int32.hpp>
#include <filesystem>

namespace orbbec_camera {
namespace tools {

class MultiCameraCloudSubscriber : public rclcpp::Node {
 public:
  MultiCameraCloudSubscriber() : Node("multi_camera_cloud_subscriber") { topic_init(); }
  //   ~MultiCameraCloudSubscriber() {
  //   }

 private:
  std::mutex image_mutex_;
  void topic_init() {
    auto custom_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    current_path = std::filesystem::current_path().string();
    stand_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/points", custom_qos,
        [this](std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg) {
          this->stand_cloud_Callback(msg);
        });
    transform_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rear_camera/depth/points", custom_qos,
        [this](std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg) {
          this->transform_cloud_Callback(msg);
        });
  }

  void stand_cloud_Callback(std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    std::stringstream ss;
    auto now = std::time(nullptr);
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    std::string filename = current_path + "/point_cloud/points_" + ss.str() + ".ply";
    if (!std::filesystem::exists(current_path + "/point_cloud")) {
      std::filesystem::create_directory(current_path + "/point_cloud");
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_camera_cloud_subscriber"), "Saving point cloud to " << filename);
    saveDepthPointsToPly(msg, filename);

  }
void transform_cloud_Callback(std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    std::stringstream ss;
    auto now = std::time(nullptr);
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    std::string filename = current_path + "/rear_camera/point_cloud/points_" + ss.str() + ".ply";
    if (!std::filesystem::exists(current_path + "/point_cloud")) {
      std::filesystem::create_directory(current_path + "/point_cloud");
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_camera_cloud_subscriber"),
                       "Saving transform_cloud to " << filename);
    saveDepthPointsToPly(msg, filename);
  }
  void saveDepthPointsToPly(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &msg,
                          const std::string &fileName) {
  FILE *fp = fopen(fileName.c_str(), "wb+");
  CHECK_NOTNULL(fp);
  CHECK_NOTNULL(msg);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  // First, count the actual number of valid points
  size_t valid_points = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      ++valid_points;
    }
  }

  // Reset the iterators
  iter_x = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "x");
  iter_y = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "y");
  iter_z = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "z");

  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", valid_points);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "end_header\n");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      fprintf(fp, "%.3f %.3f %.3f\n", *iter_x, *iter_y, *iter_z);
    }
  }

  fflush(fp);
  fclose(fp);
}
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr stand_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr transform_cloud_sub_;    
  std::string current_path;
};
}  // namespace tools
}  // namespace orbbec_camera