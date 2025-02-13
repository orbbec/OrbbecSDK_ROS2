#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>
#include "orbbec_camera_msgs/msg/metadata.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <filesystem>

namespace orbbec_camera {
namespace tools {

class StartBenchmark : public rclcpp::Node {
 public:
  explicit StartBenchmark(const rclcpp::NodeOptions& options) : Node("StartBenchmark", options) {
    params_init();
    auto custom_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    for (size_t i = 0; i < camera_name_.size(); ++i) {
      color_subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
          color_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->color_Callback(msg, i);
          }));
      depth_subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
          depth_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->depth_Callback(msg, i);
          }));
      left_ir_subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
          left_ir_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->left_ir_Callback(msg, i);
          }));
      right_ir_subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
          right_ir_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->right_ir_Callback(msg, i);
          }));
      depth_point_cloud_subs_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
          depth_point_cloud_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg) {
            this->depth_point_cloud_Callback(msg, i);
          }));
      color_point_cloud_subs_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
          color_point_cloud_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg) {
            this->color_point_cloud_Callback(msg, i);
          }));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("StartBenchmark"), color_topics_[i] << " is subed ");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("StartBenchmark"), depth_topics_[i] << " is subed ");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("StartBenchmark"), left_ir_topics_[i] << " is subed ");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("StartBenchmark"), right_ir_topics_[i] << " is subed ");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("StartBenchmark"),
                         depth_point_cloud_topics_[i] << " is subed ");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("StartBenchmark"),
                         color_point_cloud_topics_[i] << " is subed ");
    }
  }

 private:
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> color_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> depth_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> left_ir_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> right_ir_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      depth_point_cloud_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      color_point_cloud_subs_;


  std::vector<std::string> camera_name_;
  std::vector<std::string> color_topics_;
  std::vector<std::string> depth_topics_;
  std::vector<std::string> left_ir_topics_;
  std::vector<std::string> right_ir_topics_;
  std::vector<std::string> depth_point_cloud_topics_;
  std::vector<std::string> color_point_cloud_topics_;

  std::mutex image_mutex_;

  void params_init() {
    std::ifstream file(
        "install/orbbec_camera/share/orbbec_camera/config/tools/startbenchmark/"
        "start_benchmark_params.json");
    if (!file.is_open()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open JSON file.");
      return;
    }
    nlohmann::json json_data;
    file >> json_data;
    camera_name_ =
        json_data["start_benchmark_params"]["camera_name"].get<std::vector<std::string>>();
    color_topics_.resize(camera_name_.size());
    depth_topics_.resize(camera_name_.size());
    left_ir_topics_.resize(camera_name_.size());
    right_ir_topics_.resize(camera_name_.size());
    depth_point_cloud_topics_.resize(camera_name_.size());
    color_point_cloud_topics_.resize(camera_name_.size());
    for (size_t i = 0; i < camera_name_.size(); ++i) {
      color_topics_[i] = "/" + camera_name_[i] + "/color/image_raw";
      depth_topics_[i] = "/" + camera_name_[i] + "/depth/image_raw";
      left_ir_topics_[i] = "/" + camera_name_[i] + "/left_ir/image_raw";
      right_ir_topics_[i] = "/" + camera_name_[i] + "/right_ir/image_raw";
      depth_point_cloud_topics_[i] = "/" + camera_name_[i] + "/depth/points";
      color_point_cloud_topics_[i] = "/" + camera_name_[i] + "/depth_registered/points";
    }
  }

  void color_Callback(std::shared_ptr<const sensor_msgs::msg::Image> msg, size_t index) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("StartBenchmark"),
                        "time is : " << msg->step << "color is subed " << index << "is subed");
  }
  void depth_Callback(std::shared_ptr<const sensor_msgs::msg::Image> msg, size_t index) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("StartBenchmark"),
                        "time is : " << msg->step << "depth is subed " << index << "is subed");
  }
  void left_ir_Callback(std::shared_ptr<const sensor_msgs::msg::Image> msg, size_t index) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("StartBenchmark"),
                        "time is : " << msg->step << "left_ir is subed " << index << "is subed");
  }
  void right_ir_Callback(std::shared_ptr<const sensor_msgs::msg::Image> msg, size_t index) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("StartBenchmark"),
                        "time is : " << msg->step << "right_ir is subed " << index << "is subed");
  }
  void depth_point_cloud_Callback(std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg,
                                  size_t index) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    RCLCPP_DEBUG_STREAM(
        rclcpp::get_logger("StartBenchmark"),
        "time is : " << msg->point_step << "depth_point_cloud is subed " << index << "is subed");
  }
  void color_point_cloud_Callback(std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg,
                                  size_t index) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    RCLCPP_DEBUG_STREAM(
        rclcpp::get_logger("StartBenchmark"),
        "time is : " << msg->point_step << "color_point_cloud is subed " << index << "is subed");
  }
};

}  // namespace tools
}  // namespace orbbec_camera
RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::tools::StartBenchmark)
