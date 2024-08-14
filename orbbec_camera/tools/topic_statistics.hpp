#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <statistics_msgs/msg/metrics_message.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <filesystem>

namespace orbbec_camera {
namespace tools {

class TopicStatistics : public rclcpp::Node {
 public:
  TopicStatistics() : Node("topic_statistics") {
    this->declare_parameter("image_topic", "/camera/color/image_raw");
    this->declare_parameter("statistics_topic", "/statistics");

    image_topic_ = this->get_parameter("image_topic").as_string();
    statistics_topic_ = this->get_parameter("statistics_topic").as_string();

    RCLCPP_INFO(get_logger(), "TopicStatistics starting up");
    initialize();
    initialize_csv();
  }

  void initialize() {
    // Create a subscriber to the image topic
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_topic = statistics_topic_;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(1000);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 10, std::bind(&TopicStatistics::image_callback, this, std::placeholders::_1),
        sub_opt);

    // Create a subscriber to the statistics topic
    statistics_sub_ = this->create_subscription<statistics_msgs::msg::MetricsMessage>(
        statistics_topic_, 10,
        std::bind(&TopicStatistics::statistics_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribed to image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribed to statistics topic: %s", statistics_topic_.c_str());
  }

  void initialize_csv() {
    // Get the current working directory
    std::filesystem::path cwd = std::filesystem::current_path();
    csv_path_ = cwd.string() + "/statistics.csv";

    // Open the file in output mode, which will create/overwrite the file
    std::ofstream csv_file(csv_path_, std::ios::out);
    if (csv_file.is_open()) {
      // Write the header to the CSV file
      csv_file << "\"_time\",\"message_type\",\"min\",\"avg\",\"max\"\n";
      csv_file.close();
      csv_initialized_ = true;
    } else {
      RCLCPP_ERROR(get_logger(), "Unable to open statistics.csv for writing");
    }
  }

  void write_statistics_to_csv(const statistics_msgs::msg::MetricsMessage& msg) {
    if (!csv_initialized_) {
      // If the CSV hasn't been initialized, initialize it
      initialize_csv();
    }

    std::ofstream csv_file(csv_path_, std::ios::app);
    if (csv_file.is_open()) {
      // Extract the timestamp from the MetricsMessage
      auto time_ns = rclcpp::Time(msg.window_stop);
      auto time_point = std::chrono::system_clock::from_time_t(time_ns.seconds());
      std::time_t time_t_value = std::chrono::system_clock::to_time_t(time_point);
      std::tm* tm_value = std::gmtime(&time_t_value);
      char time_str[20];
      std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", tm_value);

      // Determine the message type (e.g., "age" or "period")
      std::string message_type;
      if (msg.metrics_source.find("_age") != std::string::npos) {
        message_type = "age";
      } else if (msg.metrics_source.find("_period") != std::string::npos) {
        message_type = "period";
      } else {
        message_type = "unknown";
      }

      // Write the timestamp and message type to the CSV file
      csv_file << "\"" << time_str << "\",\"" << message_type << "\",";

      // Variables to store the min, avg, and max values
      double min = 0, avg = 0, max = 0;

      // Iterate through the statistics and assign values based on the type
      for (const auto& statistic : msg.statistics) {
        switch (statistic.data_type) {
          case 1:  // avg
            avg = statistic.data;
            break;
          case 2:  // min
            min = statistic.data;
            break;
          case 3:  // max
            max = statistic.data;
            break;
          default:
            break;
        }
      }

      // Write the min, avg, and max values to the CSV file
      csv_file << min << " ms," << avg << " ms," << max << " ms\n";
      csv_file.close();
    } else {
      RCLCPP_ERROR(get_logger(), "Unable to open statistics.csv for writing");
    }
  }

 private:
  void image_callback(const sensor_msgs::msg::Image::UniquePtr msg) {
    image_count_++;
    image_size_ += msg->data.size();
    RCLCPP_DEBUG(get_logger(), "Received image %d, size: %zu bytes", image_count_,
                 msg->data.size());
  }

  void statistics_callback(const statistics_msgs::msg::MetricsMessage::UniquePtr msg) {
    RCLCPP_INFO(get_logger(), "Statistics received:\n%s", metrics_message_to_string(*msg).c_str());
    write_statistics_to_csv(*msg);
  }

  std::string metrics_message_to_string(const statistics_msgs::msg::MetricsMessage& msg) {
    std::stringstream ss;
    ss << "Metric name: " << msg.metrics_source << " source: " << msg.measurement_source_name
       << " unit: " << msg.unit;
    ss << "\nWindow start: " << msg.window_start.nanosec << " end: " << msg.window_stop.nanosec;

    for (const auto& statistic : msg.statistics) {
      ss << "\n"
         << statistic_type_to_string(statistic.data_type) << ": " << std::to_string(statistic.data);
    }

    return ss.str();
  }

  std::string statistic_type_to_string(int8_t type) {
    switch (type) {
      case 1:
        return "avg";
      case 2:
        return "min";
      case 3:
        return "max";
      case 4:
        return "std_dev";
      case 5:
        return "sample_count";
      default:
        return "unknown";
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<statistics_msgs::msg::MetricsMessage>::SharedPtr statistics_sub_;

  std::string image_topic_;
  std::string statistics_topic_;
  int image_count_ = 0;
  size_t image_size_ = 0;
  std::string csv_path_;
  bool csv_initialized_ = false;
};

}  // namespace tools
}  // namespace orbbec_camera
