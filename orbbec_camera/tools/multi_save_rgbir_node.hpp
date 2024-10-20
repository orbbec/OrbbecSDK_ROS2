#pragma once
#include <rclcpp/rclcpp.hpp>

#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <filesystem>

class MultiCameraSubscriber : public rclcpp::Node {
 public:
  MultiCameraSubscriber() : Node("multi_camera_subscriber") {
    try {
      auto context = std::make_unique<ob::Context>();
      context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
      auto list = context->queryDeviceList();
      for (size_t i = 0; i < list->deviceCount(); i++) {
        auto device = list->getDevice(i);
        auto device_info = device->getDeviceInfo();
        std::string serial = device_info->serialNumber();
        std::string uid = device_info->uid();
        auto usb_port = orbbec_camera::parseUsbPort(uid);
        int vid = device_info->vid();
        int pid = device_info->pid();
        serial_numbers_[usb_port] = serial;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), ":vid: " << std::hex << vid);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), ":pid: " << std::hex << pid);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), ":serial: " << serial);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), ":usb_port: " << usb_port);
        color_frame_counters_[count] = 0;
        ir_frame_counters_[count] = 0;
        count++;
      }
    } catch (ob::Error &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.getMessage());
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(get_logger(), "unknown error");
    }

    this->declare_parameter<std::vector<std::string>>("ir_topics", std::vector<std::string>());
    this->declare_parameter<std::vector<std::string>>("color_topics", std::vector<std::string>());
    this->declare_parameter<std::vector<std::string>>("usb_ports", std::vector<std::string>());
    this->declare_parameter<std::string>("image_number", "100");

    std::vector<std::string> ir_topics_ = this->get_parameter("ir_topics").as_string_array();
    std::vector<std::string> color_topics_ = this->get_parameter("color_topics").as_string_array();
    std::vector<std::string> usb_params_ = this->get_parameter("usb_ports").as_string_array();
    image_number_ = this->get_parameter("image_number").as_string();

    auto custom_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    for (size_t i = 0; i < usb_params_.size(); i++) {
      usb_numbers_[i] = usb_params_[i];
      usb_index_map_[usb_params_[i]] = i;
    }
    reentrant_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    for (const auto &pair : serial_numbers_) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_camera_subscriber"),
                         "usb_port: " << pair.first << ", serial: " << pair.second);
    }
    for (const auto &pair : usb_index_map_) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_camera_subscriber"),
                         "usb_port: " << pair.first << ", index: " << pair.second);
    }

    for (size_t i = 0; i < ir_topics_.size(); ++i) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_camera_subscriber"),
                         "ir_topic: " << ir_topics_[i]);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_camera_subscriber"),
                         "color_topic: " << color_topics_[i]);

      rclcpp::SubscriptionOptions ir_sub_options;
      ir_sub_options.callback_group = reentrant_callback_group_;

      rclcpp::SubscriptionOptions color_sub_options;
      color_sub_options.callback_group = reentrant_callback_group_;

      auto ir_sub = this->create_subscription<sensor_msgs::msg::Image>(
          ir_topics_[i], custom_qos,
          [this, i](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
            this->irCallback(msg, i);
          },
          ir_sub_options);

      auto color_sub = this->create_subscription<sensor_msgs::msg::Image>(
          color_topics_[i], custom_qos,
          [this, i](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
            this->colorCallback(msg, i);
          },
          color_sub_options);

      ir_subscribers_.push_back(ir_sub);
      color_subscribers_.push_back(color_sub);

      ir_image_buffers_.emplace_back();
      color_image_buffers_.emplace_back();
      ir_current_timestamp_buffers_.emplace_back();
      color_current_timestamp_buffers_.emplace_back();
      ir_timestamp_buffers_.emplace_back();
      color_timestamp_buffers_.emplace_back();
    }
  }

 private:
  std::string generateFolderName(const std::string &serial_number, size_t serial_index) {
    std::string frame_rate = "30fps";
    std::string folder_name = "Star-AE-OFF-ir-" + ir_resolution + "-" + "y8" + "-rgb-" +
                              color_resolution + "-" + "mjpg" + "-" + frame_rate;

    std::string path = std::string("multicamera_sync/output/") + folder_name + "/" +
                       "TotalModeFrames/" + "/" + "SN" + serial_number + "_Index" +
                       std::to_string(serial_index);

    std::filesystem::create_directories(path);
    return path;
  }
  std::string getTimestamp() {
    auto now = this->get_clock()->now();
    int64_t seconds = now.seconds();
    int64_t nanoseconds = now.nanoseconds() % 1000000000;
    int64_t milliseconds = nanoseconds / 1000000;
    return std::to_string(seconds) + std::to_string(milliseconds);
  }
  std::string getCurrentTimestamp(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg) {
    int64_t seconds = image_msg->header.stamp.sec;
    int64_t nanoseconds = image_msg->header.stamp.nanosec;

    int64_t milliseconds = nanoseconds / 1000000;

    std::ostringstream timestamp;
    timestamp << seconds << std::setw(3) << std::setfill('0') << milliseconds;

    return timestamp.str();
  }

  void saveAlignedImages(size_t index) {
    images_saved_ = true;
    auto &ir_images = ir_image_buffers_[index];
    auto &ir_current_timestamps = ir_current_timestamp_buffers_[index];
    auto &ir_timestamps = ir_timestamp_buffers_[index];
    auto &color_images = color_image_buffers_[index];
    auto &color_current_timestamps = color_current_timestamp_buffers_[index];
    auto &color_timestamps = color_timestamp_buffers_[index];

    if (ir_images.size() < static_cast<size_t>(std::stoi(image_number_)) ||
        color_images.size() < static_cast<size_t>(std::stoi(image_number_))) {
      return;
    }

    auto usb_iter = usb_index_map_.find(usb_numbers_[index]);
    auto serial_iter = serial_numbers_.find(usb_numbers_[index]);
    int usb_index = usb_iter->second;
    std::string serial_index = serial_iter->second;

    for (size_t i = 0; i < static_cast<size_t>(std::stoi(image_number_)); ++i) {
      std::string folder = generateFolderName(serial_index, usb_index);
      std::string ir_filename = folder + "/ir#left_SN" + serial_index + "_Index" +
                                std::to_string(usb_index) + "_d" + ir_current_timestamps[i] + "_f" +
                                std::to_string(i) + "_s" + ir_timestamps[i] + "_.jpg";

      cv::imwrite(ir_filename, ir_images[i]);
      RCLCPP_INFO(this->get_logger(), "Saved IR image to: %s", ir_filename.c_str());

      std::string color_filename = folder + "/color_SN" + serial_index + "_Index" +
                                   std::to_string(usb_index) + "_d" + color_current_timestamps[i] +
                                   "_f" + std::to_string(i) + "_s" + color_timestamps[i] + "_.jpg";
      cv::imwrite(color_filename, color_images[i]);
      RCLCPP_INFO(this->get_logger(), "Saved Color image to: %s", color_filename.c_str());
    }

    ir_images.clear();
    color_images.clear();
    ir_current_timestamps.clear();
    color_current_timestamps.clear();
    ir_timestamps.clear();
    color_timestamps.clear();
    rclcpp::shutdown();
  }

  void irCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image, size_t index) {
    cv::Mat ir_mat = cv_bridge::toCvCopy(image, image->encoding)->image;
    std::string current_timestamp_ir = getCurrentTimestamp(image);
    std::string timestamp_ir = getTimestamp();
    ir_image_buffers_[index].push_back(ir_mat);
    ir_current_timestamp_buffers_[index].push_back(current_timestamp_ir);
    ir_timestamp_buffers_[index].push_back(timestamp_ir);
    ir_resolution = std::to_string(image->width) + "x" + std::to_string(image->height);
    if (!images_saved_ &&
        ir_image_buffers_[index].size() >= static_cast<size_t>(std::stoi(image_number_)) &&
        color_image_buffers_[index].size() >= static_cast<size_t>(std::stoi(image_number_))) {
      saveAlignedImages(index);
    }
  }

  void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image, size_t index) {
    cv::Mat color_mat = cv_bridge::toCvCopy(image, image->encoding)->image;
    cv::Mat corrected_image;
    cv::cvtColor(color_mat, corrected_image, cv::COLOR_RGB2BGR);
    std::string current_timestamp_color = getCurrentTimestamp(image);
    std::string timestamp_color = getTimestamp();

    color_image_buffers_[index].push_back(corrected_image);
    color_current_timestamp_buffers_[index].push_back(current_timestamp_color);
    color_timestamp_buffers_[index].push_back(timestamp_color);
    color_resolution = std::to_string(image->width) + "x" + std::to_string(image->height);
    if (!images_saved_ &&
        ir_image_buffers_[index].size() >= static_cast<size_t>(std::stoi(image_number_)) &&
        color_image_buffers_[index].size() >= static_cast<size_t>(std::stoi(image_number_))) {
      saveAlignedImages(index);
    }
  }

  rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> ir_subscribers_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> color_subscribers_;

  std::map<std::string, int> usb_index_map_;
  std::map<std::string, std::string> serial_numbers_;
  std::array<std::string, 10> usb_numbers_;
  std::array<size_t, 10> color_frame_counters_;
  std::array<size_t, 10> ir_frame_counters_;
  size_t count = 0;

  std::vector<std::string> usb_params_;
  std::vector<std::string> ir_topics_;
  std::vector<std::string> color_topics_;
  std::string image_number_;

  std::vector<std::vector<cv::Mat>> ir_image_buffers_;
  std::vector<std::vector<cv::Mat>> color_image_buffers_;
  std::vector<std::vector<std::string>> ir_current_timestamp_buffers_;
  std::vector<std::vector<std::string>> color_current_timestamp_buffers_;
  std::vector<std::vector<std::string>> ir_timestamp_buffers_;
  std::vector<std::vector<std::string>> color_timestamp_buffers_;

  std::string color_resolution;
  std::string ir_resolution;

  bool images_saved_;
};