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
        RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_save_rgbir_node"), ":vid: " << std::hex << vid);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_save_rgbir_node"), ":pid: " << std::hex << pid);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_save_rgbir_node"), ":serial: " << serial);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_save_rgbir_node"), ":usb_port: " << usb_port);
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

    std::vector<std::string> ir_topics_ = this->get_parameter("ir_topics").as_string_array();
    std::vector<std::string> color_topics_ = this->get_parameter("color_topics").as_string_array();
    std::vector<std::string> usb_params_ = this->get_parameter("usb_ports").as_string_array();

    auto custom_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    for (size_t i = 0; i < usb_params_.size(); i++) {
      usb_numbers_[i] = usb_params_[i];
      usb_index_map_[usb_params_[i]] = i;
    }

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

      auto ir_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          this, ir_topics_[i], custom_qos.get_rmw_qos_profile());
      auto color_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          this, color_topics_[i], custom_qos.get_rmw_qos_profile());

      ir_sub->registerCallback([this, i](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        this->irCallback(msg, i);
      });

      color_sub->registerCallback([this, i](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        this->colorCallback(msg, i);
      });

      ir_subscribers_.push_back(ir_sub);
      color_subscribers_.push_back(color_sub);
    }
  }

 private:
  std::string generateFolderName(const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
                                 const std::string &serial_number, size_t serial_index) {
    std::string color_resolution =
        std::to_string(color_msg->width) + "x" + std::to_string(color_msg->height);
    std::string color_encoding = color_msg->encoding;
    std::string frame_rate = "30fps";
    std::string folder_name = "Star-AE-OFF-ir-" + color_resolution + "-" + "y8" + "-rgb-" +
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
  void irCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image, size_t index) {
    auto usb_iter = usb_index_map_.find(usb_numbers_[index]);
    auto serial_iter = serial_numbers_.find(usb_numbers_[index]);
    int usb_index = usb_iter->second;
    std::string serial_index = serial_iter->second;
    cv::Mat ir_mat = cv_bridge::toCvCopy(image, image->encoding)->image;
    std::string timestamp_ir = getCurrentTimestamp(image);
    size_t frame_index = ir_frame_counters_[index]++;
    std::string folder = generateFolderName(image, serial_index, usb_index);
    std::string filename = folder + "/ir#left_SN" + serial_index + "_Index" +
                           std::to_string(usb_index) + "_d" + timestamp_ir + "_f" +
                           std::to_string(frame_index) + "_s" + getTimestamp() + "_.jpg";
    cv::imwrite(filename, ir_mat);
    RCLCPP_INFO(this->get_logger(), "Saved IR image for camera to: %s", filename.c_str());
  }

  void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image, size_t index) {
    auto usb_iter = usb_index_map_.find(usb_numbers_[index]);
    auto serial_iter = serial_numbers_.find(usb_numbers_[index]);
    int usb_index = usb_iter->second;
    std::string serial_index = serial_iter->second;
    cv::Mat color_mat = cv_bridge::toCvCopy(image, image->encoding)->image;
    cv::Mat corrected_image;
    cv::cvtColor(color_mat, corrected_image, cv::COLOR_RGB2BGR);
    std::string timestamp_color = getCurrentTimestamp(image);
    size_t frame_index = color_frame_counters_[index]++;
    std::string folder = generateFolderName(image, serial_index, usb_index);
    std::string filename = folder + "/color_SN" + serial_index + "_Index" +
                           std::to_string(usb_index) + "_d" + timestamp_color + "_f" +
                           std::to_string(frame_index) + "_s" + getTimestamp() + "_.jpg";
    cv::imwrite(filename, corrected_image);
    RCLCPP_INFO(this->get_logger(), "Saved Color image for camera to: %s", filename.c_str());
  }

  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>>
      ir_subscribers_;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>>
      color_subscribers_;

  std::map<std::string, int> usb_index_map_;
  std::map<std::string, std::string> serial_numbers_;
  std::array<std::string, 10> usb_numbers_;
  std::array<size_t, 10> color_frame_counters_;
  std::array<size_t, 10> ir_frame_counters_;
  size_t count = 0;

  std::vector<std::string> usb_params_;
  std::vector<std::string> ir_topics_;
  std::vector<std::string> color_topics_;
};