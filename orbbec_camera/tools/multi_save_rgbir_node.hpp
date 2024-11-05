#pragma once
#include <rclcpp/rclcpp.hpp>

#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <std_msgs/msg/bool.hpp>
#include <filesystem>

class MultiCameraSubscriber : public rclcpp::Node {
 public:
  MultiCameraSubscriber() : Node("multi_camera_subscriber") {
    device_init();
    currenttimes_=getCurrentTimes();

  }
  void device_init() {
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
        serial_numbers_[usb_port] = serial;
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), ":list->deviceCount(): " << list->deviceCount());
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

    ir_topics_ = this->get_parameter("ir_topics").as_string_array();
    color_topics_ = this->get_parameter("color_topics").as_string_array();
    usb_params_ = this->get_parameter("usb_ports").as_string_array();
    image_number_ = this->get_parameter("image_number").as_string();

    auto custom_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    capture_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "start_capture", custom_qos,
        std::bind(&MultiCameraSubscriber::controlCaptureCallback, this, std::placeholders::_1));
    for (size_t i = 0; i < usb_params_.size(); i++) {
      usb_numbers_[i] = usb_params_[i];
      usb_index_map_[usb_params_[i]] = i;
    }
    reentrant_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    for (const auto &pair : serial_numbers_) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_camera_subscriber"),
                         "usb_port: " << pair.first << ", serial: " << pair.second);
    }
    for (const auto &pair : usb_index_map_) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_camera_subscriber"),
                         "usb_port: " << pair.first << ", index: " << pair.second);
    }
  }

 private:
  std::mutex buffer_mutex_;
  void topic_init() {
    auto custom_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("multi_camera_subscriber"),
                       "color_topic: " << ir_topics_.size());
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
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->irCallback(msg, i);
          },
          ir_sub_options);

      auto color_sub = this->create_subscription<sensor_msgs::msg::Image>(
          color_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->colorCallback(msg, i);
          },
          color_sub_options);

      ir_subscribers_.push_back(ir_sub);
      color_subscribers_.push_back(color_sub);

      ir_image_buffers_.resize(ir_topics_.size());
      color_image_buffers_.resize(ir_topics_.size());
      ir_current_timestamp_buffers_.resize(ir_topics_.size());
      color_current_timestamp_buffers_.resize(ir_topics_.size());
      ir_timestamp_buffers_.resize(ir_topics_.size());
      color_timestamp_buffers_.resize(ir_topics_.size());
      callback_called_ = std::vector<bool>(ir_topics_.size(), false);
    }
  }
  std::string getCurrentTimes(){
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_time_t);
    std::ostringstream date_stream;
    date_stream << std::put_time(&tm, "%Y%m%d%H%M%S");

    std::string date_str = date_stream.str();
    return date_str;
  }
  std::string generateFolderName(const std::string &serial_number, size_t serial_index) {

    std::string path = std::string("multicamera_sync/output/") + currenttimes_ + "/" +
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
    auto &ir_images = ir_image_buffers_[index];
    auto &ir_current_timestamps = ir_current_timestamp_buffers_[index];
    auto &ir_timestamps = ir_timestamp_buffers_[index];
    auto &color_images = color_image_buffers_[index];
    auto &color_current_timestamps = color_current_timestamp_buffers_[index];
    auto &color_timestamps = color_timestamp_buffers_[index];
    callback_called_[index] = true;
    if (ir_images.size() < static_cast<size_t>(std::stoi(image_number_)) ||
        color_images.size() < static_cast<size_t>(std::stoi(image_number_))) {
      return;
    }

    auto usb_iter = usb_index_map_.find(usb_numbers_[index]);
    auto serial_iter = serial_numbers_.find(usb_numbers_[index]);
    int usb_index = usb_iter->second;
    if (serial_iter == serial_numbers_.end()) {
      return;
    }
    std::string serial_index = serial_iter->second;

    for (size_t i = 0; i < static_cast<size_t>(std::stoi(image_number_)); i++) {
      std::string folder = generateFolderName(serial_index, usb_index);
      std::string ir_filename = folder + "/ir#left_SN" + serial_index + "_Index" +
                                std::to_string(usb_index) + "_d" + ir_current_timestamps[i] + "_f" +
                                std::to_string(i) + "_s" + ir_timestamps[i] + "_.jpg";
      if (ir_images[i].empty()) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "over ");
        // rclcpp::shutdown();
        continue;
      }

      cv::imwrite(ir_filename, ir_images[i]);
      // RCLCPP_INFO(this->get_logger(), "Saved IR image to: %s", ir_filename.c_str());

      std::string color_filename = folder + "/color_SN" + serial_index + "_Index" +
                                   std::to_string(usb_index) + "_d" + color_current_timestamps[i] +
                                   "_f" + std::to_string(i) + "_s" + color_timestamps[i] + "_.jpg";
      if (ir_images[i].empty()) {
        // rclcpp::shutdown();
        continue;
      }
      cv::imwrite(color_filename, color_images[i]);
      // RCLCPP_INFO(this->get_logger(), "Saved Color image to: %s", color_filename.c_str());
    }

    ir_images.clear();
    color_images.clear();
    ir_current_timestamps.clear();
    color_current_timestamps.clear();
    ir_timestamps.clear();
    color_timestamps.clear();
    ir_image_buffers_[index].clear();
    ir_current_timestamp_buffers_[index].clear();
    ir_timestamp_buffers_[index].clear();
    color_image_buffers_[index].clear();
    color_current_timestamp_buffers_[index].clear();
    color_timestamp_buffers_[index].clear();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                       "callback_called_ " << index << ":" << callback_called_[index]);
    bool all_true =
        std::all_of(callback_called_.begin(), callback_called_.end(), [](bool v) { return v; });
    if (all_true) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "over ");
      ir_image_buffers_.clear();
      ir_current_timestamp_buffers_.clear();
      ir_timestamp_buffers_.clear();
      color_image_buffers_.clear();
      color_current_timestamp_buffers_.clear();
      color_timestamp_buffers_.clear();
      rclcpp::shutdown();
    }
  }

  void controlCaptureCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    is_saving_images_ = msg->data;
    topic_init();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "jjjj " << is_saving_images_);
  }
  void irCallback(std::shared_ptr<const sensor_msgs::msg::Image> image, size_t index) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (!callback_called_[index] && is_saving_images_) {
      cv::Mat ir_mat = cv_bridge::toCvCopy(image, image->encoding)->image;
      std::string current_timestamp_ir = getCurrentTimestamp(image);
      std::string timestamp_ir = getTimestamp();
      ir_image_buffers_[index].push_back(ir_mat);
      ir_current_timestamp_buffers_[index].push_back(current_timestamp_ir);
      ir_timestamp_buffers_[index].push_back(timestamp_ir);
      ir_resolution_ = std::to_string(image->width) + "x" + std::to_string(image->height);
    //   RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), ":ir: " << index <<":"<<
    //   ir_image_buffers_[index].size());
      if (ir_image_buffers_[index].size() >= static_cast<size_t>(std::stoi(image_number_)) &&
          color_image_buffers_[index].size() >= static_cast<size_t>(std::stoi(image_number_))) {
        saveAlignedImages(index);
      }
    }
  }

  void colorCallback(std::shared_ptr<const sensor_msgs::msg::Image> image, size_t index) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (!callback_called_[index] && is_saving_images_) {
      cv::Mat color_mat = cv_bridge::toCvCopy(image, image->encoding)->image;
      cv::Mat corrected_image;
      cv::cvtColor(color_mat, corrected_image, cv::COLOR_RGB2BGR);
      std::string current_timestamp_color = getCurrentTimestamp(image);
      std::string timestamp_color = getTimestamp();
      color_image_buffers_[index].push_back(corrected_image);
      color_current_timestamp_buffers_[index].push_back(current_timestamp_color);
      color_timestamp_buffers_[index].push_back(timestamp_color);
      color_resolution_ = std::to_string(image->width) + "x" + std::to_string(image->height);
    //   RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), ":color: " << index <<":"<<
    //   color_image_buffers_[index].size());
      if (ir_image_buffers_[index].size() >= static_cast<size_t>(std::stoi(image_number_)) &&
          color_image_buffers_[index].size() >= static_cast<size_t>(std::stoi(image_number_))) {
        saveAlignedImages(index);
      }
    }
  }

  rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> ir_subscribers_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> color_subscribers_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_control_sub_;

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

  std::vector<bool> callback_called_;

  std::string color_resolution_;
  std::string ir_resolution_;
 std::string currenttimes_;

  bool is_saving_images_ = false;
};
