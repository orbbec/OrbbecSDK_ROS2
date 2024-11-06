#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <filesystem>

#include <opencv2/opencv.hpp>
#if defined(ROS_JAZZY) || defined(ROS_IRON)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>
#include "orbbec_camera_msgs/msg/metadata.hpp"

namespace orbbec_camera {
namespace tools {
const int kMetadataVectorSize = 10;

struct ImageMetadata {
  int actual_frame_rate;
  int ae_roi_bottom;
  int ae_roi_left;
  int ae_roi_right;
  int ae_roi_top;
  int auto_exposure;
  int exposure;
  int exposure_priority;
  int frame_emitter_mode;
  int frame_laser_power;
  int frame_laser_power_mode;
  int frame_number;
  int64_t frame_timestamp;
  int gain;
  int gpio_input_data;
  int hdr_sequence_index;
  int hdr_sequence_name;
  int hdr_sequence_size;
  int64_t sensor_timestamp;

  ImageMetadata()
      : actual_frame_rate(0),
        ae_roi_bottom(0),
        ae_roi_left(0),
        ae_roi_right(0),
        ae_roi_top(0),
        auto_exposure(0),
        exposure(0),
        exposure_priority(0),
        frame_emitter_mode(0),
        frame_laser_power(0),
        frame_laser_power_mode(0),
        frame_number(0),
        frame_timestamp(0),
        gain(0),
        gpio_input_data(0),
        hdr_sequence_index(0),
        hdr_sequence_name(0),
        hdr_sequence_size(0),
        sensor_timestamp(0) {}
  friend std::ostream &operator<<(std::ostream &os, const ImageMetadata &metadata) {
    os
        //  << "actual_frame_rate: " << metadata.actual_frame_rate << "\n"
        //  << "ae_roi_bottom: " << metadata.ae_roi_bottom << "\n"
        //  << "ae_roi_left: " << metadata.ae_roi_left << "\n"
        //  << "ae_roi_right: " << metadata.ae_roi_right << "\n"
        //  << "ae_roi_top: " << metadata.ae_roi_top << "\n"
        << "auto_exposure: " << metadata.auto_exposure << "\n"
        << "exposure: " << metadata.exposure
        << "\n"
        //  << "exposure_priority: " << metadata.exposure_priority << "\n"
        << "frame_emitter_mode: " << metadata.frame_emitter_mode
        << "\n"
        //  << "frame_laser_power: " << metadata.frame_laser_power << "\n"
        //  << "frame_laser_power_mode: " << metadata.frame_laser_power_mode << "\n"
        << "frame_number: " << metadata.frame_number << "\n"
        << "frame_timestamp: " << metadata.frame_timestamp << "\n"
        << "gain: " << metadata.gain
        << "\n"
        //  << "gpio_input_data: " << metadata.gpio_input_data << "\n"
        //  << "hdr_sequence_index: " << metadata.hdr_sequence_index << "\n"
        //  << "hdr_sequence_name: " << metadata.hdr_sequence_name << "\n"
        //  << "hdr_sequence_size: " << metadata.hdr_sequence_size << "\n"
        << "sensor_timestamp: " << metadata.sensor_timestamp;
    return os;
  }
};

using std::placeholders::_1;
using std::placeholders::_2;

class MetadataSaveFiles : public rclcpp::Node {
 public:
  MetadataSaveFiles() : Node("metadata_save_files") {
    initialize_params();
    initialize_directories();
    initialize_pub_sub();
  }
  void initialize_params() {
    std::ifstream file(
        "install/orbbec_camera/share/orbbec_camera/config/tools/metadatasave/"
        "metadata_save_params.json");
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file.");
      return;
    }
    nlohmann::json json_data;
    file >> json_data;
    left_ir_image_topic_ =
        json_data["metadata_save_params"]["left_ir_image_topic"].get<std::string>();
    right_ir_image_topic_ =
        json_data["metadata_save_params"]["right_ir_image_topic"].get<std::string>();
    depth_image_topic_ = json_data["metadata_save_params"]["depth_image_topic"].get<std::string>();

    left_ir_metadata_topic_ =
        json_data["metadata_save_params"]["left_ir_metadata_topic"].get<std::string>();
    right_ir_metadata_topic_ =
        json_data["metadata_save_params"]["right_ir_metadata_topic"].get<std::string>();
    depth_metadata_topic_ =
        json_data["metadata_save_params"]["depth_metadata_topic"].get<std::string>();
    RCLCPP_INFO(this->get_logger(), "Parameter 2: %s", depth_image_topic_.c_str());
    // this->declare_parameter("left_ir_image_topic", "/camera/left_ir/image_raw");
    // this->declare_parameter("right_ir_image_topic", "/camera/right_ir/image_raw");
    // this->declare_parameter("depth_image_topic", "/camera/depth/image_raw");
    // this->declare_parameter("left_ir_metadata_topic", "/camera/left_ir/metadata");
    // this->declare_parameter("right_ir_metadata_topic", "/camera/right_ir/metadata");
    // this->declare_parameter("depth_metadata_topic", "/camera/depth/metadata");

    // left_ir_image_topic_ = this->get_parameter("left_ir_image_topic").as_string();
    // right_ir_image_topic_ = this->get_parameter("right_ir_image_topic").as_string();
    // depth_image_topic_ = this->get_parameter("depth_image_topic").as_string();
    // left_ir_metadata_topic_ = this->get_parameter("left_ir_metadata_topic").as_string();
    // right_ir_metadata_topic_ = this->get_parameter("right_ir_metadata_topic").as_string();
    // depth_metadata_topic_ = this->get_parameter("depth_metadata_topic").as_string();
  }
  void initialize_pub_sub() {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    const rmw_qos_profile_t qos_filters = qos.get_rmw_qos_profile();

    left_ir_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, left_ir_image_topic_, qos_filters);
    left_ir_metadata_sub_ =
        std::make_shared<message_filters::Subscriber<orbbec_camera_msgs::msg::Metadata>>(
            this, left_ir_metadata_topic_, qos_filters);

    left_ir_sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
        MySyncPolicy(10), *left_ir_image_sub_, *left_ir_metadata_sub_);
    left_ir_sync_->setMaxIntervalDuration(rclcpp::Duration(0, 100 * 1000000));  // 100 ms

    left_ir_sync_->registerCallback(
        std::bind(&MetadataSaveFiles::left_ir_metadata_sync_callback, this, _1, _2));

    right_ir_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, right_ir_image_topic_, qos_filters);
    right_ir_metadata_sub_ =
        std::make_shared<message_filters::Subscriber<orbbec_camera_msgs::msg::Metadata>>(
            this, right_ir_metadata_topic_, qos_filters);

    right_ir_sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
        MySyncPolicy(10), *right_ir_image_sub_, *right_ir_metadata_sub_);
    right_ir_sync_->setMaxIntervalDuration(rclcpp::Duration(0, 100 * 1000000));  // 100 ms

    right_ir_sync_->registerCallback(
        std::bind(&MetadataSaveFiles::right_ir_metadata_sync_callback, this, _1, _2));

    depth_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, depth_image_topic_, qos_filters);
    depth_metadata_sub_ =
        std::make_shared<message_filters::Subscriber<orbbec_camera_msgs::msg::Metadata>>(
            this, depth_metadata_topic_, qos_filters);

    depth_sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
        MySyncPolicy(10), *depth_image_sub_, *depth_metadata_sub_);
    depth_sync_->setMaxIntervalDuration(rclcpp::Duration(0, 100 * 1000000));  // 100 ms

    depth_sync_->registerCallback(
        std::bind(&MetadataSaveFiles::depth_metadata_sync_callback, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Subscribed to left_ir_image_topic_ %s",
                left_ir_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribed to right_ir_image_topic_ %s",
                right_ir_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribed to depth_image_topic_ %s", depth_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribed to left_ir_metadata_topic_ %s",
                left_ir_metadata_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribed to right_ir_metadata_topic_ %s",
                right_ir_metadata_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribed to depth_metadata_topic_ %s",
                depth_metadata_topic_.c_str());
  }

  void initialize_directories() {
    try {
      auto context = std::make_unique<ob::Context>();
      context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
      auto list = context->queryDeviceList();
      for (size_t i = 0; i < list->deviceCount(); i++) {
        auto device = list->getDevice(i);
        auto device_info = device->getDeviceInfo();
        serial_ = device_info->serialNumber();
        std::string uid = device_info->uid();
        auto usb_port = orbbec_camera::parseUsbPort(uid);
        RCLCPP_INFO_STREAM(get_logger(), "serial: " << serial_);
        RCLCPP_INFO_STREAM(get_logger(), "usb port: " << usb_port);
      }
    } catch (ob::Error &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.getMessage());
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(get_logger(), "unknown error");
    }

    std::filesystem::path cwd = std::filesystem::current_path();
    serial_path_zero_ = cwd.string() + "/" + serial_ + "/0";
    serial_path_one_ = cwd.string() + "/" + serial_ + "/1";
    createDirectory(serial_path_zero_);
    createDirectory(serial_path_one_);
  }

 private:
  void left_ir_metadata_sync_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
      const orbbec_camera_msgs::msg::Metadata::ConstSharedPtr &metadata_msg) {
    // RCLCPP_INFO_STREAM(get_logger(),
    //                    "Left IR stamp: sec " << image_msg->header.stamp.sec << " nanosec " <<
    //                    image_msg->header.stamp.nanosec);
    // RCLCPP_INFO_STREAM(get_logger(),
    //                    "Left IR metadata stamp: sec " << metadata_msg->header.stamp.sec << "
    //                    nanosec " << metadata_msg->header.stamp.nanosec);
    int frame_emitter_mode{0};
    try {
      nlohmann::json json_data = nlohmann::json::parse(metadata_msg->json_data);

      left_ir_metadata_.actual_frame_rate = json_data["actual_frame_rate"];
      left_ir_metadata_.ae_roi_bottom = json_data["ae_roi_bottom"];
      left_ir_metadata_.ae_roi_left = json_data["ae_roi_left"];
      left_ir_metadata_.ae_roi_right = json_data["ae_roi_right"];
      left_ir_metadata_.ae_roi_top = json_data["ae_roi_top"];
      left_ir_metadata_.auto_exposure = json_data["auto_exposure"];
      left_ir_metadata_.exposure = json_data["exposure"];
      left_ir_metadata_.exposure_priority = json_data["exposure_priority"];
      left_ir_metadata_.frame_emitter_mode = json_data["frame_emitter_mode"];
      left_ir_metadata_.frame_laser_power = json_data["frame_laser_power"];
      left_ir_metadata_.frame_laser_power_mode = json_data["frame_laser_power_mode"];
      left_ir_metadata_.frame_number = json_data["frame_number"];
      left_ir_metadata_.frame_timestamp = json_data["frame_timestamp"];
      left_ir_metadata_.gain = json_data["gain"];
      left_ir_metadata_.gpio_input_data = json_data["gpio_input_data"];
      left_ir_metadata_.hdr_sequence_index = json_data["hdr_sequence_index"];
      left_ir_metadata_.hdr_sequence_name = json_data["hdr_sequence_name"];
      left_ir_metadata_.hdr_sequence_size = json_data["hdr_sequence_size"];
      left_ir_metadata_.sensor_timestamp = json_data["sensor_timestamp"];

      frame_emitter_mode = left_ir_metadata_.frame_emitter_mode;

      // RCLCPP_INFO_STREAM(get_logger(), "Left IR metadata: \n" << left_ir_metadata_);

      save_metadata_to_file(left_ir_metadata_, "irleft");
    } catch (const nlohmann::json::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse metadata JSON: %s", e.what());
    }

    try {
      save_image_to_file(image_msg, "irleft", left_ir_image_count_, image_msg->header.stamp,
                         frame_emitter_mode);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error saving image: " << e.what());
      return;
    }
    left_ir_image_count_++;
  }
  void right_ir_metadata_sync_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
      const orbbec_camera_msgs::msg::Metadata::ConstSharedPtr &metadata_msg) {
    // RCLCPP_INFO_STREAM(get_logger(),
    //                    "Right IR stamp: sec " << image_msg->header.stamp.sec << " nanosec " <<
    //                    image_msg->header.stamp.nanosec);
    // RCLCPP_INFO_STREAM(get_logger(),
    //                    "Right IR metadata stamp: sec " << metadata_msg->header.stamp.sec << "
    //                    nanosec " << metadata_msg->header.stamp.nanosec);

    int frame_emitter_mode{0};
    try {
      nlohmann::json json_data = nlohmann::json::parse(metadata_msg->json_data);

      right_ir_metadata_.actual_frame_rate = json_data["actual_frame_rate"];
      right_ir_metadata_.ae_roi_bottom = json_data["ae_roi_bottom"];
      right_ir_metadata_.ae_roi_left = json_data["ae_roi_left"];
      right_ir_metadata_.ae_roi_right = json_data["ae_roi_right"];
      right_ir_metadata_.ae_roi_top = json_data["ae_roi_top"];
      right_ir_metadata_.auto_exposure = json_data["auto_exposure"];
      right_ir_metadata_.exposure = json_data["exposure"];
      right_ir_metadata_.exposure_priority = json_data["exposure_priority"];
      right_ir_metadata_.frame_emitter_mode = json_data["frame_emitter_mode"];
      right_ir_metadata_.frame_laser_power = json_data["frame_laser_power"];
      right_ir_metadata_.frame_laser_power_mode = json_data["frame_laser_power_mode"];
      right_ir_metadata_.frame_number = json_data["frame_number"];
      right_ir_metadata_.frame_timestamp = json_data["frame_timestamp"];
      right_ir_metadata_.gain = json_data["gain"];
      right_ir_metadata_.gpio_input_data = json_data["gpio_input_data"];
      right_ir_metadata_.hdr_sequence_index = json_data["hdr_sequence_index"];
      right_ir_metadata_.hdr_sequence_name = json_data["hdr_sequence_name"];
      right_ir_metadata_.hdr_sequence_size = json_data["hdr_sequence_size"];
      right_ir_metadata_.sensor_timestamp = json_data["sensor_timestamp"];

      frame_emitter_mode = right_ir_metadata_.frame_emitter_mode;

      // RCLCPP_INFO_STREAM(get_logger(), "Right IR metadata: \n" << right_ir_metadata_);

      save_metadata_to_file(right_ir_metadata_, "irright");
    } catch (const nlohmann::json::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse metadata JSON: %s", e.what());
    }

    try {
      save_image_to_file(image_msg, "irright", right_ir_image_count_, image_msg->header.stamp,
                         frame_emitter_mode);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error saving image: " << e.what());
      return;
    }
    right_ir_image_count_++;
  }
  void depth_metadata_sync_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
      const orbbec_camera_msgs::msg::Metadata::ConstSharedPtr &metadata_msg) {
    // RCLCPP_INFO_STREAM(get_logger(),
    //                    "Depth stamp: sec " << image_msg->header.stamp.sec << " nanosec " <<
    //                    image_msg->header.stamp.nanosec);
    // RCLCPP_INFO_STREAM(get_logger(),
    //                    "Depth metadata stamp: sec " << metadata_msg->header.stamp.sec << "
    //                    nanosec " << metadata_msg->header.stamp.nanosec);
    int frame_emitter_mode{0};
    try {
      nlohmann::json json_data = nlohmann::json::parse(metadata_msg->json_data);

      depth_metadata_.actual_frame_rate = json_data["actual_frame_rate"];
      depth_metadata_.ae_roi_bottom = json_data["ae_roi_bottom"];
      depth_metadata_.ae_roi_left = json_data["ae_roi_left"];
      depth_metadata_.ae_roi_right = json_data["ae_roi_right"];
      depth_metadata_.ae_roi_top = json_data["ae_roi_top"];
      depth_metadata_.auto_exposure = json_data["auto_exposure"];
      depth_metadata_.exposure = json_data["exposure"];
      depth_metadata_.exposure_priority = json_data["exposure_priority"];
      depth_metadata_.frame_emitter_mode = json_data["frame_emitter_mode"];
      depth_metadata_.frame_laser_power = json_data["frame_laser_power"];
      depth_metadata_.frame_laser_power_mode = json_data["frame_laser_power_mode"];
      depth_metadata_.frame_number = json_data["frame_number"];
      depth_metadata_.frame_timestamp = json_data["frame_timestamp"];
      depth_metadata_.gain = json_data["gain"];
      depth_metadata_.gpio_input_data = json_data["gpio_input_data"];
      depth_metadata_.hdr_sequence_index = json_data["hdr_sequence_index"];
      depth_metadata_.hdr_sequence_name = json_data["hdr_sequence_name"];
      depth_metadata_.hdr_sequence_size = json_data["hdr_sequence_size"];
      depth_metadata_.sensor_timestamp = json_data["sensor_timestamp"];

      frame_emitter_mode = depth_metadata_.frame_emitter_mode;

      // RCLCPP_INFO_STREAM(get_logger(), "Depth metadata: \n" << depth_metadata_);

      save_metadata_to_file(depth_metadata_, "depth");
    } catch (const nlohmann::json::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse metadata JSON: %s", e.what());
    }

    try {
      save_image_to_file(image_msg, "depth", depth_image_count_, image_msg->header.stamp,
                         frame_emitter_mode);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error saving image: " << e.what());
      return;
    }
    depth_image_count_++;
  }

  void createDirectory(const std::string &path) {
    RCLCPP_INFO(this->get_logger(), "Creating directory: %s", path.c_str());
    std::filesystem::path dir_path(path);
    if (!std::filesystem::exists(dir_path)) {
      if (std::filesystem::create_directories(dir_path)) {
        std::cout << "Directory created: " << path << std::endl;
      } else {
        std::cerr << "Failed to create directory: " << path << std::endl;
      }
    } else {
      std::cout << "Directory already exists: " << path << std::endl;
    }
  }
  void save_image_to_file(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                          const std::string &prefix, int count,
                          const builtin_interfaces::msg::Time &stamp, int frame_emitter_mode) {
    (void)count;
    long long timestamp_us = static_cast<long long>(stamp.sec) * 1000000LL + stamp.nanosec / 1000LL;
    std::string serial_path{};
    cv_bridge::CvImagePtr cv_ptr;

    if ((prefix == "irleft") || (prefix == "irright")) {
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    } else if (prefix == "depth") {
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown prefix: %s", prefix.c_str());
      return;
    }

    serial_path = frame_emitter_mode == 0 ? serial_path_zero_ : serial_path_one_;

    std::ostringstream ss;
    // ss << serial_path << "/" << prefix << "_" << timestamp_us << ".png";
    ss << serial_path << "/" << timestamp_us << "_" << prefix << ".png";

    cv::imwrite(ss.str(), cv_ptr->image);
  }
  void save_metadata_to_file(const ImageMetadata &metadata, const std::string &prefix) {
    std::ostringstream ss;
    ss << (metadata.frame_emitter_mode == 0 ? serial_path_zero_ : serial_path_one_) << "/"
       << metadata.frame_timestamp << "_" << prefix << "_e" << metadata.exposure << "_g"
       << metadata.gain << ".txt";

    std::string file_path = ss.str();
    // RCLCPP_INFO(get_logger(), "Saving metadata to file: %s", file_path.c_str());

    std::ofstream outfile(file_path);
    if (outfile.is_open()) {
      outfile << "Frame Emitter Mode: " << metadata.frame_emitter_mode << "\n";
      outfile << "Exposure: " << metadata.exposure << "\n";
      outfile << "Gain: " << metadata.gain << "\n";
      outfile << "Hdr Sequence Index: " << metadata.hdr_sequence_index << "\n";
      outfile << "Frame Timestamp: " << metadata.frame_timestamp << "\n";
      outfile << "Sensor Timestamp: " << metadata.sensor_timestamp << "\n";
      outfile << "Frame Number: " << metadata.frame_number << "\n";

      outfile.close();
      // RCLCPP_INFO(get_logger(), "Metadata successfully saved to file.");
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_path.c_str());
    }
  }

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_ir_image_sub_;
  std::shared_ptr<message_filters::Subscriber<orbbec_camera_msgs::msg::Metadata>>
      left_ir_metadata_sub_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_ir_image_sub_;
  std::shared_ptr<message_filters::Subscriber<orbbec_camera_msgs::msg::Metadata>>
      right_ir_metadata_sub_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_image_sub_;
  std::shared_ptr<message_filters::Subscriber<orbbec_camera_msgs::msg::Metadata>>
      depth_metadata_sub_;

  std::string left_ir_image_topic_;
  std::string right_ir_image_topic_;
  std::string depth_image_topic_;
  std::string left_ir_metadata_topic_;
  std::string right_ir_metadata_topic_;
  std::string depth_metadata_topic_;

  int left_ir_image_count_ = 0;
  int right_ir_image_count_ = 0;
  int depth_image_count_ = 0;
  int left_ir_metadata_count_ = 0;
  int right_ir_metadata_count_ = 0;
  int depth_metadata_count_ = 0;

  bool directories_initialized_ = false;

  std::string serial_;
  std::string serial_path_zero_;
  std::string serial_path_one_;

  ImageMetadata left_ir_metadata_ = ImageMetadata();
  ImageMetadata right_ir_metadata_ = ImageMetadata();
  ImageMetadata depth_metadata_ = ImageMetadata();

  using MySyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      orbbec_camera_msgs::msg::Metadata>;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> left_ir_sync_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> right_ir_sync_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> depth_sync_;
};

}  // namespace tools
}  // namespace orbbec_camera
