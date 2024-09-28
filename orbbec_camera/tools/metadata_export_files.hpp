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
#include <cv_bridge/cv_bridge.h>
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
  int exposure;
  int frame_emitter_mode;
  int frame_number;
  int64_t frame_timestamp;
  int gain;
  int64_t sensor_timestamp;

  ImageMetadata()
      : exposure(0),
        frame_emitter_mode(0),
        frame_number(0),
        frame_timestamp(0),
        gain(0),
        sensor_timestamp(0) {}
  friend std::ostream &operator<<(std::ostream &os, const ImageMetadata &metadata) {
    os << "exposure: " << metadata.exposure << "\n"
       << "frame_emitter_mode: " << metadata.frame_emitter_mode
       << "frame_number: " << metadata.frame_number << "\n"
       << "frame_timestamp: " << metadata.frame_timestamp << "\n"
       << "gain: " << metadata.gain << "\n"
       << "sensor_timestamp: " << metadata.sensor_timestamp;
    return os;
  }
};

using std::placeholders::_1;
using std::placeholders::_2;

class MetadataExportFiles : public rclcpp::Node {
 public:
  MetadataExportFiles() : Node("metadata_export_files") {
    load_parameters();
    initialize_directories();
    initialize_pub_sub();
  }

  void load_parameters() {
<<<<<<< HEAD
    std::ifstream file("src/OrbbecSDK_ROS2/orbbec_camera/config/metadataexport/metadata_export_params.json");
=======
    std::ifstream file("src/OrbbecSDK_ROS2/orbbec_camera/config/metadata_export_params.json");
>>>>>>> e2a2e1a47ad1fe03adce9baec868053e3fe5e4ff
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file.");
      return;
    }

    nlohmann::json json_data;
    file >> json_data;

    sn_ = json_data["camera_params"]["sn"].get<std::string>();

    left_ir_image_topic_ = json_data["camera_params"]["left_ir_image_topic"].get<std::string>();
    right_ir_image_topic_ = json_data["camera_params"]["right_ir_image_topic"].get<std::string>();
    depth_image_topic_ = json_data["camera_params"]["depth_image_topic"].get<std::string>();
    color_image_topic_ = json_data["camera_params"]["color_image_topic"].get<std::string>();

    left_ir_metadata_topic_ =
        json_data["camera_params"]["left_ir_metadata_topic"].get<std::string>();
    right_ir_metadata_topic_ =
        json_data["camera_params"]["right_ir_metadata_topic"].get<std::string>();
    depth_metadata_topic_ = json_data["camera_params"]["depth_metadata_topic"].get<std::string>();
    color_metadata_topic_ = json_data["camera_params"]["color_metadata_topic"].get<std::string>();

    RCLCPP_INFO(this->get_logger(), "Parameter 1: %s", sn_.c_str());
    RCLCPP_INFO(this->get_logger(), "Parameter 2: %s", depth_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Parameter 3: %s", color_image_topic_.c_str());
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
    left_ir_sync_->setMaxIntervalDuration(
        rclcpp::Duration::from_nanoseconds(100000000LL));  // 100 ms

    left_ir_sync_->registerCallback(
        std::bind(&MetadataExportFiles::left_ir_metadata_sync_callback, this, _1, _2));

    right_ir_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, right_ir_image_topic_, qos_filters);
    right_ir_metadata_sub_ =
        std::make_shared<message_filters::Subscriber<orbbec_camera_msgs::msg::Metadata>>(
            this, right_ir_metadata_topic_, qos_filters);

    right_ir_sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
        MySyncPolicy(10), *right_ir_image_sub_, *right_ir_metadata_sub_);
    right_ir_sync_->setMaxIntervalDuration(
        rclcpp::Duration::from_nanoseconds(100000000LL));  // 100 ms

    right_ir_sync_->registerCallback(
        std::bind(&MetadataExportFiles::right_ir_metadata_sync_callback, this, _1, _2));

    depth_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, depth_image_topic_, qos_filters);
    depth_metadata_sub_ =
        std::make_shared<message_filters::Subscriber<orbbec_camera_msgs::msg::Metadata>>(
            this, depth_metadata_topic_, qos_filters);

    depth_sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
        MySyncPolicy(10), *depth_image_sub_, *depth_metadata_sub_);
    depth_sync_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(100000000LL));  // 100 ms

    depth_sync_->registerCallback(
        std::bind(&MetadataExportFiles::depth_metadata_sync_callback, this, _1, _2));

    color_image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, color_image_topic_, qos_filters);
    color_metadata_sub_ =
        std::make_shared<message_filters::Subscriber<orbbec_camera_msgs::msg::Metadata>>(
            this, color_metadata_topic_, qos_filters);

    color_sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
        MySyncPolicy(10), *color_image_sub_, *color_metadata_sub_);
    color_sync_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(100000000LL));  // 100 ms

    color_sync_->registerCallback(
        std::bind(&MetadataExportFiles::color_metadata_sync_callback, this, _1, _2));

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
        // serial_ = device_info->serialNumber();
        std::string uid = device_info->uid();
        auto usb_port = orbbec_camera::parseUsbPort(uid);
        // RCLCPP_INFO_STREAM(get_logger(), "serial: " << serial_);
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
    serial_path_left_ir_ = cwd.string() + "/" + "sn_" + sn_ + "/IR_LEFT";
    serial_path_right_ir_ = cwd.string() + "/" + "sn_" + sn_ + "/IR_RIGHT";
    serial_path_color_ = cwd.string() + "/" + "sn_" + sn_ + "/Color";
    serial_path_depth_ = cwd.string() + "/" + "sn_" + sn_ + "/Depth";
    createDirectory(serial_path_left_ir_);
    createDirectory(serial_path_right_ir_);
    createDirectory(serial_path_color_);
    createDirectory(serial_path_depth_);
  }

 private:
  void left_ir_metadata_sync_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
      const orbbec_camera_msgs::msg::Metadata::ConstSharedPtr &metadata_msg) {
    try {
      nlohmann::json json_data = nlohmann::json::parse(metadata_msg->json_data);

      left_ir_metadata_.exposure = json_data["exposure"];
      left_ir_metadata_.frame_emitter_mode = json_data["frame_emitter_mode"];
      left_ir_metadata_.frame_number = json_data["frame_number"];
      left_ir_metadata_.frame_timestamp = json_data["frame_timestamp"];
      left_ir_metadata_.gain = json_data["gain"];
      left_ir_metadata_.sensor_timestamp = json_data["sensor_timestamp"];

      save_metadata_to_file(left_ir_metadata_, "irleft");
    } catch (const nlohmann::json::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse metadata JSON: %s", e.what());
    }
    if (image_msg) {
      leftir_frame_index_++;
    }
    try {
      save_image_to_file(image_msg, "irleft", left_ir_image_count_, image_msg->header.stamp,
                         leftir_frame_index_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error saving image: " << e.what());
      return;
    }
    left_ir_image_count_++;
  }
  void right_ir_metadata_sync_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
      const orbbec_camera_msgs::msg::Metadata::ConstSharedPtr &metadata_msg) {
    try {
      nlohmann::json json_data = nlohmann::json::parse(metadata_msg->json_data);

      right_ir_metadata_.exposure = json_data["exposure"];
      right_ir_metadata_.frame_emitter_mode = json_data["frame_emitter_mode"];
      right_ir_metadata_.frame_number = json_data["frame_number"];
      right_ir_metadata_.frame_timestamp = json_data["frame_timestamp"];
      right_ir_metadata_.gain = json_data["gain"];
      right_ir_metadata_.sensor_timestamp = json_data["sensor_timestamp"];

      save_metadata_to_file(right_ir_metadata_, "irright");
    } catch (const nlohmann::json::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse metadata JSON: %s", e.what());
    }
    if (image_msg) {
      rightir_frame_index_++;
    }
    try {
      save_image_to_file(image_msg, "irright", right_ir_image_count_, image_msg->header.stamp,
                         rightir_frame_index_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error saving image: " << e.what());
      return;
    }
    right_ir_image_count_++;
  }
  void depth_metadata_sync_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
      const orbbec_camera_msgs::msg::Metadata::ConstSharedPtr &metadata_msg) {
    try {
      nlohmann::json json_data = nlohmann::json::parse(metadata_msg->json_data);

      depth_metadata_.exposure = json_data["exposure"];
      depth_metadata_.frame_emitter_mode = json_data["frame_emitter_mode"];
      depth_metadata_.frame_number = json_data["frame_number"];
      depth_metadata_.frame_timestamp = json_data["frame_timestamp"];
      depth_metadata_.gain = json_data["gain"];
      depth_metadata_.sensor_timestamp = json_data["sensor_timestamp"];

      save_metadata_to_file(depth_metadata_, "depth");
    } catch (const nlohmann::json::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse metadata JSON: %s", e.what());
    }
    if (image_msg) {
      depth_frame_index_++;
    }
    try {
      save_image_to_file(image_msg, "depth", depth_image_count_, image_msg->header.stamp,
                         depth_frame_index_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error saving image: " << e.what());
      return;
    }
    depth_image_count_++;
  }

  void color_metadata_sync_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
      const orbbec_camera_msgs::msg::Metadata::ConstSharedPtr &metadata_msg) {
    try {
      nlohmann::json json_data = nlohmann::json::parse(metadata_msg->json_data);

      color_metadata_.exposure = json_data["exposure"];
      color_metadata_.frame_number = json_data["frame_number"];
      color_metadata_.frame_timestamp = json_data["frame_timestamp"];
      color_metadata_.gain = json_data["gain"];
      color_metadata_.sensor_timestamp = json_data["sensor_timestamp"];

      save_metadata_to_file(color_metadata_, "color");
    } catch (const nlohmann::json::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse metadata JSON: %s", e.what());
    }
    if (image_msg) {
      color_frame_index_++;
    }
    try {
      save_image_to_file(image_msg, "color", color_image_count_, image_msg->header.stamp,
                         color_frame_index_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Error saving image: " << e.what());
      return;
    }
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
                          const builtin_interfaces::msg::Time &stamp, int frame_index) {
    (void)count;
    long long camera_timestamp_us =
        static_cast<long long>(stamp.sec) * 1000000LL + stamp.nanosec / 1000LL;
    auto now = this->get_clock()->now();
    int64_t seconds = now.seconds();
    int64_t nanoseconds = now.nanoseconds() % 1000000000;
    int64_t microseconds = nanoseconds / 1000;
    std::ostringstream timestamp_us;
    timestamp_us << seconds << std::setw(3) << std::setfill('0') << microseconds;
    std::string resolution = std::to_string(msg->width) + "x" + std::to_string(msg->height);
    std::string serial_path{};
    cv_bridge::CvImagePtr cv_ptr;
    if (prefix == "irleft") {
      try {
        serial_path = serial_path_left_ir_;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    } else if (prefix == "irright") {
      try {
        serial_path = serial_path_right_ir_;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    } else if (prefix == "depth") {
      try {
        serial_path = serial_path_depth_;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    } else if (prefix == "color") {
      try {
        serial_path = serial_path_color_;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown prefix: %s", prefix.c_str());
      return;
    }

    std::ostringstream ss;
    ss << serial_path << "/" << std::to_string(frame_index) << "_" << "0000" << "_"
       << camera_timestamp_us << "_" << timestamp_us.str() << "_" << prefix << "_" << resolution
       << ".png";
    cv::imwrite(ss.str(), cv_ptr->image);
  }
  void save_metadata_to_file(const ImageMetadata &metadata, const std::string &prefix) {
    std::ostringstream ss;
    std::string serial_path{};
    if (prefix == "irleft") {
      try {
        serial_path = serial_path_left_ir_;
      } catch (cv_bridge::Exception &e) {
        return;
      }
    } else if (prefix == "irright") {
      try {
        serial_path = serial_path_right_ir_;
      } catch (cv_bridge::Exception &e) {
        return;
      }
    } else if (prefix == "depth") {
      try {
        serial_path = serial_path_depth_;
      } catch (cv_bridge::Exception &e) {
        return;
      }
    } else if (prefix == "color") {
      try {
        serial_path = serial_path_color_;
      } catch (cv_bridge::Exception &e) {
        return;
      }
    }
    ss << serial_path << "/" << metadata.frame_timestamp << "_" << prefix << "_e"
       << metadata.exposure << "_g" << metadata.gain << ".txt";
    std::string file_path = ss.str();

    std::ofstream outfile(file_path);
    if (outfile.is_open()) {
      if (prefix != "color") {
        outfile << "Frame Emitter Mode: " << metadata.frame_emitter_mode << "\n";
      } else {
        outfile << "Frame Emitter Mode: " << 0 << "\n";
      }
      outfile << "Exposure: " << metadata.exposure << "\n";
      outfile << "Gain: " << metadata.gain << "\n";
      outfile << "Frame Timestamp: " << metadata.frame_timestamp << "\n";
      outfile << "Sensor Timestamp: " << metadata.sensor_timestamp << "\n";
      outfile << "Frame Number: " << metadata.frame_number << "\n";

      outfile.close();
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

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> color_image_sub_;
  std::shared_ptr<message_filters::Subscriber<orbbec_camera_msgs::msg::Metadata>>
      color_metadata_sub_;

  std::string left_ir_image_topic_;
  std::string right_ir_image_topic_;
  std::string depth_image_topic_;
  std::string color_image_topic_;
  std::string left_ir_metadata_topic_;
  std::string right_ir_metadata_topic_;
  std::string depth_metadata_topic_;
  std::string color_metadata_topic_;
  std::string sn_;

  int left_ir_image_count_ = 0;
  int right_ir_image_count_ = 0;
  int depth_image_count_ = 0;
  int color_image_count_ = 0;
  int left_ir_metadata_count_ = 0;
  int right_ir_metadata_count_ = 0;
  int depth_metadata_count_ = 0;
  int color_metadata_count_ = 0;
  int leftir_frame_index_ = 0;
  int rightir_frame_index_ = 0;
  int depth_frame_index_ = 0;
  int color_frame_index_ = 0;

  bool directories_initialized_ = false;

  std::string serial_path_left_ir_;
  std::string serial_path_right_ir_;
  std::string serial_path_color_;
  std::string serial_path_depth_;

  ImageMetadata left_ir_metadata_ = ImageMetadata();
  ImageMetadata right_ir_metadata_ = ImageMetadata();
  ImageMetadata depth_metadata_ = ImageMetadata();
  ImageMetadata color_metadata_ = ImageMetadata();

  using MySyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      orbbec_camera_msgs::msg::Metadata>;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> left_ir_sync_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> right_ir_sync_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> depth_sync_;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> color_sync_;
};

}  // namespace tools
}  // namespace orbbec_camera
