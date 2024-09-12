#pragma once
#include <rclcpp/rclcpp.hpp>

#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class MultiCameraSubscriber : public rclcpp::Node {
 public:
  MultiCameraSubscriber() : Node("multi_camera_subscriber") {
    // 初始化 context，获取设备信息
    auto context = std::make_unique<ob::Context>();
    context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
    auto list = context->queryDeviceList();
    for (size_t i = 0; i < list->deviceCount(); i++) {
      auto device = list->getDevice(i);
      auto device_info = device->getDeviceInfo();
      std::string serial = device_info->serialNumber();
      int vid = device_info->vid();
      int pid = device_info->pid();
      serial_numbers[count] = serial;
      count++;
      RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "vid: " << std::hex << vid);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "pid: " << std::hex << pid);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "serial: " << serial);
    }

    // 声明 ir 和 color 图像话题参数
    this->declare_parameter<std::vector<std::string>>("ir_topics", std::vector<std::string>());
    this->declare_parameter<std::vector<std::string>>("color_topics", std::vector<std::string>());

    std::vector<std::string> ir_topics = this->get_parameter("ir_topics").as_string_array();
    std::vector<std::string> color_topics = this->get_parameter("color_topics").as_string_array();

    rclcpp::QoS custom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 动态创建 ir 和 color 订阅器
    for (size_t i = 0; i < ir_topics.size(); ++i) {
      auto ir_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          this, ir_topics[i], custom_qos.get_rmw_qos_profile());
      auto color_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          this, color_topics[i], custom_qos.get_rmw_qos_profile());
      ir_subscribers.push_back(ir_sub);
      color_subscribers.push_back(color_sub);
    }

    // 动态生成同步器
    setupSynchronizer(ir_topics.size());
  }

 private:
  std::string generateFolderName(const sensor_msgs::msg::Image::ConstSharedPtr &ir_msg,
                                 const sensor_msgs::msg::Image::ConstSharedPtr &color_msg,
                                 const std::string &serial_number, size_t serial_index) {
    std::string ir_resolution =
        std::to_string(ir_msg->width) + "x" + std::to_string(ir_msg->height);
    std::string ir_encoding = ir_msg->encoding;

    std::string color_resolution =
        std::to_string(color_msg->width) + "x" + std::to_string(color_msg->height);
    std::string color_encoding = color_msg->encoding;

    // 假设帧率为 60fps（可以根据实际情况修改）
    std::string frame_rate = "60fps";

    // 生成符合格式的文件夹名称
    std::string folder_name = "Star-AE-OFF-ir-" + ir_resolution + "-" + ir_encoding + "-rgb-" +
                              color_resolution + "-" + color_encoding + "-" + frame_rate;

    // 创建文件夹
    std::string path = std::string("multicamera_sync/output/") + folder_name + "/" +
                       "TotalModeFrames/" + "/" + "SN" + serial_number + "_Index" +
                       std::to_string(serial_index);
    std::filesystem::create_directories(path);
    return path;
  }
  std::string getCurrentTimestamp() {
    auto now = this->get_clock()->now();
    int64_t seconds = now.seconds();
    int64_t nanoseconds = now.nanoseconds() % 1000000000;
    int64_t milliseconds = nanoseconds / 100000;
    return std::to_string(seconds) + std::to_string(milliseconds);
  }
  // 根据传入的图像话题数量动态生成同步器
  // 根据传入的图像话题数量动态生成同步器
  void setupSynchronizer(size_t num_topics) {
    if (num_topics == 2) {
      sync_2 = std::make_shared<message_filters::Synchronizer<SyncPolicy2>>(
          SyncPolicy2(10), *ir_subscribers[0], *color_subscribers[0], *ir_subscribers[1],
          *color_subscribers[1]);
      sync_2->registerCallback(std::bind(static_cast<void (MultiCameraSubscriber::*)(
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &)>(
                                             &MultiCameraSubscriber::imageCallback2),
                                         this, std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4));
    } else if (num_topics == 3) {
      sync_3 = std::make_shared<message_filters::Synchronizer<SyncPolicy3>>(
          SyncPolicy3(10), *ir_subscribers[0], *color_subscribers[0], *ir_subscribers[1],
          *color_subscribers[1], *ir_subscribers[2], *color_subscribers[2]);
      sync_3->registerCallback(std::bind(static_cast<void (MultiCameraSubscriber::*)(
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &)>(
                                             &MultiCameraSubscriber::imageCallback3),
                                         this, std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4,
                                         std::placeholders::_5, std::placeholders::_6));
    } else if (num_topics == 4) {
      sync_4 = std::make_shared<message_filters::Synchronizer<SyncPolicy4>>(
          SyncPolicy4(10), *ir_subscribers[0], *color_subscribers[0], *ir_subscribers[1],
          *color_subscribers[1], *ir_subscribers[2], *color_subscribers[2], *ir_subscribers[3],
          *color_subscribers[3]);
      sync_4->registerCallback(std::bind(static_cast<void (MultiCameraSubscriber::*)(
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &,
                                             const sensor_msgs::msg::Image::ConstSharedPtr &)>(
                                             &MultiCameraSubscriber::imageCallback4),
                                         this, std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4,
                                         std::placeholders::_5, std::placeholders::_6,
                                         std::placeholders::_7, std::placeholders::_8));
    }
  }
  // 处理  ir 和  color 图像的回调函数
  void imageCallback2(const sensor_msgs::msg::Image::ConstSharedPtr &ir_msg0,
                      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg0,
                      const sensor_msgs::msg::Image::ConstSharedPtr &ir_msg1,
                      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg1) {
    imageCallback({ir_msg0, color_msg0, ir_msg1, color_msg1});
  }

  void imageCallback3(const sensor_msgs::msg::Image::ConstSharedPtr &ir_msg0,
                      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg0,
                      const sensor_msgs::msg::Image::ConstSharedPtr &ir_msg1,
                      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg1,
                      const sensor_msgs::msg::Image::ConstSharedPtr &ir_msg2,
                      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg2) {
    imageCallback({ir_msg0, color_msg0, ir_msg1, color_msg1, ir_msg2, color_msg2});
  }
  void imageCallback4(const sensor_msgs::msg::Image::ConstSharedPtr &ir_msg0,
                      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg0,
                      const sensor_msgs::msg::Image::ConstSharedPtr &ir_msg1,
                      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg1,
                      const sensor_msgs::msg::Image::ConstSharedPtr &ir_msg2,
                      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg2,
                      const sensor_msgs::msg::Image::ConstSharedPtr &ir_msg3,
                      const sensor_msgs::msg::Image::ConstSharedPtr &color_msg3) {
    imageCallback(
        {ir_msg0, color_msg0, ir_msg1, color_msg1, ir_msg2, color_msg2, ir_msg3, color_msg3});
  }

  // 同时处理多个 ir 和 color 图像，并将其保存到文件
  void imageCallback(const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> &images) {
    try {
      for (size_t i = 0; i < images.size(); i += 2) {
        auto ir_image = images[i];
        auto color_image = images[i + 1];

        cv::Mat ir_mat = cv_bridge::toCvCopy(ir_image, ir_image->encoding)->image;
        cv::Mat color_mat = cv_bridge::toCvCopy(color_image, color_image->encoding)->image;
        cv::Mat corrected_image;
        cv::cvtColor(color_mat, corrected_image, cv::COLOR_RGB2BGR);
        std::string folder =
            generateFolderName(ir_image, color_image, serial_numbers[i / 2], i / 2);
        std::string filename_ir = folder + "/ir#left_SN" + serial_numbers[i / 2] + "_Index" +
                                  std::to_string(i / 2) + "_d" + getCurrentTimestamp() + "_.jpg";
        std::string filename_color = folder + "/color_SN" + serial_numbers[i / 2] + "_Index" +
                                     std::to_string(i / 2) + "_d" + getCurrentTimestamp() + "_.jpg";

        // 保存 ir 和 color 图像
        cv::imwrite(filename_ir, ir_mat);
        cv::imwrite(filename_color, corrected_image);

        RCLCPP_INFO(this->get_logger(), "Saved IR and Color images for camera %zu to: %s, %s",
                    i / 2, filename_ir.c_str(), filename_color.c_str());
      }

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  // 存储 ir 和 color 订阅器
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> ir_subscribers;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>>
      color_subscribers;

  // 为不同数量的话题生成同步策略
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>
      SyncPolicy2;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy2>> sync_2;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      SyncPolicy3;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy3>> sync_3;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      SyncPolicy4;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy4>> sync_4;

  std::array<std::string, 10> serial_numbers;
  size_t count = 0;
};