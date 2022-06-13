/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2022 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#include "orbbec_camera/ob_camera_node_factory.h"
namespace orbbec_camera {
OBCameraNodeFactory::OBCameraNodeFactory(const rclcpp::NodeOptions &node_options)
    : Node("orbbec_camera_node", "/", node_options),
      ctx_(std::make_unique<ob::Context>()),
      logger_(this->get_logger()) {
  init();
}

OBCameraNodeFactory::OBCameraNodeFactory(const std::string &node_name, const std::string &ns,
                                         const rclcpp::NodeOptions &node_options)
    : Node(node_name, ns, node_options),
      ctx_(std::make_unique<ob::Context>()),
      logger_(this->get_logger()) {
  init();
}

OBCameraNodeFactory::~OBCameraNodeFactory() {
  is_alive_.store(false);
  if (query_thread_.joinable()) {
    query_thread_.join();
  }
}

void OBCameraNodeFactory::init() {
  ob_log_level_ = declare_parameter<std::string>("ob_log_level", "none");
  ctx_->setLoggerSeverity(obLogSeverityFromString(ob_log_level_));
  is_alive_.store(true);
  parameters_ = std::make_shared<Parameters>(this);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  wait_for_device_timeout_ = declare_parameter<double>("wait_for_device_timeout", 2.0);
  reconnect_timeout_ = declare_parameter<double>("reconnect_timeout", 2.0);
  query_thread_ = std::thread([=]() {
    std::chrono::milliseconds timespan(static_cast<int>(reconnect_timeout_ * 1e3));
    rclcpp::Time first_try_time = this->now();
    while (is_alive_ && !device_) {
      CHECK_NOTNULL(ctx_);
      auto list = ctx_->queryDeviceList();
      getDevice(list);
      if (device_) {
        ctx_->setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removed_list,
                                              std::shared_ptr<ob::DeviceList> added_list) {
          deviceDisconnectCallback(removed_list);
          deviceConnectCallback(added_list);
        });
        startDevice();
      } else {
        std::chrono::milliseconds actual_timespan(timespan);
        if (wait_for_device_timeout_ > 0) {
          auto time_to_timeout(wait_for_device_timeout_ -
                               (this->get_clock()->now() - first_try_time).seconds());
          if (time_to_timeout < 0) {
            RCLCPP_ERROR_STREAM(logger_, "wait for device timeout of " << wait_for_device_timeout_
                                                                       << " secs expired");
            exit(1);
          } else {
            double max_timespan_secs = static_cast<double>(
                std::chrono::duration_cast<std::chrono::seconds>(timespan).count());
            actual_timespan = std::chrono::milliseconds(
                static_cast<int>(std::min(max_timespan_secs, time_to_timeout) * 1e3));
          }
        }
        std::this_thread::sleep_for(actual_timespan);
      }
    }
  });
}

void OBCameraNodeFactory::deviceConnectCallback(
    const std::shared_ptr<ob::DeviceList> &device_list) {
  if (device_list->deviceCount() == 0) {
    return;
  }
  RCLCPP_ERROR_STREAM(logger_, "deviceConnectCallback");
  CHECK_NOTNULL(device_list);
  if (!device_) {
    getDevice(device_list);
    if (device_) {
      startDevice();
    } else {
    }
  }
}

void OBCameraNodeFactory::deviceDisconnectCallback(
    const std::shared_ptr<ob::DeviceList> &device_list) {
  if (device_list->deviceCount() == 0) {
    return;
  }
  RCLCPP_ERROR_STREAM(logger_, "deviceDisconnectCallback");
  CHECK_NOTNULL(device_list);
  try {
    for (size_t i = 0; i < device_list->deviceCount(); i++) {
      std::string serial_number = device_list->serialNumber(i);
      std::string device_serial_no = device_info_->serialNumber();
      if (serial_number == device_serial_no) {
        ob_camera_node_.reset(nullptr);
        device_.reset();
      }
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, e.getMessage());
  }
}

void OBCameraNodeFactory::printDeviceInfo(const std::shared_ptr<ob::DeviceInfo> &device_info) {
  RCLCPP_INFO_STREAM(logger_, "name " << device_info->name());
  RCLCPP_INFO_STREAM(logger_, "pid " << device_info->pid());
  RCLCPP_INFO_STREAM(logger_, "vid " << device_info->vid());
  RCLCPP_INFO_STREAM(logger_, "serial_name " << device_info->serialNumber());
  RCLCPP_INFO_STREAM(logger_, "firmware version " << device_info->firmwareVersion());
  RCLCPP_INFO_STREAM(logger_,
                     "supported min sdk version " << device_info->supportedMinSdkVersion());
  RCLCPP_INFO_STREAM(logger_, "hardware version " << device_info->hardwareVersion());
}

OBLogSeverity OBCameraNodeFactory::obLogSeverityFromString(const std::string &log_level) {
  if (log_level == "debug") {
    return OB_LOG_SEVERITY_DEBUG;
  } else if (log_level == "info") {
    return OB_LOG_SEVERITY_INFO;
  } else if (log_level == "warn" || log_level == "warning") {
    return OB_LOG_SEVERITY_WARN;
  } else if (log_level == "fatal" || log_level == "error") {
    return OB_LOG_SEVERITY_FATAL;
  } else {
    // Default None
    return OB_LOG_SEVERITY_NONE;
  }
}

void OBCameraNodeFactory::getDevice(const std::shared_ptr<ob::DeviceList> &list) {
  if (device_) {
    return;
  }
  if (0 == list->deviceCount()) {
    RCLCPP_WARN_STREAM(logger_, "No orbbec devices were found!");
    return;
  }
  if (serial_number_.empty()) {
    for (size_t i = 0; i < list->deviceCount(); i++) {
      auto dev = list->getDevice(i);
      if (dev != nullptr) {
        device_ = dev;
        printDeviceInfo(device_->getDeviceInfo());
        updateDeviceInfo();
      }
    }
  } else {
    device_ = list->getDeviceBySN(serial_number_.c_str());
    if (device_ == nullptr) {
      RCLCPP_ERROR_STREAM(logger_, "can not found device with SN " << serial_number_);
    } else {
      printDeviceInfo(device_->getDeviceInfo());
      updateDeviceInfo();
    }
  }
}

void OBCameraNodeFactory::updateDeviceInfo() {
  if (device_ == nullptr) {
    return;
  }
  device_info_ = device_->getDeviceInfo();
}

void OBCameraNodeFactory::startDevice() {
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  CHECK_NOTNULL(device_);
  ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_, parameters_);
}
}  // namespace orbbec_camera
