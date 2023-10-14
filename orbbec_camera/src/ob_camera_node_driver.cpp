/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include "orbbec_camera/ob_camera_node_driver.h"
#include <fcntl.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <csignal>
#include <sys/mman.h>

namespace orbbec_camera {
OBCameraNodeDriver::OBCameraNodeDriver(const rclcpp::NodeOptions &node_options)
    : Node("orbbec_camera_node", "/", node_options),
      config_path_(ament_index_cpp::get_package_share_directory("orbbec_camera") +
                   "/config/OrbbecSDKConfig_v1.0.xml"),
      ctx_(std::make_unique<ob::Context>(config_path_.c_str())),
      logger_(this->get_logger()) {
  init();
}

OBCameraNodeDriver::OBCameraNodeDriver(const std::string &node_name, const std::string &ns,
                                       const rclcpp::NodeOptions &node_options)
    : Node(node_name, ns, node_options),
      ctx_(std::make_unique<ob::Context>()),
      logger_(this->get_logger()) {
  init();
}

OBCameraNodeDriver::~OBCameraNodeDriver() {
  is_alive_.store(false);
  if (device_count_update_thread_ && device_count_update_thread_->joinable()) {
    device_count_update_thread_->join();
  }
  if (sync_time_thread_ && sync_time_thread_->joinable()) {
    sync_time_thread_->join();
  }
  if (query_thread_ && query_thread_->joinable()) {
    query_thread_->join();
  }
  if (reset_device_thread_ && reset_device_thread_->joinable()) {
    reset_device_cond_.notify_all();
    reset_device_thread_->join();
  }
}

void OBCameraNodeDriver::init() {
  auto log_level_str = declare_parameter<std::string>("log_level", "none");
  auto log_level = obLogSeverityFromString(log_level_str);
  ob::Context::setLoggerSeverity(log_level);
  orb_device_lock_shm_fd_ = shm_open(ORB_DEFAULT_LOCK_NAME.c_str(), O_CREAT | O_RDWR, 0666);
  if (orb_device_lock_shm_fd_ < 0) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to open shared memory " << ORB_DEFAULT_LOCK_NAME);
    return;
  }
  int ret = ftruncate(orb_device_lock_shm_fd_, sizeof(pthread_mutex_t));
  if (ret < 0) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to truncate shared memory " << ORB_DEFAULT_LOCK_NAME);
    return;
  }
  orb_device_lock_shm_addr_ =
      static_cast<uint8_t *>(mmap(NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED,
                                  orb_device_lock_shm_fd_, 0));
  if (orb_device_lock_shm_addr_ == MAP_FAILED) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to map shared memory " << ORB_DEFAULT_LOCK_NAME);
    return;
  }
  pthread_mutexattr_init(&orb_device_lock_attr_);
  pthread_mutexattr_setpshared(&orb_device_lock_attr_, PTHREAD_PROCESS_SHARED);
  orb_device_lock_ = (pthread_mutex_t *)orb_device_lock_shm_addr_;
  pthread_mutex_init(orb_device_lock_, &orb_device_lock_attr_);
  is_alive_.store(true);
  parameters_ = std::make_shared<Parameters>(this);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  device_num_ = static_cast<int>(declare_parameter<int>("device_num", 1));
  usb_port_ = declare_parameter<std::string>("usb_port", "");
  auto enumerate_net_device_ = declare_parameter<bool>("enumerate_net_device", "false");
  ctx_->enableNetDeviceEnumeration(enumerate_net_device_);
  ctx_->setDeviceChangedCallback([this](const std::shared_ptr<ob::DeviceList> &removed_list,
                                        const std::shared_ptr<ob::DeviceList> &added_list) {
    onDeviceConnected(added_list);
    onDeviceDisconnected(removed_list);
  });
  check_connect_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { checkConnectTimer(); });
  CHECK_NOTNULL(check_connect_timer_);
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
  sync_time_thread_ = std::make_shared<std::thread>([this]() { syncTime(); });
  reset_device_thread_ = std::make_shared<std::thread>([this]() { resetDevice(); });
}

void OBCameraNodeDriver::onDeviceConnected(const std::shared_ptr<ob::DeviceList> &device_list) {
  CHECK_NOTNULL(device_list);
  if (device_list->deviceCount() == 0) {
    return;
  }
  if (!device_) {
    startDevice(device_list);
  }
}

void OBCameraNodeDriver::onDeviceDisconnected(const std::shared_ptr<ob::DeviceList> &device_list) {
  CHECK_NOTNULL(device_list);
  if (device_list->deviceCount() == 0) {
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "onDeviceDisconnected");
  for (size_t i = 0; i < device_list->deviceCount(); i++) {
    std::string uid = device_list->uid(i);
    std::string serial_number = device_list->serialNumber(i);
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    RCLCPP_INFO_STREAM(logger_, "device with " << uid << " disconnected");
    if (uid == device_unique_id_ || serial_number_ == serial_number) {
      RCLCPP_INFO_STREAM(logger_,
                         "device with " << uid << " disconnected, notify reset device thread.");
      std::unique_lock<decltype(reset_device_mutex_)> reset_device_lock(reset_device_mutex_);
      reset_device_flag_ = true;
      reset_device_cond_.notify_all();
      break;
    }
  }
}

OBLogSeverity OBCameraNodeDriver::obLogSeverityFromString(const std::string_view &log_level) {
  if (log_level == "debug") {
    return OBLogSeverity::OB_LOG_SEVERITY_DEBUG;
  } else if (log_level == "info") {
    return OBLogSeverity::OB_LOG_SEVERITY_INFO;
  } else if (log_level == "warn") {
    return OBLogSeverity::OB_LOG_SEVERITY_WARN;
  } else if (log_level == "error") {
    return OBLogSeverity::OB_LOG_SEVERITY_ERROR;
  } else if (log_level == "fatal") {
    return OBLogSeverity::OB_LOG_SEVERITY_FATAL;
  } else {
    return OBLogSeverity::OB_LOG_SEVERITY_NONE;
  }
}

void OBCameraNodeDriver::checkConnectTimer() {
  if (!device_connected_.load()) {
    RCLCPP_DEBUG_STREAM(logger_,
                        "checkConnectTimer: device " << serial_number_ << " not connected");
    return;
  } else if (!ob_camera_node_) {
    device_connected_.store(false);
  }
}

void OBCameraNodeDriver::queryDevice() {
  if (!device_connected_.load()) {
    auto device_list = ctx_->queryDeviceList();
    if (device_list->deviceCount() == 0) {
      RCLCPP_INFO_STREAM(logger_, "queryDevice : No device found");
      return;
    }
    startDevice(device_list);
  }
}

void OBCameraNodeDriver::syncTime() {
  while (is_alive_ && rclcpp::ok()) {
    if (device_ && device_info_ && !isOpenNIDevice(device_info_->pid())) {
      ctx_->enableDeviceClockSync(0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }
}

void OBCameraNodeDriver::resetDevice() {
  while (is_alive_ && rclcpp::ok()) {
    std::unique_lock<decltype(reset_device_mutex_)> lock(reset_device_mutex_);
    reset_device_cond_.wait(lock,
                            [this]() { return !is_alive_ || !rclcpp::ok() || reset_device_flag_; });
    if (!is_alive_ || !rclcpp::ok()) {
      break;
    }
    RCLCPP_INFO_STREAM(logger_, "resetDevice : Reset device uid: " << device_unique_id_);
    std::lock_guard<decltype(device_lock_)> device_lock(device_lock_);
    {
      ob_camera_node_.reset();
      device_.reset();
      device_info_.reset();
      device_connected_ = false;
      device_unique_id_.clear();
      serial_number_.clear();
      reset_device_flag_ = false;
    }
    RCLCPP_INFO_STREAM(logger_, "Reset device uid: " << device_unique_id_ << " done");
  }
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDevice(
    const std::shared_ptr<ob::DeviceList> &list) {
  if (device_num_ == 1) {
    RCLCPP_INFO_STREAM(logger_, "Connecting to the default device");
    return list->getDevice(0);
  }

  std::shared_ptr<ob::Device> device = nullptr;
  if (!serial_number_.empty()) {
    RCLCPP_INFO_STREAM(logger_, "Connecting to device with serial number: " << serial_number_);
    device = selectDeviceBySerialNumber(list, serial_number_);
  } else if (!usb_port_.empty()) {
    RCLCPP_INFO_STREAM(logger_, "Connecting to device with usb port: " << usb_port_);
    device = selectDeviceByUSBPort(list, usb_port_);
  }
  if (device == nullptr) {
    RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Device with serial number %s not found",
                         serial_number_.c_str());
    device_connected_ = false;
    return nullptr;
  }
  return device;
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDeviceBySerialNumber(
    const std::shared_ptr<ob::DeviceList> &list, const std::string &serial_number) {
  std::string lower_sn;
  std::transform(serial_number.begin(), serial_number.end(), std::back_inserter(lower_sn),
                 [](auto ch) { return isalpha(ch) ? tolower(ch) : static_cast<int>(ch); });
  for (size_t i = 0; i < list->deviceCount(); i++) {
    RCLCPP_INFO_STREAM(logger_, "Before lock: Select device serial number: " << serial_number);
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    RCLCPP_INFO_STREAM(logger_, "After lock: Select device serial number: " << serial_number);
    try {
      auto pid = list->pid(i);
      if (isOpenNIDevice(pid)) {
        // openNI device
        auto device = list->getDevice(i);
        auto device_info = device->getDeviceInfo();
        if (device_info->serialNumber() == serial_number) {
          RCLCPP_INFO_STREAM(logger_,
                             "Device serial number " << device_info->serialNumber() << " matched");
          return device;
        }
      } else {
        std::string sn = list->serialNumber(i);
        RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000, "Device serial number: " << sn);
        if (sn == serial_number) {
          RCLCPP_INFO_STREAM(logger_, "Device serial number " << sn << " matched");
          return list->getDevice(i);
        }
      }
    } catch (ob::Error &e) {
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 1000,
                                   "Failed to get device info " << e.getMessage());
    } catch (std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get device info " << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get device info");
    }
  }
  return nullptr;
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDeviceByUSBPort(
    const std::shared_ptr<ob::DeviceList> &list, const std::string &usb_port) {
  try {
    RCLCPP_INFO_STREAM(logger_, "Before lock: Select device usb port: " << usb_port);
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    RCLCPP_INFO_STREAM(logger_, "After lock: Select device usb port: " << usb_port);
    auto device = list->getDeviceByUid(usb_port.c_str());
    RCLCPP_INFO_STREAM(logger_, "Device usb port " << usb_port << " done");
    return device;
  } catch (ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to get device info " << e.getMessage());
  } catch (std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to get device info " << e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to get device info");
  }

  return nullptr;
}

void OBCameraNodeDriver::initializeDevice(const std::shared_ptr<ob::Device> &device) {
  device_ = device;
  CHECK_NOTNULL(device_);
  CHECK_NOTNULL(device_.get());
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_, parameters_);
  ob_camera_node_->startStreams();
  ob_camera_node_->startIMU();
  device_connected_ = true;
  device_info_ = device_->getDeviceInfo();
  serial_number_ = device_info_->serialNumber();
  CHECK_NOTNULL(device_info_.get());
  device_unique_id_ = device_info_->uid();
  if (!isOpenNIDevice(device_info_->pid())) {
    ctx_->enableDeviceClockSync(0);  // sync time stamp
  }
  RCLCPP_INFO_STREAM(logger_, "Device " << device_info_->name() << " connected");
  RCLCPP_INFO_STREAM(logger_, "Serial number: " << device_info_->serialNumber());
  RCLCPP_INFO_STREAM(logger_, "Firmware version: " << device_info_->firmwareVersion());
  RCLCPP_INFO_STREAM(logger_, "Hardware version: " << device_info_->hardwareVersion());
  RCLCPP_INFO_STREAM(logger_, "device unique id: " << device_unique_id_);
}

void OBCameraNodeDriver::startDevice(const std::shared_ptr<ob::DeviceList> &list) {
  if (device_connected_) {
    return;
  }
  if (list->deviceCount() == 0) {
    RCLCPP_WARN(logger_, "No device found");
    return;
  }
  if (device_) {
    device_.reset();
  }
  pthread_mutex_lock(orb_device_lock_);
  std::shared_ptr<int> lock_holder(nullptr,
                                   [this](int *) { pthread_mutex_unlock(orb_device_lock_); });
  bool start_device_failed = false;
  try {
    auto device = selectDevice(list);
    if (device == nullptr) {
      RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Device with serial number %s not found",
                           serial_number_.c_str());
      device_connected_ = false;
      return;
    }
    initializeDevice(device);
  } catch (ob::Error &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device " << e.getMessage());
    start_device_failed = true;
  } catch (std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device " << e.what());
    start_device_failed = true;
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device");
    start_device_failed = true;
  }
  if (start_device_failed) {
    device_connected_ = false;
    std::unique_lock<decltype(reset_device_mutex_)> reset_device_lock(reset_device_mutex_);
    reset_device_flag_ = true;
    reset_device_cond_.notify_all();
  }
}
}  // namespace orbbec_camera

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::OBCameraNodeDriver)
