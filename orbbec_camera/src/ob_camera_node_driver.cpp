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

#include "orbbec_camera/ob_camera_node_driver.h"
#include <fcntl.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <signal.h>

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
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  if (int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT); shm_id != -1) {
    shmctl(shm_id, IPC_RMID, nullptr);
  }
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
  is_alive_.store(true);
  parameters_ = std::make_shared<Parameters>(this);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  device_num_ = static_cast<int>(declare_parameter<int>("device_num", 1));
  usb_port_ = declare_parameter<std::string>("usb_port", "");
  ctx_->setDeviceChangedCallback([this](const std::shared_ptr<ob::DeviceList> &removed_list,
                                        const std::shared_ptr<ob::DeviceList> &added_list) {
    (void)added_list;
    onDeviceDisconnected(removed_list);
  });
  check_connect_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { checkConnectTimer(); });
  CHECK_NOTNULL(check_connect_timer_);
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
  device_count_update_thread_ = std::make_shared<std::thread>([this]() { deviceCountUpdate(); });
  sync_time_thread_ = std::make_shared<std::thread>([this]() { syncTime(); });
  reset_device_thread_ = std::make_shared<std::thread>([this]() { resetDevice(); });
  signal(SIGINT, [](int) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("orbbec_camera_node_driver"), "SIGINT received");
    exit(0);
  });
  signal(SIGTERM, [](int) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("orbbec_camera_node_driver"), "SIGTERM received");
    exit(0);
  });

}

void OBCameraNodeDriver::onDeviceConnected(const std::shared_ptr<ob::DeviceList> &device_list) {
  CHECK_NOTNULL(device_list);
  if (device_list->deviceCount() == 0) {
    return;
  }
  RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000, "onDeviceConnected");
  if (!device_) {
    try {
      startDevice(device_list);
    } catch (ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_, "startDevice failed: " << e.getMessage());
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "startDevice failed: " << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "startDevice failed");
    }
  }
}

void OBCameraNodeDriver::onDeviceDisconnected(const std::shared_ptr<ob::DeviceList> &device_list) {
  CHECK_NOTNULL(device_list);
  if (device_list->deviceCount() == 0) {
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "onDeviceDisconnected");
  bool current_device_disconnected = false;
  for (size_t i = 0; i < device_list->deviceCount(); i++) {
    std::string uid = device_list->uid(i);
    std::scoped_lock<decltype(device_lock_)> lock(device_lock_);
    RCLCPP_INFO_STREAM(logger_, "device with " << uid << " disconnected");
    if (uid == device_unique_id_) {
      std::unique_lock<decltype(reset_device_mutex_)> reset_device_lock(reset_device_mutex_);
      reset_device_flag_ = true;
      reset_device_cond_.notify_all();
      current_device_disconnected = true;
      break;
    }
  }
  auto connect_event = current_device_disconnected
                           ? DeviceConnectionEvent::kDeviceDisconnected
                           : DeviceConnectionEvent::kOtherDeviceDisconnected;
  updateConnectedDeviceCount(num_devices_connected_, connect_event);
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
    RCLCPP_ERROR_STREAM(logger_,
                        "checkConnectTimer: device " << serial_number_ << " not connected");
    return;
  } else if (!ob_camera_node_) {
    device_connected_.store(false);
  }
}

void OBCameraNodeDriver::queryDevice() {
  while (is_alive_ && rclcpp::ok()) {
    if (!device_connected_.load()) {
      RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000, "Waiting for device connection...");
      auto device_list = ctx_->queryDeviceList();
      if (device_list->deviceCount() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      onDeviceConnected(device_list);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
}

void OBCameraNodeDriver::deviceCountUpdate() {
  while (is_alive_ && rclcpp::ok()) {
    updateConnectedDeviceCount(num_devices_connected_, DeviceConnectionEvent::kDeviceCountUpdate);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void OBCameraNodeDriver::syncTime() {
  while (is_alive_ && rclcpp::ok()) {
    if (device_ && device_info_ && !isOpenNIDevice(device_info_->pid())) {
      ctx_->enableMultiDeviceSync(0);
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
    ob_camera_node_.reset();
    device_.reset();
    device_info_.reset();
    device_connected_ = false;
    device_unique_id_.clear();
    reset_device_flag_ = false;
  }
}
void OBCameraNodeDriver::releaseDeviceSemaphore(sem_t *device_sem, int &num_devices_connected) {
  RCLCPP_INFO_THROTTLE(logger_, *get_clock(), 1000, "Release device semaphore");
  sem_post(device_sem);
  int sem_value = 0;
  sem_getvalue(device_sem, &sem_value);
  RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000, "semaphore value: " << sem_value);
  RCLCPP_INFO_THROTTLE(logger_, *get_clock(), 1000, "Release device semaphore done");
  if (num_devices_connected >= device_num_) {
    sem_destroy(device_sem);
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

void OBCameraNodeDriver::updateConnectedDeviceCount(int &num_devices_connected,
                                                    DeviceConnectionEvent connection_event) {
  // write connected device count to file
  int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
  if (shm_id == -1) {
    RCLCPP_INFO_STREAM(logger_, "Failed to create shared memory " << strerror(errno));
    return;
  }
  auto shm_ptr = (int *)shmat(shm_id, nullptr, 0);
  if (shm_ptr == (void *)-1) {
    RCLCPP_INFO_STREAM(logger_, "Failed to attach shared memory " << strerror(errno));
    return;
  }
  if (connection_event == DeviceConnectionEvent::kDeviceConnected) {
    num_devices_connected = *shm_ptr + 1;
  } else if (connection_event == DeviceConnectionEvent::kDeviceDisconnected && *shm_ptr > 0) {
    num_devices_connected = *shm_ptr - 1;
  } else {
    num_devices_connected = *shm_ptr;
  }
  RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                               "Current connected device " << num_devices_connected);
  *shm_ptr = static_cast<int>(num_devices_connected);
  shmdt(shm_ptr);
  if (connection_event == DeviceConnectionEvent::kDeviceDisconnected &&
      num_devices_connected == 0) {
    shmctl(shm_id, IPC_RMID, nullptr);
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDevice(
    const std::shared_ptr<ob::DeviceList> &list) {
  if (device_num_ == 1) {
    RCLCPP_INFO_STREAM(logger_, "Connecting to the default device");
    return list->getDevice(0);
  }
  sem_t *device_sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 1);
  if (device_sem == SEM_FAILED) {
    RCLCPP_INFO_STREAM(logger_, "Failed to open semaphore");
    return nullptr;
  }
  RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000,
                              "Connecting to device with serial number: " << serial_number_);
  int sem_value = 0;
  sem_getvalue(device_sem, &sem_value);
  RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000, "semaphore value: " << sem_value);
  int ret = sem_wait(device_sem);
  if (ret != 0) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to wait semaphore " << strerror(errno));
    releaseDeviceSemaphore(device_sem, num_devices_connected_);
    return nullptr;
  }
  std::shared_ptr<ob::Device> device = nullptr;
  if (!serial_number_.empty()) {
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000,
                                "Connecting to device with serial number: " << serial_number_);
    device = selectDeviceBySerialNumber(list, serial_number_);
  } else if (!usb_port_.empty()) {
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000,
                                "Connecting to device with usb port: " << usb_port_);
    device = selectDeviceByUSBPort(list, usb_port_);
  }
  std::shared_ptr<int> sem_guard(nullptr, [&, device](int const *) {
    auto connect_event = device != nullptr ? DeviceConnectionEvent::kDeviceConnected
                                           : DeviceConnectionEvent::kOtherDeviceConnected;
    updateConnectedDeviceCount(num_devices_connected_, connect_event);

    releaseDeviceSemaphore(device_sem, num_devices_connected_);
  });
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
  for (size_t i = 0; i < list->deviceCount(); i++) {
    try {
      auto pid = list->pid(i);
      if (isOpenNIDevice(pid)) {
        // openNI device
        auto dev = list->getDevice(i);
        auto device_info = dev->getDeviceInfo();
        std::string uid = device_info->uid();
        auto port_id = parseUsbPort(uid);
        if (port_id == usb_port) {
          RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000,
                                      "Device port id " << port_id << " matched");
          return dev;
        }
      } else {
        std::string uid = list->uid(i);
        auto port_id = parseUsbPort(uid);
        RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 1000, "Device usb port: " << uid);
        if (port_id == usb_port) {
          RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 1000,
                                      "Device usb port " << uid << " matched");
          return list->getDevice(i);
        }
      }
    } catch (ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get device info " << e.getMessage());
    } catch (std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get device info " << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get device info");
    }
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
  device_connected_ = true;
  device_info_ = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info_.get());
  device_unique_id_ = device_info_->uid();
  if (!isOpenNIDevice(device_info_->pid())) {
    ctx_->enableMultiDeviceSync(0);  // sync time stamp
  }
  RCLCPP_INFO_STREAM(logger_, "Device " << device_info_->name() << " connected");
  RCLCPP_INFO_STREAM(logger_, "Serial number: " << device_info_->serialNumber());
  RCLCPP_INFO_STREAM(logger_, "Firmware version: " << device_info_->firmwareVersion());
  RCLCPP_INFO_STREAM(logger_, "Hardware version: " << device_info_->hardwareVersion());
  RCLCPP_INFO_STREAM(logger_, "device unique id: " << device_unique_id_);
}

void OBCameraNodeDriver::startDevice(const std::shared_ptr<ob::DeviceList> &list) {
  std::scoped_lock<decltype(device_lock_)> lock(device_lock_);
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
  auto device = selectDevice(list);
  if (device == nullptr) {
    RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Device with serial number %s not found",
                         serial_number_.c_str());
    device_connected_ = false;
    return;
  }
  initializeDevice(device);
}
}  // namespace orbbec_camera

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::OBCameraNodeDriver)