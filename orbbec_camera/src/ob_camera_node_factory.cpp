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
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/shm.h>

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
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  if (int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT); shm_id != -1) {
    shmctl(shm_id, IPC_RMID, nullptr);
  }
  if (query_thread_ && query_thread_->joinable()) {
    query_thread_->join();
  }
}

void OBCameraNodeFactory::init() {
  auto log_level_str = declare_parameter<std::string>("log_level", "none");
  auto log_level = obLogSeverityFromString(log_level_str);
  ob::Context::setLoggerSeverity(log_level);
  is_alive_.store(true);
  parameters_ = std::make_shared<Parameters>(this);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  device_num_ = declare_parameter<int>("device_num", 1);
  ctx_->setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removed_list,
                                        std::shared_ptr<ob::DeviceList> added_list) {
    onDeviceDisconnected(removed_list);
    onDeviceConnected(added_list);
  });
  check_connect_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { checkConnectTimer(); });
  CHECK_NOTNULL(check_connect_timer_);
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
}

void OBCameraNodeFactory::onDeviceConnected(const std::shared_ptr<ob::DeviceList> &device_list) {
  if (device_list->deviceCount() == 0) {
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "onDeviceConnected");
  CHECK_NOTNULL(device_list);
  if (!device_) {
    try {
      startDevice(device_list);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "startDevice failed: " << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "startDevice failed");
    }
  }
}

void OBCameraNodeFactory::onDeviceDisconnected(const std::shared_ptr<ob::DeviceList> &device_list) {
  if (device_list->deviceCount() == 0) {
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "onDeviceDisconnected");
  CHECK_NOTNULL(device_list);
  for (size_t i = 0; i < device_list->deviceCount(); i++) {
    std::string serial_number = device_list->serialNumber(i);
    std::scoped_lock<decltype(device_lock_)> lock(device_lock_);
    RCLCPP_INFO_STREAM(logger_, "onDeviceDisconnected: " << serial_number);
    if (device_info_ && device_info_->serialNumber() == serial_number) {
      ob_camera_node_.reset();
      device_.reset();
      device_connected_ = false;
      break;
    }
  }
}

OBLogSeverity OBCameraNodeFactory::obLogSeverityFromString(const std::string_view &log_level) {
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

void OBCameraNodeFactory::checkConnectTimer() const {
  if (!device_connected_.load()) {
    RCLCPP_ERROR_STREAM(logger_, "checkConnectTimer: device not connected");
    return;
  }
}

void OBCameraNodeFactory::queryDevice() {
  while (is_alive_ && rclcpp::ok()) {
    if (!device_connected_) {
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

void OBCameraNodeFactory::startDevice(const std::shared_ptr<ob::DeviceList> &list) {
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
  size_t connected_device_num = 0;
  sem_t *device_sem = nullptr;
  std::shared_ptr<int> sem_guard(nullptr, [&](int const *) {
    if (device_num_ > 1 && device_sem) {
      RCLCPP_INFO(logger_, "Release device semaphore");
      sem_post(device_sem);
      int sem_value = 0;
      sem_getvalue(device_sem, &sem_value);
      RCLCPP_INFO_STREAM(logger_, "semaphore value: " << sem_value);
      RCLCPP_INFO_STREAM(logger_, "Release device semaphore done");
      if (connected_device_num >= device_num_) {
        RCLCPP_INFO_STREAM(logger_, "All devices connected,  sem_unlink");
        sem_destroy(device_sem);
        sem_unlink(DEFAULT_SEM_NAME.c_str());
        RCLCPP_INFO_STREAM(logger_, "All devices connected,  sem_unlink done..");
      }
    }
  });
  if (device_num_ == 1) {
    RCLCPP_INFO_STREAM(logger_, "Connecting to the default device");
    device_ = list->getDevice(0);
  } else {
    std::string lower_sn;
    std::transform(serial_number_.begin(), serial_number_.end(), std::back_inserter(lower_sn),
                   [](auto ch) { return isalpha(ch) ? tolower(ch) : static_cast<int>(ch); });
    device_sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 1);
    if (device_sem == SEM_FAILED) {
      RCLCPP_INFO_STREAM(logger_, "Failed to open semaphore");
      return;
    }
    RCLCPP_INFO_STREAM(logger_, "Connecting to device with serial number: " << serial_number_);
    int sem_value = 0;
    sem_getvalue(device_sem, &sem_value);
    RCLCPP_INFO_STREAM(logger_, "semaphore value: " << sem_value);
    if (int ret = sem_wait(device_sem); ret != 0) {
      RCLCPP_INFO_STREAM(logger_, "Failed to wait semaphore " << strerror(errno));
      return;
    }
    try {
      auto device = list->getDeviceBySN(serial_number_.c_str());
      device_ = device;
    } catch (ob::Error &e) {
      RCLCPP_INFO_STREAM(logger_, "Failed to get device info " << e.getMessage());
    } catch (std::exception &e) {
      RCLCPP_INFO_STREAM(logger_, "Failed to get device info " << e.what());
    } catch (...) {
      RCLCPP_INFO_STREAM(logger_, "Failed to get device info");
    }

    if (device_ == nullptr) {
      RCLCPP_WARN(logger_, "Device with serial number %s not found", serial_number_.c_str());
      device_connected_ = false;
      return;
    } else {
      // write connected device info to file
      int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
      if (shm_id == -1) {
        RCLCPP_INFO_STREAM(logger_, "Failed to create shared memory " << strerror(errno));
      } else {
        RCLCPP_INFO_STREAM(logger_, "Created shared memory");
        auto shm_ptr = (int *)shmat(shm_id, nullptr, 0);
        if (shm_ptr == (void *)-1) {
          RCLCPP_INFO_STREAM(logger_, "Failed to attach shared memory " << strerror(errno));
        } else {
          RCLCPP_INFO_STREAM(logger_, "Attached shared memory");
          connected_device_num = *shm_ptr + 1;
          RCLCPP_INFO_STREAM(logger_, "Current connected device " << connected_device_num);
          *shm_ptr = static_cast<int>(connected_device_num);
          RCLCPP_INFO_STREAM(logger_, "Wrote to shared memory");
          shmdt(shm_ptr);
          if (connected_device_num >= device_num_) {
            RCLCPP_INFO_STREAM(logger_, "All devices connected, removing shared memory");
            shmctl(shm_id, IPC_RMID, nullptr);
          }
        }
      }
    }
  }
  CHECK_NOTNULL(device_);
  CHECK_NOTNULL(device_.get());
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_, parameters_);
  device_connected_ = true;
  device_info_ = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info_.get());
  RCLCPP_INFO_STREAM(logger_, "Device " << device_info_->name() << " connected");
  RCLCPP_INFO_STREAM(logger_, "Serial number: " << device_info_->serialNumber());
  RCLCPP_INFO_STREAM(logger_, "Firmware version: " << device_info_->firmwareVersion());
  RCLCPP_INFO_STREAM(logger_, "Hardware version: " << device_info_->hardwareVersion());
  RCLCPP_INFO_STREAM(logger_, "device type: " << ObDeviceTypeToString(device_info_->deviceType()));
}
}  // namespace orbbec_camera
