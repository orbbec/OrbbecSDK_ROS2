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
#include <sys/stat.h>
#include <sys/types.h>
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
  if (query_thread_ && query_thread_->joinable()) {
    query_thread_->join();
  }
}

void OBCameraNodeFactory::init() {
  log_level_ = declare_parameter<std::string>("ob_log_level", "info");
  auto ob_log_level = obLogSeverityFromString(log_level_);
  ctx_->setLoggerSeverity(ob_log_level);
  is_alive_.store(true);
  parameters_ = std::make_shared<Parameters>(this);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  ctx_->setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removed_list,
                                        std::shared_ptr<ob::DeviceList> added_list) {
    deviceDisconnectCallback(removed_list);
    deviceConnectCallback(added_list);
  });
  check_connect_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { checkConnectTimer(); });
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
}

void OBCameraNodeFactory::deviceConnectCallback(
    const std::shared_ptr<ob::DeviceList> &device_list) {
  if (device_list->deviceCount() == 0) {
    return;
  }
  RCLCPP_ERROR_STREAM(logger_, "deviceConnectCallback");
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

void OBCameraNodeFactory::deviceDisconnectCallback(
    const std::shared_ptr<ob::DeviceList> &device_list) {
  if (device_list->deviceCount() == 0) {
    return;
  }
  RCLCPP_ERROR_STREAM(logger_, "deviceDisconnectCallback");
  CHECK_NOTNULL(device_list);
  for (size_t i = 0; i < device_list->deviceCount(); i++) {
    std::string serial_number = device_list->serialNumber(i);
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    RCLCPP_INFO_STREAM(logger_, "deviceDisconnectCallback: " << serial_number);
    if (device_info_ && device_info_->serialNumber() == serial_number) {
      ob_camera_node_.reset();
      device_.reset();
      device_connected_ = false;
      break;
    }
  }
}

OBLogSeverity OBCameraNodeFactory::obLogSeverityFromString(const std::string &log_level) {
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

void OBCameraNodeFactory::checkConnectTimer() {
  if (!device_connected_) {
    RCLCPP_ERROR_STREAM(logger_, "checkConnectTimer: device not connected");
    return;
  }
}
void OBCameraNodeFactory::queryDevice() {
  while (is_alive_ && rclcpp::ok()) {
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    if (device_) {
      break;
    }
    auto list = ctx_->queryDeviceList();
    CHECK_NOTNULL(list);
    if (list->deviceCount() > 0) {
      try {
        startDevice(list);
      } catch (const ob::Error &e) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to start device: " << e.getMessage());
      } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to start device: " << e.what());
      } catch (...) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to start device");
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void OBCameraNodeFactory::startDevice(const std::shared_ptr<ob::DeviceList> &list) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (device_) {
    return;
  }
  if (list->deviceCount() == 0) {
    RCLCPP_WARN(logger_, "No device found");
    return;
  }
  if (serial_number_.empty()) {
    RCLCPP_INFO_STREAM(logger_, "Connecting to the default device");
    device_ = list->getDevice(0);
  } else {
    std::string lower_sn;
    std::transform(serial_number_.begin(), serial_number_.end(), std::back_inserter(lower_sn),
                   [](auto ch) { return isalpha(ch) ? tolower(ch) : static_cast<int>(ch); });
    auto device_sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 1);
    if (device_sem == SEM_FAILED) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to open semaphore");
      return;
    }
    size_t connected_device_num = 0;
    RCLCPP_INFO_STREAM(logger_, "Connecting to device with serial number: " << serial_number_);
    int sem_value = 0;
    sem_getvalue(device_sem, reinterpret_cast<int *>(&sem_value));
    RCLCPP_INFO_STREAM(logger_, "semaphore value: " << sem_value);
    int ret = sem_wait(device_sem);
    std::shared_ptr<int> sem_guard(nullptr, [&](auto) {
      RCLCPP_INFO_STREAM(logger_, "release semaphore");
      sem_post(device_sem);
      sem_value = 0;
      sem_getvalue(device_sem, reinterpret_cast<int *>(&sem_value));
      RCLCPP_INFO_STREAM(logger_, "semaphore value: " << sem_value);
      RCLCPP_INFO_STREAM(logger_, "Release device semaphore done");
      if (connected_device_num >= device_num_) {
        RCLCPP_INFO_STREAM(logger_, "All devices connected,  sem_unlink");
        sem_destroy(device_sem);
        sem_unlink(DEFAULT_SEM_NAME.c_str());
        RCLCPP_INFO_STREAM(logger_, "All devices connected,  sem_unlink done..");
      }
    });
    RCLCPP_INFO_STREAM(logger_, "sem_wait ret: " << ret);
    if (!ret) {
      for (size_t i = 0; i < list->deviceCount(); ++i) {
        auto device = list->getDevice(i);
        auto info = device->getDeviceInfo();
        std::string serial = info->serialNumber();
        if (serial == serial_number_ || serial == lower_sn) {
          RCLCPP_INFO_STREAM(logger_, "Connecting to device " << serial);
          device_ = device;
          break;
        }
      }
    } else {
      RCLCPP_ERROR_STREAM(logger_, "Failed to wait semaphore " << strerror(errno));
    }

    if (device_ == nullptr) {
      RCLCPP_WARN(logger_, "Device with serial number %s not found", serial_number_.c_str());
      RCLCPP_ERROR(logger_, "Release device semaphore");

      return;
    } else {
      // write connected device info to file
      int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
      if (shm_id == -1) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to create shared memory " << strerror(errno));
      } else {
        RCLCPP_INFO_STREAM(logger_, "Created shared memory");
        auto shm_ptr = (int *)shmat(shm_id, nullptr, 0);
        if (shm_ptr == (void *)-1) {
          RCLCPP_ERROR_STREAM(logger_, "Failed to attach shared memory " << strerror(errno));
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
    ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_, parameters_);
    device_connected_ = true;
    device_info_ = device_->getDeviceInfo();
    CHECK_NOTNULL(device_info_);
    RCLCPP_INFO_STREAM(logger_, "Device " << device_info_->name() << " connected");
    RCLCPP_INFO_STREAM(logger_, "Serial number: " << device_info_->serialNumber());
    RCLCPP_INFO_STREAM(logger_, "Firmware version: " << device_info_->firmwareVersion());
    RCLCPP_INFO_STREAM(logger_, "Hardware version: " << device_info_->hardwareVersion());
    RCLCPP_INFO_STREAM(logger_,
                       "device type: " << ObDeviceTypeToString(device_info_->deviceType()));
  }
}
}  // namespace orbbec_camera
