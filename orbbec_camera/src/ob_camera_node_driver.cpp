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
#include "orbbec_camera/utils.h"
#include <fcntl.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <csignal>
#include <sys/mman.h>
#include <unistd.h>
#include <filesystem>

#include <fstream>
#include <iomanip>  // For std::put_time
#include <malloc.h>

std::string g_camera_name = "orbbec_camera";  // Assuming this is declared elsewhere
std::string g_time_domain = "global";         // Assuming this is declared elsewhere

void signalHandler(int sig) {
  std::cout << "Received signal: " << sig << std::endl;
  if (sig == SIGINT || sig == SIGTERM) {
    static int signal_count = 0;
    signal_count++;

    if (signal_count <= 3) {
      rclcpp::shutdown();
      // Give some time for graceful shutdown
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    } else if (signal_count >= 5) {
      // Force exit after second signal
      std::cout << "Force exit due to multiple signals" << std::endl;
      exit(sig);
    }
  } else {
    std::string log_dir = "Log/";

    // get current time
    std::time_t now = std::time(nullptr);
    std::tm *local_time = std::localtime(&now);

    // format date and time to string, format as "2024_05_20_12_34_56"
    std::ostringstream time_stream;
    time_stream << std::put_time(local_time, "%Y_%m_%d_%H_%M_%S");

    // generate log file name
    std::string log_file_name = g_camera_name + "_crash_stack_trace_" + time_stream.str() + ".log";
    std::string log_file_path = log_dir + log_file_name;

    if (!std::filesystem::exists(log_dir)) {
      std::filesystem::create_directories(log_dir);
    }

    std::cout << "Log crash stack trace to " << log_file_path << std::endl;
    std::ofstream log_file(log_file_path, std::ios::app);

    if (log_file.is_open()) {
      log_file << "Received signal: " << sig << std::endl;

      backward::StackTrace st;
      st.load_here(32);  // Capture stack
      backward::Printer p;
      p.print(st, log_file);  // Print stack to log file
    }

    log_file.close();
    exit(sig);  // Exit program
  }
}

namespace orbbec_camera {
backward::SignalHandling OBCameraNodeDriver::sh;
OBCameraNodeDriver::OBCameraNodeDriver(const rclcpp::NodeOptions &node_options)
    : Node("orbbec_camera_node", "/", node_options),
      node_options_(node_options),
      config_path_(ament_index_cpp::get_package_share_directory("orbbec_camera") +
                   "/config/OrbbecSDKConfig_v2.0.xml"),
      logger_(this->get_logger()),
      extension_path_(ament_index_cpp::get_package_prefix("orbbec_camera") + "/lib/extensions") {
  node_name_ = "orbbec_camera_node";
  init();
}

OBCameraNodeDriver::OBCameraNodeDriver(const std::string &node_name, const std::string &ns,
                                       const rclcpp::NodeOptions &node_options)
    : Node(node_name, ns, node_options),
      node_options_(node_options),
      config_path_(ament_index_cpp::get_package_share_directory("orbbec_camera") +
                   "/config/OrbbecSDKConfig_v2.0.xml"),
      logger_(this->get_logger()),
      extension_path_(ament_index_cpp::get_package_prefix("orbbec_camera") + "/lib/extensions") {
  node_name_ = node_name;
  init();
}

OBCameraNodeDriver::~OBCameraNodeDriver() {
  is_alive_.store(false);

  // First stop the camera node cleanly before stopping threads
  if (ob_camera_node_) {
    try {
      ob_camera_node_->clean();
    } catch (...) {
      RCLCPP_WARN_STREAM(logger_, "Exception during camera node cleanup in destructor");
    }
    ob_camera_node_->stopGmslTrigger();
  }

  // Stop timers that might access the device
  if (sync_host_time_timer_) {
    try {
      sync_host_time_timer_->cancel();
      sync_host_time_timer_.reset();
    } catch (...) {
      RCLCPP_WARN_STREAM(logger_, "Exception during sync timer cleanup in destructor");
    }
  }

  if (check_connect_timer_) {
    try {
      check_connect_timer_->cancel();
      check_connect_timer_.reset();
    } catch (...) {
      RCLCPP_WARN_STREAM(logger_, "Exception during check connect timer cleanup in destructor");
    }
  }

  if (device_status_timer_) {
    try {
      device_status_timer_->cancel();
      device_status_timer_.reset();
    } catch (...) {
      RCLCPP_WARN_STREAM(logger_, "Exception during device status timer cleanup in destructor");
    }
  }

  // Now stop threads
  if (device_count_update_thread_ && device_count_update_thread_->joinable()) {
    device_count_update_thread_->join();
  }
  if (query_thread_ && query_thread_->joinable()) {
    query_thread_->join();
  }
  if (reset_device_thread_ && reset_device_thread_->joinable()) {
    reset_device_cond_.notify_all();
    reset_device_thread_->join();
  }

  // Clean up shared memory
  if (orb_device_lock_shm_fd_ != -1) {
    close(orb_device_lock_shm_fd_);
    orb_device_lock_shm_fd_ = -1;
  }
  shm_unlink(ORB_DEFAULT_LOCK_NAME.c_str());
}

void OBCameraNodeDriver::init() {
  // Set signal handlers for crash reporting
  signal(SIGSEGV, signalHandler);  // segment fault
  signal(SIGABRT, signalHandler);  // abort
  signal(SIGFPE, signalHandler);   // float point exception
  signal(SIGILL, signalHandler);   // illegal instruction
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  ob::Context::setExtensionsDirectory(extension_path_.c_str());
  g_camera_name = declare_parameter<std::string>("camera_name", g_camera_name);
  auto log_level_str = declare_parameter<std::string>("log_level", "none");
  auto log_level = obLogSeverityFromString(log_level_str);
  auto log_file_name = declare_parameter<std::string>("log_file_name", "");
  std::string pwd_dir = std::getenv("PWD") ? std::getenv("PWD") : std::getenv("HOME");
  std::string log_path = pwd_dir + "/Log/" + g_camera_name;
  // Set logger to console
  ob::Context::setLoggerToConsole(log_level);
  ob::Context::setLoggerToFile(log_level, log_path.c_str());
  // Set custom log file name if specified
  if (!log_file_name.empty()) {
    try {
      ob::Context::setLoggerFileName(log_file_name);
      RCLCPP_INFO_STREAM(logger_, "SDK log file name set to: " << log_file_name);
    } catch (const ob::Error &e) {
      RCLCPP_WARN_STREAM(logger_, "Failed to set SDK log file name: " << e.getMessage());
    }
  }
  // Force IP
  force_ip_enable_ = declare_parameter<bool>("force_ip_enable", false);
  force_ip_mac_ = declare_parameter<std::string>("force_ip_mac", "");
  force_ip_address_ = declare_parameter<std::string>("force_ip_address", "192.168.1.10");
  force_ip_subnet_mask_ = declare_parameter<std::string>("force_ip_subnet_mask", "255.255.255.0");
  force_ip_gateway_ = declare_parameter<std::string>("force_ip_gateway", "192.168.1.1");
  applyForceIpConfig();
  if (config_path_.empty()) {
    ctx_ = std::make_unique<ob::Context>();
  } else {
    ctx_ = std::make_unique<ob::Context>(config_path_.c_str());
  }
  connection_delay_ = static_cast<int>(declare_parameter<int>("connection_delay", 100));
  enable_sync_host_time_ = declare_parameter<bool>("enable_sync_host_time", true);
  double time_sync_period = declare_parameter<double>("time_sync_period", 60.0);
  time_sync_period_ = std::chrono::milliseconds((int)(time_sync_period * 1000));
  upgrade_firmware_ = declare_parameter<std::string>("upgrade_firmware", "");
  g_time_domain = declare_parameter<std::string>("time_domain", g_time_domain);
  preset_firmware_path_ =
      declare_parameter<std::string>("preset_firmware_path", preset_firmware_path_);
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
  reboot_device_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reboot_device", std::bind(&OBCameraNodeDriver::rebootDeviceCallback, this,
                                 std::placeholders::_1, std::placeholders::_2));
  pthread_mutexattr_init(&orb_device_lock_attr_);
  pthread_mutexattr_setpshared(&orb_device_lock_attr_, PTHREAD_PROCESS_SHARED);
  orb_device_lock_ = (pthread_mutex_t *)orb_device_lock_shm_addr_;
  pthread_mutex_init(orb_device_lock_, &orb_device_lock_attr_);
  is_alive_.store(true);
  // Initialize the reset device completion time to allow immediate device connection on startup
  last_reset_device_completion_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
  parameters_ = std::make_shared<Parameters>(this);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  device_num_ = static_cast<int>(declare_parameter<int>("device_num", 1));
  usb_port_ = declare_parameter<std::string>("usb_port", "");
  net_device_ip_ = declare_parameter<std::string>("net_device_ip", "");
  net_device_port_ = static_cast<int>(declare_parameter<int>("net_device_port", 0));
  enumerate_net_device_ = declare_parameter<bool>("enumerate_net_device", false);
  uvc_backend_ = declare_parameter<std::string>("uvc_backend", "libuvc");
  device_access_mode_str_ = declare_parameter<std::string>("device_access_mode", "default");
  device_access_mode_ = stringToAccessMode(device_access_mode_str_);
  RCLCPP_INFO_STREAM(logger_, "Device access mode: " << device_access_mode_str_ << " ("
                                                     << device_access_mode_ << ")");
  if (uvc_backend_ == "libuvc") {
    ctx_->setUvcBackendType(OB_UVC_BACKEND_TYPE_LIBUVC);
    RCLCPP_INFO_STREAM(logger_, "setUvcBackendType:" << uvc_backend_);
  } else if (uvc_backend_ == "v4l2") {
    ctx_->setUvcBackendType(OB_UVC_BACKEND_TYPE_V4L2);
    RCLCPP_INFO_STREAM(logger_, "setUvcBackendType:" << uvc_backend_);
  } else {
    ctx_->setUvcBackendType(OB_UVC_BACKEND_TYPE_LIBUVC);
    RCLCPP_INFO_STREAM(logger_, "setUvcBackendType:" << uvc_backend_);
  }
  ctx_->enableNetDeviceEnumeration(enumerate_net_device_);
  ctx_->setDeviceChangedCallback([this](const std::shared_ptr<ob::DeviceList> &removed_list,
                                        const std::shared_ptr<ob::DeviceList> &added_list) {
    onDeviceDisconnected(removed_list);
    onDeviceConnected(added_list);
  });
  check_connect_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { checkConnectTimer(); });
  CHECK_NOTNULL(check_connect_timer_);
  device_status_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1000 / device_status_interval_hz),
                              [this]() { deviceStatusTimer(); });
  auto qos = rclcpp::QoS(1).transient_local();
  if (node_options_.use_intra_process_comms()) {
    qos = rclcpp::QoS(1);
  }
  device_status_pub_ =
      this->create_publisher<orbbec_camera_msgs::msg::DeviceStatus>("device_status", qos);
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
  reset_device_thread_ = std::make_shared<std::thread>([this]() { resetDevice(); });
}

void OBCameraNodeDriver::onDeviceConnected(const std::shared_ptr<ob::DeviceList> &device_list) {
  CHECK_NOTNULL(device_list);
  {
    RCLCPP_INFO_STREAM(logger_, "onDeviceConnected called");
    std::unique_lock<decltype(reset_device_mutex_)> reset_lock(reset_device_mutex_);
    if (reset_device_flag_) {
      RCLCPP_INFO_STREAM(logger_, "onDeviceConnected : device reset in progress, waiting...");
      reset_device_cond_.wait(
          reset_lock, [this]() { return !reset_device_flag_ || !is_alive_ || !rclcpp::ok(); });
      if (!is_alive_) {
        return;
      }
      RCLCPP_INFO_STREAM(logger_,
                         "onDeviceConnected : device reset completed, continuing connection");
    }
  }
  if (device_list->getCount() == 0) {
    return;
  }

  // Check if device is already connected or connecting
  if (device_connected_.load() || device_connecting_.load()) {
    RCLCPP_DEBUG_STREAM(logger_, "onDeviceConnected: device already connected or connecting");
    return;
  }

  if (!device_) {
    startDevice(device_list);
  }
}

void OBCameraNodeDriver::onDeviceDisconnected(const std::shared_ptr<ob::DeviceList> &device_list) {
  CHECK_NOTNULL(device_list);
  if (device_list->getCount() == 0) {
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "onDeviceDisconnected called");

  // Check if device connection/initialization is in progress
  if (device_connecting_.load()) {
    RCLCPP_INFO_STREAM(logger_,
                       "onDeviceDisconnected: device connection/initialization in progress, "
                       "ignoring disconnect event");
    return;
  }

  std::unique_lock<decltype(reset_device_mutex_)> reset_device_lock(reset_device_mutex_);

  std::lock_guard<decltype(device_lock_)> lock(device_lock_);

  if (!device_connected_.load()) {
    RCLCPP_DEBUG_STREAM(logger_, "onDeviceDisconnected: device already disconnected");
    return;
  }

  for (size_t i = 0; i < device_list->getCount(); i++) {
    std::string uid = device_list->getUid(i);
    std::string serial_number = device_list->getSerialNumber(i);
    RCLCPP_INFO_STREAM(logger_, "device with " << uid << " disconnected");
    if (uid == device_unique_id_ || serial_number_ == serial_number) {
      RCLCPP_INFO_STREAM(logger_,
                         "device with " << uid << " disconnected, notify reset device thread 1.");
      reset_device_flag_ = true;
      reset_device_cond_.notify_all();
      RCLCPP_INFO_STREAM(logger_,
                         "device with " << uid << " disconnected, notify reset device thread 2.");
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
  while (is_alive_ && rclcpp::ok()) {
    // Check if device reset is in progress before attempting to connect
    {
      std::unique_lock<decltype(reset_device_mutex_)> reset_lock(reset_device_mutex_);
      if (reset_device_flag_) {
        RCLCPP_INFO_STREAM(logger_, "queryDevice: device reset in progress, waiting...");
        reset_device_cond_.wait(
            reset_lock, [this]() { return !reset_device_flag_ || !is_alive_ || !rclcpp::ok(); });
        if (!is_alive_ || !rclcpp::ok()) {
          return;
        }
        RCLCPP_INFO_STREAM(logger_, "queryDevice: device reset completed, continuing connection");
      }
    }

    // Check if connection is already in progress
    if (device_connecting_.load()) {
      RCLCPP_DEBUG_STREAM(logger_,
                          "queryDevice: device connection already in progress, waiting...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // If device is already connected, skip connection attempt
    if (!device_connected_.load()) {
      // Check if sufficient time has passed since last reset device completion
      auto now = std::chrono::steady_clock::now();
      auto time_since_last_reset = std::chrono::duration_cast<std::chrono::seconds>(
          now - last_reset_device_completion_time_);

      if (time_since_last_reset.count() < 10) {
        RCLCPP_DEBUG_STREAM(
            logger_,
            "queryDevice: Only "
                << time_since_last_reset.count()
                << " seconds since last reset completion, waiting before starting device...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        continue;
      }

      if (!enumerate_net_device_ && !net_device_ip_.empty() && net_device_port_ != 0) {
        connectNetDevice(net_device_ip_, net_device_port_);
      } else {
        auto device_list = ctx_->queryDeviceList();
        if (device_list->getCount() != 0) {
          startDevice(device_list);
        }
      }
    }
    // Add a delay to prevent tight loop
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void OBCameraNodeDriver::resetDevice() {
  while (is_alive_ && rclcpp::ok()) {
    {
      std::unique_lock<decltype(reset_device_mutex_)> lock(reset_device_mutex_);
      // Use a timeout to make the wait interruptible
      auto timeout = std::chrono::milliseconds(1000);
      bool notified = reset_device_cond_.wait_for(
          lock, timeout, [this]() { return !is_alive_ || !rclcpp::ok() || reset_device_flag_; });

      // Check if we should exit due to shutdown
      if (!is_alive_ || !rclcpp::ok()) {
        break;
      }

      // If not notified by reset flag, continue waiting
      if (!notified || !reset_device_flag_) {
        continue;
      }

      // Stop sync timer to prevent it from accessing the device during reset
      if (sync_host_time_timer_) {
        try {
          sync_host_time_timer_->cancel();
          sync_host_time_timer_.reset();
        } catch (...) {
          RCLCPP_WARN_STREAM(logger_, "Exception during sync timer cleanup during reset");
        }
      }

      RCLCPP_INFO_STREAM(logger_, "resetDevice : Reset device uid: " << device_unique_id_);
      std::lock_guard<decltype(device_lock_)> device_lock(device_lock_);
      {
        // Mark device as disconnected immediately to prevent other threads from accessing it
        device_connected_ = false;
        device_connecting_ = false;  // Clear connecting flag

        // Reset objects in order, with additional safety checks
        if (ob_camera_node_) {
          try {
            RCLCPP_INFO_STREAM(logger_, "Resetting ob_camera_node_");
            ob_camera_node_.reset();
            RCLCPP_INFO_STREAM(logger_, "ob_camera_node_ reset completed");
          } catch (...) {
            RCLCPP_WARN_STREAM(logger_, "Exception during ob_camera_node reset");
          }
        }

        // Allow more time for internal SDK cleanup
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (device_) {
          try {
            RCLCPP_INFO_STREAM(logger_, "Resetting device_");
            // Force free any idle memory before device reset
            if (ctx_) {
              try {
                ctx_->freeIdleMemory();
              } catch (...) {
                // Ignore exceptions during memory cleanup
              }
            }
            device_.reset();
            RCLCPP_INFO_STREAM(logger_, "device_ reset completed");
          } catch (const ob::Error &e) {
            RCLCPP_WARN_STREAM(logger_, "OB Exception during device reset: " << e.getMessage());
          } catch (const std::exception &e) {
            RCLCPP_WARN_STREAM(logger_, "Standard exception during device reset: " << e.what());
          } catch (...) {
            RCLCPP_WARN_STREAM(logger_, "Unknown exception during device reset");
          }
        }

        if (device_info_) {
          try {
            RCLCPP_INFO_STREAM(logger_, "Resetting device_info_");
            device_info_.reset();
            RCLCPP_INFO_STREAM(logger_, "device_info_ reset completed");
          } catch (...) {
            RCLCPP_WARN_STREAM(logger_, "Exception during device_info reset");
          }
        }

        device_unique_id_.clear();
      }
      reset_device_flag_ = false;
      last_reset_device_completion_time_ = std::chrono::steady_clock::now();
    }
    reset_device_cond_.notify_all();
    malloc_trim(0);
    RCLCPP_INFO_STREAM(logger_, "Reset device uid: " << device_unique_id_ << " done");
  }
}

void OBCameraNodeDriver::deviceStatusTimer() {
  // Always publish device status regardless of device connection state
  orbbec_camera_msgs::msg::DeviceStatus status_msg;
  status_msg.header.stamp = this->now();
  status_msg.device_online = device_connected_.load();
  status_msg.header.frame_id = node_name_;

  // Initialize default values for when device is not connected
  status_msg.connection_type = "";
  status_msg.calibration_from_factory = false;
  status_msg.calibration_from_launch_param = false;
  status_msg.customer_calibration_ready = false;

  // Flag to track if device communication error occurs
  bool device_communication_error = false;

  // Only try to get device information if device is connected and stable
  if (device_connected_.load() && !device_connecting_.load()) {
    // Check if reset is in progress
    std::unique_lock<decltype(reset_device_mutex_)> reset_lock(reset_device_mutex_,
                                                               std::try_to_lock);
    if (reset_lock.owns_lock() && !reset_device_flag_) {
      // Only get device-specific info if we have a valid camera node and device
      if (ob_camera_node_) {
        // Safely get color and depth status - these may access device
        try {
          ob_camera_node_->getColorStatus(status_msg);
          ob_camera_node_->getDepthStatus(status_msg);
        } catch (const ob::Error &e) {
          std::string error_msg = e.getMessage() ? e.getMessage() : "Unknown OB error";
          if (error_msg.find("Device is deactivated") != std::string::npos ||
              error_msg.find("disconnected") != std::string::npos ||
              error_msg.find("Send control transfer failed") != std::string::npos) {
            RCLCPP_WARN(
                logger_,
                "Device communication error in %s at line %d: %s - Device may be disconnected",
                __FUNCTION__, __LINE__, error_msg.c_str());
            device_communication_error = true;
          } else {
            RCLCPP_ERROR(logger_, "Error in %s at line %d: %s", __FUNCTION__, __LINE__,
                         error_msg.c_str());
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(logger_, "Exception in %s at line %d: %s", __FUNCTION__, __LINE__, e.what());
        } catch (...) {
          RCLCPP_ERROR(logger_, "Unknown exception in %s at line %d", __FUNCTION__, __LINE__);
        }

        // These should be safe as they don't directly access hardware
        status_msg.calibration_from_launch_param = ob_camera_node_->isParamCalibrated();
      }

      // Safely get connection type
      try {
        if (device_info_) {
          status_msg.connection_type = device_info_->getConnectionType();
        }
      } catch (const ob::Error &e) {
        std::string error_msg = e.getMessage() ? e.getMessage() : "Unknown OB error";
        if (error_msg.find("Device is deactivated") != std::string::npos ||
            error_msg.find("disconnected") != std::string::npos ||
            error_msg.find("Send control transfer failed") != std::string::npos) {
          RCLCPP_WARN(
              logger_,
              "Device communication error in %s at line %d: %s - Device may be disconnected",
              __FUNCTION__, __LINE__, error_msg.c_str());
          device_communication_error = true;
        } else {
          RCLCPP_ERROR(logger_, "Error in %s at line %d: %s", __FUNCTION__, __LINE__,
                       error_msg.c_str());
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "Exception in %s at line %d: %s", __FUNCTION__, __LINE__, e.what());
      } catch (...) {
        RCLCPP_ERROR(logger_, "Unknown exception in %s at line %d", __FUNCTION__, __LINE__);
      }

      // Safely get calibration info
      try {
        if (device_) {
          auto camera_params = device_->getCalibrationCameraParamList();
          bool calibration_from_factory = (camera_params != nullptr && camera_params->count() > 0);
          status_msg.calibration_from_factory = calibration_from_factory;
        }
      } catch (const ob::Error &e) {
        std::string error_msg = e.getMessage() ? e.getMessage() : "Unknown OB error";
        if (error_msg.find("Device is deactivated") != std::string::npos ||
            error_msg.find("disconnected") != std::string::npos ||
            error_msg.find("Send control transfer failed") != std::string::npos) {
          RCLCPP_WARN(
              logger_,
              "Device communication error in %s at line %d: %s - Device may be disconnected",
              __FUNCTION__, __LINE__, error_msg.c_str());
          device_communication_error = true;
        } else {
          RCLCPP_ERROR(logger_, "Error in %s at line %d: %s", __FUNCTION__, __LINE__,
                       error_msg.c_str());
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "Exception in %s at line %d: %s", __FUNCTION__, __LINE__, e.what());
      } catch (...) {
        RCLCPP_ERROR(logger_, "Unknown exception in %s at line %d", __FUNCTION__, __LINE__);
      }

      // Safely check user calibration readiness - this may also access device
      // Only execute for 435LE devices (check by device name for network devices)
      try {
        if (device_info_ && ob_camera_node_) {
          std::string device_name = device_info_->getName();
          if (device_name.find("435Le") != std::string::npos ||
              device_name.find("435LE") != std::string::npos) {
            if (!ob_camera_node_->checkUserCalibrationReady()) {
              status_msg.customer_calibration_ready = false;
            } else {
              status_msg.customer_calibration_ready = true;
            }
          } else {
            // For non-435LE devices, set a default value or skip this check
            status_msg.customer_calibration_ready = false;
          }
        } else {
          status_msg.customer_calibration_ready = false;
        }
      } catch (const ob::Error &e) {
        std::string error_msg = e.getMessage() ? e.getMessage() : "Unknown OB error";
        if (error_msg.find("Device is deactivated") != std::string::npos ||
            error_msg.find("disconnected") != std::string::npos ||
            error_msg.find("Send control transfer failed") != std::string::npos) {
          RCLCPP_WARN(
              logger_,
              "Device communication error in %s at line %d: %s - Device may be disconnected",
              __FUNCTION__, __LINE__, error_msg.c_str());
          device_communication_error = true;
        } else {
          RCLCPP_ERROR(logger_, "Error in %s at line %d: %s", __FUNCTION__, __LINE__,
                       error_msg.c_str());
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "Exception in %s at line %d: %s", __FUNCTION__, __LINE__, e.what());
      } catch (...) {
        RCLCPP_ERROR(logger_, "Unknown exception in %s at line %d", __FUNCTION__, __LINE__);
      }
    }
  }

  // If device communication error occurred, set device_online to false
  if (device_communication_error) {
    status_msg.device_online = false;
  }

  // if status_msg.connection_type is empty, set it to "unknown"
  if (status_msg.connection_type.empty()) {
    status_msg.connection_type = "unknown";
    status_msg.device_online = false;
  }

  // Always publish the status message, regardless of device state
  if (device_status_pub_) {
    device_status_pub_->publish(status_msg);
  }
  // RCLCPP_INFO_STREAM(logger_, "deviceStatusTimer() ");
}

void OBCameraNodeDriver::rebootDeviceCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  (void)request;
  (void)response;

  malloc_trim(0);
  RCLCPP_INFO(logger_, "Reboot device service called");

  struct timespec timeout;
  clock_gettime(CLOCK_REALTIME, &timeout);
  timeout.tv_sec += 15;

  int lock_result = pthread_mutex_timedlock(orb_device_lock_, &timeout);
  if (lock_result != 0) {
    RCLCPP_WARN(logger_, "Failed to acquire process lock for reboot: %s", strerror(lock_result));
    return;
  }

  std::shared_ptr<int> process_lock_guard(
      nullptr, [this](int *) { pthread_mutex_unlock(orb_device_lock_); });

  try {
    std::unique_lock<decltype(reset_device_mutex_)> reset_lock(reset_device_mutex_);
    reset_device_flag_ = true;
    {
      std::lock_guard<decltype(device_lock_)> device_lock(device_lock_);

      if (!device_connected_ || !ob_camera_node_) {
        RCLCPP_INFO(logger_, "Device not connected");
        reset_device_flag_ = false;
      } else {
        std::string current_device_uid = device_unique_id_;
        RCLCPP_INFO_STREAM(logger_, "Rebooting device with UID: " << current_device_uid);
        ob_camera_node_->rebootDevice();
      }
    }
    if (reset_device_flag_) {
      RCLCPP_INFO(logger_, "Device reboot initiated, waiting for reconnection");
    }

  } catch (std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to reboot device: " << e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to reboot device: unknown error");
  }
  process_lock_guard.reset();
  if (reset_device_flag_) {
    reset_device_cond_.notify_all();
  }
  malloc_trim(0);
  return;
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDevice(
    const std::shared_ptr<ob::DeviceList> &list) {
  std::shared_ptr<ob::Device> device = nullptr;
  if (!net_device_ip_.empty() && net_device_port_ != 0) {
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                "Connecting to device with net ip: " << net_device_ip_);
    device = selectDeviceByNetIP(list, net_device_ip_);
  } else if (!serial_number_.empty()) {
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                "Connecting to device with serial number: " << serial_number_);
    device = selectDeviceBySerialNumber(list, serial_number_);
  } else if (!usb_port_.empty()) {
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                "Connecting to device with usb port: " << usb_port_);
    device = selectDeviceByUSBPort(list, usb_port_);
  } else if (device_num_ == 1) {
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000, "Connecting to the default device");
    return list->getDevice(0, device_access_mode_);
  }
  if (device == nullptr) {
    RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000, "Device with serial number %s not found",
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
  for (size_t i = 0; i < list->getCount(); i++) {
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                "Before lock: Select device serial number: " << serial_number);
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                "After lock: Select device serial number: " << serial_number);
    try {
      auto pid = list->getPid(i);
      if (isOpenNIDevice(pid)) {
        // openNI device
        auto device = list->getDevice(i, device_access_mode_);
        auto device_info = device->getDeviceInfo();
        if (device_info->getSerialNumber() == serial_number) {
          RCLCPP_INFO_STREAM_THROTTLE(
              logger_, *get_clock(), 5000,
              "Device serial number " << device_info->getSerialNumber() << " matched");
          return device;
        }
      } else {
        std::string sn = list->getSerialNumber(i);
        RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000, "Device serial number: " << sn);
        if (sn == serial_number) {
          RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                      "Device serial number " << sn << " matched");
          return list->getDevice(i, device_access_mode_);
        }
      }
    } catch (ob::Error &e) {
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                   "Failed to get device info " << e.getMessage());
    } catch (std::exception &e) {
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                   "Failed to get device info " << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000, "Failed to get device info");
    }
  }
  return nullptr;
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDeviceByUSBPort(
    const std::shared_ptr<ob::DeviceList> &list, const std::string &usb_port) {
  try {
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                "Before lock: Select device usb port: " << usb_port);
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                "After lock: Select device usb port: " << usb_port);
    auto device = list->getDeviceByUid(usb_port.c_str(), device_access_mode_);
    if (device) {
      RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                  "getDeviceByUid device usb port " << usb_port << " done");
    } else {
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                   "getDeviceByUid device usb port " << usb_port << " failed");
      RCLCPP_ERROR_STREAM_THROTTLE(
          logger_, *get_clock(), 5000,
          "Please use script to get usb port: ros2 run orbbec_camera list_devices_node");
    }
    return device;
  } catch (ob::Error &e) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                 "Failed to get device info " << e.getMessage());
  } catch (std::exception &e) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                 "Failed to get device info " << e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000, "Failed to get device info");
  }

  return nullptr;
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDeviceByNetIP(
    const std::shared_ptr<ob::DeviceList> &list, const std::string &net_ip) {
  RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                              "Before lock: Select device net ip: " << net_ip);
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                              "After lock: Select device net ip: " << net_ip);
  std::shared_ptr<ob::Device> device = nullptr;
  for (size_t i = 0; i < list->getCount(); i++) {
    try {
      if (std::string(list->getConnectionType(i)) != "Ethernet") {
        continue;
      }
      if (list->getIpAddress(i) == nullptr) {
        continue;
      }
      RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                  "FindDeviceByNetIP device net ip " << list->getIpAddress(i));
      if (std::string(list->getIpAddress(i)) == net_ip) {
        RCLCPP_INFO_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                    "getDeviceByNetIP device net ip " << net_ip << " done");
        return list->getDevice(i, device_access_mode_);
      }
    } catch (ob::Error &e) {
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                   "Failed to get device info " << e.getMessage());
      continue;
    } catch (std::exception &e) {
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
                                   "Failed to get device info " << e.what());
      continue;
    } catch (...) {
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000, "Failed to get device info");
      continue;
    }
  }

  return nullptr;
}

void OBCameraNodeDriver::initializeDevice(const std::shared_ptr<ob::Device> &device) {
  device_ = device;
  updatePresetFirmware(preset_firmware_path_);
  CHECK_NOTNULL(device_);
  CHECK_NOTNULL(device_.get());
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  int retry_count = 0;
  constexpr int max_retries = 3;
  bool initialized = false;
  device_info_ = device_->getDeviceInfo();
  RCLCPP_INFO_STREAM(logger_, "Try to connect device via " << device_info_->connectionType());

  while (retry_count < max_retries && !initialized) {
    try {
      ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_, parameters_,
                                                       node_options_.use_intra_process_comms());
      initialized = true;
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device (Attempt "
                                       << retry_count + 1 << " of " << max_retries
                                       << "): " << e.getMessage());
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device (Attempt " << retry_count + 1
                                                                           << " of " << max_retries
                                                                           << "): " << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device (Attempt "
                                       << retry_count + 1 << " of " << max_retries << ")");
    }
    retry_count++;
  }

  if (!initialized) {
    RCLCPP_ERROR_STREAM(logger_,
                        "Device initialization failed after " << max_retries << " attempts.");
    throw std::runtime_error("Device initialization failed after " + std::to_string(max_retries) +
                             " attempts.");
  }

  device_connected_ = true;
  device_info_ = device_->getDeviceInfo();
  serial_number_ = device_info_->getSerialNumber();
  CHECK_NOTNULL(device_info_.get());
  device_unique_id_ = device_info_->getUid();

  if (enable_sync_host_time_ && !isOpenNIDevice(device_info_->pid())) {
    TRY_EXECUTE_BLOCK(device_->timerSyncWithHost());
    if (g_time_domain != "global") {
      sync_host_time_timer_ = this->create_wall_timer(time_sync_period_, [this]() {
        // Multiple safety checks before attempting time sync
        if (!device_) {
          RCLCPP_DEBUG_STREAM(logger_, "sync_host_time_timer_: device is null, skip time sync");
          return;
        }

        // Check device connection status
        if (!device_connected_.load()) {
          RCLCPP_DEBUG_STREAM(logger_,
                              "sync_host_time_timer_: device not connected, skip time sync");
          return;
        }

        // Check if device is in connecting state
        if (device_connecting_.load()) {
          RCLCPP_DEBUG_STREAM(logger_, "sync_host_time_timer_: device connecting, skip time sync");
          return;
        }

        // Check if reset is in progress
        {
          std::unique_lock<decltype(reset_device_mutex_)> reset_lock(reset_device_mutex_,
                                                                     std::try_to_lock);
          if (!reset_lock.owns_lock() || reset_device_flag_) {
            RCLCPP_DEBUG_STREAM(logger_,
                                "sync_host_time_timer_: device reset in progress, skip time sync");
            return;
          }
        }

        // Additional safety check with device lock
        std::unique_lock<decltype(device_lock_)> device_lock(device_lock_, std::try_to_lock);
        if (!device_lock.owns_lock()) {
          RCLCPP_DEBUG_STREAM(logger_,
                              "sync_host_time_timer_: cannot acquire device lock, skip time sync");
          return;
        }

        // Verify device is still valid after acquiring lock
        if (!device_) {
          RCLCPP_DEBUG_STREAM(
              logger_, "sync_host_time_timer_: device became null after lock, skip time sync");
          return;
        }

        RCLCPP_INFO_STREAM(logger_, "Sync device time with host");
        TRY_EXECUTE_BLOCK(device_->timerSyncWithHost());
      });
    }
  }

  // Safely log device information - these calls can throw if device disconnects
  TRY_EXECUTE_BLOCK({
    RCLCPP_INFO_STREAM(logger_, "Device " << device_info_->getName() << " connected");
    RCLCPP_INFO_STREAM(logger_, "Serial number: " << device_info_->getSerialNumber());
    RCLCPP_INFO_STREAM(logger_, "Firmware version: " << device_info_->getFirmwareVersion());
    RCLCPP_INFO_STREAM(logger_, "Hardware version: " << device_info_->getHardwareVersion());
    RCLCPP_INFO_STREAM(logger_, "usb connect type: " << device_info_->getConnectionType());
  });
  RCLCPP_INFO_STREAM(logger_, "device unique id: " << device_unique_id_);
  RCLCPP_INFO_STREAM(logger_, "Current node pid: " << getpid());
  auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start_time_);
  RCLCPP_INFO_STREAM(logger_, "Start device cost " << time_cost.count() << " ms");

  if (!upgrade_firmware_.empty()) {
    firmware_update_success_ = false;

    TRY_EXECUTE_BLOCK({
      ob_camera_node_->withDeviceLock([&]() {
        device_->updateFirmware(
            upgrade_firmware_.c_str(),
            std::bind(&OBCameraNodeDriver::firmwareUpdateCallback, this, std::placeholders::_1,
                      std::placeholders::_2, std::placeholders::_3),
            false);
      });
    });
    if (firmware_update_success_) {
      return;
    }
  }

  if (ob_camera_node_) {
    ob_camera_node_->startIMU();
    ob_camera_node_->startStreams();
  } else {
    RCLCPP_INFO_STREAM(logger_, "ob_camera_node_ is nullptr");
  }

}  // namespace orbbec_camera

bool OBCameraNodeDriver::applyForceIpConfig() {
  if (!force_ip_enable_) {
    RCLCPP_DEBUG(logger_, "[ForceIP] Disabled, skip config");
    return false;
  }
  if (force_ip_success_) {
    RCLCPP_DEBUG(logger_, "[ForceIP] Already applied, skip");
    return false;
  }

  OBNetIpConfig config{};
  config.dhcp = force_ip_dhcp_ ? 1 : 0;

  if (config.dhcp == 0) {
    RCLCPP_INFO(logger_, "[ForceIP] Static config mode");
    auto strToIp = [&](const std::string &s, uint8_t out[4]) -> bool {
      std::stringstream ss(s);
      std::string item;
      int i = 0;
      while (std::getline(ss, item, '.') && i < 4) {
        int val = std::stoi(item);
        if (val < 0 || val > 255) return false;
        out[i++] = static_cast<uint8_t>(val);
      }
      return i == 4;
    };
    uint8_t ip[4], mask[4], gw[4];
    if (!strToIp(force_ip_address_, ip)) {
      RCLCPP_ERROR(logger_, "[ForceIP] Invalid IP: %s", force_ip_address_.c_str());
      return false;
    }
    if (!strToIp(force_ip_subnet_mask_, mask)) {
      RCLCPP_ERROR(logger_, "[ForceIP] Invalid Mask: %s", force_ip_subnet_mask_.c_str());
      return false;
    }
    if (!strToIp(force_ip_gateway_, gw)) {
      RCLCPP_ERROR(logger_, "[ForceIP] Invalid Gateway: %s", force_ip_gateway_.c_str());
      return false;
    }
    std::memcpy(config.address, ip, 4);
    std::memcpy(config.mask, mask, 4);
    std::memcpy(config.gateway, gw, 4);
  }

  force_ip_success_ = false;
  try {
    auto device_list = ctx_->queryDeviceList();
    uint32_t index = 0;
    std::string mac;
    if (!force_ip_mac_.empty()) {
      mac = force_ip_mac_;
    } else if (device_list->getCount() == 1) {
      mac = device_list->getUid(index);
    } else {
      RCLCPP_ERROR(logger_, "[ForceIP] MAC address is empty");
      return false;
    }

    if (ctx_->forceIp(mac.c_str(), config)) {
      RCLCPP_INFO(logger_, "[ForceIP] Config applied. dhcp=%d ip=%s mask=%s gw=%s", config.dhcp,
                  force_ip_address_.c_str(), force_ip_subnet_mask_.c_str(),
                  force_ip_gateway_.c_str());
      force_ip_success_ = true;
    } else {
      RCLCPP_ERROR(logger_, "[ForceIP] Failed to apply config (SDK returned false)");
    }
  } catch (const ob::Error &e) {
    RCLCPP_ERROR(logger_, "[ForceIP] ob::Error: %s", e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[ForceIP] std::exception: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger_, "[ForceIP] Unknown error");
  }

  return force_ip_success_;
}

void OBCameraNodeDriver::connectNetDevice(const std::string &net_device_ip, int net_device_port) {
  if (net_device_ip.empty() || net_device_port == 0) {
    RCLCPP_ERROR_STREAM(logger_, "Invalid net device ip or port");
    return;
  }

  // Check if already connecting
  bool expected = false;
  if (!device_connecting_.compare_exchange_strong(expected, true)) {
    RCLCPP_DEBUG_STREAM(logger_, "connectNetDevice: connection already in progress");
    return;
  }

  // Use RAII to ensure connecting flag is cleared
  std::shared_ptr<int> connecting_guard(nullptr,
                                        [this](int *) { device_connecting_.store(false); });

  std::this_thread::sleep_for(std::chrono::milliseconds(connection_delay_));
  auto device = ctx_->createNetDevice(net_device_ip.c_str(), net_device_port, device_access_mode_);
  if (device == nullptr) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to connect to net device " << net_device_ip);
    return;
  }
  try {
    initializeDevice(device);
    if (!device_connected_) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to initialize net device " << net_device_ip);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "Exception during net device initialization: " << e.what());
    device_connected_ = false;
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "Unknown exception during net device initialization");
    device_connected_ = false;
  }
}

void OBCameraNodeDriver::startDevice(const std::shared_ptr<ob::DeviceList> &list) {
  if (device_connected_.load()) {
    return;
  }

  // Try to set connecting flag atomically
  bool expected = false;
  if (!device_connecting_.compare_exchange_strong(expected, true)) {
    RCLCPP_DEBUG_STREAM(logger_, "startDevice: connection already in progress by another thread");
    return;
  }

  // Use RAII to ensure connecting flag is cleared
  std::shared_ptr<int> connecting_guard(nullptr, [this](int *) {
    device_connecting_.store(false);
    RCLCPP_DEBUG_STREAM(logger_, "startDevice: connecting flag cleared");
  });

  if (list->getCount() == 0) {
    RCLCPP_WARN(logger_, "No device found");
    return;
  }

  RCLCPP_INFO_THROTTLE(logger_, *get_clock(), 5000, "startDevice called");

  start_time_ = std::chrono::high_resolution_clock::now();
  if (device_) {
    device_.reset();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(connection_delay_));
  int try_lock_count = 0;
  int max_try_lock_count = 5;

  while (try_lock_count < max_try_lock_count) {
    int try_lock_result = pthread_mutex_trylock(orb_device_lock_);

    if (try_lock_result == 0) {
      // success get lock,break
      break;
    } else if (try_lock_result == EBUSY) {
      RCLCPP_INFO_STREAM(logger_, "Device lock is held by another process, waiting 100ms");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else {
      RCLCPP_ERROR_STREAM(logger_, "Failed to lock orb_device_lock_");
      return;  // Not EBUSY, return
    }

    try_lock_count++;
  }
  if (try_lock_count >= max_try_lock_count) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to lock orb_device_lock_");
    return;
  }

  std::shared_ptr<int> lock_holder(nullptr,
                                   [this](int *) { pthread_mutex_unlock(orb_device_lock_); });
  bool start_device_failed = false;
  try {
    auto start_time = std::chrono::high_resolution_clock::now();
    auto device = selectDevice(list);
    if (device == nullptr) {
      device_connected_ = false;
      return;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO_STREAM(logger_, "Select device cost " << time_cost.count() << " ms");
    start_time = std::chrono::high_resolution_clock::now();
    initializeDevice(device);
    end_time = std::chrono::high_resolution_clock::now();
    time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO_STREAM(logger_, "Initialize device cost " << time_cost.count() << " ms");

    if (firmware_update_success_) {
      firmware_update_success_ = false;
      device_connected_ = false;
      {
        std::unique_lock<decltype(reset_device_mutex_)> reset_device_lock(reset_device_mutex_);
        reset_device_flag_ = true;
      }
      reset_device_cond_.notify_all();
      return;
    }

    auto pid = device->getDeviceInfo()->getPid();
    if (GEMINI_335LG_PID == pid) {
      ob_camera_node_->startGmslTrigger();
    }
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
    {
      std::unique_lock<decltype(reset_device_mutex_)> reset_device_lock(reset_device_mutex_);
      reset_device_flag_ = true;
    }
    reset_device_cond_.notify_all();
  }
}
void OBCameraNodeDriver::updatePresetFirmware(std::string path) {
  if (path.empty()) {
    return;
  } else {
    std::stringstream ss(path);
    std::string path_segment;
    std::vector<std::string> paths;
    OBFwUpdateState updateState = STAT_START;
    bool firstCall = true;

    while (std::getline(ss, path_segment, ',')) {
      paths.push_back(path_segment);
    }
    uint8_t index = 0;
    uint8_t count = static_cast<uint8_t>(paths.size());
    char(*filePaths)[OB_PATH_MAX] = new char[count][OB_PATH_MAX];
    RCLCPP_INFO_STREAM(this->get_logger(), "paths.cout : " << (uint32_t)count);
    for (const auto &p : paths) {
      strcpy(filePaths[index], p.c_str());
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "path: " << (uint32_t)index << ":" << filePaths[index]);
      index++;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Start to update optional depth preset, please wait a moment...");
    try {
      device_->updateOptionalDepthPresets(
          filePaths, count,
          [this, &updateState, &firstCall](OBFwUpdateState state, const char *message,
                                           uint8_t percent) {
            updateState = state;
            presetUpdateCallback(firstCall, state, message, percent);
            // firstCall = false;
          });

      delete[] filePaths;
      filePaths = nullptr;
      if (updateState == STAT_DONE || updateState == STAT_DONE_WITH_DUPLICATES) {
        RCLCPP_INFO_STREAM(this->get_logger(), "After updating the preset: ");
        auto presetList = device_->getAvailablePresetList();
        RCLCPP_INFO_STREAM(this->get_logger(), "Preset count: " << presetList->getCount());
        for (uint32_t i = 0; i < presetList->getCount(); ++i) {
          RCLCPP_INFO_STREAM(this->get_logger(), "  - " << presetList->getName(i));
        }
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Current preset: " << device_->getCurrentPresetName());
        std::string key = "PresetVer";
        if (device_->isExtensionInfoExist(key)) {
          std::string value = device_->getExtensionInfo(key);
          RCLCPP_INFO_STREAM(this->get_logger(), "Preset version: " << value);
        } else {
          RCLCPP_INFO_STREAM(this->get_logger(), "PresetVer: ");
        }
      }
    } catch (ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to update Preset Firmware " << e.getMessage());
    } catch (std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to update Preset Firmware " << e.what());
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to update Preset Firmware");
    }
  }
}
void OBCameraNodeDriver::presetUpdateCallback(bool firstCall, OBFwUpdateState state,
                                              const char *message, uint8_t percent) {
  if (!firstCall) {
    std::cout << "\033[3F";
  }

  std::cout << "\033[K";
  std::cout << "Progress: " << static_cast<uint32_t>(percent) << "%" << std::endl;

  std::cout << "\033[K";
  std::cout << "Status  : ";
  switch (state) {
    case STAT_VERIFY_SUCCESS:
      std::cout << "Image file verification success" << std::endl;
      break;
    case STAT_FILE_TRANSFER:
      std::cout << "File transfer in progress" << std::endl;
      break;
    case STAT_DONE:
      std::cout << "Update completed" << std::endl;
      break;
    case STAT_DONE_WITH_DUPLICATES:
      std::cout << "Update completed, duplicated presets have been ignored" << std::endl;
      break;
    case STAT_IN_PROGRESS:
      std::cout << "Update in progress" << std::endl;
      break;
    case STAT_START:
      std::cout << "Starting the update" << std::endl;
      break;
    case STAT_VERIFY_IMAGE:
      std::cout << "Verifying image file" << std::endl;
      break;
    default:
      std::cout << "Unknown status or error" << std::endl;
      break;
  }

  std::cout << "\033[K";
  std::cout << "Message : " << message << std::endl << std::flush;
}
void OBCameraNodeDriver::firmwareUpdateCallback(OBFwUpdateState state, const char *message,
                                                uint8_t percent) {
  std::cout << "\033[K";  // Clear the current line
  std::cout << "Progress: " << static_cast<uint32_t>(percent) << "%" << std::endl;

  std::cout << "\033[K";
  std::cout << "Status  : ";
  switch (state) {
    case STAT_VERIFY_SUCCESS:
      std::cout << "Image file verification success" << std::endl;
      break;
    case STAT_FILE_TRANSFER:
      std::cout << "File transfer in progress" << std::endl;
      break;
    case STAT_DONE:
      std::cout << "Update completed" << std::endl;
      break;
    case STAT_IN_PROGRESS:
      std::cout << "Upgrade in progress" << std::endl;
      break;
    case STAT_START:
      std::cout << "Starting the upgrade" << std::endl;
      break;
    case STAT_VERIFY_IMAGE:
      std::cout << "Verifying image file" << std::endl;
      break;
    default:
      std::cout << "Unknown status or error" << std::endl;
      break;
  }

  std::cout << "\033[K";
  std::cout << "Message : " << message << std::endl << std::flush;
  if (state == STAT_DONE) {
    RCLCPP_INFO(logger_, "Reboot device");

    // Don't call clean() here to avoid deadlock - just stop timers and reboot
    // The resetDevice thread will handle proper cleanup when device disconnects
    if (sync_host_time_timer_) {
      try {
        sync_host_time_timer_->cancel();
        sync_host_time_timer_.reset();
      } catch (...) {
        RCLCPP_WARN_STREAM(logger_, "Exception during sync timer cleanup in firmware update");
      }
    }

    device_->reboot();
    device_connected_ = false;
    upgrade_firmware_ = "";

    firmware_update_success_ = true;
  }
}

OBDeviceAccessMode OBCameraNodeDriver::stringToAccessMode(const std::string &mode_str) {
  std::string lower_mode;
  std::transform(mode_str.begin(), mode_str.end(), std::back_inserter(lower_mode),
                 [](auto ch) { return tolower(ch); });

  if (lower_mode == "exclusive") {
    return OB_DEVICE_EXCLUSIVE_ACCESS;
  } else if (lower_mode == "control") {
    return OB_DEVICE_CONTROL_ACCESS;
  } else if (lower_mode == "monitor") {
    return OB_DEVICE_MONITOR_ACCESS;
  } else if (lower_mode == "default") {
    return OB_DEVICE_DEFAULT_ACCESS;
  } else {
    RCLCPP_WARN_STREAM(logger_, "Unknown access mode: " << mode_str << ", using default");
    return OB_DEVICE_DEFAULT_ACCESS;
  }
}

std::string OBCameraNodeDriver::accessModeToString(OBDeviceAccessMode mode) {
  switch (mode) {
    case OB_DEVICE_EXCLUSIVE_ACCESS:
      return "exclusive";
    case OB_DEVICE_CONTROL_ACCESS:
      return "control";
    case OB_DEVICE_MONITOR_ACCESS:
      return "monitor";
    case OB_DEVICE_ACCESS_DENIED:
      return "denied";
    case OB_DEVICE_DEFAULT_ACCESS:
    default:
      return "default";
  }
}
}  // namespace orbbec_camera

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::OBCameraNodeDriver)
