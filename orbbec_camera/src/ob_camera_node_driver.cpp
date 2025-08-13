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

namespace orbbec_camera {
backward::SignalHandling OBCameraNodeDriver::sh;
OBCameraNodeDriver::OBCameraNodeDriver(const rclcpp::NodeOptions &node_options)
    : Node("orbbec_camera_node", "/", node_options),
      node_options_(node_options),
      config_path_(ament_index_cpp::get_package_share_directory("orbbec_camera") +
                   "/config/OrbbecSDKConfig_v2.0.xml"),
      logger_(this->get_logger()),
      extension_path_(ament_index_cpp::get_package_prefix("orbbec_camera") + "/lib/extensions") {
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
  init();
}

OBCameraNodeDriver::~OBCameraNodeDriver() {
  is_alive_.store(false);

  if (check_connect_timer_) {
    check_connect_timer_.reset();
  }

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
  ob_camera_node_->stopGmslTrigger();
  if (orb_device_lock_shm_fd_ != -1) {
    close(orb_device_lock_shm_fd_);
    orb_device_lock_shm_fd_ = -1;
  }
  shm_unlink(ORB_DEFAULT_LOCK_NAME.c_str());
}

void OBCameraNodeDriver::init() {
  // Set memory allocation parameters to reduce memory fragmentation
  mallopt(M_TRIM_THRESHOLD, 1024*1024*10);
  mallopt(M_TOP_PAD, 128*1024);

  // Set signal handlers for crash reporting
  signal(SIGSEGV, signalHandler);  // segment fault
  signal(SIGABRT, signalHandler);  // abort
  signal(SIGFPE, signalHandler);   // float point exception
  signal(SIGILL, signalHandler);   // illegal instruction
  ob::Context::setExtensionsDirectory(extension_path_.c_str());
  if (config_path_.empty()) {
    ctx_ = std::make_unique<ob::Context>();
  } else {
    ctx_ = std::make_unique<ob::Context>(config_path_.c_str());
  }

  auto log_level_str = declare_parameter<std::string>("log_level", "none");
  auto log_level = obLogSeverityFromString(log_level_str);
  connection_delay_ = static_cast<int>(declare_parameter<int>("connection_delay", 100));
  enable_sync_host_time_ = declare_parameter<bool>("enable_sync_host_time", true);
  upgrade_firmware_ = declare_parameter<std::string>("upgrade_firmware", "");
  g_camera_name = declare_parameter<std::string>("camera_name", g_camera_name);
  g_time_domain = declare_parameter<std::string>("time_domain", g_time_domain);
  preset_firmware_path_ =
      declare_parameter<std::string>("preset_firmware_path", preset_firmware_path_);
  ob::Context::setLoggerToConsole(log_level);
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
  parameters_ = std::make_shared<Parameters>(this);
  serial_number_ = declare_parameter<std::string>("serial_number", "");
  device_num_ = static_cast<int>(declare_parameter<int>("device_num", 1));
  usb_port_ = declare_parameter<std::string>("usb_port", "");
  net_device_ip_ = declare_parameter<std::string>("net_device_ip", "");
  net_device_port_ = static_cast<int>(declare_parameter<int>("net_device_port", 0));
  enumerate_net_device_ = declare_parameter<bool>("enumerate_net_device", false);
  uvc_backend_ = declare_parameter<std::string>("uvc_backend", "libuvc");
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
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
  reset_device_thread_ = std::make_shared<std::thread>([this]() { resetDevice(); });
}

void OBCameraNodeDriver::onDeviceConnected(const std::shared_ptr<ob::DeviceList> &device_list) {
  CHECK_NOTNULL(device_list);
  {
    std::unique_lock<decltype(reset_device_mutex_)> reset_lock(reset_device_mutex_);
    if (reset_device_flag_) {
      RCLCPP_INFO_STREAM(logger_, "onDeviceConnected : device reset in progress, waiting...");
      reset_device_cond_.wait(reset_lock, [this]() { return !reset_device_flag_ || !is_alive_ || !rclcpp::ok(); });
      if (!is_alive_) {
        return;
      }
      RCLCPP_INFO_STREAM(logger_, "onDeviceConnected : device reset completed, continuing connection");
    }
  }
  if (device_list->getCount() == 0) {
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
  RCLCPP_INFO_STREAM(logger_, "onDeviceDisconnected");
  for (size_t i = 0; i < device_list->getCount(); i++) {
    std::string uid = device_list->getUid(i);
    std::string serial_number = device_list->getSerialNumber(i);
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
  while (is_alive_ && rclcpp::ok() && !device_connected_.load()) {
    if (!enumerate_net_device_ && !net_device_ip_.empty() && net_device_port_ != 0) {
      connectNetDevice(net_device_ip_, net_device_port_);
    } else {
      auto device_list = ctx_->queryDeviceList();
      if (device_list->getCount() == 0) {
        RCLCPP_INFO_STREAM(logger_,
                           "queryDevice :No Device found, using usb event to trigger  "
                           "OBCameraNodeDriver::onDeviceConnected");
        return;
      }
      startDevice(device_list);
    }
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
    }
    reset_device_flag_ = false;
    reset_device_cond_.notify_all();
    malloc_trim(0);
    RCLCPP_INFO_STREAM(logger_, "Reset device uid: " << device_unique_id_ << " done");
  }
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

  std::shared_ptr<int> process_lock_guard(nullptr,
                                        [this](int *) { pthread_mutex_unlock(orb_device_lock_); });

  try {
    std::unique_lock<decltype(reset_device_mutex_)> reset_lock(reset_device_mutex_);
    {
      std::lock_guard<decltype(device_lock_)> device_lock(device_lock_);

      if (!device_connected_ || !ob_camera_node_) {
        RCLCPP_INFO(logger_, "Device not connected");
        return;
      }

      std::string current_device_uid = device_unique_id_;
      RCLCPP_INFO_STREAM(logger_, "Rebooting device with UID: " << current_device_uid);

      ob_camera_node_->rebootDevice();
    }

    RCLCPP_INFO(logger_, "Device reboot initiated, waiting for reconnection");
    malloc_trim(0);
    return;

  } catch (std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to reboot device: " << e.what());
    return;
  } catch (...) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to reboot device: unknown error");
    return;
  }
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDevice(
    const std::shared_ptr<ob::DeviceList> &list) {
  std::shared_ptr<ob::Device> device = nullptr;
  if (!net_device_ip_.empty() && net_device_port_ != 0) {
    RCLCPP_INFO_STREAM(logger_, "Connecting to device with net ip: " << net_device_ip_);
    device = selectDeviceByNetIP(list, net_device_ip_);
  } else if (!serial_number_.empty()) {
    RCLCPP_INFO_STREAM(logger_, "Connecting to device with serial number: " << serial_number_);
    device = selectDeviceBySerialNumber(list, serial_number_);
  } else if (!usb_port_.empty()) {
    RCLCPP_INFO_STREAM(logger_, "Connecting to device with usb port: " << usb_port_);
    device = selectDeviceByUSBPort(list, usb_port_);
  } else if (device_num_ == 1) {
    RCLCPP_INFO_STREAM(logger_, "Connecting to the default device");
    return list->getDevice(0);
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
  for (size_t i = 0; i < list->getCount(); i++) {
    RCLCPP_INFO_STREAM(logger_, "Before lock: Select device serial number: " << serial_number);
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    RCLCPP_INFO_STREAM(logger_, "After lock: Select device serial number: " << serial_number);
    try {
      auto pid = list->getPid(i);
      if (isOpenNIDevice(pid)) {
        // openNI device
        auto device = list->getDevice(i);
        auto device_info = device->getDeviceInfo();
        if (device_info->getSerialNumber() == serial_number) {
          RCLCPP_INFO_STREAM(
              logger_, "Device serial number " << device_info->getSerialNumber() << " matched");
          return device;
        }
      } else {
        std::string sn = list->getSerialNumber(i);
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
    if (device) {
      RCLCPP_INFO_STREAM(logger_, "getDeviceByUid device usb port " << usb_port << " done");
    } else {
      RCLCPP_ERROR_STREAM(logger_, "getDeviceByUid device usb port " << usb_port << " failed");
      RCLCPP_ERROR_STREAM(logger_,
                          "Please use script to get usb port: "
                          "ros2 run orbbec_camera list_devices_node");
    }
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

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDeviceByNetIP(
    const std::shared_ptr<ob::DeviceList> &list, const std::string &net_ip) {
  RCLCPP_INFO_STREAM(logger_, "Before lock: Select device net ip: " << net_ip);
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  RCLCPP_INFO_STREAM(logger_, "After lock: Select device net ip: " << net_ip);
  std::shared_ptr<ob::Device> device = nullptr;
  for (size_t i = 0; i < list->getCount(); i++) {
    try {
      if (std::string(list->getConnectionType(i)) != "Ethernet") {
        continue;
      }
      if (list->getIpAddress(i) == nullptr) {
        continue;
      }
      RCLCPP_INFO_STREAM(logger_, "FindDeviceByNetIP device net ip " << list->getIpAddress(i));
      if (std::string(list->getIpAddress(i)) == net_ip) {
        RCLCPP_INFO_STREAM(logger_, "getDeviceByNetIP device net ip " << net_ip << " done");
        return list->getDevice(i);
      }
    } catch (ob::Error &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get device info " << e.getMessage());
      continue;
    } catch (std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get device info " << e.what());
      continue;
    } catch (...) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to get device info");
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
      sync_host_time_timer_ = this->create_wall_timer(std::chrono::milliseconds(60000), [this]() {
        if (device_) {
          TRY_EXECUTE_BLOCK(device_->timerSyncWithHost());
        }
      });
    }
  }

  RCLCPP_INFO_STREAM(logger_, "Device " << device_info_->getName() << " connected");
  RCLCPP_INFO_STREAM(logger_, "Serial number: " << device_info_->getSerialNumber());
  RCLCPP_INFO_STREAM(logger_, "Firmware version: " << device_info_->getFirmwareVersion());
  RCLCPP_INFO_STREAM(logger_, "Hardware version: " << device_info_->getHardwareVersion());
  RCLCPP_INFO_STREAM(logger_, "device unique id: " << device_unique_id_);
  RCLCPP_INFO_STREAM(logger_, "Current node pid: " << getpid());
  RCLCPP_INFO_STREAM(logger_, "usb connect type: " << device_info_->getConnectionType());
  auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start_time_);
  RCLCPP_INFO_STREAM(logger_, "Start device cost " << time_cost.count() << " ms");

  if (!upgrade_firmware_.empty()) {
    firmware_update_success_ = false;

    ob_camera_node_->withDeviceLock([&]() {
      device_->updateFirmware(
          upgrade_firmware_.c_str(),
          std::bind(&OBCameraNodeDriver::firmwareUpdateCallback, this, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3),
          false);
    });
    if(firmware_update_success_)
    {
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

void OBCameraNodeDriver::connectNetDevice(const std::string &net_device_ip, int net_device_port) {
  if (net_device_ip.empty() || net_device_port == 0) {
    RCLCPP_ERROR_STREAM(logger_, "Invalid net device ip or port");
    return;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(connection_delay_));
  auto device = ctx_->createNetDevice(net_device_ip.c_str(), net_device_port);
  if (device == nullptr) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to connect to net device " << net_device_ip);
    return;
  }
  initializeDevice(device);
}

void OBCameraNodeDriver::startDevice(const std::shared_ptr<ob::DeviceList> &list) {
  if (device_connected_) {
    return;
  }
  if (list->getCount() == 0) {
    RCLCPP_WARN(logger_, "No device found");
    return;
  }
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
    auto end_time = std::chrono::high_resolution_clock::now();
    auto time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO_STREAM(logger_, "Select device cost " << time_cost.count() << " ms");
    if (device == nullptr) {
      RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Device with serial number %s not found",
                           serial_number_.c_str());
      device_connected_ = false;
      return;
    }
    start_time = std::chrono::high_resolution_clock::now();
    initializeDevice(device);
    end_time = std::chrono::high_resolution_clock::now();
    time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO_STREAM(logger_, "Initialize device cost " << time_cost.count() << " ms");

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
    std::unique_lock<decltype(reset_device_mutex_)> reset_device_lock(reset_device_mutex_);
    reset_device_flag_ = true;
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
    char (*filePaths)[OB_PATH_MAX] = new char[count][OB_PATH_MAX];
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
    device_->reboot();
    device_connected_ = false;
    upgrade_firmware_ = "";

    firmware_update_success_ = true;
  }
}
}  // namespace orbbec_camera

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::OBCameraNodeDriver)
