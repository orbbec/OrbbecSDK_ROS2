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
#pragma once
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <semaphore.h>
#include "ob_camera_node.h"
#include "utils.h"
#include "dynamic_params.h"

#include "libobsensor/ObSensor.hpp"
#include <pthread.h>
#include <std_srvs/srv/empty.hpp>
#include <backward_ros/backward.hpp>


namespace orbbec_camera {

class OBCameraNodeDriver : public rclcpp::Node {
 public:
  explicit OBCameraNodeDriver(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  OBCameraNodeDriver(const std::string& node_name, const std::string& ns,
                     const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~OBCameraNodeDriver() override;

 private:
  void init();

  std::shared_ptr<ob::Device> selectDevice(const std::shared_ptr<ob::DeviceList>& list);

  std::shared_ptr<ob::Device> selectDeviceBySerialNumber(
      const std::shared_ptr<ob::DeviceList>& list, const std::string& serial_number);

  std::shared_ptr<ob::Device> selectDeviceByUSBPort(const std::shared_ptr<ob::DeviceList>& list,
                                                    const std::string& usb_port);

  void initializeDevice(const std::shared_ptr<ob::Device>& device);

  void startDevice(const std::shared_ptr<ob::DeviceList>& list);

  void connectNetDevice(const std::string& net_device_ip, int net_device_port);

  void onDeviceConnected(const std::shared_ptr<ob::DeviceList>& device_list);

  void onDeviceDisconnected(const std::shared_ptr<ob::DeviceList>& device_list);

  static OBLogSeverity obLogSeverityFromString(const std::string_view& log_level);

  void checkConnectTimer();

  void queryDevice();

  void resetDevice();

  void rebootDeviceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response);

 private:
  const rclcpp::NodeOptions node_options_;
  std::string config_path_;
  std::unique_ptr<ob::Context> ctx_ = nullptr;
  rclcpp::Logger logger_;
  std::unique_ptr<OBCameraNode> ob_camera_node_ = nullptr;
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::shared_ptr<ob::DeviceInfo> device_info_ = nullptr;
  std::atomic_bool is_alive_{false};
  std::atomic_bool device_connected_{false};
  std::string serial_number_;
  std::string device_unique_id_;
  std::string usb_port_;
  bool enumerate_net_device_ = false;  // default false
  std::shared_ptr<Parameters> parameters_ = nullptr;
  std::shared_ptr<std::thread> query_thread_ = nullptr;
  std::shared_ptr<std::thread> device_count_update_thread_ = nullptr;
  std::recursive_mutex device_lock_;
  int device_num_ = 1;
  rclcpp::TimerBase::SharedPtr check_connect_timer_ = nullptr;
  rclcpp::TimerBase::SharedPtr sync_host_time_timer_ = nullptr;
  std::shared_ptr<std::thread> reset_device_thread_ = nullptr;
  std::mutex reset_device_mutex_;
  std::condition_variable reset_device_cond_;
  std::atomic_bool reset_device_flag_{false};
  pthread_mutex_t* orb_device_lock_ = nullptr;
  pthread_mutexattr_t orb_device_lock_attr_;
  uint8_t* orb_device_lock_shm_addr_ = nullptr;
  int orb_device_lock_shm_fd_ = -1;
  // net config
  std::string net_device_ip_;
  int net_device_port_ = 0;
  int connection_delay_ = 100;
  bool enable_sync_host_time_ = true;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reboot_device_srv_ = nullptr;
  std::chrono::time_point<std::chrono::system_clock> start_time_;
  static backward::SignalHandling sh;  // for stack trace
  bool enable_hardware_reset_ = false;
  bool hardware_reset_done_ = false;
};
}  // namespace orbbec_camera
