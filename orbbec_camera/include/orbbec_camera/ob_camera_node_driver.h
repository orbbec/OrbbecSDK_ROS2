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
#pragma once
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <semaphore.h>
#include "ob_camera_node.h"
#include "utils.h"
#include "dynamic_params.h"

#include "libobsensor/ObSensor.hpp"

namespace orbbec_camera {

enum DeviceConnectionEvent {
  kDeviceConnected = 0,
  kDeviceDisconnected,
  kOtherDeviceConnected,
  kOtherDeviceDisconnected,
  kDeviceCountUpdate,
};

class OBCameraNodeDriver : public rclcpp::Node {
 public:
  explicit OBCameraNodeDriver(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  OBCameraNodeDriver(const std::string& node_name, const std::string& ns,
                     const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~OBCameraNodeDriver() override;

 private:
  void init();

  void releaseDeviceSemaphore(sem_t* device_sem, int& num_devices_connected);

  void updateConnectedDeviceCount(int& num_devices_connected,
                                  DeviceConnectionEvent connection_event);

  std::shared_ptr<ob::Device> selectDevice(const std::shared_ptr<ob::DeviceList>& list);

  std::shared_ptr<ob::Device> selectDeviceBySerialNumber(
      const std::shared_ptr<ob::DeviceList>& list, const std::string& serial_number);

  std::shared_ptr<ob::Device> selectDeviceByUSBPort(const std::shared_ptr<ob::DeviceList>& list,
                                                    const std::string& usb_port);

  void initializeDevice(const std::shared_ptr<ob::Device>& device);

  void startDevice(const std::shared_ptr<ob::DeviceList>& list);

  void onDeviceConnected(const std::shared_ptr<ob::DeviceList>& device_list);

  void onDeviceDisconnected(const std::shared_ptr<ob::DeviceList>& device_list);

  static OBLogSeverity obLogSeverityFromString(const std::string_view& log_level);

  void checkConnectTimer();

  void queryDevice();

  void deviceCountUpdate();

  void syncTime();

  void resetDevice();

 private:
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
  std::shared_ptr<Parameters> parameters_ = nullptr;
  std::shared_ptr<std::thread> query_thread_ = nullptr;
  std::shared_ptr<std::thread> device_count_update_thread_ = nullptr;
  std::recursive_mutex device_lock_;
  int device_num_ = 1;
  int num_devices_connected_ = 0;
  rclcpp::TimerBase::SharedPtr check_connect_timer_ = nullptr;
  std::shared_ptr<std::thread> sync_time_thread_ = nullptr;
  std::shared_ptr<std::thread> reset_device_thread_ = nullptr;
  std::mutex reset_device_mutex_;
  std::condition_variable reset_device_cond_;
  std::atomic_bool reset_device_flag_{false};
};
}  // namespace orbbec_camera
