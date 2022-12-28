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
#include "ob_camera_node.h"
#include "utils.h"
#include "dynamic_params.h"

#include "libobsensor/ObSensor.hpp"

namespace orbbec_camera {
class OBCameraNodeFactory : public rclcpp::Node {
 public:
  explicit OBCameraNodeFactory(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  OBCameraNodeFactory(const std::string& node_name, const std::string& ns,
                      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~OBCameraNodeFactory() override;

 private:
  void init();

  void startDevice(const std::shared_ptr<ob::DeviceList>& list);

  void deviceConnectCallback(const std::shared_ptr<ob::DeviceList>& device_list);

  void deviceDisconnectCallback(const std::shared_ptr<ob::DeviceList>& device_list);

  static OBLogSeverity obLogSeverityFromString(const std::string& log_level);

  void queryDevice();

 private:
  std::unique_ptr<ob::Context> ctx_ = nullptr;
  rclcpp::Logger logger_;
  std::unique_ptr<OBCameraNode> ob_camera_node_ = nullptr;
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::shared_ptr<ob::DeviceInfo> device_info_ = nullptr;
  std::atomic_bool is_alive_{false};
  std::atomic_bool device_connected_{false};
  std::string log_level_;
  std::string serial_number_;
  std::shared_ptr<Parameters> parameters_;
  std::shared_ptr<std::thread> query_thread_ = nullptr;
  std::recursive_mutex device_lock_;
  size_t device_num_ = 1;
};
}  // namespace orbbec_camera

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::OBCameraNodeFactory)
