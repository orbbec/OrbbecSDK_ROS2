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

#include "libobsensor/ObSensor.hpp"

namespace orbbec_camera {
class OBCameraNodeFactory : public rclcpp::Node {
 public:
  explicit OBCameraNodeFactory(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  OBCameraNodeFactory(const std::string& node_name, const std::string& ns,
                      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~OBCameraNodeFactory() override ;

 private:
  void init();
  void startDevice();
  void getDevice(const std::shared_ptr<ob::DeviceList>& list);

  void deviceConnectCallback(const std::shared_ptr<ob::DeviceList>& device_list);

  void deviceDisconnectCallback(const std::shared_ptr<ob::DeviceList>& device_list);

 private:
  std::unique_ptr<ob::Context> ctx_;
  rclcpp::Logger logger_;
  std::unique_ptr<OBCameraNode> ob_camera_node_;
  std::shared_ptr<ob::Device> device_;
  std::atomic_bool is_alive_{false};
  std::thread query_thread_;
  std::string serial_number_;
  std::string usb_port_id_;
  double reconnect_timeout_;
  double wait_for_device_timeout_;
};
}  // namespace orbbec_camera

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::OBCameraNodeFactory)
