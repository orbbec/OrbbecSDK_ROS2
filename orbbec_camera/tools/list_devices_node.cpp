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
#include <rclcpp/rclcpp.hpp>

#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>

int main() {
  try {
    ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
    auto context = std::make_unique<ob::Context>();
    auto list = context->queryDeviceList();
    for (size_t i = 0; i < list->deviceCount(); i++) {
      if (std::string(list->getConnectionType(i)) != "Ethernet") {
        std::string serial = list->serialNumber(i);
        std::string uid = list->uid(i);
        auto usb_port = orbbec_camera::parseUsbPort(uid);
        auto connection_type = list->getConnectionType(i);
        std::stringstream pid_hex;
        pid_hex << std::hex << std::setw(4) << std::setfill('0') << list->getPid(i);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "- Name: " << list->getName(i) << ", PID: 0x" << pid_hex.str()
                                      << ", SN/ID: " << serial
                                      << ", Connection: " << connection_type);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "serial: " << serial);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "usb port: " << usb_port);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "usb connect type: " << connection_type);
        std::cout << std::endl;
      } else {
        std::string serial = list->serialNumber(i);
        auto connection_type = list->getConnectionType(i);
        auto ip_address = list->getIpAddress(i);
        std::stringstream pid_hex;
        pid_hex << std::hex << std::setw(4) << std::setfill('0') << list->getPid(i);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "- Name: " << list->getName(i) << ", PID: 0x" << pid_hex.str()
                                      << ", SN/ID: " << serial
                                      << ", Connection: " << connection_type);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "serial: " << serial);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "ip address: " << ip_address);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "usb connect type: " << connection_type);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "MAC address: " << list->getUid(i));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "subnet mask: " << list->getSubnetMask(i));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "gateway: " << list->getGateway(i));
        std::cout << std::endl;
      }
    }
  } catch (ob::Error &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("list_device_node"), e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("list_device_node"), e.what());
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("list_device_node"), "unknown error");
  }
  return 0;
}
