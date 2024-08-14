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
    auto context = std::make_unique<ob::Context>();
    context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
    auto list = context->queryDeviceList();
    for (size_t i = 0; i < list->deviceCount(); i++) {
      auto device = list->getDevice(i);
      auto device_info = device->getDeviceInfo();
      std::string serial = device_info->serialNumber();
      std::string uid = device_info->uid();
      auto usb_port = orbbec_camera::parseUsbPort(uid);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "serial: " << serial);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "usb port: " << usb_port);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("list_device_node"), e.what());
  } catch (ob::Error &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("list_device_node"), e.getMessage());
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("list_device_node"), "unknown error");
  }
  return 0;
}
