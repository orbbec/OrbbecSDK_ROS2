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
