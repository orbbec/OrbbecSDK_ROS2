#include <rclcpp/rclcpp.hpp>

#include <orbbec_camera/ob_camera_node_driver.h>

int main() {
  auto context = std::make_unique<ob::Context>();
  context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
  auto list = context->queryDeviceList();
  for (size_t i = 0; i < list->deviceCount(); i++) {
    auto serial = list->getDevice(i)->getDeviceInfo()->serialNumber();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"), "serial: " << serial);
  }
  return 0;
}
