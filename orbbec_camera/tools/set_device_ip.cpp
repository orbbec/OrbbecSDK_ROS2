#include "rclcpp/rclcpp.hpp"
#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>
#include <sstream>
#include <string>

using namespace ob;

bool parseIpString(const std::string &ip_str, uint8_t ip[4]) {
  std::stringstream ss(ip_str);
  std::string item;
  int i = 0;
  while (std::getline(ss, item, '.')) {
    if (i >= 4) return false;
    try {
      int num = std::stoi(item);
      if (num < 0 || num > 255) return false;
      ip[i++] = static_cast<uint8_t>(num);
    } catch (...) {
      return false;
    }
  }
  return i == 4;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("set_device_ip");
  auto logger = node->get_logger();

  std::string device_ip_str = node->declare_parameter<std::string>("old_ip", "192.168.1.10");
  int port = node->declare_parameter<int>("port", 8090);
  bool dhcp = node->declare_parameter<bool>("dhcp", false);
  std::string new_ip_str = node->declare_parameter<std::string>("new_ip", "192.168.1.200");
  std::string mask_str = node->declare_parameter<std::string>("mask", "255.255.255.0");
  std::string gateway_str = node->declare_parameter<std::string>("gateway", "192.168.1.1");

  ob_net_ip_config ip_config{};
  ip_config.dhcp = dhcp ? 1 : 0;

  if (!parseIpString(new_ip_str, ip_config.address)) {
    RCLCPP_ERROR(logger, "Invalid new_ip format: %s", new_ip_str.c_str());
    return 1;
  }
  if (!parseIpString(mask_str, ip_config.mask)) {
    RCLCPP_ERROR(logger, "Invalid mask format: %s", mask_str.c_str());
    return 1;
  }
  if (!parseIpString(gateway_str, ip_config.gateway)) {
    RCLCPP_ERROR(logger, "Invalid gateway format: %s", gateway_str.c_str());
    return 1;
  }

  try {
    RCLCPP_INFO(logger, "Connecting to device %s:%d ...", device_ip_str.c_str(), port);
    ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
    auto context = std::make_shared<ob::Context>();
    auto device = context->createNetDevice(device_ip_str.c_str(), port);

    RCLCPP_INFO(logger, "Setting new IP configuration...");
    device->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG,
                              reinterpret_cast<const uint8_t *>(&ip_config), sizeof(ip_config));

    RCLCPP_INFO(logger, "IP configuration applied successfully.");
    if (dhcp) {
      RCLCPP_INFO(logger, "DHCP mode enabled.");
    } else {
      RCLCPP_INFO(logger, "Static IP set to %d.%d.%d.%d", ip_config.address[0],
                  ip_config.address[1], ip_config.address[2], ip_config.address[3]);
      RCLCPP_INFO(logger, "Mask: %d.%d.%d.%d", ip_config.mask[0], ip_config.mask[1],
                  ip_config.mask[2], ip_config.mask[3]);
      RCLCPP_INFO(logger, "Gateway: %d.%d.%d.%d", ip_config.gateway[0], ip_config.gateway[1],
                  ip_config.gateway[2], ip_config.gateway[3]);
    }

  } catch (ob::Error &e) {
    RCLCPP_ERROR(logger, "set_device_ip: %s", e.getMessage());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "set_device_ip: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger, "set_device_ip: unknown error");
  }

  rclcpp::shutdown();
  return 0;
}