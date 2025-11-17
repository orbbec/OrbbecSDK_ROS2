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

#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>

// SRS_Internal_Change
// Device attributes are additionally emitted as srs_yaml_prefix| tagged lines
// so this tool's output can be parsed by other tools.
// End of SRS_Internal_Change

namespace {
constexpr int kFirmwareLogDrainDelaySec = 5;
const std::string kYamlPrefix = "srs_yaml_prefix|";

struct CliArgs {
  bool help = false;
  std::string sdk_log_level = "off";
};

void printUsage() {
  std::cout << "Usage:\n"
            << "  ros2 run orbbec_camera list_devices_node -- [options]\n\n"
            << "Options:\n"
            << "  --sdk_log_level LEVEL  SDK file log level: debug/info/warn/error/fatal/off "
               "(default: off).\n\n"
            << "Examples:\n"
            << "  ros2 run orbbec_camera list_devices_node -- --sdk_log_level debug\n";
}

bool parseArgs(int argc, char **argv, CliArgs &args, std::string &error) {
  for (int i = 1; i < argc; ++i) {
    const std::string current = argv[i];
    if (current == "-h" || current == "--help") {
      args.help = true;
      return true;
    }
    if (current.rfind("--sdk_log_level=", 0) == 0) {
      args.sdk_log_level = current.substr(std::strlen("--sdk_log_level="));
      continue;
    }
    if (current == "--sdk_log_level") {
      if (++i >= argc) {
        error = "--sdk_log_level requires a value";
        return false;
      }
      args.sdk_log_level = argv[i];
      continue;
    }
    error = "Unknown argument: " + current;
    return false;
  }

  const auto log_severity = orbbec_camera::obLogSeverityFromString(args.sdk_log_level);
  if (log_severity == OBLogSeverity::OB_LOG_SEVERITY_OFF && args.sdk_log_level != "off" &&
      args.sdk_log_level != "none") {
    error = "--sdk_log_level expects one of: debug, info, warn, error, fatal, off";
    return false;
  }

  return true;
}

std::string ipSourceTypeToString(int ip_source_type) {
  switch (ip_source_type) {
    case 0:
      return "NONE";
    case 1:
      return "LLA";
    case 2:
      return "DHCP";
    case 3:
      return "PERSISTENT";
    default:
      return std::string("UNKNOWN(") + std::to_string(ip_source_type) + ")";
  }
}

std::string deviceAccessStateToString(OBDeviceAccessState state) {
  switch (state) {
    case OB_DEVICE_ACCESS_STATE_UNKNOWN:
      return "UNKNOWN";
    case OB_DEVICE_ACCESS_STATE_UNSUPPORTED:
      return "UNSUPPORTED";
    case OB_DEVICE_ACCESS_STATE_AVAILABLE:
      return "AVAILABLE";
    case OB_DEVICE_ACCESS_STATE_CONTROLLED:
      return "CONTROLLED";
    case OB_DEVICE_ACCESS_STATE_EXCLUSIVE:
      return "EXCLUSIVE";
    case OB_DEVICE_ACCESS_STATE_UNREACHABLE:
      return "UNREACHABLE";
    case OB_DEVICE_ACCESS_STATE_FW_NOT_SUPPORTED:
      return "FW_NOT_SUPPORTED";
    default:
      return "UNKNOWN(" + std::to_string(static_cast<int>(state)) + ")";
  }
}

void printDeviceAccessState(const std::shared_ptr<ob::DeviceList> &list, uint32_t index) {
  try {
    const auto state = list->queryDeviceAccessState(index);
    std::cout << kYamlPrefix << "  device access state [serial: " << list->getSerialNumber(index)
               << ", ip: " << list->getIpAddress(index) << "]: "
               << deviceAccessStateToString(state) << std::endl;
  } catch (const ob::Error &e) {
    std::cout << kYamlPrefix
               << "  device access state: UNKNOWN (" << orbbec_camera::formatObErrorWithStatus(e)
               << ")" << std::endl;
  } catch (const std::exception &e) {
    std::cout << kYamlPrefix << "  device access state: UNKNOWN (" << e.what() << ")"
               << std::endl;
  }
}

std::string boolToString(bool value) { return value ? "true" : "false"; }

bool isPropertyReadable(const std::shared_ptr<ob::Device> &device, OBPropertyID property_id) {
  return device->isPropertySupported(property_id, OB_PERMISSION_READ) ||
         device->isPropertySupported(property_id, OB_PERMISSION_READ_WRITE);
}

void printIpConfigStatus(const std::shared_ptr<ob::Device> &device) {
  std::cout << kYamlPrefix << "  IP config status:" << std::endl;

  const bool v2_read_supported = isPropertyReadable(device, OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2);
  const bool legacy_read_supported = isPropertyReadable(device, OB_STRUCT_DEVICE_IP_ADDR_CONFIG);

  if (v2_read_supported) {
    OBNetIpConfigV2 ip_config_v2{};
    uint32_t data_size = sizeof(ip_config_v2);
    device->getStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2,
                              reinterpret_cast<uint8_t *>(&ip_config_v2), &data_size);
    std::cout << kYamlPrefix
               << "    DHCP: " << boolToString(ip_config_v2.flags & OB_NET_IP_FLAG_DHCP)
               << std::endl;
    std::cout << kYamlPrefix << "    persistent IP: "
               << boolToString(ip_config_v2.flags & OB_NET_IP_FLAG_PERSISTENT) << std::endl;
  } else if (legacy_read_supported) {
    OBNetIpConfig ip_config{};
    uint32_t data_size = sizeof(ip_config);
    device->getStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG,
                              reinterpret_cast<uint8_t *>(&ip_config), &data_size);
    std::cout << kYamlPrefix << "    DHCP: " << boolToString(ip_config.dhcp != 0) << std::endl;
    std::cout << kYamlPrefix << "    persistent IP: " << boolToString(ip_config.dhcp == 0)
               << std::endl;
  } else {
    std::cout << kYamlPrefix << "    DHCP: not supported" << std::endl;
    std::cout << kYamlPrefix << "    persistent IP: not supported" << std::endl;
  }
}

void printPresetInfo(const std::shared_ptr<ob::Device> &device) {
  try {
    auto preset_list = device->getAvailablePresetList();
    const uint32_t preset_count = preset_list ? preset_list->getCount() : 0;
    std::cout << kYamlPrefix << "  device_preset count: " << preset_count << std::endl;
    for (uint32_t i = 0; i < preset_count; ++i) {
      const char *preset_name = preset_list->getName(i);
      if (preset_name != nullptr && preset_name[0] != '\0') {
        std::cout << kYamlPrefix << "    - " << preset_name << std::endl;
      }
    }

    if (device->isColorPresetSupported()) {
      auto color_preset_list = device->getColorPresetList();
      const uint32_t color_preset_count = color_preset_list ? color_preset_list->getCount() : 0;
      std::cout << kYamlPrefix << "  color_preset count: " << color_preset_count << std::endl;
      for (uint32_t i = 0; i < color_preset_count; ++i) {
        const char *preset_name = color_preset_list->getName(i);
        if (preset_name != nullptr && preset_name[0] != '\0') {
          std::cout << kYamlPrefix << "    - " << preset_name << std::endl;
        }
      }
    }

    std::string key = "PresetVer";
    if (device->isExtensionInfoExist(key)) {
      std::string value = device->getExtensionInfo(key);
      std::cout << kYamlPrefix << "  preset version: " << value << std::endl;
    } else {
      std::cout << kYamlPrefix << "  preset version: not available" << std::endl;
    }
  } catch (ob::Error &e) {
    std::cout << kYamlPrefix
               << "  Failed to get preset info: " << orbbec_camera::formatObErrorWithStatus(e)
               << std::endl;
  } catch (const std::exception &e) {
    std::cout << kYamlPrefix << "  Failed to get preset info: " << e.what() << std::endl;
  } catch (...) {
    std::cout << kYamlPrefix << "  Failed to get preset info" << std::endl;
  }
}

void waitForFirmwareLogDrain() {
  auto logger = rclcpp::get_logger("list_device_node");
  RCLCPP_INFO(logger, "Waiting %d seconds to keep firmware log alive...",
              kFirmwareLogDrainDelaySec);
  std::this_thread::sleep_for(std::chrono::seconds(kFirmwareLogDrainDelaySec));
}

bool enableFirmwareLog(const std::shared_ptr<ob::Device> &device) {
  auto logger = rclcpp::get_logger("list_device_node");
  try {
    device->enableFirmwareLog(true);
    RCLCPP_INFO(logger, "Firmware log enabled.");
    return true;
  } catch (const ob::Error &e) {
    RCLCPP_WARN(logger, "Failed to enable firmware log: %s",
                orbbec_camera::formatObErrorWithStatus(e).c_str());
  } catch (const std::exception &e) {
    RCLCPP_WARN(logger, "Failed to enable firmware log: %s", e.what());
  }
  return false;
}

bool isSdkLogEnabled(const std::string &log_level) {
  return orbbec_camera::obLogSeverityFromString(log_level) != OBLogSeverity::OB_LOG_SEVERITY_OFF;
}
}  // namespace

int main(int argc, char **argv) {
  CliArgs args;
  std::string parse_error;
  if (!parseArgs(argc, argv, args, parse_error)) {
    std::cerr << "Argument error: " << parse_error << std::endl;
    printUsage();
    return 1;
  }
  if (args.help) {
    printUsage();
    return 0;
  }

  try {
    const auto sdk_log_path =
        orbbec_camera::configureObSdkLoggerForTool("list_devices_node", args.sdk_log_level);
    if (!sdk_log_path.empty()) {
      RCLCPP_INFO(rclcpp::get_logger("list_device_node"), "SDK file log enabled: %s",
                  sdk_log_path.c_str());
    }
    auto context = std::make_unique<ob::Context>();
    auto list = context->queryDeviceList();
    bool firmware_log_enabled = false;
    const size_t device_count = list->deviceCount();
    std::cout << kYamlPrefix << "orbbec_camera_device_count: " << device_count << std::endl;
    for (size_t i = 0; i < device_count; i++) {
      try {
        if (std::string(list->getConnectionType(i)) == "Ethernet") {
          printDeviceAccessState(list, static_cast<uint32_t>(i));
        }
        auto device_ = list->getDevice(i);
        if (isSdkLogEnabled(args.sdk_log_level)) {
          firmware_log_enabled = enableFirmwareLog(device_) || firmware_log_enabled;
        }
        auto device_info_ = device_->getDeviceInfo();
        if (std::string(list->getConnectionType(i)) != "Ethernet") {
          std::string serial = list->serialNumber(i);
          std::string uid = list->uid(i);
          auto usb_port = orbbec_camera::parseUsbPort(uid);
          auto connection_type = list->getConnectionType(i);
          auto firmware_version = device_info_->getFirmwareVersion();
          std::stringstream pid_hex;
          pid_hex << std::hex << std::setw(4) << std::setfill('0') << list->getPid(i);
          std::cout << kYamlPrefix << "  name: " << list->getName(i) << std::endl;
          std::cout << kYamlPrefix << "  pid: 0x" << pid_hex.str() << std::endl;
          std::cout << kYamlPrefix << "  serial: " << serial << std::endl;
          std::cout << kYamlPrefix << "  connection: " << connection_type << std::endl;
          std::cout << kYamlPrefix << "  firmware version: " << firmware_version << std::endl;
          std::cout << kYamlPrefix << "  usb port: " << usb_port << std::endl;
          printPresetInfo(device_);
          std::cout << std::endl;
        } else {
          std::string serial = list->serialNumber(i);
          auto connection_type = list->getConnectionType(i);
          auto ip_address = list->getIpAddress(i);
          std::stringstream pid_hex;
          auto firmware_version = device_info_->getFirmwareVersion();
          pid_hex << std::hex << std::setw(4) << std::setfill('0') << list->getPid(i);
          std::cout << kYamlPrefix << "  name: " << list->getName(i) << std::endl;
          std::cout << kYamlPrefix << "  pid: 0x" << pid_hex.str() << std::endl;
          std::cout << kYamlPrefix << "  serial: " << serial << std::endl;
          std::cout << kYamlPrefix << "  connection: " << connection_type << std::endl;
          std::cout << kYamlPrefix << "  firmware version: " << firmware_version << std::endl;
          std::cout << kYamlPrefix << "  ip address: " << ip_address << std::endl;
          std::cout << kYamlPrefix << "  MAC address: " << list->getUid(i) << std::endl;
          std::cout << kYamlPrefix << "  subnet mask: " << list->getSubnetMask(i) << std::endl;
          std::cout << kYamlPrefix << "  gateway: " << list->getGateway(i) << std::endl;
          std::cout << kYamlPrefix << "  local net interface: "
                     << list->getLocalNetInterfaceName(static_cast<uint32_t>(i)) << std::endl;
          std::cout << kYamlPrefix << "  local MAC address: "
                     << list->getLocalMacAddress(static_cast<uint32_t>(i)) << std::endl;
          std::cout << kYamlPrefix << "  ip source type: "
                     << ipSourceTypeToString(
                            static_cast<int>(list->getIpSourceType(static_cast<uint32_t>(i))))
                     << std::endl;
          printIpConfigStatus(device_);
          printPresetInfo(device_);
          std::cout << std::endl;
        }
      } catch (ob::Error &e) {
        std::cerr << kYamlPrefix << "error: Failed to list device at index " << i << ": "
                   << orbbec_camera::formatObErrorWithStatus(e) << std::endl;
      } catch (const std::exception &e) {
        std::cerr << kYamlPrefix << "error: Failed to list device at index " << i << ": "
                   << e.what() << std::endl;
      } catch (...) {
        std::cerr << kYamlPrefix << "error: Failed to list device at index " << i
                   << ": unknown error" << std::endl;
      }
    }
    if (firmware_log_enabled) {
      waitForFirmwareLogDrain();
    }
  } catch (ob::Error &e) {
    std::cerr << kYamlPrefix << "error: " << orbbec_camera::formatObErrorWithStatus(e)
               << std::endl;
  } catch (const std::exception &e) {
    std::cerr << kYamlPrefix << "exception: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << kYamlPrefix << "unknown_error:" << std::endl;
  }
  return 0;
}
