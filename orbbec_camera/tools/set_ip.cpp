#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>

#include "libobsensor/ObSensor.hpp"

// The camera mapping provided
const std::map<std::string, std::string> camera_ips = {{"mast_front_downward", "192.168.20.201"},
                                                       {"mast_left_downward", "192.168.20.202"},
                                                       {"mast_right_downward", "192.168.20.203"},
                                                       {"mast_rear_downward", "192.168.20.204"},
                                                       {"rear_upward", "192.168.20.205"}};

bool parseIpString(const std::string& ipStr, uint8_t* ipArr) {
  struct in_addr addr;
  if (inet_pton(AF_INET, ipStr.c_str(), &addr) != 1) return false;
  uint32_t ipInt = ntohl(addr.s_addr);
  ipArr[0] = (ipInt >> 24) & 0xFF;
  ipArr[1] = (ipInt >> 16) & 0xFF;
  ipArr[2] = (ipInt >> 8) & 0xFF;
  ipArr[3] = ipInt & 0xFF;
  return true;
}

void printUsage() {
  std::cout << "\nGemini 335Le Hybrid IP Config Tool\n"
            << "----------------------------------------\n"
            << "Usage: gemini_ip_tool [ -i <IP> | -n <NAME> ] [options]\n\n"
            << "Required (Choose one):\n"
            << "  -i <IP_ADDRESS>    Manually specify the target IP\n"
            << "  -n <CAMERA_NAME>   Use a preset name (overrides -i if both are provided)\n\n"
            << "Valid Camera Names:\n";
  for (const auto& pair : camera_ips) {
    std::cout << "  - " << pair.first << " (" << pair.second << ")\n";
  }
  std::cout << "\nOptions:\n"
            << "  -m <MASK>          (Default: 255.255.255.0)\n"
            << "  -g <GATEWAY>       (Default: 192.168.20.1)\n"
            << "  -h                 (Show help)\n\n";
}

int main(int argc, char** argv) {
  std::string ipStr = "";
  std::string cameraName = "";
  std::string maskStr = "255.255.255.0";
  std::string gatewayStr = "192.168.20.1";
  // Not using dhcp hardcode to false
  bool dhcp = false;
  int opt;

  // Added 'n:' to the getopt string
  while ((opt = getopt(argc, argv, "i:n:m:g:d:h")) != -1) {
    switch (opt) {
      case 'i':
        ipStr = optarg;
        break;
      case 'n':
        cameraName = optarg;
        break;
      case 'm':
        maskStr = optarg;
        break;
      case 'g':
        gatewayStr = optarg;
        break;
      case 'h':
      default:
        printUsage();
        return EXIT_SUCCESS;
    }
  }

  // --- PRIORITY LOGIC ---
  std::string finalIp = "";
  if (!cameraName.empty()) {
    // If name is provided, look it up.
    auto it = camera_ips.find(cameraName);
    if (it != camera_ips.end()) {
      finalIp = it->second;
      std::cout << "Found name mapping: " << cameraName << " -> " << finalIp << "\n";
    } else {
      std::cerr << "Error: Unknown camera name '" << cameraName << "'\n";
      return EXIT_FAILURE;
    }
  } else if (!ipStr.empty()) {
    // If no name but manual IP is provided.
    finalIp = ipStr;
    std::cout << "Using manual IP: " << finalIp << "\n";
  } else {
    // Neither provided.
    std::cerr << "Error: You must provide either an IP address (-i) or a camera name (-n).\n";
    printUsage();
    return EXIT_FAILURE;
  }

  try {
    ob::Context ctx;
    ctx.enableNetDeviceEnumeration(true);

    auto devList = ctx.queryDeviceList();
    if (devList->deviceCount() != 1) {
      std::cerr << "Error: No Orbbec devices found on the network or multiple devices found!\n";
      return EXIT_FAILURE;
    }

    auto dev = devList->getDevice(0);
    auto devInfo = dev->getDeviceInfo();
    std::cout << "Targeting Hardware: " << devInfo->name() << " (SN: " << devInfo->serialNumber()
              << ")\n";

    OBDeviceIpAddrConfig ipConfig;
    ipConfig.dhcp = dhcp;

    if (!parseIpString(finalIp, ipConfig.address) || !parseIpString(maskStr, ipConfig.mask) ||
        !parseIpString(gatewayStr, ipConfig.gateway)) {
      std::cerr << "Error: Invalid network address format.\n";
      return EXIT_FAILURE;
    }

    std::cout << "Writing Configuration: IP=" << finalIp << ", Mask=" << maskStr
              << ", GW=" << gatewayStr << "\n";

    dev->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG, reinterpret_cast<uint8_t*>(&ipConfig),
                           sizeof(OBDeviceIpAddrConfig));

    std::cout << "SUCCESS! Device updated. Rebooting camera...\n";
    dev->reboot();

  } catch (ob::Error& e) {
    std::cerr << "Orbbec SDK Error: " << e.getMessage() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
