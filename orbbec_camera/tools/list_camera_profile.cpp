#include <rclcpp/rclcpp.hpp>
#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/ob_camera_node.h>
#include <memory>
#include <magic_enum/magic_enum.hpp>
#include <iostream>

using namespace orbbec_camera;

std::shared_ptr<ob::Device> initializeDevice(std::shared_ptr<ob::Pipeline> pipeline) {
  auto device = pipeline->getDevice();
  if (!device) {
    std::cout << "No device found" << std::endl;
    return nullptr;
  }
  return device;
}

void listSensorProfiles(const std::shared_ptr<ob::Device>& device) {
  auto sensor_list = device->getSensorList();
  for (size_t i = 0; i < sensor_list->getCount(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profile_list = sensor->getStreamProfileList();
    for (size_t j = 0; j < profile_list->getCount(); j++) {
      auto origin_profile = profile_list->getProfile(j);
      if (sensor->getType() == OB_SENSOR_COLOR || sensor->getType() == OB_SENSOR_DEPTH ||
          sensor->getType() == OB_SENSOR_IR || sensor->getType() == OB_SENSOR_IR_LEFT ||
          sensor->getType() == OB_SENSOR_IR_RIGHT) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->getType()) << " profile: " << profile->getWidth()
                  << "x" << profile->getHeight() << " " << profile->getFps() << "fps "
                  << magic_enum::enum_name(profile->getFormat()) << std::endl;
      } else if (sensor->getType() == OB_SENSOR_ACCEL) {
        auto profile = origin_profile->as<ob::AccelStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->getType()) << " profile: " << profile->getSampleRate()
                  << "  full scale_range " << profile->getFullScaleRange() << std::endl;
      } else if (sensor->getType() == OB_SENSOR_GYRO) {
        auto profile = origin_profile->as<ob::GyroStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->getType()) << " profile: " << profile->getSampleRate()
                  << "  full scale_range " << profile->getFullScaleRange() << std::endl;
      } else {
        std::cout << "Unknown profile: " << magic_enum::enum_name(sensor->getType()) << std::endl;
      }
    }
  }
}

void printDeviceProperties(const std::shared_ptr<ob::Device>& device) {
  if (!device->isPropertySupported(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_READ_WRITE)) {
    std::cout << "Current device not support depth work mode!" << std::endl;
    return;
  }
  auto current_depth_mode = device->getCurrentDepthWorkMode();
  std::cout << "Current depth mode: " << current_depth_mode.name << std::endl;
  auto depth_mode_list = device->getDepthWorkModeList();
  std::cout << "Depth mode list: " << std::endl;
  for (uint32_t i = 0; i < depth_mode_list->getCount(); i++) {
    std::cout << "Depth_mode_list[" << i << "]: " << (*depth_mode_list)[i].name << std::endl;
  }
}

void printPreset(const std::shared_ptr<ob::Device>& device) {
  auto preset_list = device->getAvailablePresetList();
  if (!preset_list || preset_list->getCount() == 0) {
    return;
  }
  std::cout << "Preset list:" << std::endl;
  for (uint32_t i = 0; i < preset_list->getCount(); i++) {
    auto name = preset_list->getName(i);
    std::cout << "Preset list[" << i << "]: " << name << std::endl;
  }
}
int main() {
  auto pipeline = std::make_shared<ob::Pipeline>();
  auto device = initializeDevice(pipeline);
  if (!device) {
    return -1;  // Device initialization failed
  }
  listSensorProfiles(device);
  printDeviceProperties(device);
  printPreset(device);
  return 0;
}
