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
  for (size_t i = 0; i < sensor_list->count(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profile_list = sensor->getStreamProfileList();
    for (size_t j = 0; j < profile_list->count(); j++) {
      auto origin_profile = profile_list->getProfile(j);
      if (sensor->type() == OB_SENSOR_COLOR || sensor->type() == OB_SENSOR_DEPTH ||
          sensor->type() == OB_SENSOR_IR || sensor->type() == OB_SENSOR_IR_LEFT ||
          sensor->type() == OB_SENSOR_IR_RIGHT) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->type()) << " profile: " << profile->width()
                  << "x" << profile->height() << " " << profile->fps() << "fps "
                  << magic_enum::enum_name(profile->format()) << std::endl;
      } else if (sensor->type() == OB_SENSOR_ACCEL) {
        auto profile = origin_profile->as<ob::AccelStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->type()) << " profile: " << profile->sampleRate()
                  << "  full scale_range " << profile->fullScaleRange() << std::endl;
      } else if (sensor->type() == OB_SENSOR_GYRO) {
        auto profile = origin_profile->as<ob::GyroStreamProfile>();
        std::cout << magic_enum::enum_name(sensor->type()) << " profile: " << profile->sampleRate()
                  << "  full scale_range " << profile->fullScaleRange() << std::endl;
      } else {
        std::cout << "Unknown profile: " << magic_enum::enum_name(sensor->type()) << std::endl;
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
  for (uint32_t i = 0; i < depth_mode_list->count(); i++) {
    std::cout << "Depth_mode_list[" << i << "]: " << (*depth_mode_list)[i].name << std::endl;
  }
}

void printPreset(const std::shared_ptr<ob::Device>& device) {
  auto preset_list = device->getAvailablePresetList();
  if (!preset_list || preset_list->count() == 0) {
    return;
  }
  std::cout << "Preset list:" << std::endl;
  for (uint32_t i = 0; i < preset_list->count(); i++) {
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
