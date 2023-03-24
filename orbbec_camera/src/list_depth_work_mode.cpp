#include <orbbec_camera/ob_camera_node.h>

int main() {
  std::shared_ptr<ob::Pipeline> pipeline = std::make_shared<ob::Pipeline>();
  auto device = pipeline->getDevice();
  if (!device) {
    std::cout << "No device found" << std::endl;
    return -1;
  }
  if (!device->isPropertySupported(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_READ_WRITE)) {
    std::cout << "Current device not support depth work mode!" << std::endl;
    return -1;
  }
  auto current_depth_mode = device->getCurrentDepthWorkMode();
  std::cout << "current depth mode: " << current_depth_mode.name << std::endl;
  auto depth_mode_list = device->getDepthWorkModeList();
  std::cout << "depth mode list: " << std::endl;
  for (uint32_t i = 0; i < depth_mode_list->count(); i++) {
    std::cout << "depth_mode_list[" << i << "]: " << (*depth_mode_list)[i].name;

    std::cout << std::endl;
  }
  return 0;
}
