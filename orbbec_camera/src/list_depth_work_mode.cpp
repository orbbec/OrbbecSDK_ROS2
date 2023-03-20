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
  auto depthModeList = device->getDepthWorkModeList();
  std::cout << "depth mode list: " << std::endl;
  for (uint32_t i = 0; i < depthModeList->count(); i++) {
    std::cout << "depthModeList[" << i << "]: " << (*depthModeList)[i].name;

    std::cout << std::endl;
  }
  return 0;
}
