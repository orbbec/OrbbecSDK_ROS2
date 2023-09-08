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
