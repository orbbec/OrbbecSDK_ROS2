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
#pragma once

#include <string>
#include <vector>
#include "libobsensor/ObSensor.hpp"
#include "utils.h"

namespace orbbec_camera {

class JPEGDecoder {
 public:
  JPEGDecoder(int width, int height);

  virtual ~JPEGDecoder();

  virtual bool decode(const std::shared_ptr<ob::ColorFrame> &frame, uint8_t *dest) = 0;

  std::string getErrorMsg() const { return error_msg_; }

 protected:
  int width_ = 0;
  int height_ = 0;
  std::string error_msg_;
};
}  // namespace orbbec_camera
