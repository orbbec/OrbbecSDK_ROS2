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
#include "utils.h"
#include <opencv2/opencv.hpp>

#include "jpeg_decoder.h"
#include <NvJpegDecoder.h>
#include <NvUtils.h>
#include <NvV4l2Element.h>
#include <NvJpegDecoder.h>
#include <NvV4l2Element.h>

namespace orbbec_camera {
class JetsonNvJPEGDecoder : public JPEGDecoder {
 public:
  JetsonNvJPEGDecoder(int width, int height);
  ~JetsonNvJPEGDecoder() override;

  bool decode(const std::shared_ptr<ob::ColorFrame>& frame, uint8_t* dest) override;

 private:
  NvJPEGDecoder* decoder_;
};
}  // namespace orbbec_camera