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

#include "jpeg_decoder.h"
#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_err.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_log.h>
#include <rockchip/mpp_packet.h>
#include <rockchip/mpp_rc_defs.h>
#include <rockchip/mpp_task.h>
#include <rockchip/rk_mpi.h>
#include "utils.h"

#if defined(USE_LIBYUV)
#include <libyuv.h>
#else
#include <rga/RgaApi.h>
#endif
#define MPP_ALIGN(x, a) (((x) + (a)-1) & ~((a)-1))

namespace orbbec_camera {
class RKJPEGDecoder : public JPEGDecoder {
 public:
  RKJPEGDecoder(int width, int height);

  ~RKJPEGDecoder() override;

  bool decode(const std::shared_ptr<ob::ColorFrame>& frame, uint8_t* dest) override;

  bool mppFrame2RGB(const MppFrame frame, uint8_t* data);

 private:
  MppCtx mpp_ctx_ = nullptr;
  MppApi* mpp_api_ = nullptr;
  MppPacket mpp_packet_ = nullptr;
  MppFrame mpp_frame_ = nullptr;
  MppDecCfg mpp_dec_cfg_ = nullptr;
  MppBuffer mpp_frame_buffer_ = nullptr;
  MppBuffer mpp_packet_buffer_ = nullptr;
  uint8_t* data_buffer_ = nullptr;
  MppBufferGroup mpp_frame_group_ = nullptr;
  MppBufferGroup mpp_packet_group_ = nullptr;
  MppTask mpp_task_ = nullptr;
  uint32_t need_split_ = 0;
  uint8_t* rgb_buffer_ = nullptr;
};

}  // namespace orbbec_camera