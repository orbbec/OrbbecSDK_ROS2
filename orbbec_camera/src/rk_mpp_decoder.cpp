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

#include "orbbec_camera/rk_mpp_decoder.h"
#include <rclcpp/rclcpp.hpp>
#include <magic_enum/magic_enum.hpp>

namespace orbbec_camera {

RKJPEGDecoder::RKJPEGDecoder(int width, int height) : JPEGDecoder(width, height) {
  rgb_buffer_ = new uint8_t[width_ * height_ * 3];
  MPP_RET ret = mpp_create(&mpp_ctx_, &mpp_api_);
  if (ret != MPP_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"), "mpp_create failed, ret = " << ret);
    throw std::runtime_error("mpp_create failed");
  }
  MpiCmd mpi_cmd = MPP_CMD_BASE;
  MppParam mpp_param = nullptr;

  mpi_cmd = MPP_DEC_SET_PARSER_SPLIT_MODE;
  mpp_param = &need_split_;
  ret = mpp_api_->control(mpp_ctx_, mpi_cmd, mpp_param);
  if (ret != MPP_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"),
                        "mpp_api_->control failed, ret = " << ret);
    throw std::runtime_error("mpp_api_->control failed");
  }
  ret = mpp_init(mpp_ctx_, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);
  if (ret != MPP_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"), "mpp_init failed, ret = " << ret);
    throw std::runtime_error("mpp_init failed");
  }
  MppFrameFormat fmt = MPP_FMT_YUV420SP_VU;
  mpp_param = &fmt;
  ret = mpp_api_->control(mpp_ctx_, MPP_DEC_SET_OUTPUT_FORMAT, mpp_param);
  if (ret != MPP_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"),
                        "mpp_api_->control failed, ret = " << ret);
    throw std::runtime_error("mpp_api_->control failed");
  }
  ret = mpp_frame_init(&mpp_frame_);
  if (ret != MPP_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"),
                        "mpp_frame_init failed, ret = " << ret);
    throw std::runtime_error("mpp_frame_init failed");
  }
  ret = mpp_buffer_group_get_internal(&mpp_frame_group_, MPP_BUFFER_TYPE_ION);
  if (ret != MPP_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"),
                        "mpp_buffer_group_get_internal failed, ret = " << ret);
    throw std::runtime_error("mpp_buffer_group_get_internal failed");
  }
  ret = mpp_buffer_group_get_internal(&mpp_packet_group_, MPP_BUFFER_TYPE_ION);
  if (ret != MPP_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"),
                        "mpp_buffer_group_get_internal failed, ret = " << ret);
    throw std::runtime_error("mpp_buffer_group_get_internal failed");
  }
  RK_U32 hor_stride = MPP_ALIGN(width_, 16);
  RK_U32 ver_stride = MPP_ALIGN(height_, 16);
  ret = mpp_buffer_get(mpp_frame_group_, &mpp_frame_buffer_, hor_stride * ver_stride * 4);
  if (ret != MPP_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"),
                        "mpp_buffer_get failed, ret = " << ret);
    throw std::runtime_error("mpp_buffer_get failed");
  }
  mpp_frame_set_buffer(mpp_frame_, mpp_frame_buffer_);
  ret = mpp_buffer_get(mpp_packet_group_, &mpp_packet_buffer_, width_ * height_ * 3);
  if (ret != MPP_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"),
                        "mpp_buffer_get failed, ret = " << ret);
    throw std::runtime_error("mpp_buffer_get failed");
  }
  mpp_packet_init_with_buffer(&mpp_packet_, mpp_packet_buffer_);
  data_buffer_ = (uint8_t *)mpp_buffer_get_ptr(mpp_packet_buffer_);
}

RKJPEGDecoder::~RKJPEGDecoder() {
  if (mpp_frame_buffer_) {
    mpp_buffer_put(mpp_frame_buffer_);
    mpp_frame_buffer_ = nullptr;
  }
  if (mpp_packet_buffer_) {
    mpp_buffer_put(mpp_packet_buffer_);
    mpp_packet_buffer_ = nullptr;
  }
  if (mpp_frame_group_) {
    mpp_buffer_group_put(mpp_frame_group_);
    mpp_frame_group_ = nullptr;
  }
  if (mpp_packet_group_) {
    mpp_buffer_group_put(mpp_packet_group_);
    mpp_packet_group_ = nullptr;
  }
  if (mpp_frame_) {
    mpp_frame_deinit(&mpp_frame_);
    mpp_frame_ = nullptr;
  }
  if (mpp_packet_) {
    mpp_packet_deinit(&mpp_packet_);
    mpp_packet_ = nullptr;
  }
  if (mpp_ctx_) {
    mpp_destroy(mpp_ctx_);
    mpp_ctx_ = nullptr;
  }
  if (rgb_buffer_) {
    delete[] rgb_buffer_;
  }
}

bool RKJPEGDecoder::mppFrame2RGB(const MppFrame frame, uint8_t *data) {
  int width = mpp_frame_get_width(frame);
  int height = mpp_frame_get_height(frame);
  MppBuffer buffer = mpp_frame_get_buffer(frame);
  CHECK_EQ(width, width_);
  CHECK_EQ(height, height_);
  CHECK_NOTNULL(data);
  CHECK_EQ(width, width_);
  CHECK_EQ(height, height_);
  memset(data, 0, width * height * 3);
  auto buffer_ptr = mpp_buffer_get_ptr(buffer);
#if defined(USE_LIBYUV)
  auto *y = (const uint8_t *)buffer_ptr;
  auto *uv = y + width * height;
  int ret = libyuv::NV12ToRGB24(y, width, uv, width, data, width * 3, width, height);
  if (ret) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"), "libyuv error " << ret);
    return false;
  }
  return true;
#else
  rga_info_t src_info;
  rga_info_t dst_info;
  // NOTE: memset to zero is MUST
  memset(&src_info, 0, sizeof(rga_info_t));
  memset(&dst_info, 0, sizeof(rga_info_t));
  src_info.fd = -1;
  src_info.mmuFlag = 1;
  src_info.virAddr = buffer_ptr;
  src_info.format = RK_FORMAT_YCbCr_420_SP;
  dst_info.fd = -1;
  dst_info.mmuFlag = 1;
  dst_info.virAddr = data;
  dst_info.format = RK_FORMAT_BGR_888;
  rga_set_rect(&src_info.rect, 0, 0, width, height, width, height, RK_FORMAT_YCbCr_420_SP);
  rga_set_rect(&dst_info.rect, 0, 0, width, height, width, height, RK_FORMAT_BGR_888);
  int ret = c_RkRgaBlit(&src_info, &dst_info, nullptr);
  if (ret) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"),
                        "c_RkRgaBlit error " << ret << " errno " << strerror(errno));
    return false;
  }
  return true;
#endif
}

bool RKJPEGDecoder::decode(const std::shared_ptr<ob::ColorFrame> &frame, uint8_t *dest) {
  MPP_RET ret = MPP_OK;
  memset(data_buffer_, 0, width_ * height_ * 3);
  memcpy(data_buffer_, frame->data(), frame->dataSize());
  mpp_packet_set_pos(mpp_packet_, data_buffer_);
  mpp_packet_set_length(mpp_packet_, frame->dataSize());
  mpp_packet_set_eos(mpp_packet_);
  CHECK_NOTNULL(mpp_ctx_);
  ret = mpp_api_->poll(mpp_ctx_, MPP_PORT_INPUT, MPP_POLL_BLOCK);
  if (ret != MPP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rk_mpp_decoder"), "mpp poll failed %d", ret);
    return false;
  }
  ret = mpp_api_->dequeue(mpp_ctx_, MPP_PORT_INPUT, &mpp_task_);
  if (ret != MPP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rk_mpp_decoder"), "mpp dequeue failed %d", ret);
    return false;
  }
  mpp_task_meta_set_packet(mpp_task_, KEY_INPUT_PACKET, mpp_packet_);
  mpp_task_meta_set_frame(mpp_task_, KEY_OUTPUT_FRAME, mpp_frame_);
  ret = mpp_api_->enqueue(mpp_ctx_, MPP_PORT_INPUT, mpp_task_);
  if (ret != MPP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rk_mpp_decoder"), "mpp enqueue failed %d", ret);
    return false;
  }
  ret = mpp_api_->poll(mpp_ctx_, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);
  if (ret != MPP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rk_mpp_decoder"), "mpp poll failed %d", ret);
    return false;
  }
  ret = mpp_api_->dequeue(mpp_ctx_, MPP_PORT_OUTPUT, &mpp_task_);
  if (ret != MPP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("rk_mpp_decoder"), "mpp dequeue failed %d", ret);
    return false;
  }
  if (mpp_task_) {
    MppFrame output_frame = nullptr;
    mpp_task_meta_get_frame(mpp_task_, KEY_OUTPUT_FRAME, &output_frame);
    if (mpp_frame_) {
      int width = mpp_frame_get_width(mpp_frame_);
      int height = mpp_frame_get_height(mpp_frame_);
      if (width != width_ || height != height_) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"),
                            "mpp frame size error " << width << " " << height);
        return false;
      }
      if (!mppFrame2RGB(mpp_frame_, rgb_buffer_)) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rk_mpp_decoder"), "mpp frame to rgb error");
        return false;
      }
      if (mpp_frame_get_eos(output_frame)) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rk_mpp_decoder"), "mpp frame get eos");
      }
    }
    ret = mpp_api_->enqueue(mpp_ctx_, MPP_PORT_OUTPUT, mpp_task_);
    if (ret != MPP_OK) {
      RCLCPP_ERROR(rclcpp::get_logger("rk_mpp_decoder"), "mpp enqueue failed %d", ret);
      return false;
    }
    CHECK_NOTNULL(dest);
    memcpy(dest, rgb_buffer_, width_ * height_ * 3);
    return true;
  }
  return false;
}

}  // namespace orbbec_camera