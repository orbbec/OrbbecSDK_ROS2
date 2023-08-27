#pragma once
#include "mjpeg_decoder.h"
#include <gst/gst.h>

namespace orbbec_camera {
enum class HWDecoder : int {
  ROCKCHIP_MPP = 0,
  NV_JPEG_DEC = 1,
  AMLOGIC_CODEC = 2,
};

std::string hwDecoderToString(HWDecoder hw_decoder);

class GstreamerMjpegDecoder : public MjpegDecoder {
 public:
  GstreamerMjpegDecoder(int width, int height, HWDecoder hw_decoder);
  ~GstreamerMjpegDecoder() override;

  bool decode(const std::shared_ptr<ob::ColorFrame>& frame, uint8_t* dest) override;

 private:
  std::string hw_decoder_;
  guint buffer_size_ = 0;
  GstBufferPool* buffer_pool_ = nullptr;
  GstElement* pipeline_ = nullptr;
  GstElement* appsrc_ = nullptr;
  GstElement* appsink_ = nullptr;
};

}  // namespace orbbec_camera