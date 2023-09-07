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