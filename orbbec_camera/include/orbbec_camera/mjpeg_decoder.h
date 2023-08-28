#pragma once

#include <string>
#include <vector>
#include "libobsensor/ObSensor.hpp"

namespace orbbec_camera {
enum HWDecoder {
  ROCKCHIP_MPP = 0,
  NV_JPEG_DEC = 1,
  AMLOGIC_CODEC = 2,
  AV_CODEC = 3,
};

class MjpegDecoder {
 public:
  MjpegDecoder(int width, int height);

  virtual ~MjpegDecoder();

  virtual bool decode(const std::shared_ptr<ob::ColorFrame> &frame, uint8_t *dest) = 0;

  std::string getErrorMsg() const { return error_msg_; }

 protected:
  int width_ = 0;
  int height_ = 0;
  std::string error_msg_;
};
}  // namespace orbbec_camera
