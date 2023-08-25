
#include <orbbec_camera/mjpeg_decoder.h>

namespace orbbec_camera {
MjpegDecoder::MjpegDecoder(int width, int height) : width_(width), height_(height) {
  rgb_buffer_ = new uint8_t[width_ * height_ * 3];
}
MjpegDecoder::~MjpegDecoder() {
  if (rgb_buffer_) {
    delete[] rgb_buffer_;
    rgb_buffer_ = nullptr;
  }
}

}  // namespace orbbec_camera