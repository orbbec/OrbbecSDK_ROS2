
#include <orbbec_camera/mjpeg_decoder.h>

namespace orbbec_camera {
MjpegDecoder::MjpegDecoder(int width, int height) : width_(width), height_(height) {}
MjpegDecoder::~MjpegDecoder() {}

}  // namespace orbbec_camera