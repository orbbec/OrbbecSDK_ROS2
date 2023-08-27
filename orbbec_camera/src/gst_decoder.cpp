#include "orbbec_camera/gst_decoder.h"
#include <rclcpp/rclcpp.hpp>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

namespace orbbec_camera {
std::string hwDecoderToString(HWDecoder hw_decoder) {
  switch (hw_decoder) {
    case HWDecoder::ROCKCHIP_MPP:
      return "mppjpegdec";
    case HWDecoder::NV_JPEG_DEC:
      return "nvjpegdec";
    case HWDecoder::AMLOGIC_CODEC:
      return "amlvenc";
    default:
      return "unknown";
  }
}

GstreamerMjpegDecoder::GstreamerMjpegDecoder(int width, int height, HWDecoder hw_decoder)
    : MjpegDecoder(width, height),
      hw_decoder_(hwDecoderToString(hw_decoder)),
      buffer_size_(width * height * 3) {
  buffer_pool_ = gst_buffer_pool_new();
  GstStructure* config = gst_buffer_pool_get_config(buffer_pool_);
  gst_buffer_pool_config_set_params(config, NULL, buffer_size_, 0, 0);
  if (hw_decoder_ == "unknown") {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"), "hw decoder is unknown");
    throw std::runtime_error("hw decoder is unknown");
  }
  if (!gst_buffer_pool_set_config(buffer_pool_, config)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"),
                        "gst buffer pool set config error");
    throw std::runtime_error("gst buffer pool set config error");
  }
  if (gst_buffer_pool_set_active(buffer_pool_, TRUE) != TRUE) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"),
                        "gst buffer pool set active error");
    throw std::runtime_error("gst buffer pool set active error");
  }
  std::string pipeline_str = "appsrc name=appsrc0 is-live=true do-timestamp=true ! " + hw_decoder_ +
                             " ! videoconvert ! video/x-raw,format=RGB ! appsink name=appsink0";
  pipeline_ = gst_parse_launch(pipeline_str.c_str(), NULL);
  if (!pipeline_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"), "gst parse launch error");
    throw std::runtime_error("gst parse launch error");
  }
  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc0");
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink0");
}

GstreamerMjpegDecoder::~GstreamerMjpegDecoder() {
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  if (buffer_pool_) {
    gst_buffer_pool_set_active(buffer_pool_, FALSE);
    g_object_unref(buffer_pool_);
  }
}

bool GstreamerMjpegDecoder::decode(const std::shared_ptr<ob::ColorFrame>& frame, uint8_t* dest) {
  GstBuffer* buffer = NULL;
  GstMapInfo map;

  if (gst_buffer_pool_acquire_buffer(buffer_pool_, &buffer, NULL) != GST_FLOW_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"),
                        "gst buffer pool acquire buffer error");
    return false;
  }

  gst_buffer_map(buffer, &map, GST_MAP_WRITE);
  memcpy(map.data, frame->data(), frame->dataSize());
  gst_buffer_unmap(buffer, &map);  // Unmap the buffer after copying

  gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink_));
  if (sample) {
    GstBuffer* outBuffer = gst_sample_get_buffer(sample);
    GstMapInfo mapInfo;
    gst_buffer_map(outBuffer, &mapInfo, GST_MAP_READ);

    // Copy data to destination buffer
    memcpy(dest, mapInfo.data, mapInfo.size);

    gst_buffer_unmap(outBuffer, &mapInfo);
    gst_sample_unref(sample);
    return true;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"), "Failed to decode frame");
    return false;
  }
}

}  // namespace orbbec_camera