#include "orbbec_camera/gst_decoder.h"
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <glog/logging.h>

namespace orbbec_camera {

GstreamerMjpegDecoder::GstreamerMjpegDecoder(int width, int height, std::string jpeg_decoder,
                                             std::string video_convert, std::string jpeg_parse)
    : MjpegDecoder(width, height),
      jpeg_decoder_(std::move(jpeg_decoder)),
      video_convert_(std::move(video_convert)),
      jpeg_parse_(std::move(jpeg_parse)),
      buffer_size_(width * height * 3) {
  gst_init(NULL, NULL);
  buffer_pool_ = gst_buffer_pool_new();
  CHECK_NOTNULL(buffer_pool_);
  GstStructure* config = gst_buffer_pool_get_config(buffer_pool_);
  CHECK_NOTNULL(config);
  gst_buffer_pool_config_set_params(config, NULL, buffer_size_, 0, 0);
  if (jpeg_decoder_ == "unknown") {
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
  // Create GStreamer elements
  RCLCPP_INFO_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"),
                     "hw decoder: " << jpeg_decoder_ << ", video convert: " << video_convert_
                                    << ", jpeg parse: " << jpeg_parse_);
  appsrc_ = gst_element_factory_make("appsrc", "appsrc");
  jpegparse_ = gst_element_factory_make(jpeg_parse_.c_str(), jpeg_parse_.c_str());
  jpegdec_ = gst_element_factory_make(jpeg_decoder_.c_str(), jpeg_decoder_.c_str());
  videoconvert_ = gst_element_factory_make(video_convert_.c_str(), video_convert_.c_str());
  appsink_ = gst_element_factory_make("appsink", "appsink");

  // Check for null pointers
  if (!appsrc_ || !jpegparse_ || !jpegdec_ || !videoconvert_ || !appsink_) {
    throw std::runtime_error("Failed to create GStreamer elements");
  }

  // Set element properties
  g_object_set(G_OBJECT(appsrc_), "caps",
               gst_caps_new_simple("image/jpeg", "width", G_TYPE_INT, width, "height", G_TYPE_INT,
                                   height, "framerate", GST_TYPE_FRACTION, 0, 1, NULL),
               NULL);

  g_object_set(G_OBJECT(appsink_), "blocksize", buffer_size_, "sync", FALSE, "max-buffers", 1,
               "drop", TRUE, NULL);

  g_object_set(G_OBJECT(appsink_), "caps",
               gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "RGB", "width",
                                   G_TYPE_INT, width, "height", G_TYPE_INT, height, NULL),
               NULL);

  // Create pipeline and add elements
  pipeline_ = gst_pipeline_new("pipeline");
  if (!pipeline_) {
    throw std::runtime_error("Failed to create pipeline");
  }

  gst_bin_add_many(GST_BIN(pipeline_), appsrc_, jpegparse_, jpegdec_, videoconvert_, appsink_,
                   NULL);
  if (!gst_element_link_many(appsrc_, jpegparse_, jpegdec_, videoconvert_, appsink_, NULL)) {
    throw std::runtime_error("Failed to link GStreamer elements");
  }
  // Set pipeline to playing state
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    throw std::runtime_error("Failed to set GStreamer pipeline to playing state");
  }
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
  gst_deinit();
}

bool GstreamerMjpegDecoder::decode(const std::shared_ptr<ob::ColorFrame>& frame, uint8_t* dest) {
  GstBuffer* buffer = NULL;
  GstMapInfo map;
  // Acquire buffer from pool
  if (gst_buffer_pool_acquire_buffer(buffer_pool_, &buffer, NULL) != GST_FLOW_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"),
                        "gst buffer pool acquire buffer error");
    return false;
  }
  // Map buffer and copy frame data
  if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    if (frame->dataSize() <= map.size) {
      memcpy(map.data, frame->data(), frame->dataSize());
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"),
                          "Frame data size exceeds buffer size");
      gst_buffer_unmap(buffer, &map);
      return false;
    }
    gst_buffer_unmap(buffer, &map);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"), "Failed to map buffer");
    return false;
  }

  // Push buffer to appsrc
  GstFlowReturn flow_return = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  if (flow_return != GST_FLOW_OK) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"),
                        "Failed to push buffer to appsrc, GstFlowReturn: " << flow_return);
    return false;
  }

  // Pull sample from appsink
  GstClockTime timeout = GST_SECOND;
  GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), timeout);

  if (sample) {
    GstBuffer* outBuffer = gst_sample_get_buffer(sample);
    GstMapInfo mapInfo;
    if (gst_buffer_map(outBuffer, &mapInfo, GST_MAP_READ)) {
      // Copy data to destination buffer
      memcpy(dest, mapInfo.data, mapInfo.size);
      gst_buffer_unmap(outBuffer, &mapInfo);
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"),
                          "Failed to map output buffer");
      gst_sample_unref(sample);
      return false;
    }
    gst_sample_unref(sample);
    return true;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("gstreamer_mjpeg_decoder"),
                        "Failed to decode frame " << strerror(errno) << " " << errno);
    return false;
  }
}

}  // namespace orbbec_camera