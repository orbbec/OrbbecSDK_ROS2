extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/time.h>
}
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "orbbec_camera/ob_camera_node.h"
#include <thread>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "orbbec_camera/utils.h"
#include <filesystem>
#include <fstream>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "libobsensor/hpp/Utils.hpp"

class H264DecoderNode : public rclcpp::Node {
 public:
  H264DecoderNode() : Node("h264_decoder_node") {
    avformat_network_init();
    rclcpp::QoS qos_settings(30);
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_settings.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    compressed_image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/camera/color/h26x_encoded_data", qos_settings,
        std::bind(&H264DecoderNode::compressedImageCallback, this, std::placeholders::_1));

    rgb_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/color/h26x_decoder/image_raw", qos_settings);
  }

 private:
  void decode_init(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    if (msg->format == "h264") {
      codec_ = std::shared_ptr<const AVCodec>(avcodec_find_decoder(AV_CODEC_ID_H264),
                                              [](const AVCodec*) {});
      codec_context_ = std::shared_ptr<AVCodecContext>(avcodec_alloc_context3(codec_.get()),
                                                       [](AVCodecContext* ctx) {
                                                         if (ctx) {
                                                           avcodec_free_context(&ctx);
                                                         }
                                                       });

      if (avcodec_open2(codec_context_.get(), codec_.get(), nullptr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open codec");
        return;
      }

      frame_ = std::shared_ptr<AVFrame>(av_frame_alloc(), [](AVFrame* f) {
        if (f) av_frame_free(&f);
      });

      packet_ = std::shared_ptr<AVPacket>(av_packet_alloc(), [](AVPacket* p) {
        if (p) av_packet_free(&p);
      });
      codec_init_ = 0;
    } else if (msg->format == "h265") {
      codec_ = std::shared_ptr<const AVCodec>(avcodec_find_decoder(AV_CODEC_ID_HEVC),
                                              [](const AVCodec*) {});
      codec_context_ = std::shared_ptr<AVCodecContext>(avcodec_alloc_context3(codec_.get()),
                                                       [](AVCodecContext* ctx) {
                                                         if (ctx) {
                                                           avcodec_free_context(&ctx);
                                                         }
                                                       });

      if (avcodec_open2(codec_context_.get(), codec_.get(), nullptr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open codec");
        return;
      }

      frame_ = std::shared_ptr<AVFrame>(av_frame_alloc(), [](AVFrame* f) {
        if (f) av_frame_free(&f);
      });

      packet_ = std::shared_ptr<AVPacket>(av_packet_alloc(), [](AVPacket* p) {
        if (p) av_packet_free(&p);
      });
      codec_init_ = 0;
    }
  }
  void decode_frame(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    av_packet_unref(packet_.get());
    packet_->data = const_cast<uint8_t*>(msg->data.data());
    packet_->size = msg->data.size();
    std::stringstream ss;
    const size_t bytes_to_print = std::min<size_t>(msg->data.size(), 32);
    for (size_t i = 0; i < bytes_to_print; ++i) {
      ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(msg->data[i]) << " ";
    }
    RCLCPP_INFO(this->get_logger(), "Data (hex): %s", ss.str().c_str());

    send_ret_ = avcodec_send_packet(codec_context_.get(), packet_.get());
    if (send_ret_ >= 0) {
      receive_ret_ = avcodec_receive_frame(codec_context_.get(), frame_.get());
      if (receive_ret_ >= 0) {
        cv::Mat rgb_image(frame_->height, frame_->width, CV_8UC3);
        SwsContext* sws_context = sws_getContext(frame_->width, frame_->height,
                                                 static_cast<AVPixelFormat>(frame_->format),
                                                 frame_->width, frame_->height, AV_PIX_FMT_BGR24,
                                                 SWS_BILINEAR, nullptr, nullptr, nullptr);

        uint8_t* dest[1] = {rgb_image.data};
        int linesize[1] = {static_cast<int>(rgb_image.step1())};
        sws_scale(sws_context, frame_->data, frame_->linesize, 0, frame_->height, dest, linesize);
        sws_freeContext(sws_context);
        auto rgb_image_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_image).toImageMsg();
        rgb_image_msg->header.stamp = this->now();

        rgb_image_publisher_->publish(*rgb_image_msg);
      }
    }
  }
  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Format: %s", msg->format.c_str());
    if (codec_init_) {
      decode_init(msg);
    }
    decode_frame(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_image_publisher_;
  std::shared_ptr<const AVCodec> codec_;
  std::shared_ptr<AVCodecContext> codec_context_;
  std::shared_ptr<AVFrame> frame_;
  std::shared_ptr<AVPacket> packet_;
  int codec_init_ = 1;
  int send_ret_;
  int receive_ret_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<H264DecoderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
