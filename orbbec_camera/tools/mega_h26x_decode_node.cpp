extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/time.h>
#include <libavutil/log.h>
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
    av_log_set_level(AV_LOG_QUIET);
    avformat_network_init();
    params_init();
    rclcpp::QoS qos_settings(30);
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_settings.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    // compressed_image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    //     "/camera/color/h26x_encoded_data", qos_settings,
    //     std::bind(&H264DecoderNode::compressedImageCallback, this, std::placeholders::_1));

    // rgb_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    //     "/camera/color/h26x_decoder/image_raw", qos_settings);
    for (size_t i = 0; i < encode_topics_.size(); ++i) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("mega_h26x_decode"),
                         "encode_topics_: " << encode_topics_[i]);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("mega_h26x_decode"),
                         "decode_topics_: " << decode_topics_[i]);
      auto encode_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
          encode_topics_[i], qos_settings,
          [this, i](sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            this->compressedImageCallback(msg, i);
          });

      auto decode_pub =
          this->create_publisher<sensor_msgs::msg::Image>(decode_topics_[i], qos_settings);
      encode_subscribers_.push_back(encode_sub);
      decode_publishers_.push_back(decode_pub);
    }
    codec_.resize(encode_topics_.size());
    codec_context_.resize(encode_topics_.size());
    frame_.resize(encode_topics_.size());
    packet_.resize(encode_topics_.size());
    codec_init_.resize(encode_topics_.size());
    send_ret_.resize(encode_topics_.size());
    receive_ret_.resize(encode_topics_.size());
  }

 private:
  std::mutex buffer_mutex_;
  void params_init() {
    std::ifstream file(
        "install/orbbec_camera/share/orbbec_camera/config/tools/megah26xdecode/"
        "mega_h26x_decode_params.json");
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file.");
      return;
    }
    nlohmann::json json_data;
    file >> json_data;
    encode_topics_ =
        json_data["h26x_decode_params"]["encode_topics"].get<std::vector<std::string>>();
    decode_topics_ =
        json_data["h26x_decode_params"]["decode_topics"].get<std::vector<std::string>>();
  }
  void decode_init(const sensor_msgs::msg::CompressedImage::SharedPtr msg, size_t index) {
    if (msg->format == "h264") {
      codec_[index] = std::shared_ptr<const AVCodec>(avcodec_find_decoder(AV_CODEC_ID_H264),
                                                     [](const AVCodec*) {});
      codec_context_[index] = std::shared_ptr<AVCodecContext>(avcodec_alloc_context3(codec_[index].get()),
                                                              [](AVCodecContext* ctx) {
                                                                if (ctx) {
                                                                  avcodec_free_context(&ctx);
                                                                }
                                                              });

      if (avcodec_open2(codec_context_[index].get(), codec_[index].get(), nullptr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open codec");
        return;
      }

      frame_[index] = std::shared_ptr<AVFrame>(av_frame_alloc(), [](AVFrame* f) {
        if (f) av_frame_free(&f);
      });

      packet_[index] = std::shared_ptr<AVPacket>(av_packet_alloc(), [](AVPacket* p) {
        if (p) av_packet_free(&p);
      });
    } else if (msg->format == "h265") {
      codec_[index] = std::shared_ptr<const AVCodec>(avcodec_find_decoder(AV_CODEC_ID_HEVC),
                                              [](const AVCodec*) {});
      codec_context_[index] = std::shared_ptr<AVCodecContext>(avcodec_alloc_context3(codec_[index].get()),
                                                       [](AVCodecContext* ctx) {
                                                         if (ctx) {
                                                           avcodec_free_context(&ctx);
                                                         }
                                                       });

      if (avcodec_open2(codec_context_[index].get(), codec_[index].get(), nullptr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open codec");
        return;
      }

      frame_[index] = std::shared_ptr<AVFrame>(av_frame_alloc(), [](AVFrame* f) {
        if (f) av_frame_free(&f);
      });

      packet_[index] = std::shared_ptr<AVPacket>(av_packet_alloc(), [](AVPacket* p) {
        if (p) av_packet_free(&p);
      });
    }
  }
  void decode_frame(const sensor_msgs::msg::CompressedImage::SharedPtr msg, size_t index) {
    av_packet_unref(packet_[index].get());
    packet_[index]->data = const_cast<uint8_t*>(msg->data.data());
    packet_[index]->size = msg->data.size();
    std::stringstream ss;
    const size_t bytes_to_print = std::min<size_t>(msg->data.size(), 32);
    for (size_t i = 0; i < bytes_to_print; ++i) {
      ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(msg->data[i]) << " ";
    }
    // RCLCPP_INFO(this->get_logger(), "Data (hex): %s", ss.str().c_str());

    send_ret_[index]= avcodec_send_packet(codec_context_[index].get(), packet_[index].get());
    if (send_ret_[index] >= 0) {
      receive_ret_[index] = avcodec_receive_frame(codec_context_[index].get(), frame_[index].get());
      if (receive_ret_[index] >= 0) {
        cv::Mat rgb_image(frame_[index]->height, frame_[index]->width, CV_8UC3);
        SwsContext* sws_context = sws_getContext(frame_[index]->width, frame_[index]->height,
                                                 static_cast<AVPixelFormat>(frame_[index]->format),
                                                 frame_[index]->width, frame_[index]->height, AV_PIX_FMT_BGR24,
                                                 SWS_BILINEAR, nullptr, nullptr, nullptr);

        uint8_t* dest[1] = {rgb_image.data};
        int linesize[1] = {static_cast<int>(rgb_image.step1())};
        sws_scale(sws_context, frame_[index]->data, frame_[index]->linesize, 0, frame_[index]->height, dest, linesize);
        sws_freeContext(sws_context);
        auto rgb_image_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_image).toImageMsg();
        rgb_image_msg->header.stamp = this->now();

        decode_publishers_[index]->publish(*rgb_image_msg);
      }
    }
  }
  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg,
                               size_t index) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    // RCLCPP_INFO(this->get_logger(), "Format: %s", msg->format.c_str());
    if (codec_init_[index] == 0) {
      decode_init(msg, index);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("mega_h26x_decode"), encode_topics_[index] << ":is decoding" );
      codec_init_[index] = 1;
    }
    decode_frame(msg, index);
  }

  std::vector<std::string> encode_topics_;
  std::vector<std::string> decode_topics_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr>
      encode_subscribers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> decode_publishers_;

  //   rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
  //   compressed_image_subscriber_; rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
  //   rgb_image_publisher_;
  std::vector<std::shared_ptr<const AVCodec>> codec_;
  std::vector<std::shared_ptr<AVCodecContext>> codec_context_;
  std::vector<std::shared_ptr<AVFrame>> frame_;
  std::vector<std::shared_ptr<AVPacket>> packet_;
  std::vector<int> codec_init_;
  std::vector<int> send_ret_;
  std::vector<int> receive_ret_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<H264DecoderNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 1);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
