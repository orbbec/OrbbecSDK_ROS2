#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "orbbec_camera_msgs/msg/camera_status.hpp"
#include "orbbec_camera/stats_helper.h"

using CameraStatusMsg = orbbec_camera_msgs::msg::CameraStatus;

class CameraStatusPublisher 
{
public:
    CameraStatusPublisher(rclcpp::Node* node);

    void setColorExpectedFrameRate(double frame_rate);
    void setDepthExpectedFrameRate(double frame_rate);
    void setIMUExpectedFrameRate(double frame_rate);

    void colorFrameReceived(rclcpp::Time timestamp);
    void depthFrameReceived(rclcpp::Time timestamp);
    void imuFrameReceived(rclcpp::Time timestamp);

    void updateColorStreamActive(bool active);
    void updateDepthStreamActive(bool active);
    void updateIMUStreamActive(bool active);

    void setMessageAndPublish(std::function<void(orbbec_camera_msgs::msg::CameraStatus&)> func);

    void connected();
    void disconnected();
    void initialized();

private:
    std::mutex mutex_;

    // All these internal functions are not thread safe
    void resetStats();
    void publishStats();
    void publishMessage();

    rclcpp::Node* node_;

    rclcpp::Publisher<CameraStatusMsg>::SharedPtr publisher_;
    CameraStatusMsg camera_state_;
    int64_t last_color_frame_timestamp_;
    int64_t last_depth_frame_timestamp_;
    int64_t last_imu_frame_timestamp_;
    int64_t max_color_frame_timeout_;
    int64_t max_depth_frame_timeout_;
    int64_t max_imu_frame_timeout_;

    rclcpp::TimerBase::SharedPtr color_frame_timeout_timer_;
    rclcpp::TimerBase::SharedPtr depth_frame_timeout_timer_;
    rclcpp::TimerBase::SharedPtr imu_frame_timeout_timer_;

    FrameStats color_frame_stats_;
    FrameStats depth_frame_stats_;
    FrameStats imu_frame_stats_;

    rclcpp::TimerBase::SharedPtr stats_timer_;
    bool enable_timeouts_ = false;

    void colorFrameTimedOut();
    void depthFrameTimedOut();
    void imuFrameTimedOut();
};
