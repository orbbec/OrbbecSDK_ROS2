#include "orbbec_camera/camera_state_publisher.h"

CameraStatusPublisher::CameraStatusPublisher(rclcpp::Node* node):
    node_(node)
{
    publisher_ = node->create_publisher<orbbec_camera_msgs::msg::CameraStatus>("camera_status", 10);
    enable_timeouts_ = node->declare_parameter("enable_timeouts_in_camera_status", false);
    // Initialize the default CameraStateMsg
    camera_state_.connected = false;
    camera_state_.initialized = false;
    camera_state_.color_stream_enabled = false;
    camera_state_.depth_stream_enabled = false;
    camera_state_.imu_stream_enabled = false;
    camera_state_.color_frame_timeout = false;
    camera_state_.depth_frame_timeout = false;
    camera_state_.imu_frame_timeout = false;
    camera_state_.stats_valid = false;

    // Publish stats every second
    stats_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CameraStatusPublisher::publishStats, this));
}

void CameraStatusPublisher::setColorExpectedFrameRate(double frame_rate)
{
    max_color_frame_timeout_ = 2*(1000000000 / frame_rate);
    camera_state_.color_expected_frame_rate = frame_rate;
}

void CameraStatusPublisher::setDepthExpectedFrameRate(double frame_rate)
{
    max_depth_frame_timeout_ = 2*(1000000000 / frame_rate);
    camera_state_.depth_expected_frame_rate = frame_rate;
}

void CameraStatusPublisher::setIMUExpectedFrameRate(double frame_rate)
{
    max_imu_frame_timeout_ = 2*(1000000000 / frame_rate);
    camera_state_.imu_expected_frame_rate = frame_rate;
}

void CameraStatusPublisher::colorFrameReceived(rclcpp::Time timestamp){
    std::lock_guard<std::mutex> lock(mutex_);
    auto latency_ms = (node_->now().nanoseconds() - timestamp.nanoseconds()) / 1e6;
    color_frame_stats_.update(timestamp.nanoseconds(), latency_ms);
    last_color_frame_timestamp_ = timestamp.nanoseconds();
    if(enable_timeouts_){
        if(color_frame_timeout_timer_ == nullptr){
            color_frame_timeout_timer_ = node_->create_wall_timer(
                std::chrono::nanoseconds(max_color_frame_timeout_), 
                [this]() {
                    RCLCPP_INFO(node_->get_logger(), "Color frame timed out");
                    std::lock_guard<std::mutex> lock(mutex_);
                    colorFrameTimedOut();
                }
            );
        }else{
            camera_state_.color_frame_timeout = false;
            color_frame_timeout_timer_->reset();
        }
    }
}

void CameraStatusPublisher::depthFrameReceived(rclcpp::Time timestamp){
    std::lock_guard<std::mutex> lock(mutex_);
    auto latency_ms = (node_->now().nanoseconds() - timestamp.nanoseconds()) / 1e6;
    depth_frame_stats_.update(timestamp.nanoseconds(), latency_ms);
    last_depth_frame_timestamp_ = rclcpp::Clock().now().nanoseconds();
    if(enable_timeouts_){
        if(depth_frame_timeout_timer_ == nullptr){
            depth_frame_timeout_timer_ = node_->create_wall_timer(
                std::chrono::nanoseconds(max_depth_frame_timeout_), 
                [this]() {
                    RCLCPP_INFO(node_->get_logger(), "Depth frame timed out");
                    std::lock_guard<std::mutex> lock(mutex_);
                    depthFrameTimedOut();
                }
            );
        }else{
            camera_state_.depth_frame_timeout = false;
            depth_frame_timeout_timer_->reset();
        }
    }
}

void CameraStatusPublisher::imuFrameReceived(rclcpp::Time timestamp){
    std::lock_guard<std::mutex> lock(mutex_);
    auto latency_ms = (node_->now().nanoseconds() - timestamp.nanoseconds()) / 1e6;
    imu_frame_stats_.update(timestamp.nanoseconds(), latency_ms);
    last_imu_frame_timestamp_ = rclcpp::Clock().now().nanoseconds();
    if(enable_timeouts_){
        if(imu_frame_timeout_timer_ == nullptr){
            imu_frame_timeout_timer_ = node_->create_wall_timer(
            std::chrono::nanoseconds(max_imu_frame_timeout_), 
            [this]() {
                RCLCPP_INFO(node_->get_logger(), "IMU frame timed out");
                std::lock_guard<std::mutex> lock(mutex_);
                imuFrameTimedOut();
            }
            );
        }else{
            camera_state_.imu_frame_timeout = false;
            imu_frame_timeout_timer_->reset();
        }
    }
}

void CameraStatusPublisher::updateColorStreamActive(bool active)
{
    std::lock_guard<std::mutex> lock(mutex_);
    camera_state_.color_stream_enabled = active;
    if (!active) {
        camera_state_.color_frame_timeout = false;
        if (color_frame_timeout_timer_) {
            color_frame_timeout_timer_->cancel();
            color_frame_timeout_timer_ = nullptr;
        }
    }
    publishMessage();
}

void CameraStatusPublisher::updateDepthStreamActive(bool active)
{
    std::lock_guard<std::mutex> lock(mutex_);
    camera_state_.depth_stream_enabled = active;
    if (!active) {
        camera_state_.depth_frame_timeout = false;
        if (depth_frame_timeout_timer_) {
            depth_frame_timeout_timer_->cancel();
            depth_frame_timeout_timer_ = nullptr;
        }
    }
    publishMessage();
}

void CameraStatusPublisher::colorFrameTimedOut(){
    if(camera_state_.color_stream_enabled){
        camera_state_.color_frame_timeout = true;
        publishMessage();
    }
}

void CameraStatusPublisher::depthFrameTimedOut(){
    if(camera_state_.depth_stream_enabled){
        camera_state_.depth_frame_timeout = true;
        publishMessage();
    }
}

void CameraStatusPublisher::imuFrameTimedOut(){
    if(camera_state_.imu_stream_enabled){
        camera_state_.imu_frame_timeout = true;
        publishMessage();
    }
}

void CameraStatusPublisher::updateIMUStreamActive(bool active)
{
    std::lock_guard<std::mutex> lock(mutex_);
    camera_state_.imu_stream_enabled = active;
    if (!active) {
        camera_state_.imu_frame_timeout = false;
        if (imu_frame_timeout_timer_) {
            imu_frame_timeout_timer_->cancel();
            imu_frame_timeout_timer_ = nullptr;
        }
    }
    publishMessage();
}

void CameraStatusPublisher::setMessageAndPublish(std::function<void(orbbec_camera_msgs::msg::CameraStatus&)> func)
{
    std::lock_guard<std::mutex> lock(mutex_);
    func(camera_state_);
    publishMessage();
}

void CameraStatusPublisher::publishMessage()
{
    publisher_->publish(camera_state_);
}

void CameraStatusPublisher::connected(){
    std::lock_guard<std::mutex> lock(mutex_);
    camera_state_.connected = true;
    //camera_state_.initialized = false;
    camera_state_.color_stream_enabled = false;
    camera_state_.depth_stream_enabled = false;
    camera_state_.imu_stream_enabled = false;
    resetStats();
    publishMessage();
}

void CameraStatusPublisher::disconnected(){
    std::lock_guard<std::mutex> lock(mutex_);
    camera_state_.connected = false;
    camera_state_.initialized = false;
    camera_state_.color_stream_enabled = false;
    camera_state_.depth_stream_enabled = false;
    camera_state_.imu_stream_enabled = false;
    resetStats();
    publishMessage();
}

void CameraStatusPublisher::resetStats(){
    RCLCPP_INFO(node_->get_logger(), "Resetting stats");
    camera_state_.color_frame_timeout = false;
    camera_state_.depth_frame_timeout = false;
    camera_state_.imu_frame_timeout = false;
    camera_state_.color_frame_period = StatsMsg();
    camera_state_.depth_frame_period = StatsMsg();
    camera_state_.imu_frame_period = StatsMsg();
    camera_state_.color_frame_latency = StatsMsg();
    camera_state_.depth_frame_latency = StatsMsg();
    camera_state_.imu_frame_latency = StatsMsg();
    color_frame_stats_.reset();
    depth_frame_stats_.reset();
    imu_frame_stats_.reset();
}

// This function assumes that all other fields are already set correctly
void CameraStatusPublisher::initialized(){
    std::lock_guard<std::mutex> lock(mutex_);
    camera_state_.initialized = true;
    resetStats();
    publishMessage();
}

void CameraStatusPublisher::publishStats(){
    std::lock_guard<std::mutex> lock(mutex_);

    camera_state_.stats_valid = true;
    auto color_report = color_frame_stats_.get_report();
    camera_state_.color_frame_period = color_report.period;
    camera_state_.color_frame_latency = color_report.latency;

    auto depth_report = depth_frame_stats_.get_report();
    camera_state_.depth_frame_period = depth_report.period;
    camera_state_.depth_frame_latency = depth_report.latency;

    auto imu_report = imu_frame_stats_.get_report();
    camera_state_.imu_frame_period = imu_report.period;
    camera_state_.imu_frame_latency = imu_report.latency;

    camera_state_.color_frame_rate = color_report.count;
    camera_state_.depth_frame_rate = depth_report.count;
    camera_state_.imu_frame_rate = imu_report.count;

    publishMessage();
    camera_state_.stats_valid = false;
}
