#include <rclcpp/rclcpp.hpp>
#include "orbbec_camera_msgs/srv/get_user_calib_params.hpp"
#include "orbbec_camera_msgs/srv/set_user_calib_params.hpp"
#include "orbbec_camera_msgs/srv/set_arrays.hpp"
#include "orbbec_camera_msgs/msg/device_status.hpp"
#include <orbbec_camera_msgs/srv/get_device_info.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <iostream>

using namespace std::chrono_literals;

class CameraExampleNode : public rclcpp::Node {
 public:
  CameraExampleNode() : Node("camera_example_node") {
    set_params_client_ = this->create_client<orbbec_camera_msgs::srv::SetUserCalibParams>(
        "/camera/set_user_calib_params");
    get_params_client_ = this->create_client<orbbec_camera_msgs::srv::GetUserCalibParams>(
        "/camera/get_user_calib_params");
    set_streams_client_ = this->create_client<std_srvs::srv::SetBool>("/camera/set_streams_enable");
    set_color_ae_roi_client_ = this->create_client<orbbec_camera_msgs::srv::SetArrays>(
        "/camera/set_color_ae_roi");
    get_device_info_client_ =
        this->create_client<orbbec_camera_msgs::srv::GetDeviceInfo>("/camera/get_device_info");
  }

  // Feature 1: Write camera parameters
  void exampleWriteParams() {
    if (!set_params_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "SetUserCalibParams service not available");
      return;
    }

    auto req = std::make_shared<orbbec_camera_msgs::srv::SetUserCalibParams::Request>();
    req->k = {614.9613647460938,
              0.0,
              634.91552734375,
              0.0,
              614.65771484375,
              391.407470703125,
              0.0,
              0.0,
              1.0};
    req->d = {-0.03131488710641861,
              0.032955970615148544,
              9.096559369936585e-05,
              -0.0003368517500348389,
              -0.01115430984646082,
              0.0,
              0.0,
              0.0};
    req->rotation = {0.9999880790710449,     0.0003024190664291382,  -0.004874417092651129,
                     -0.0002965621242765337, 0.9999992251396179,     0.001202247804030776,
                     0.004874777048826218,   -0.0012007878394797444, 0.9999874234199524};
    req->translation = {-0.023897956848144532, -9.439220279455185e-05, -6.804073229432106e-06};

    auto result = set_params_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call SetUserCalibParams");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Set result: %s", result.get()->message.c_str());
  }

  // Feature 2: Read camera parameters
  void exampleReadParams() {
    if (!get_params_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "GetUserCalibParams service not available");
      return;
    }

    auto req = std::make_shared<orbbec_camera_msgs::srv::GetUserCalibParams::Request>();
    auto result = get_params_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call GetUserCalibParams");
      return;
    }

    auto res = result.get();
    RCLCPP_INFO(this->get_logger(), "Get result: %s", res->message.c_str());

    std::cout << "K: ";
    for (auto v : res->k) std::cout << v << " ";
    std::cout << std::endl;

    std::cout << "D: ";
    for (auto v : res->d) std::cout << v << " ";
    std::cout << std::endl;

    std::cout << "Rotation: ";
    for (auto v : res->rotation) std::cout << v << " ";
    std::cout << std::endl;

    std::cout << "Translation: ";
    for (auto v : res->translation) std::cout << v << " ";
    std::cout << std::endl;
  }

  // Feature 3: Enable camera streams
  void exampleEnableStreams() { exampleSetStreamsEnable(true); }

  // Feature 4: Disable camera streams
  void exampleDisableStreams() { exampleSetStreamsEnable(false); }

  // Feature 5: Get device info
  void exampleGetDeviceInfo() {
    if (!get_device_info_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "GetDeviceInfo service not available");
      return;
    }

    auto req = std::make_shared<orbbec_camera_msgs::srv::GetDeviceInfo::Request>();
    auto result = get_device_info_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call GetDeviceInfo");
      return;
    }

    auto res = result.get();
    if (!res->success) {
      RCLCPP_ERROR(this->get_logger(), "GetDeviceInfo failed: %s", res->message.c_str());
      return;
    }

    std::cout << "Device Info:" << std::endl;
    std::cout << "Name: " << res->info.name << std::endl;
    std::cout << "Serial Number: " << res->info.serial_number << std::endl;
    std::cout << "Firmware Version: " << res->info.firmware_version << std::endl;
    std::cout << "Supported Min SDK Version: " << res->info.supported_min_sdk_version << std::endl;
    std::cout << "Current SDK Version: " << res->info.current_sdk_version << std::endl;
    std::cout << "Hardware Version: " << res->info.hardware_version << std::endl;
  }

  // Feature 6: Show device status
  void exampleShowdeviceStatus() {
    device_status_sub_ = this->create_subscription<orbbec_camera_msgs::msg::DeviceStatus>(
        "/camera/device_status", 10,
        std::bind(&CameraExampleNode::deviceStatusCallback, this, std::placeholders::_1));

    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
    }
  }

  // Feature 7: Set Color AE ROI
  void exampleSetColorAERoi(float left, float right, float top, float bottom) {
    if (!set_color_ae_roi_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "SetColorAERoi service not available");
      return;
    }

    auto req = std::make_shared<orbbec_camera_msgs::srv::SetArrays::Request>();
    req->data_param = {left, right, top, bottom};

    auto result = set_color_ae_roi_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call SetColorAERoi");
      return;
    }

    auto res = result.get();
    if (res->success) {
      RCLCPP_INFO(this->get_logger(), "SetColorAERoi success: %s", res->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "SetColorAERoi failed: %s", res->message.c_str());
    }
  }

 private:
  void exampleSetStreamsEnable(bool enable) {
    if (!set_streams_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "SetStreamsEnable service not available");
      return;
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = enable;

    auto result = set_streams_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call SetStreamsEnable");
      return;
    }

    auto res = result.get();
    RCLCPP_INFO(this->get_logger(), "SetStreamsEnable result: %s", res->message.c_str());
  }

  void deviceStatusCallback(const orbbec_camera_msgs::msg::DeviceStatus::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "-------orbbec camera real-time status-------");
    RCLCPP_INFO(this->get_logger(), "--------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Color Frame Rate: Cur: %.2f, Avg: %.2f",
                msg->color_frame_rate_cur, msg->color_frame_rate_avg);
    RCLCPP_INFO(this->get_logger(), "Depth Frame Rate: Cur: %.2f, Avg: %.2f",
                msg->depth_frame_rate_cur, msg->depth_frame_rate_avg);
    RCLCPP_INFO(this->get_logger(), "Color Delay (ms): Cur: %.2f, Avg: %.2f",
                msg->color_delay_ms_cur, msg->color_delay_ms_avg);
    RCLCPP_INFO(this->get_logger(), "Depth Delay (ms): Cur: %.2f, Avg: %.2f",
                msg->depth_delay_ms_cur, msg->depth_delay_ms_avg);

    RCLCPP_INFO(this->get_logger(), "Device Online: %s", msg->device_online ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "Connection Type: %s", msg->connection_type.c_str());
    RCLCPP_INFO(this->get_logger(), "customer_calibration_ready: %s",
                msg->customer_calibration_ready ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "calibration_from_factory: %s",
                msg->calibration_from_factory ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "Calibration From Launch Param: %s",
                msg->calibration_from_launch_param ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "--------------------------------------------\n");
  }

  rclcpp::Client<orbbec_camera_msgs::srv::SetUserCalibParams>::SharedPtr set_params_client_;
  rclcpp::Client<orbbec_camera_msgs::srv::GetUserCalibParams>::SharedPtr get_params_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_streams_client_;
  rclcpp::Client<orbbec_camera_msgs::srv::GetDeviceInfo>::SharedPtr get_device_info_client_;
  rclcpp::Client<orbbec_camera_msgs::srv::SetArrays>::SharedPtr set_color_ae_roi_client_;
  rclcpp::Subscription<orbbec_camera_msgs::msg::DeviceStatus>::SharedPtr device_status_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraExampleNode>();

  while (rclcpp::ok()) {
    std::cout << "\n========= Example Menu =========\n";
    std::cout << "1. Write camera parameters\n";
    std::cout << "2. Read camera parameters\n";
    std::cout << "3. Enable camera streams\n";
    std::cout << "4. Disable camera streams\n";
    std::cout << "5. Get device info\n";
    std::cout << "6. Show device status\n";
    std::cout << "7. Set Color AE ROI\n";
    std::cout << "0. Exit\n";
    std::cout << "Choose option: ";

    int choice;
    std::cin >> choice;

    switch (choice) {
      case 1:
        node->exampleWriteParams();
        break;
      case 2:
        node->exampleReadParams();
        break;
      case 3:
        node->exampleEnableStreams();
        break;
      case 4:
        node->exampleDisableStreams();
        break;
      case 5:
        node->exampleGetDeviceInfo();
        break;
      case 6:
        node->exampleShowdeviceStatus();
        break;
      case 7:
        float l, r, t, b;
        std::cout << "Enter ROI (left right top bottom): ";
        std::cin >> l >> r >> t >> b;
        node->exampleSetColorAERoi(l, r, t, b);
        break;
      case 0:
        rclcpp::shutdown();
        return 0;
      default:
        std::cout << "Invalid choice, try again.\n";
        break;
    }
  }

  rclcpp::shutdown();
  return 0;
}
