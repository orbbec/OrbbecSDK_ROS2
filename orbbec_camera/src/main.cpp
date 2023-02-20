#include <rclcpp/rclcpp.hpp>

#include <orbbec_camera/ob_camera_node_driver.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  using namespace orbbec_camera;
  auto node = std::make_shared<OBCameraNodeDriver>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
