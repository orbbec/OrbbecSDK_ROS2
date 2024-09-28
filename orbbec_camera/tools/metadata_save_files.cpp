#include "metadata_save_files.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orbbec_camera::tools::MetadataSaveFiles>());
  rclcpp::shutdown();
  return 0;
}
