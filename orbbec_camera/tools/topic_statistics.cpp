#include "topic_statistics.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orbbec_camera::tools::TopicStatistics>());
  rclcpp::shutdown();
  return 0;
}
