#include <rclcpp/rclcpp.hpp>

#include <orbbec_camera/ob_camera_node_driver.h>
#include <memory>
#include <magic_enum/magic_enum.hpp>
#include <orbbec_camera/utils.h>
int main() {
  using namespace orbbec_camera;
  auto context = std::make_unique<ob::Context>();
  context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
  auto device_list = context->queryDeviceList();
  auto device = device_list->getDevice(0);
  auto sensor_list = device->getSensorList();
  for (size_t i = 0; i < sensor_list->count(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profile_list = sensor->getStreamProfileList();
    for (size_t j = 0; j < profile_list->count(); j++) {
      auto origin_profile = profile_list->getProfile(j);
      if (sensor->type() == OB_SENSOR_COLOR) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "color profile: " << profile->width() << "x" << profile->height() << " "
                                             << profile->fps() << "fps "
                                             << magic_enum::enum_name(profile->format()));
      } else if (sensor->type() == OB_SENSOR_DEPTH) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "depth profile: " << profile->width() << "x" << profile->height() << " "
                                             << profile->fps() << "fps "
                                             << magic_enum::enum_name(profile->format()));
      } else if (sensor->type() == OB_SENSOR_IR) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "ir profile: " << profile->width() << "x" << profile->height() << " "
                                          << profile->fps() << "fps "
                                          << magic_enum::enum_name(profile->format()));
      } else if (sensor->type() == OB_SENSOR_ACCEL) {
        auto profile = origin_profile->as<ob::AccelStreamProfile>();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "accel profile: sampleRate "
                               << sampleRateToString(profile->sampleRate()) << "  full scale_range "
                               << fullAccelScaleRangeToString(profile->fullScaleRange()));
      } else if (sensor->type() == OB_SENSOR_GYRO) {
        auto profile = origin_profile->as<ob::GyroStreamProfile>();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "gyro profile: sampleRate "
                               << sampleRateToString(profile->sampleRate()) << "  full scale_range "
                               << fullGyroScaleRangeToString(profile->fullScaleRange()));
      } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device_node"),
                           "unknown profile: " << magic_enum::enum_name(sensor->type()));
      }
    }
  }
  return 0;
}
