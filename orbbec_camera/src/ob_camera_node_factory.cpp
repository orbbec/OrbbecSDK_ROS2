#include "orbbec_camera/ob_camera_node_factory.h"
namespace orbbec_camera {
OBCameraNodeFactory::OBCameraNodeFactory(const rclcpp::NodeOptions &node_options)
    : Node("orbbec_camera_node", "/", node_options),
      ctx_(std::make_unique<ob::Context>()),
      logger_(this->get_logger()) {
  init();
}
OBCameraNodeFactory::OBCameraNodeFactory(const std::string &node_name, const std::string &ns,
                                         const rclcpp::NodeOptions &node_options)
    : Node(node_name, ns, node_options),
      ctx_(std::make_unique<ob::Context>()),
      logger_(this->get_logger()) {
  init();
}

OBCameraNodeFactory::~OBCameraNodeFactory() {
  is_alive_.store(false);
  if (query_thread_.joinable()) {
    query_thread_.join();
  }
}
void OBCameraNodeFactory::init() {
  is_alive_.store(true);
  serial_number_ = declare_parameter<std::string>("serial_number", "BX4NC10000S");
  query_thread_ = std::thread([=]() {
    std::chrono::milliseconds timespan(static_cast<int>(reconnect_timeout_ * 1e3));
    rclcpp::Time first_try_time = this->now();
    while (is_alive_ && !device_) {
      CHECK_NOTNULL(ctx_);
      auto list = ctx_->queryDeviceList();
      getDevice(list);
      if (device_) {
        ctx_->setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removed_list,
                                              std::shared_ptr<ob::DeviceList> added_list) {
          deviceDisconnectCallback(removed_list);
          deviceConnectCallback(added_list);
        });
        startDevice();
      } else {
        std::chrono::milliseconds actual_timespan(timespan);
        if (wait_for_device_timeout_ > 0) {
          auto time_to_timeout(wait_for_device_timeout_ -
                               (this->get_clock()->now() - first_try_time).seconds());
          if (time_to_timeout < 0) {
            RCLCPP_ERROR_STREAM(logger_, "wait for device timeout of " << wait_for_device_timeout_
                                                                       << " secs expired");
            exit(1);
          } else {
            double max_timespan_secs = static_cast<double>(
                std::chrono::duration_cast<std::chrono::seconds>(timespan).count());
            actual_timespan = std::chrono::milliseconds(
                static_cast<int>(std::min(max_timespan_secs, time_to_timeout) * 1e3));
          }
        }
        std::this_thread::sleep_for(actual_timespan);
      }
    }
  });
}

void OBCameraNodeFactory::deviceConnectCallback(
    const std::shared_ptr<ob::DeviceList> &device_list) {
  CHECK_NOTNULL(device_list);
  if (!device_) {
    getDevice(device_list);
    if (device_) {
      startDevice();
    } else {
    }
  }
}

void OBCameraNodeFactory::deviceDisconnectCallback(
    const std::shared_ptr<ob::DeviceList> &device_list) {
  CHECK_NOTNULL(device_list);
  auto dev = device_list->getDeviceBySN(serial_number_.c_str());
  if (dev) {
    RCLCPP_ERROR_STREAM(logger_, "The device has been disconnected!");
    ob_camera_node_.reset(nullptr);
    device_.reset();
  }
}

void OBCameraNodeFactory::getDevice(const std::shared_ptr<ob::DeviceList> &list) {
  if (device_) {
    return;
  }
  if (0 == list->deviceCount()) {
    RCLCPP_WARN_STREAM(logger_, "No orbbec devices were found!");
    return;
  }
  for (size_t i = 0; i < list->deviceCount(); i++) {
    auto dev = list->getDevice(i);
    if (dev != nullptr) {
      device_ = dev;
      RCLCPP_INFO_STREAM(logger_, "device name " << dev->getDeviceInfo()->name());
      RCLCPP_INFO_STREAM(logger_, "device pid " << dev->getDeviceInfo()->pid());
      RCLCPP_INFO_STREAM(logger_, "device vid " << dev->getDeviceInfo()->vid());
      RCLCPP_INFO_STREAM(logger_, "device serial_name " << dev->getDeviceInfo()->serialNumber());
      RCLCPP_INFO_STREAM(logger_,
                         "device firmware version " << dev->getDeviceInfo()->firmwareVersion());
      RCLCPP_INFO_STREAM(logger_, "device supported min sdk version "
                                      << dev->getDeviceInfo()->supportedMinSdkVersion());
      RCLCPP_INFO_STREAM(logger_, "device hardware version " << dev->getDeviceInfo()->hardwareVersion());
    }
  }
}

void OBCameraNodeFactory::startDevice() {
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  CHECK_NOTNULL(device_);
  ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_);
}
}  // namespace orbbec_camera
