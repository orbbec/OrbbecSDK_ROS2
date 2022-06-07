#include "orbbec_camera/ob_camera_node.h"
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "orbbec_camera/utils.h"
namespace orbbec_camera {
void OBCameraNode::setupCameraCtrlServices() {
  using std_srvs::srv::SetBool;
  for (auto stream_index : IMAGE_STREAMS) {
    auto stream_name = stream_name_[stream_index.first];
    if (enable_[stream_index]) {
      std::string service_name = "get_" + stream_name + "_exposure";
      get_exposure_srv_[stream_index] = node_->create_service<GetInt32>(
          service_name, [this, stream_index = stream_index](
                            const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<GetInt32::Request> request,
                            std::shared_ptr<GetInt32::Response> response) {
            getExposureCallback(request, response, stream_index);
          });

      service_name = "set_" + stream_name + "_exposure";
      set_exposure_srv_[stream_index] = node_->create_service<SetInt32>(
          service_name, [this, stream_index = stream_index](
                            const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<SetInt32::Request> request,
                            std::shared_ptr<SetInt32::Response> response) {
            setExposureCallback(request, response, stream_index);
          });
      service_name = "get_" + stream_name + "_gain";
      get_gain_srv_[stream_index] = node_->create_service<GetInt32>(
          service_name, [this, stream_index = stream_index](
                            const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<GetInt32::Request> request,
                            std::shared_ptr<GetInt32::Response> response) {
            getGainCallback(request, response, stream_index);
          });

      service_name = "set_" + stream_name + "_gain";
      set_gain_srv_[stream_index] = node_->create_service<SetInt32>(
          service_name, [this, stream_index = stream_index](
                            const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<SetInt32::Request> request,
                            std::shared_ptr<SetInt32::Response> response) {
            setGainCallback(request, response, stream_index);
          });
      if (stream_index.first == OB_STREAM_COLOR || stream_index.first == OB_STREAM_DEPTH) {
        service_name = "set_" + stream_name + "_auto_exposure";
        set_auto_exposure_srv_[stream_index] = node_->create_service<SetBool>(
            service_name, [this, stream_index = stream_index](
                              const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<SetBool::Request> request,
                              std::shared_ptr<SetBool::Response> response) {
              setAutoExposureCallback(request, response, stream_index);
            });
      }
    }
  }
  set_fan_mode_srv_ = node_->create_service<SetInt32>(
      "set_fan_mode", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                             const std::shared_ptr<SetInt32::Request> request,
                             std::shared_ptr<SetInt32::Response> response) {
        setFanModeCallback(request_header, request, response);
      });
  set_floor_enable_srv_ = node_->create_service<SetBool>(
      "set_floor_enable", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<SetBool::Request> request,
                                 std::shared_ptr<SetBool::Response> response) {
        setFloorEnableCallback(request_header, request, response);
      });
  set_laser_enable_srv_ = node_->create_service<SetBool>(
      "set_laser_enable", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<SetBool::Request> request,
                                 std::shared_ptr<SetBool::Response> response) {
        setLaserEnableCallback(request_header, request, response);
      });
  set_ldp_enable_srv_ = node_->create_service<SetBool>(
      "set_ldp_enable", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<SetBool::Request> request,
                               std::shared_ptr<SetBool::Response> response) {
        setLdpEnableCallback(request_header, request, response);
      });

  get_white_balance_srv_ = node_->create_service<GetInt32>(
      "get_white_balance", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                  const std::shared_ptr<GetInt32::Request> request,
                                  std::shared_ptr<GetInt32::Response> response) {
        getWhiteBalanceCallback(request_header, request, response);
      });

  set_white_balance_srv_ = node_->create_service<SetInt32>(
      "set_white_balance", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                  const std::shared_ptr<SetInt32::Request> request,
                                  std::shared_ptr<SetInt32::Response> response) {
        setWhiteBalanceCallback(request_header, request, response);
      });
}

void OBCameraNode::setExposureCallback(const std::shared_ptr<SetInt32::Request>& request,
                                       std::shared_ptr<SetInt32::Response>& response,
                                       const stream_index_pair& stream_index) {
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR:
        device_->setIntProperty(OB_PROP_IR_EXPOSURE_INT, request->data);
        break;
      case OB_STREAM_DEPTH:
        device_->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, request->data);
        break;
      case OB_STREAM_COLOR:
        device_->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, request->data);
        break;
      default:
        RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    RCLCPP_ERROR(logger_, "%s unknown error %d", __FUNCTION__, __LINE__);
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::getGainCallback(const std::shared_ptr<GetInt32::Request>& request,
                                   std::shared_ptr<GetInt32::Response>& response,
                                   const stream_index_pair& stream_index) {
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR:
        response->data = device_->getIntProperty(OB_PROP_IR_GAIN_INT);
        break;
      case OB_STREAM_DEPTH:
        response->data = device_->getIntProperty(OB_PROP_DEPTH_GAIN_INT);
        break;
      case OB_STREAM_COLOR:
        response->data = device_->getIntProperty(OB_PROP_COLOR_GAIN_INT);
        break;
      default:
        RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
        break;
    }
  } catch (ob::Error& e) {
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->message = e.what();
  } catch (...) {
    response->message = "unknown error";
  }
}

void OBCameraNode::setGainCallback(const std::shared_ptr<SetInt32 ::Request>& request,
                                   std::shared_ptr<SetInt32::Response>& response,
                                   const stream_index_pair& stream_index) {
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR:
        device_->setIntProperty(OB_PROP_IR_GAIN_INT, request->data);
        break;
      case OB_STREAM_DEPTH:
        device_->setIntProperty(OB_PROP_DEPTH_GAIN_INT, request->data);
        break;
      case OB_STREAM_COLOR:
        device_->setIntProperty(OB_PROP_COLOR_GAIN_INT, request->data);
        break;
      default:
        RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::getWhiteBalanceCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                                           const std::shared_ptr<GetInt32::Request>& request,
                                           std::shared_ptr<GetInt32::Response>& response) {
  try {
    response->data = device_->getIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT);
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->message = e.what();
  } catch (...) {
    response->message = "unknown error";
  }
}

void OBCameraNode::setWhiteBalanceCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                                           const std::shared_ptr<SetInt32 ::Request>& request,
                                           std::shared_ptr<SetInt32 ::Response>& response) {
  try {
    device_->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, request->data);
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::setAutoExposureCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response,
    const stream_index_pair& stream_index) {
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR:
        response->success = false;
        response->message = "IR not support set auto exposure";
        break;
      case OB_STREAM_DEPTH:
        device_->setIntProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, request->data);
        response->success = true;
        break;
      case OB_STREAM_COLOR:
        device_->setIntProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, request->data);
        response->success = true;
        break;
      default:
        RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
        response->success = false;
        response->message = "NOT a video stream";
        break;
    }
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::setFanModeCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                                      const std::shared_ptr<SetInt32::Request>& request,
                                      std::shared_ptr<SetInt32::Response>& response) {
  (void)request_header;
  (void)response;
  bool fan_mode = request->data;
  try {
    device_->setBoolProperty(OB_PROP_FAN_WORK_MODE_INT, fan_mode);
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::setFloorEnableCallback(
    const std::shared_ptr<rmw_request_id_t>& request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  (void)request_header;
  (void)response;
  bool floor_enable = request->data;
  try {
    device_->setBoolProperty(OB_PROP_FLOOD_BOOL, floor_enable);
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::setLaserEnableCallback(
    const std::shared_ptr<rmw_request_id_t>& request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  (void)request_header;
  (void)response;
  bool laser_enable = request->data;
  try {
    device_->setBoolProperty(OB_PROP_LASER_BOOL, laser_enable);
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->message = e.what();
  } catch (...) {
    response->message = "unknown error";
  }
}

void OBCameraNode::setLdpEnableCallback(
    const std::shared_ptr<rmw_request_id_t>& request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  (void)request_header;
  (void)response;
  bool ldp_enable = request->data;
  try {
    device_->setBoolProperty(OB_PROP_LDP_BOOL, ldp_enable);
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}
void OBCameraNode::getExposureCallback(const std::shared_ptr<GetInt32::Request>& request,
                                       std::shared_ptr<GetInt32 ::Response>& response,
                                       const stream_index_pair& stream_index) {
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR:
        response->data = device_->getIntProperty(OB_PROP_IR_EXPOSURE_INT);
        break;
      case OB_STREAM_DEPTH:
        response->data = device_->getIntProperty(OB_PROP_DEPTH_EXPOSURE_INT);
        break;
      case OB_STREAM_COLOR:
        response->data = device_->getIntProperty(OB_PROP_COLOR_EXPOSURE_INT);
        break;
      default:
        RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
        break;
    }
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
  } catch (const std::exception& e) {
    response->message = e.what();
  } catch (...) {
    response->message = "unknown error";
  }
}
}  // namespace orbbec_camera
