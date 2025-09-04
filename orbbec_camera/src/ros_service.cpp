/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include "orbbec_camera/ob_camera_node.h"
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <thread>

#include "orbbec_camera/utils.h"
namespace orbbec_camera {

void OBCameraNode::setupCameraCtrlServices() {
  using std_srvs::srv::SetBool;
  for (auto stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    auto stream_name = stream_name_[stream_index];
    std::string service_name = "get_" + stream_name + "_exposure";
    get_exposure_srv_[stream_index] = node_->create_service<GetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetInt32::Request> request,
                                            std::shared_ptr<GetInt32::Response> response) {
          getExposureCallback(request, response, stream_index);
        });

    service_name = "set_" + stream_name + "_exposure";
    set_exposure_srv_[stream_index] = node_->create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          setExposureCallback(request, response, stream_index);
        });
    service_name = "get_" + stream_name + "_gain";
    get_gain_srv_[stream_index] = node_->create_service<GetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetInt32::Request> request,
                                            std::shared_ptr<GetInt32::Response> response) {
          getGainCallback(request, response, stream_index);
        });

    service_name = "set_" + stream_name + "_gain";
    set_gain_srv_[stream_index] = node_->create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          setGainCallback(request, response, stream_index);
        });
    service_name = "set_" + stream_name + "_auto_exposure";
    set_auto_exposure_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          setAutoExposureCallback(request, response, stream_index);
        });

    service_name = "set_" + stream_name + "_ae_roi";
    set_ae_roi_srv_[stream_index] = node_->create_service<SetArrays>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetArrays::Request> request,
                                            std::shared_ptr<SetArrays::Response> response) {
          setAeRoiCallback(request, response, stream_index);
        });

    service_name = "toggle_" + stream_name;

    toggle_sensor_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          toggleSensorCallback(request, response, stream_index);
        });
    service_name = "set_" + stream_name + "_mirror";
    set_mirror_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          setMirrorCallback(request, response, stream_index);
        });
    service_name = "set_" + stream_name + "_flip";
    set_flip_srv_[stream_index] = node_->create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          setFlipCallback(request, response, stream_index);
        });
    service_name = "set_" + stream_name + "_rotation";
    set_rotation_srv_[stream_index] = node_->create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          setRotationCallback(request, response, stream_index);
        });
  }
  set_fan_work_mode_srv_ = node_->create_service<SetInt32>(
      "set_fan_work_mode", [this](const std::shared_ptr<SetInt32::Request> request,
                                  std::shared_ptr<SetInt32::Response> response) {
        setFanWorkModeCallback(request, response);
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
  get_ldp_status_srv_ = node_->create_service<GetBool>(
      "get_ldp_status", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<GetBool::Request> request,
                               std::shared_ptr<GetBool::Response> response) {
        (void)request_header;
        getLdpStatusCallback(request, response);
      });

  get_white_balance_srv_ = node_->create_service<GetInt32>(
      "get_white_balance", [this](const std::shared_ptr<GetInt32::Request> request,
                                  std::shared_ptr<GetInt32::Response> response) {
        getWhiteBalanceCallback(request, response);
      });

  set_white_balance_srv_ = node_->create_service<SetInt32>(
      "set_white_balance", [this](const std::shared_ptr<SetInt32::Request> request,
                                  std::shared_ptr<SetInt32::Response> response) {
        setWhiteBalanceCallback(request, response);
      });
  get_auto_white_balance_srv_ = node_->create_service<GetInt32>(
      "get_auto_white_balance", [this](const std::shared_ptr<GetInt32::Request> request,
                                       std::shared_ptr<GetInt32::Response> response) {
        getAutoWhiteBalanceCallback(request, response);
      });
  set_auto_white_balance_srv_ = node_->create_service<SetBool>(
      "set_auto_white_balance", [this](const std::shared_ptr<SetBool::Request> request,
                                       std::shared_ptr<SetBool::Response> response) {
        setAutoWhiteBalanceCallback(request, response);
      });
  get_device_srv_ = node_->create_service<GetDeviceInfo>(
      "get_device_info", [this](const std::shared_ptr<GetDeviceInfo::Request> request,
                                std::shared_ptr<GetDeviceInfo::Response> response) {
        getDeviceInfoCallback(request, response);
      });
  get_sdk_version_srv_ = node_->create_service<GetString>(
      "get_sdk_version",
      [this](const std::shared_ptr<GetString::Request> request,
             std::shared_ptr<GetString::Response> response) { getSDKVersion(request, response); });
  save_images_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "save_images", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        saveImageCallback(request, response);
      });
  save_point_cloud_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "save_point_cloud", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                 std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        savePointCloudCallback(request, response);
      });
  switch_ir_camera_srv_ = node_->create_service<SetString>(
      "switch_ir", [this](const std::shared_ptr<SetString::Request> request,
                          std::shared_ptr<SetString::Response> response) {
        switchIRCameraCallback(request, response);
      });
  set_ir_long_exposure_srv_ = node_->create_service<SetBool>(
      "set_ir_long_exposure", [this](const std::shared_ptr<SetBool::Request> request,
                                     std::shared_ptr<SetBool::Response> response) {
        setIRLongExposureCallback(request, response);
      });
  get_lrm_measure_distance_srv_ = node_->create_service<GetInt32>(
      "get_lrm_measure_distance", [this](const std::shared_ptr<GetInt32::Request> request,
                                         std::shared_ptr<GetInt32::Response> response) {
        getLrmMeasureDistanceCallback(request, response);
      });
  set_reset_timestamp_srv_ = node_->create_service<SetBool>(
      "set_reset_timestamp", [this](const std::shared_ptr<SetBool::Request> request,
                                    std::shared_ptr<SetBool::Response> response) {
        setRESETTimestampCallback(request, response);
      });
  set_interleaver_laser_sync_srv_ = node_->create_service<SetInt32>(
      "set_sync_interleaverlaser", [this](const std::shared_ptr<SetInt32::Request> request,
                                          std::shared_ptr<SetInt32::Response> response) {
        setSYNCInterleaveLaserCallback(request, response);
      });
  set_sync_host_time_srv_ = node_->create_service<SetBool>(
      "set_sync_hosttime", [this](const std::shared_ptr<SetBool::Request> request,
                                  std::shared_ptr<SetBool::Response> response) {
        setSYNCHostimeCallback(request, response);
      });
  send_service_trigger_srv_ = node_->create_service<CameraTrigger>(
      "send_service_trigger", [this](const std::shared_ptr<CameraTrigger::Request> request,
                                     std::shared_ptr<CameraTrigger::Response> response) {
        sendSoftwareTriggerCallback(request, response);
      });
  set_write_customerdata_srv_ = node_->create_service<SetString>(
      "set_write_customer_data", [this](const std::shared_ptr<SetString::Request> request,
                                        std::shared_ptr<SetString::Response> response) {
        setWriteCustomerData(request, response);
      });
  set_read_customerdata_srv_ = node_->create_service<SetString>(
      "set_read_customer_data", [this](const std::shared_ptr<SetString::Request> request,
                                       std::shared_ptr<SetString::Response> response) {
        setReadCustomerData(request, response);
      });
  change_state_srv_ = node_->create_service<lifecycle_msgs::srv::ChangeState>(
      "change_state", std::bind(&OBCameraNode::handleChangeStateRequest, this,
                                std::placeholders::_1, std::placeholders::_2));
}

void OBCameraNode::setExposureCallback(const std::shared_ptr<SetInt32::Request>& request,
                                       std::shared_ptr<SetInt32::Response>& response,
                                       const stream_index_pair& stream_index) {
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR_LEFT:
      case OB_STREAM_IR_RIGHT:
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
  (void)request;
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR_LEFT:
      case OB_STREAM_IR_RIGHT:
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
    response->success = true;
  } catch (ob::Error& e) {
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

void OBCameraNode::setGainCallback(const std::shared_ptr<SetInt32 ::Request>& request,
                                   std::shared_ptr<SetInt32::Response>& response,
                                   const stream_index_pair& stream_index) {
  auto stream = stream_index.first;
  OBPropertyID prop_id = OB_PROP_IR_GAIN_INT;
  try {
    switch (stream) {
      case OB_STREAM_IR_LEFT:
      case OB_STREAM_IR_RIGHT:
      case OB_STREAM_IR:
        prop_id = OB_PROP_IR_GAIN_INT;
        break;
      case OB_STREAM_DEPTH:
        prop_id = OB_PROP_DEPTH_GAIN_INT;
        break;
      case OB_STREAM_COLOR:
        prop_id = OB_PROP_COLOR_GAIN_INT;
        break;
      default:
        RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
        response->success = false;
        response->message = "NOT a video stream";
        return;
    }
    auto range = device_->getIntPropertyRange(prop_id);
    if (request->data < range.min || request->data > range.max) {
      response->success = false;
      RCLCPP_INFO_STREAM(logger_, "set gain value out of range");
      response->message = "value out of range";
      return;
    }
    device_->setIntProperty(prop_id, request->data);
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

void OBCameraNode::setAeRoiCallback(const std::shared_ptr<SetArrays ::Request>& request,
                                    std::shared_ptr<SetArrays::Response>& response,
                                    const stream_index_pair& stream_index) {
  auto stream = stream_index.first;
  auto config = OBRegionOfInterest();
  uint32_t data_size = sizeof(config);
  try {
    switch (stream) {
      case OB_STREAM_IR_LEFT:
      case OB_STREAM_IR_RIGHT:
      case OB_STREAM_IR:
      case OB_STREAM_DEPTH:
        config.x0_left = (static_cast<short int>(request->data_param[0]) < 0)
                             ? 0
                             : static_cast<short int>(request->data_param[0]);
        config.x0_left = (static_cast<short int>(request->data_param[0]) > width_[DEPTH] - 1)
                             ? width_[DEPTH] - 1
                             : config.x0_left;
        config.y0_top = (static_cast<short int>(request->data_param[2]) < 0)
                            ? 0
                            : static_cast<short int>(request->data_param[2]);
        config.y0_top = (static_cast<short int>(request->data_param[2]) > height_[DEPTH] - 1)
                            ? height_[DEPTH] - 1
                            : config.y0_top;
        config.x1_right = (static_cast<short int>(request->data_param[1]) < 0)
                              ? 0
                              : static_cast<short int>(request->data_param[1]);
        config.x1_right = (static_cast<short int>(request->data_param[1]) > width_[DEPTH] - 1)
                              ? width_[DEPTH] - 1
                              : config.x1_right;
        config.y1_bottom = (static_cast<short int>(request->data_param[3]) < 0)
                               ? 0
                               : static_cast<short int>(request->data_param[3]);
        config.y1_bottom = (static_cast<short int>(request->data_param[3]) > height_[DEPTH] - 1)
                               ? height_[DEPTH] - 1
                               : config.y1_bottom;
        device_->setStructuredData(OB_STRUCT_DEPTH_AE_ROI,
                                   reinterpret_cast<const uint8_t*>(&config), sizeof(config));
        device_->getStructuredData(OB_STRUCT_DEPTH_AE_ROI, reinterpret_cast<uint8_t*>(&config),
                                   &data_size);
        RCLCPP_INFO_STREAM(logger_,
                           "set depth AE ROI : " << "[Left: " << config.x0_left << ", Right: "
                                                 << config.x1_right << ", Top: " << config.y0_top
                                                 << ", Bottom: " << config.y1_bottom << " ]");
        break;
      case OB_STREAM_COLOR:
        config.x0_left = (static_cast<short int>(request->data_param[0]) < 0)
                             ? 0
                             : static_cast<short int>(request->data_param[0]);
        config.x0_left = (static_cast<short int>(request->data_param[0]) > width_[COLOR] - 1)
                             ? width_[COLOR] - 1
                             : config.x0_left;
        config.y0_top = (static_cast<short int>(request->data_param[2]) < 0)
                            ? 0
                            : static_cast<short int>(request->data_param[2]);
        config.y0_top = (static_cast<short int>(request->data_param[2]) > height_[COLOR] - 1)
                            ? height_[COLOR] - 1
                            : config.y0_top;
        config.x1_right = (static_cast<short int>(request->data_param[1]) < 0)
                              ? 0
                              : static_cast<short int>(request->data_param[1]);
        config.x1_right = (static_cast<short int>(request->data_param[1]) > width_[COLOR] - 1)
                              ? width_[COLOR] - 1
                              : config.x1_right;
        config.y1_bottom = (static_cast<short int>(request->data_param[3]) < 0)
                               ? 0
                               : static_cast<short int>(request->data_param[3]);
        config.y1_bottom = (static_cast<short int>(request->data_param[3]) > height_[COLOR] - 1)
                               ? height_[COLOR] - 1
                               : config.y1_bottom;
        device_->setStructuredData(OB_STRUCT_COLOR_AE_ROI,
                                   reinterpret_cast<const uint8_t*>(&config), sizeof(config));
        device_->getStructuredData(OB_STRUCT_COLOR_AE_ROI, reinterpret_cast<uint8_t*>(&config),
                                   &data_size);
        RCLCPP_INFO_STREAM(logger_,
                           "set color AE ROI : " << "[Left: " << config.x0_left << ", Right: "
                                                 << config.x1_right << ", Top: " << config.y0_top
                                                 << ", Bottom: " << config.y1_bottom << " ]");
        break;
      default:
        RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
        response->success = false;
        response->message = "NOT a video stream";
        return;
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

void OBCameraNode::getWhiteBalanceCallback(const std::shared_ptr<GetInt32::Request>& request,
                                           std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  try {
    response->data = device_->getIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT);
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

void OBCameraNode::setWhiteBalanceCallback(const std::shared_ptr<SetInt32 ::Request>& request,
                                           std::shared_ptr<SetInt32 ::Response>& response) {
  try {
    auto range = device_->getIntPropertyRange(OB_PROP_COLOR_WHITE_BALANCE_INT);
    if (request->data < range.min || request->data > range.max) {
      response->success = false;
      RCLCPP_INFO_STREAM(logger_, "set white balance value out of range");
      response->message = "value out of range";
      return;
    }
    bool auto_white_balance = device_->getBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
    if (auto_white_balance) {
      RCLCPP_WARN(logger_, "auto white balance is enabled, set white balance will be ignored");
      response->success = false;
      response->message = "auto white balance is enabled";
      return;
    }
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

void OBCameraNode::getAutoWhiteBalanceCallback(const std::shared_ptr<GetInt32::Request>& request,
                                               std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  try {
    response->data = device_->getBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
    response->success = true;
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

void OBCameraNode::setAutoWhiteBalanceCallback(const std::shared_ptr<SetBool::Request>& request,
                                               std::shared_ptr<SetBool::Response>& response) {
  try {
    device_->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, request->data);
    response->success = true;
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

void OBCameraNode::setAutoExposureCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response,
    const stream_index_pair& stream_index) {
  auto stream = stream_index.first;
  OBPropertyID prop_id = OB_PROP_IR_AUTO_EXPOSURE_BOOL;
  try {
    switch (stream) {
      case OB_STREAM_IR_LEFT:
      case OB_STREAM_IR_RIGHT:
      case OB_STREAM_IR:
        prop_id = OB_PROP_IR_AUTO_EXPOSURE_BOOL;
        break;
      case OB_STREAM_DEPTH:
        prop_id = OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL;
        break;
      case OB_STREAM_COLOR:
        prop_id = OB_PROP_COLOR_AUTO_EXPOSURE_BOOL;
        break;
      default:
        RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
        response->success = false;
        response->message = "NOT a video stream";
        return;
    }
    auto range = device_->getIntPropertyRange(prop_id);
    if (request->data < range.min || request->data > range.max) {
      response->success = false;
      RCLCPP_INFO_STREAM(logger_, "set auto exposure value out of range");
      response->message = "value out of range";
      return;
    }
    device_->setIntProperty(prop_id, request->data);
    response->success = true;
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

void OBCameraNode::setFanWorkModeCallback(const std::shared_ptr<SetInt32::Request>& request,
                                          std::shared_ptr<SetInt32::Response>& response) {
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
  int laser_enable = request->data ? 1 : 0;
  try {
    if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, laser_enable);
    } else if (device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_LASER_BOOL, laser_enable);
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
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
    if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
      auto laser_enable = device_->getIntProperty(OB_PROP_LASER_CONTROL_INT);
      device_->setBoolProperty(OB_PROP_LDP_BOOL, ldp_enable);
      device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, laser_enable);
    } else if (device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
      if (!ldp_enable) {
        auto laser_enable = device_->getIntProperty(OB_PROP_LASER_BOOL);
        device_->setBoolProperty(OB_PROP_LDP_BOOL, ldp_enable);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        device_->setIntProperty(OB_PROP_LASER_BOOL, laser_enable);
      } else {
        device_->setBoolProperty(OB_PROP_LDP_BOOL, ldp_enable);
      }
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

void OBCameraNode::getExposureCallback(const std::shared_ptr<GetInt32::Request>& request,
                                       std::shared_ptr<GetInt32 ::Response>& response,
                                       const stream_index_pair& stream_index) {
  (void)request;
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR_LEFT:
      case OB_STREAM_IR_RIGHT:
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
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::getDeviceInfoCallback(const std::shared_ptr<GetDeviceInfo::Request>& request,
                                         std::shared_ptr<GetDeviceInfo::Response>& response) {
  (void)request;
  try {
    auto device_info = device_->getDeviceInfo();
    response->info.name = device_info->getName();
    response->info.serial_number = device_info->getSerialNumber();
    response->info.firmware_version = device_info->getFirmwareVersion();
    response->info.supported_min_sdk_version = device_info->getSupportedMinSdkVersion();
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

void OBCameraNode::getSDKVersion(const std::shared_ptr<GetString::Request>& request,
                                 std::shared_ptr<GetString::Response>& response) {
  (void)request;
  try {
    auto device_info = device_->getDeviceInfo();
    nlohmann::json data;
    data["firmware_version"] = device_info->getFirmwareVersion();
    data["supported_min_sdk_version"] = device_info->getSupportedMinSdkVersion();
    data["ros_sdk_version"] = OB_ROS_VERSION_STR;
    data["ob_sdk_version"] = getObSDKVersion();
    response->data = data.dump(2);
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

void OBCameraNode::setMirrorCallback(const std::shared_ptr<SetBool::Request>& request,
                                     std::shared_ptr<SetBool::Response>& response,
                                     const stream_index_pair& stream_index) {
  (void)request;
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR_RIGHT:
        device_->setBoolProperty(OB_PROP_IR_RIGHT_MIRROR_BOOL, request->data);
        break;
      case OB_STREAM_IR_LEFT:
      case OB_STREAM_IR:
        device_->setBoolProperty(OB_PROP_IR_MIRROR_BOOL, request->data);
        break;
      case OB_STREAM_DEPTH:
        device_->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, request->data);
        break;
      case OB_STREAM_COLOR:
        device_->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, request->data);
        break;
      default:
        RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::setFlipCallback(const std::shared_ptr<SetBool::Request>& request,
                                   std::shared_ptr<SetBool::Response>& response,
                                   const stream_index_pair& stream_index) {
  (void)request;
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR_RIGHT:
        device_->setBoolProperty(OB_PROP_IR_RIGHT_FLIP_BOOL, request->data);
        break;
      case OB_STREAM_IR_LEFT:
        device_->setBoolProperty(OB_PROP_IR_FLIP_BOOL, request->data);
        break;
      case OB_STREAM_IR:
        device_->setBoolProperty(OB_PROP_IR_FLIP_BOOL, request->data);
        break;
      case OB_STREAM_DEPTH:
        device_->setBoolProperty(OB_PROP_DEPTH_FLIP_BOOL, request->data);
        break;
      case OB_STREAM_COLOR:
        device_->setBoolProperty(OB_PROP_COLOR_FLIP_BOOL, request->data);
        break;
      default:
        RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::setRotationCallback(const std::shared_ptr<SetInt32::Request>& request,
                                       std::shared_ptr<SetInt32::Response>& response,
                                       const stream_index_pair& stream_index) {
  (void)request;
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR_RIGHT:
        device_->setIntProperty(OB_PROP_IR_RIGHT_ROTATE_INT, request->data);
        break;
      case OB_STREAM_IR_LEFT:
        device_->setIntProperty(OB_PROP_IR_ROTATE_INT, request->data);
        break;
      case OB_STREAM_IR:
        device_->setIntProperty(OB_PROP_IR_ROTATE_INT, request->data);
        break;
      case OB_STREAM_DEPTH:
        device_->setIntProperty(OB_PROP_DEPTH_ROTATE_INT, request->data);
        break;
      case OB_STREAM_COLOR:
        device_->setIntProperty(OB_PROP_COLOR_ROTATE_INT, request->data);
        break;
      default:
        RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::getLdpStatusCallback(const std::shared_ptr<GetBool::Request>& request,
                                        std::shared_ptr<GetBool::Response>& response) {
  (void)request;
  try {
    response->data = device_->getBoolProperty(OB_PROP_LDP_STATUS_BOOL);
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::getLrmMeasureDistanceCallback(const std::shared_ptr<GetInt32::Request>& request,
                                                 std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  try {
    response->data = device_->getIntProperty(OB_PROP_LDP_MEASURE_DISTANCE_INT);
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::toggleSensorCallback(const std::shared_ptr<SetBool::Request>& request,
                                        std::shared_ptr<SetBool::Response>& response,
                                        const stream_index_pair& stream_index) {
  std::string msg;
  if (request->data) {
    if (enable_stream_[stream_index]) {
      msg = stream_name_[stream_index] + " Already ON";
    }
    RCLCPP_INFO_STREAM(logger_, "toggling sensor " << stream_name_[stream_index] << " ON");

  } else {
    if (!enable_stream_[stream_index]) {
      msg = stream_name_[stream_index] + " Already OFF";
    }
    RCLCPP_INFO_STREAM(logger_, "toggling sensor " << stream_name_[stream_index] << " OFF");
  }
  if (!msg.empty()) {
    RCLCPP_ERROR_STREAM(logger_, msg);
    response->success = false;
    response->message = msg;
    return;
  }
  response->success = toggleSensor(stream_index, request->data, response->message);
}

bool OBCameraNode::toggleSensor(const stream_index_pair& stream_index, bool enabled,
                                std::string& msg) {
  try {
    pipeline_->stop();
    enable_stream_[stream_index] = enabled;
    setupProfiles();
    startStreams();
    return true;
  } catch (const ob::Error& e) {
    msg = e.getMessage();
    return false;
  } catch (const std::exception& e) {
    msg = e.what();
    return false;
  } catch (...) {
    msg = "unknown error";
    return false;
  }
}

void OBCameraNode::saveImageCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>& request,
                                     std::shared_ptr<std_srvs::srv::Empty::Response>& response) {
  (void)request;
  (void)response;
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      save_images_[stream_index] = true;
      save_images_count_[stream_index] = 0;
    }
  }
}

void OBCameraNode::savePointCloudCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>& request,
    std::shared_ptr<std_srvs::srv::Empty::Response>& response) {
  (void)request;
  (void)response;
  if (enable_point_cloud_) {
    save_point_cloud_ = true;
  }
  if (enable_colored_point_cloud_) {
    save_colored_point_cloud_ = true;
  }
}

void OBCameraNode::switchIRCameraCallback(const std::shared_ptr<SetString::Request>& request,
                                          std::shared_ptr<SetString::Response>& response) {
  if (request->data != "left" && request->data != "right") {
    response->success = false;
    response->message = "invalid ir camera name";
    return;
  }
  try {
    int data = request->data == "left" ? 0 : 1;
    device_->setIntProperty(OB_PROP_IR_CHANNEL_DATA_SOURCE_INT, data);
    response->success = true;
    return;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}
void OBCameraNode::setIRLongExposureCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  try {
    device_->setBoolProperty(OB_PROP_IR_LONG_EXPOSURE_BOOL, request->data);
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::setRESETTimestampCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  (void)request;
  try {
    device_->setBoolProperty(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL, true);
    device_->setBoolProperty(OB_PROP_TIMER_RESET_SIGNAL_BOOL, true);
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}
void OBCameraNode::setSYNCInterleaveLaserCallback(
    const std::shared_ptr<SetInt32 ::Request>& request,
    std::shared_ptr<SetInt32 ::Response>& response) {
  (void)request;
  try {
    device_->setIntProperty(OB_PROP_FRAME_INTERLEAVE_LASER_PATTERN_SYNC_DELAY_INT, request->data);
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}
void OBCameraNode::setSYNCHostimeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  (void)request;
  try {
    device_->timerSyncWithHost();
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::resetCaptureServiceVariables() {
  service_capture_started_ = false;
  number_of_rgb_frames_captured_ = 0;
  number_of_depth_frames_captured_ = 0;
}

void OBCameraNode::handleChangeStateRequest(
    const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request,
        std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> response) {
    
  RCLCPP_INFO(logger_, "Received request to change state '%d' with label '%s'",
                     request->transition.id, request->transition.label.c_str());
  
  response->success = true;
  if(request->transition.id == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
    RCLCPP_INFO_STREAM(logger_, "Recieved request to turn ON camera streams");
    if (pipeline_started_){
      RCLCPP_WARN_STREAM(logger_, "Camera streams already ON");
      response->success = true;
      return;
    }
    try {
      setupProfiles();
      startStreams();
      RCLCPP_INFO_STREAM(logger_, "Camera streams are now ON");
    } catch (const ob::Error& e) {
      response->success = false;
      RCLCPP_ERROR_STREAM(logger_, "Failed to start camera streams: " << e.getMessage());
    } catch (const std::exception& e) {
      response->success = false;
      RCLCPP_ERROR_STREAM(logger_, "Failed to start camera streams: " << e.what());
    } catch (...) {
      response->success = false;
      RCLCPP_ERROR_STREAM(logger_, "Failed to start camera streams: Unknown Error");
    }
  }
  else if(request->transition.id == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) {
    RCLCPP_INFO_STREAM(logger_, "Recieved request to turn OFF camera streams");
    if (!pipeline_ || !pipeline_started_){
      RCLCPP_WARN_STREAM(logger_, "Camera streams already OFF");
      response->success = true;
      return;
    }
    try {
      stopStreams();
      RCLCPP_INFO_STREAM(logger_, "Camera streams are now OFF");
    } catch (const ob::Error& e) {
      response->success = false;
      RCLCPP_ERROR_STREAM(logger_, "Failed to stop camera streams: " << e.getMessage());
    } catch (const std::exception& e) {
      response->success = false;
      RCLCPP_ERROR_STREAM(logger_, "Failed to stop camera streams: " << e.what());
    } catch (...) {
      response->success = false;
      RCLCPP_ERROR_STREAM(logger_, "Failed to stop camera streams: Unknown Error");
    }
  }
  else {
    response->success = false;
    RCLCPP_ERROR_STREAM(logger_, "Unsupported transition ID: " << request->transition.id);
  }
}

void OBCameraNode::sendSoftwareTriggerCallback(
    const std::shared_ptr<CameraTrigger::Request>& request,
    std::shared_ptr<CameraTrigger::Response>& response) {
  (void)request;
  if (service_trigger_enabled_ and !software_trigger_enabled_) {
    service_capture_started_ = true;
    try {
      RCLCPP_DEBUG_STREAM(logger_, "Triggering");
      device_->triggerCapture();

      RCLCPP_DEBUG_STREAM(logger_, "Capturing images");
      std::unique_lock<std::mutex> lock(service_capture_lock_);
      service_capture_cv_.wait_until(
          lock, std::chrono::system_clock::now() + std::chrono::seconds(1), [this]() {
            return (number_of_rgb_frames_captured_ >= frames_per_trigger_ &&
                    number_of_depth_frames_captured_ >= frames_per_trigger_);
          });
      RCLCPP_DEBUG_STREAM(logger_, "Captured " << number_of_rgb_frames_captured_ << " "
                                               << number_of_depth_frames_captured_ << " "
                                               << (color_image_ == nullptr) << " "
                                               << (depth_image_ == nullptr));

      if (number_of_rgb_frames_captured_ >= frames_per_trigger_ &&
          number_of_depth_frames_captured_ >= frames_per_trigger_ && color_image_ && depth_image_) {
        response->success = true;
        response->rgb_image = *(color_image_);
        response->depth_image = *(depth_image_);
        response->rgb_camera_info = color_image_camera_info_;
        response->depth_camera_info = depth_image_camera_info_;
      } else {
        response->success = false;
        response->message = "Failed to capture images";
      }
    } catch (const ob::Error& e) {
      response->message = e.getMessage();
      response->success = false;
    } catch (const std::exception& e) {
      response->message = e.what();
      response->success = false;
    } catch (...) {
      response->message = "unknown error";
      response->success = false;
    }
    resetCaptureServiceVariables();
  } else {
    RCLCPP_INFO_STREAM(logger_, "Service not enabled");
  }
}

void OBCameraNode::setWriteCustomerData(const std::shared_ptr<SetString::Request>& request,
                                        std::shared_ptr<SetString::Response>& response) {
  if (request->data.empty()) {
    response->success = false;
    response->message = "set write customer data is empty";
    return;
  }
  try {
    device_->writeCustomerData(request->data.c_str(), request->data.size());
    response->message = "set write customer data is " + request->data;
    response->success = true;
    return;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}
void OBCameraNode::setReadCustomerData(const std::shared_ptr<SetString::Request>& request,
                                       std::shared_ptr<SetString::Response>& response) {
  (void)request;
  try {
    std::vector<uint8_t> customer_date;
    customer_date.resize(40960);
    uint32_t customer_date_len = 0;
    device_->readCustomerData(customer_date.data(), &customer_date_len);
    std::string customer_date_str(customer_date.begin(), customer_date.end());
    response->message = "read customer data is " + customer_date_str;
    response->success = true;
    return;
  } catch (const ob::Error& e) {
    response->message = e.getMessage();
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}
}  // namespace orbbec_camera
