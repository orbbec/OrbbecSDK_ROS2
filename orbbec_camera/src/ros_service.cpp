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
  get_ldp_measure_distance_srv_ = node_->create_service<GetInt32>(
      "get_ldp_measure_distance", [this](const std::shared_ptr<GetInt32::Request> request,
                                         std::shared_ptr<GetInt32::Response> response) {
        getLdpMeasureDistanceCallback(request, response);
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
  set_decimation_filter_enable_srv_ = node_->create_service<SetBool>(
      "set_decimation_filter_enable", [this](const std::shared_ptr<SetBool::Request> request,
                                             std::shared_ptr<SetBool::Response> response) {
        setDecimationFilterEnableCallback(request, response);
      });
  set_sequence_id_filter_enable_srv_ = node_->create_service<SetBool>(
      "set_sequence_id_filter_enable", [this](const std::shared_ptr<SetBool::Request> request,
                                              std::shared_ptr<SetBool::Response> response) {
        setSequenceIdFilterEnableCallback(request, response);
      });
  set_threshold_filter_enable_srv_ = node_->create_service<SetBool>(
      "set_threshold_filter_enable", [this](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
        setThresholdFilterEnableCallback(request, response);
      });
  set_spatial_filter_enable_srv_ = node_->create_service<SetBool>(
      "set_spatial_filter_enable", [this](const std::shared_ptr<SetBool::Request> request,
                                          std::shared_ptr<SetBool::Response> response) {
        setSpatialFilterEnableCallback(request, response);
      });
  set_temporal_filter_enable_srv_ = node_->create_service<SetBool>(
      "set_temporal_filter_enable", [this](const std::shared_ptr<SetBool::Request> request,
                                           std::shared_ptr<SetBool::Response> response) {
        setTemporalFilterEnableCallback(request, response);
      });
  set_hole_filling_filter_enable_srv_ = node_->create_service<SetBool>(
      "set_hole_filling_filter_enable", [this](const std::shared_ptr<SetBool::Request> request,
                                               std::shared_ptr<SetBool::Response> response) {
        setHoleFillingFilterEnableCallback(request, response);
      });
  set_all_software_filter_enable_srv_ = node_->create_service<SetBool>(
      "set_all_software_filter_enable", [this](const std::shared_ptr<SetBool::Request> request,
                                               std::shared_ptr<SetBool::Response> response) {
        setAllSoftwareFilterEnableCallback(request, response);
      });
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

void OBCameraNode::getLdpMeasureDistanceCallback(const std::shared_ptr<GetInt32::Request>& request,
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
void OBCameraNode::setDecimationFilterEnableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  try {
    enable_decimation_filter_ = request->data;
    if (enable_decimation_filter_) {
      decimation_filter_scale_ = node_->get_parameter("decimation_filter_scale").as_int();
      if (decimation_filter_scale_ == -1) {
        RCLCPP_WARN(logger_, "Please configure the parameter 'decimation_filter_scale'");
        return;
      }
    } else {
      decimation_filter_scale_ = -1;
      node_->set_parameter(rclcpp::Parameter("decimation_filter_scale", decimation_filter_scale_));
    }
    setupDepthPostProcessFilter();
    node_->set_parameter(rclcpp::Parameter("enable_decimation_filter", enable_decimation_filter_));
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
void OBCameraNode::setSequenceIdFilterEnableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  try {
    enable_sequence_id_filter_ = request->data;
    if (enable_sequence_id_filter_) {
      sequence_id_filter_id_ = node_->get_parameter("sequence_id_filter_id").as_int();
      if (sequence_id_filter_id_ == -1) {
        RCLCPP_WARN(logger_, "Please configure the parameter 'sequence_id_filter_id'");
        return;
      }
    } else {
      sequence_id_filter_id_ = -1;
      node_->set_parameter(rclcpp::Parameter("sequence_id_filter_id", sequence_id_filter_id_));
    }
    setupDepthPostProcessFilter();
    node_->set_parameter(
        rclcpp::Parameter("enable_sequence_id_filter", enable_sequence_id_filter_));
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
void OBCameraNode::setThresholdFilterEnableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  try {
    enable_threshold_filter_ = request->data;
    if (enable_threshold_filter_) {
      threshold_filter_max_ = node_->get_parameter("threshold_filter_max").as_int();
      threshold_filter_min_ = node_->get_parameter("threshold_filter_min").as_int();
      if (threshold_filter_max_ == -1 || threshold_filter_min_ == -1) {
        RCLCPP_WARN(
            logger_,
            "Please configure the parameter 'threshold_filter_max' and 'threshold_filter_min'");
        return;
      }
    } else {
      threshold_filter_max_ = -1;
      threshold_filter_min_ = -1;
      node_->set_parameter(rclcpp::Parameter("threshold_filter_max", threshold_filter_max_));
      node_->set_parameter(rclcpp::Parameter("threshold_filter_min", threshold_filter_min_));
    }
    setupDepthPostProcessFilter();
    node_->set_parameter(rclcpp::Parameter("enable_threshold_filter", enable_threshold_filter_));
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
void OBCameraNode::setSpatialFilterEnableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  try {
    enable_spatial_filter_ = request->data;
    if (enable_spatial_filter_) {
      spatial_filter_alpha_ =
          static_cast<float>(node_->get_parameter("spatial_filter_alpha").as_double());
      spatial_filter_diff_threshold_ =
          node_->get_parameter("spatial_filter_diff_threshold").as_int();
      spatial_filter_magnitude_ = node_->get_parameter("spatial_filter_magnitude").as_int();
      spatial_filter_radius_ = node_->get_parameter("spatial_filter_radius").as_int();
      if (spatial_filter_alpha_ == -1.0 || spatial_filter_diff_threshold_ == -1 ||
          spatial_filter_magnitude_ == -1 || spatial_filter_radius_ == -1) {
        return;
      }
    } else {
      spatial_filter_alpha_ = -1.0;
      spatial_filter_diff_threshold_ = -1;
      spatial_filter_magnitude_ = -1;
      spatial_filter_radius_ = -1;
      node_->set_parameter(rclcpp::Parameter("spatial_filter_alpha", spatial_filter_alpha_));
      node_->set_parameter(
          rclcpp::Parameter("spatial_filter_diff_threshold", spatial_filter_diff_threshold_));
      node_->set_parameter(
          rclcpp::Parameter("spatial_filter_magnitude", spatial_filter_magnitude_));
      node_->set_parameter(rclcpp::Parameter("spatial_filter_radius", spatial_filter_radius_));
    }
    setupDepthPostProcessFilter();
    node_->set_parameter(rclcpp::Parameter("enable_spatial_filter", enable_spatial_filter_));
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
void OBCameraNode::setTemporalFilterEnableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  try {
    enable_temporal_filter_ = request->data;
    if (enable_temporal_filter_) {
      temporal_filter_diff_threshold_ =
          static_cast<float>(node_->get_parameter("temporal_filter_diff_threshold").as_double());
      temporal_filter_weight_ =
          static_cast<float>(node_->get_parameter("temporal_filter_weight").as_double());
      if (temporal_filter_diff_threshold_ == -1.0 || temporal_filter_weight_ == -1.0) {
        return;
      }
    } else {
      temporal_filter_diff_threshold_ = -1.0;
      temporal_filter_weight_ = -1.0;
      node_->set_parameter(
          rclcpp::Parameter("temporal_filter_diff_threshold", temporal_filter_diff_threshold_));
      node_->set_parameter(rclcpp::Parameter("temporal_filter_weight", temporal_filter_weight_));
    }
    setupDepthPostProcessFilter();
    node_->set_parameter(rclcpp::Parameter("enable_temporal_filter", enable_temporal_filter_));
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
void OBCameraNode::setHoleFillingFilterEnableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  try {
    enable_hole_filling_filter_ = request->data;
    if (enable_hole_filling_filter_) {
      hole_filling_filter_mode_ = node_->get_parameter("hole_filling_filter_mode").as_string();
      if (hole_filling_filter_mode_.empty()) {
        return;
      }
    } else {
      hole_filling_filter_mode_ = "";
      node_->set_parameter(
          rclcpp::Parameter("hole_filling_filter_mode", hole_filling_filter_mode_));
    }
    setupDepthPostProcessFilter();
    node_->set_parameter(
        rclcpp::Parameter("enable_hole_filling_filter", enable_hole_filling_filter_));
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
void OBCameraNode::setAllSoftwareFilterEnableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  try {
    if (request->data) {
      decimation_filter_scale_ = node_->get_parameter("decimation_filter_scale").as_int();
      sequence_id_filter_id_ = node_->get_parameter("sequence_id_filter_id").as_int();
      threshold_filter_max_ = node_->get_parameter("threshold_filter_max").as_int();
      threshold_filter_min_ = node_->get_parameter("threshold_filter_min").as_int();
      spatial_filter_alpha_ =
          static_cast<float>(node_->get_parameter("spatial_filter_alpha").as_double());
      spatial_filter_diff_threshold_ =
          node_->get_parameter("spatial_filter_diff_threshold").as_int();
      spatial_filter_magnitude_ = node_->get_parameter("spatial_filter_magnitude").as_int();
      spatial_filter_radius_ = node_->get_parameter("spatial_filter_radius").as_int();
      temporal_filter_diff_threshold_ =
          static_cast<float>(node_->get_parameter("temporal_filter_diff_threshold").as_double());
      temporal_filter_weight_ =
          static_cast<float>(node_->get_parameter("temporal_filter_weight").as_double());
      hole_filling_filter_mode_ = node_->get_parameter("hole_filling_filter_mode").as_string();
      if (decimation_filter_scale_ == -1 || sequence_id_filter_id_ == -1 ||
          threshold_filter_max_ == -1 || threshold_filter_min_ == -1 ||
          spatial_filter_alpha_ == -1.0 || spatial_filter_diff_threshold_ == -1 ||
          spatial_filter_magnitude_ == -1 || spatial_filter_radius_ == -1 ||
          temporal_filter_diff_threshold_ == -1.0 || temporal_filter_weight_ == -1.0 ||
          hole_filling_filter_mode_.empty()) {
        return;
      }
    } else {
      decimation_filter_scale_ = sequence_id_filter_id_ = threshold_filter_max_ =
          threshold_filter_min_ = spatial_filter_diff_threshold_ = spatial_filter_magnitude_ =
              spatial_filter_radius_ = -1;
      spatial_filter_alpha_ = temporal_filter_diff_threshold_ = temporal_filter_weight_ = -1.0;
      hole_filling_filter_mode_ = "";
      node_->set_parameter(rclcpp::Parameter("decimation_filter_scale", decimation_filter_scale_));
      node_->set_parameter(rclcpp::Parameter("sequence_id_filter_id", sequence_id_filter_id_));
      node_->set_parameter(rclcpp::Parameter("threshold_filter_max", threshold_filter_max_));
      node_->set_parameter(rclcpp::Parameter("threshold_filter_min", threshold_filter_min_));
      node_->set_parameter(
          rclcpp::Parameter("spatial_filter_diff_threshold", spatial_filter_diff_threshold_));
      node_->set_parameter(
          rclcpp::Parameter("spatial_filter_magnitude", spatial_filter_magnitude_));
      node_->set_parameter(rclcpp::Parameter("spatial_filter_radius", spatial_filter_radius_));
      node_->set_parameter(rclcpp::Parameter("spatial_filter_alpha", spatial_filter_alpha_));
      node_->set_parameter(
          rclcpp::Parameter("temporal_filter_diff_threshold", temporal_filter_diff_threshold_));
      node_->set_parameter(rclcpp::Parameter("temporal_filter_weight", temporal_filter_weight_));
      node_->set_parameter(
          rclcpp::Parameter("hole_filling_filter_mode", hole_filling_filter_mode_));
    }
    setupDepthPostProcessFilter();
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
}  // namespace orbbec_camera
