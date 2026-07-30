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
#include <algorithm>
#include <cctype>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <thread>

#include "orbbec_camera/utils.h"
namespace orbbec_camera {

namespace {

bool isGemini330SeriesForDisparity(uint32_t pid) {
  return pid == GEMINI_335_PID || pid == GEMINI_336_PID || pid == GEMINI_330_PID ||
         pid == GEMINI_335L_PID || pid == GEMINI_336L_PID || pid == GEMINI_330L_PID ||
         pid == GEMINI_335LG_PID || pid == GEMINI_335LE_PID || pid == GEMINI_338_PID ||
         pid == GEMINI_338LG_PID || pid == GEMINI_338LE_PID || pid == GEMINI_338L_PID ||
         pid == GEMINI_331L_PID;
}

bool isSupportedDisparityResolutionForPid(uint32_t pid, int width, int height) {
  if (pid == GEMINI_335LE_PID || pid == GEMINI_338LE_PID) {
    return (width == 1280 && height == 800) || (width == 640 && height == 400) ||
           (width == 424 && height == 266) || (width == 320 && height == 200);
  }

  return (width == 1280 && height == 800) || (width == 1280 && height == 720) ||
         (width == 640 && height == 400) || (width == 424 && height == 266);
}

std::string getDisparityResolutionHintByPid(uint32_t pid) {
  if (pid == GEMINI_335LE_PID || pid == GEMINI_338LE_PID) {
    return "Supported resolutions for the current device: 1280x800/640x400/424x266/320x200";
  }

  return "Supported resolutions for the current device: "
         "1280x800/1280x720/640x400/424x266";
}

std::string alignTargetStreamToString(OBStreamType stream_type) {
  switch (stream_type) {
    case OB_STREAM_COLOR:
      return "COLOR";
    case OB_STREAM_DEPTH:
      return "DEPTH";
    default:
      return "UNKNOWN";
  }
}

std::string disparityToDepthModeToString(bool hardware_enabled, bool software_enabled) {
  if (hardware_enabled) {
    return "HW";
  }
  if (software_enabled) {
    return "SW";
  }
  return "disable";
}

bool isPropertySupported(const std::shared_ptr<ob::Device>& device, OBPropertyID property_id,
                         OBPermissionType permission) {
  if (!device) {
    return false;
  }
  try {
    return device->isPropertySupported(property_id, permission);
  } catch (...) {
    return false;
  }
}

bool isPropertyReadable(const std::shared_ptr<ob::Device>& device, OBPropertyID property_id) {
  return isPropertySupported(device, property_id, OB_PERMISSION_READ) ||
         isPropertySupported(device, property_id, OB_PERMISSION_READ_WRITE);
}

bool isPropertyWritable(const std::shared_ptr<ob::Device>& device, OBPropertyID property_id) {
  return isPropertySupported(device, property_id, OB_PERMISSION_WRITE) ||
         isPropertySupported(device, property_id, OB_PERMISSION_READ_WRITE);
}

std::string OBSyncModeToString(const OBMultiDeviceSyncMode& mode) {
  switch (mode) {
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN:
      return "FREE_RUN";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_STANDALONE:
      return "STANDALONE";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_PRIMARY:
      return "PRIMARY";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SECONDARY:
      return "SECONDARY";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED:
      return "SECONDARY_SYNCED";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING:
      return "SOFTWARE_TRIGGERING";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING:
      return "HARDWARE_TRIGGERING";
    default:
      return "FREE_RUN";
  }
}

}  // namespace

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
  if (isPropertyWritable(device_, OB_PROP_FAN_WORK_MODE_INT)) {
    set_fan_work_mode_srv_ = node_->create_service<SetInt32>(
        "set_fan_work_mode", [this](const std::shared_ptr<SetInt32::Request> request,
                                    std::shared_ptr<SetInt32::Response> response) {
          setFanWorkModeCallback(request, response);
        });
  }
  if (isPropertyWritable(device_, OB_PROP_FLOOD_BOOL)) {
    set_floor_enable_srv_ = node_->create_service<SetBool>(
        "set_floor_enable", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<SetBool::Request> request,
                                   std::shared_ptr<SetBool::Response> response) {
          setFloorEnableCallback(request_header, request, response);
        });
  }
  if (isPropertyWritable(device_, OB_PROP_LASER_CONTROL_INT) ||
      isPropertyWritable(device_, OB_PROP_LASER_BOOL)) {
    set_laser_enable_srv_ = node_->create_service<SetBool>(
        "set_laser_enable", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<SetBool::Request> request,
                                   std::shared_ptr<SetBool::Response> response) {
          setLaserEnableCallback(request_header, request, response);
        });
  }
  if (isPropertyWritable(device_, OB_PROP_LDP_BOOL) &&
      ((isPropertyReadable(device_, OB_PROP_LASER_CONTROL_INT) &&
        isPropertyWritable(device_, OB_PROP_LASER_CONTROL_INT)) ||
       (isPropertyReadable(device_, OB_PROP_LASER_BOOL) &&
        isPropertyWritable(device_, OB_PROP_LASER_BOOL)))) {
    set_ldp_enable_srv_ = node_->create_service<SetBool>(
        "set_ldp_enable", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<SetBool::Request> request,
                                 std::shared_ptr<SetBool::Response> response) {
          setLdpEnableCallback(request_header, request, response);
        });
  }
  if (isPropertyReadable(device_, OB_PROP_LDP_BOOL) &&
      isPropertyReadable(device_, OB_PROP_LDP_STATUS_BOOL)) {
    get_ldp_status_srv_ = node_->create_service<GetBool>(
        "get_ldp_status", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<GetBool::Request> request,
                                 std::shared_ptr<GetBool::Response> response) {
          (void)request_header;
          getLdpStatusCallback(request, response);
        });
  }
  if (isPropertyReadable(device_, OB_PROP_LASER_CONTROL_INT) ||
      isPropertyReadable(device_, OB_PROP_LASER_BOOL)) {
    get_laser_status_srv_ = node_->create_service<GetBool>(
        "get_laser_status", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<GetBool::Request> request,
                                   std::shared_ptr<GetBool::Response> response) {
          (void)request_header;
          getLaserStatusCallback(request, response);
        });
  }
  if (isPropertyReadable(device_, OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL) &&
      isPropertyWritable(device_, OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL)) {
    set_ptp_config_srv_ = node_->create_service<SetBool>(
        "set_ptp_config", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<SetBool::Request> request,
                                 std::shared_ptr<SetBool::Response> response) {
          setPtpConfigCallback(request_header, request, response);
        });
  }
  if (isPropertyReadable(device_, OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL)) {
    get_ptp_config_srv_ = node_->create_service<GetBool>(
        "get_ptp_config", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<GetBool::Request> request,
                                 std::shared_ptr<GetBool::Response> response) {
          (void)request_header;
          getPtpConfigCallback(request, response);
        });
  }

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
  get_device_config_srv_ = node_->create_service<GetDeviceConfig>(
      "get_device_config", [this](const std::shared_ptr<GetDeviceConfig::Request> request,
                                  std::shared_ptr<GetDeviceConfig::Response> response) {
        getDeviceConfigCallback(request, response);
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
  export_config_json_srv_ = node_->create_service<SetString>(
      "export_config_json", [this](const std::shared_ptr<SetString::Request> request,
                                   std::shared_ptr<SetString::Response> response) {
        exportConfigJsonCallback(request, response);
      });
  if (isPropertyWritable(device_, OB_PROP_IR_CHANNEL_DATA_SOURCE_INT)) {
    switch_ir_camera_srv_ = node_->create_service<SetString>(
        "switch_ir", [this](const std::shared_ptr<SetString::Request> request,
                            std::shared_ptr<SetString::Response> response) {
          switchIRCameraCallback(request, response);
        });
  }
  if (isPropertyWritable(device_, OB_PROP_IR_LONG_EXPOSURE_BOOL)) {
    set_ir_long_exposure_srv_ = node_->create_service<SetBool>(
        "set_ir_long_exposure", [this](const std::shared_ptr<SetBool::Request> request,
                                       std::shared_ptr<SetBool::Response> response) {
          setIRLongExposureCallback(request, response);
        });
  }
  if (isPropertyReadable(device_, OB_PROP_LDP_MEASURE_DISTANCE_INT)) {
    get_lrm_measure_distance_srv_ = node_->create_service<GetInt32>(
        "get_lrm_measure_distance", [this](const std::shared_ptr<GetInt32::Request> request,
                                           std::shared_ptr<GetInt32::Response> response) {
          getLrmMeasureDistanceCallback(request, response);
        });
  }
  if (isPropertyWritable(device_, OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL) &&
      isPropertyWritable(device_, OB_PROP_TIMER_RESET_SIGNAL_BOOL)) {
    set_reset_timestamp_srv_ = node_->create_service<SetBool>(
        "set_reset_timestamp", [this](const std::shared_ptr<SetBool::Request> request,
                                      std::shared_ptr<SetBool::Response> response) {
          setRESETTimestampCallback(request, response);
        });
  }
  if (isPropertyWritable(device_, OB_PROP_FRAME_INTERLEAVE_LASER_PATTERN_SYNC_DELAY_INT)) {
    set_interleaver_laser_sync_srv_ = node_->create_service<SetInt32>(
        "set_sync_interleaverlaser", [this](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          setSYNCInterleaveLaserCallback(request, response);
        });
  }
  set_sync_host_time_srv_ = node_->create_service<SetBool>(
      "set_sync_hosttime", [this](const std::shared_ptr<SetBool::Request> request,
                                  std::shared_ptr<SetBool::Response> response) {
        setSYNCHostimeCallback(request, response);
      });
  send_software_trigger_srv_ = node_->create_service<SetBool>(
      "send_software_trigger", [this](const std::shared_ptr<SetBool::Request> request,
                                      std::shared_ptr<SetBool::Response> response) {
        sendSoftwareTriggerCallback(request, response);
      });
  if (device_->getDeviceInfo()->getPid() == GEMINI_435Le_PID) {
    write_customerdata_srv_ = node_->create_service<SetString>(
        "write_customer_data", [this](const std::shared_ptr<SetString::Request> request,
                                      std::shared_ptr<SetString::Response> response) {
          writeCustomerDataCallback(request, response);
        });
    read_customerdata_srv_ = node_->create_service<GetString>(
        "read_customer_data", [this](const std::shared_ptr<GetString::Request> request,
                                     std::shared_ptr<GetString::Response> response) {
          readCustomerDataCallback(request, response);
        });
    set_user_calib_params_srv_ = node_->create_service<SetUserCalibParams>(
        "set_user_calib_params", [this](const std::shared_ptr<SetUserCalibParams::Request> request,
                                        std::shared_ptr<SetUserCalibParams::Response> response) {
          setUserCalibParamsCallback(request, response);
        });
    get_user_calib_params_srv_ = node_->create_service<GetUserCalibParams>(
        "get_user_calib_params", [this](const std::shared_ptr<GetUserCalibParams::Request> request,
                                        std::shared_ptr<GetUserCalibParams::Response> response) {
          getUserCalibParamsCallback(request, response);
        });
  }
  set_ae_reference_stream_srv_ = node_->create_service<SetString>(
      "set_ae_reference_stream", [this](const std::shared_ptr<SetString::Request> request,
                                        std::shared_ptr<SetString::Response> response) {
        setAEReferenceStreamCallback(request, response);
      });
  set_ae_strategy_srv_ = node_->create_service<SetString>(
      "set_ae_strategy", [this](const std::shared_ptr<SetString::Request> request,
                                std::shared_ptr<SetString::Response> response) {
        setAEStrategyCallback(request, response);
      });
  set_streams_enable_srv_ = node_->create_service<SetBool>(
      "set_streams_enable", [this](const std::shared_ptr<SetBool::Request> request,
                                   std::shared_ptr<SetBool::Response> response) {
        setStreamsEnableCallback(request, response);
      });
  set_image_registration_mode_srv_ = node_->create_service<SetString>(
      "set_image_registration_mode", [this](const std::shared_ptr<SetString::Request> request,
                                            std::shared_ptr<SetString::Response> response) {
        setImageRegistrationModeCallback(request, response);
      });
  set_stream_profile_srv_ = node_->create_service<SetStreamProfile>(
      "set_stream_profile", [this](const std::shared_ptr<SetStreamProfile::Request> request,
                                   std::shared_ptr<SetStreamProfile::Response> response) {
        setStreamProfileCallback(request, response);
      });
  get_streams_enable_srv_ = node_->create_service<GetBool>(
      "get_streams_enable", [this](const std::shared_ptr<GetBool::Request> request,
                                   std::shared_ptr<GetBool::Response> response) {
        getStreamsEnableCallback(request, response);
      });
  set_point_cloud_decimation_srv_ = node_->create_service<SetInt32>(
      "set_point_cloud_decimation", [this](const std::shared_ptr<SetInt32::Request> request,
                                           std::shared_ptr<SetInt32::Response> response) {
        setPointCloudDecimationCallback(request, response);
      });
  get_point_cloud_decimation_srv_ = node_->create_service<GetInt32>(
      "get_point_cloud_decimation", [this](const std::shared_ptr<GetInt32::Request> request,
                                           std::shared_ptr<GetInt32::Response> response) {
        getPointCloudDecimationCallback(request, response);
      });
  if (isPropertyWritable(device_, OB_PROP_DISP_SEARCH_RANGE_MODE_INT)) {
    set_disparity_range_mode_srv_ = node_->create_service<SetInt32>(
        "set_disparity_range_mode", [this](const std::shared_ptr<SetInt32::Request> request,
                                           std::shared_ptr<SetInt32::Response> response) {
          setDisparityRangeModeCallback(request, response);
        });
  }
  if (isPropertyWritable(device_, OB_PROP_DISP_SEARCH_OFFSET_INT)) {
    set_disparity_search_offset_srv_ = node_->create_service<SetInt32>(
        "set_disparity_search_offset", [this](const std::shared_ptr<SetInt32::Request> request,
                                              std::shared_ptr<SetInt32::Response> response) {
          setDisparitySearchOffsetCallback(request, response);
        });
  }
  if (isPropertyWritable(device_, OB_PROP_USB_SYNC_VOLTAGE_LEVEL_INT)) {
    set_sync_io_voltage_level_srv_ = node_->create_service<SetInt32>(
        "set_sync_io_voltage_level", [this](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          setSyncIoVoltageLevelCallback(request, response);
        });
  }
}

void OBCameraNode::getPointCloudDecimationCallback(
    const std::shared_ptr<GetInt32::Request>& request,
    std::shared_ptr<GetInt32::Response>& response) {
  (void)request;
  try {
    response->data = point_cloud_decimation_filter_factor_;
    response->success = true;
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::setPointCloudDecimationCallback(
    const std::shared_ptr<SetInt32::Request>& request,
    std::shared_ptr<SetInt32::Response>& response) {
  if (!request) {
    response->success = false;
    response->message = "Invalid request";
    return;
  }

  if (request->data <= 0 || request->data > 8) {
    response->success = false;
    response->message = "Decimation factor must be between 1 and 8";
    RCLCPP_WARN_STREAM(logger_, "Invalid decimation factor: " << request->data);
    return;
  }

  try {
    point_cloud_decimation_filter_factor_ = request->data;
    RCLCPP_INFO_STREAM(logger_, "Set point_cloud_decimation_filter_factor to "
                                    << point_cloud_decimation_filter_factor_);
    response->success = true;
    response->message = "Point cloud decimation factor updated successfully";
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Failed to set decimation factor: ") + e.what();
    RCLCPP_ERROR_STREAM(logger_, response->message);
  }
}

void OBCameraNode::setDisparityRangeModeCallback(const std::shared_ptr<SetInt32::Request>& request,
                                                 std::shared_ptr<SetInt32::Response>& response) {
  if (!request) {
    response->success = false;
    response->message = "Invalid request";
    return;
  }

  try {
    if (!device_->isPropertySupported(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, OB_PERMISSION_WRITE)) {
      response->success = false;
      response->message = "Current device does not support disparity range mode";
      return;
    }

    const bool allow_set = isGemini435LePID(pid_) || enable_stream_[DEPTH];
    if (!allow_set) {
      response->success = false;
      response->message = "Disparity range mode can only be set when depth stream is enabled";
      return;
    }

    if (isGemini330SeriesForDisparity(pid_) &&
        !isSupportedDisparityResolutionForPid(pid_, width_[DEPTH], height_[DEPTH])) {
      response->success = false;
      response->message = "Current depth resolution " + std::to_string(width_[DEPTH]) + "x" +
                          std::to_string(height_[DEPTH]) + " is not supported. " +
                          getDisparityResolutionHintByPid(pid_);
      return;
    }

    auto range = device_->getIntPropertyRange(OB_PROP_DISP_SEARCH_RANGE_MODE_INT);
    const int requested_mode_value = request->data;
    int hw_mode_index = -1;
    if (requested_mode_value == 64) {
      hw_mode_index = 0;
    } else if (requested_mode_value == 128) {
      hw_mode_index = 1;
    } else if (requested_mode_value == 256) {
      hw_mode_index = 2;
    }

    if (hw_mode_index < range.min || hw_mode_index > range.max) {
      response->success = false;
      std::string supported_mode;
      for (int i = range.min; i <= range.max; ++i) {
        supported_mode += (i == 0)   ? "64"
                          : (i == 1) ? "/128"
                          : (i == 2) ? "/256"
                                     : "/" + std::to_string(i);
      }
      response->message = "Invalid disparity range mode. Allowed values:" + supported_mode;
      return;
    }

    device_->setIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, hw_mode_index);
    auto current_mode_index = device_->getIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT);
    auto current_mode_value = (current_mode_index == 0)   ? 64
                              : (current_mode_index == 1) ? 128
                              : (current_mode_index == 2) ? 256
                                                          : current_mode_index;
    disparity_range_mode_ = current_mode_value;
    response->success = true;
    response->message = "disparity_range_mode updated to " + std::to_string(current_mode_value);
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::setDisparitySearchOffsetCallback(
    const std::shared_ptr<SetInt32::Request>& request,
    std::shared_ptr<SetInt32::Response>& response) {
  if (!request) {
    response->success = false;
    response->message = "Invalid request";
    return;
  }

  try {
    if (!device_->isPropertySupported(OB_PROP_DISP_SEARCH_OFFSET_INT, OB_PERMISSION_WRITE)) {
      response->success = false;
      response->message = "Current device does not support disparity search offset";
      return;
    }

    const bool allow_set = isGemini435LePID(pid_) || enable_stream_[DEPTH];
    if (!allow_set) {
      response->success = false;
      response->message = "Disparity search offset can only be set when depth stream is enabled";
      return;
    }

    if (isGemini330SeriesForDisparity(pid_) &&
        !isSupportedDisparityResolutionForPid(pid_, width_[DEPTH], height_[DEPTH])) {
      response->success = false;
      response->message = "Current depth resolution " + std::to_string(width_[DEPTH]) + "x" +
                          std::to_string(height_[DEPTH]) + " is not supported. " +
                          getDisparityResolutionHintByPid(pid_);
      return;
    }

    auto range = device_->getIntPropertyRange(OB_PROP_DISP_SEARCH_OFFSET_INT);
    if (request->data < range.min || request->data > range.max) {
      response->success = false;
      response->message =
          "Invalid disparity search offset. Allowed values:" + std::to_string(range.min) + " to " +
          std::to_string(range.max);
      return;
    }

    device_->setIntProperty(OB_PROP_DISP_SEARCH_OFFSET_INT, request->data);
    auto current_offset = device_->getIntProperty(OB_PROP_DISP_SEARCH_OFFSET_INT);
    disparity_search_offset_ = current_offset;
    RCLCPP_INFO_STREAM(logger_, "Set disparity_search_offset to " << current_offset);
    response->success = true;
    response->message = "disparity_search_offset updated to " + std::to_string(current_offset);
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::setSyncIoVoltageLevelCallback(const std::shared_ptr<SetInt32::Request>& request,
                                                 std::shared_ptr<SetInt32::Response>& response) {
  if (!request) {
    response->success = false;
    response->message = "Invalid request";
    return;
  }

  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    if (!device_->isPropertySupported(OB_PROP_USB_SYNC_VOLTAGE_LEVEL_INT,
                                      OB_PERMISSION_READ_WRITE)) {
      response->success = false;
      response->message = "Current device does not support sync IO voltage level";
      return;
    }

    auto range = device_->getIntPropertyRange(OB_PROP_USB_SYNC_VOLTAGE_LEVEL_INT);
    if (request->data < range.min || request->data > range.max) {
      response->success = false;
      response->message =
          "Invalid sync IO voltage level. Allowed values:" + std::to_string(range.min) + " to " +
          std::to_string(range.max);
      return;
    }

    device_->setIntProperty(OB_PROP_USB_SYNC_VOLTAGE_LEVEL_INT, request->data);
    sync_io_voltage_level_ = device_->getIntProperty(OB_PROP_USB_SYNC_VOLTAGE_LEVEL_INT);
    response->success = true;
    response->message =
        "sync_io_voltage_level updated to " + std::to_string(sync_io_voltage_level_);
    RCLCPP_INFO_STREAM(logger_, response->message);
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::setStreamsEnableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  try {
    if (request->data) {
      startStreams();
      response->success = true;
      response->message = "streams started";
    } else {
      stopStreams();
      response->success = true;
      response->message = "streams stopped";
    }
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::setImageRegistrationModeCallback(
    const std::shared_ptr<SetString::Request> request,
    std::shared_ptr<SetString::Response> response) {
  auto mode = request->data;
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
  if (mode != "OFF" && mode != "HW_D2C" && mode != "SW_D2C" && mode != "SW_C2D") {
    response->success = false;
    response->message = "Invalid image registration mode '" + request->data +
                        "'. Valid values: OFF, HW_D2C, SW_D2C, SW_C2D";
    return;
  }

  std::lock_guard<decltype(device_lock_)> lock(device_lock_);

  if (mode != "OFF" && (!enable_stream_[COLOR] || !enable_stream_[DEPTH])) {
    response->success = false;
    response->message =
        "Image registration mode " + mode + " requires both color and depth streams to be enabled";
    return;
  }

  const bool old_depth_registration = depth_registration_;
  const std::string old_align_mode = align_mode_;
  const OBStreamType old_align_target_stream = align_target_stream_;
  const bool was_running = pipeline_started_.load();

  auto mode_from_state = [](bool depth_registration, const std::string& align_mode,
                            OBStreamType align_target_stream) {
    if (!depth_registration) {
      return std::string("OFF");
    }
    if (align_mode == "HW") {
      return std::string("HW_D2C");
    }
    return align_target_stream == OB_STREAM_DEPTH ? std::string("SW_C2D") : std::string("SW_D2C");
  };
  const auto old_mode =
      mode_from_state(old_depth_registration, old_align_mode, old_align_target_stream);

  auto apply_image_registration_mode = [this](const std::string& mode) {
    if (mode == "OFF") {
      depth_registration_ = false;
      align_mode_ = "HW";
      align_target_stream_ = OB_STREAM_COLOR;
    } else if (mode == "HW_D2C") {
      depth_registration_ = true;
      align_mode_ = "HW";
      align_target_stream_ = OB_STREAM_COLOR;
    } else {
      depth_registration_ = true;
      align_mode_ = "SW";
      align_target_stream_ = mode == "SW_C2D" ? OB_STREAM_DEPTH : OB_STREAM_COLOR;
    }
    align_filter_.reset();
    syncSoftwareAlignment();
  };

  auto restore_old_mode = [this, old_depth_registration, old_align_mode,
                           old_align_target_stream]() {
    depth_registration_ = old_depth_registration;
    align_mode_ = old_align_mode;
    align_target_stream_ = old_align_target_stream;
    align_filter_.reset();
    syncSoftwareAlignment();
  };

  auto rollback_after_error = [&](const std::string& error_message) {
    try {
      restore_old_mode();
      if (was_running && !pipeline_started_.load()) {
        startStreams();
      }
      response->message = "Failed to set image registration mode to " + mode + ": " +
                          error_message + ". Rolled back to " + old_mode;
    } catch (const std::exception& rollback_error) {
      response->message = "Failed to set image registration mode to " + mode + ": " +
                          error_message + ". Rollback to " + old_mode +
                          " also failed: " + rollback_error.what();
    } catch (...) {
      response->message = "Failed to set image registration mode to " + mode + ": " +
                          error_message + ". Rollback to " + old_mode + " also failed";
    }
    response->success = false;
  };

  try {
    if (was_running) {
      stopStreams();
    }

    apply_image_registration_mode(mode);

    if (was_running) {
      startStreams();
      response->message = "Image registration mode changed from " + old_mode + " to " + mode +
                          "; streams restarted";
    } else {
      response->message = "Image registration mode set to " + mode + "; streams remain stopped";
    }
    response->success = true;
  } catch (const ob::Error& e) {
    rollback_after_error(orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    rollback_after_error(e.what());
  } catch (...) {
    rollback_after_error("unknown error");
  }
}

void OBCameraNode::setStreamProfileCallback(
    const std::shared_ptr<SetStreamProfile::Request>& request,
    std::shared_ptr<SetStreamProfile::Response>& response) {
  try {
    std::vector<PendingStreamProfile> pending_profiles;
    std::string message;
    if (!validateStreamProfileRequest(request, pending_profiles, message)) {
      response->success = false;
      response->message = message;
      return;
    }
    if (!applyStreamProfiles(pending_profiles, message)) {
      response->success = false;
      response->message = message;
      return;
    }
    response->success = true;
    response->message = message;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::getStreamsEnableCallback(
    const std::shared_ptr<orbbec_camera_msgs::srv::GetBool::Request> request,
    std::shared_ptr<orbbec_camera_msgs::srv::GetBool::Response> response) {
  (void)request;
  try {
    response->data = pipeline_started_.load();
    response->success = true;
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
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
        ir_exposure_ = device_->getIntProperty(OB_PROP_IR_EXPOSURE_INT);
        break;
      case OB_STREAM_DEPTH:
        device_->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, request->data);
        depth_exposure_ = device_->getIntProperty(OB_PROP_DEPTH_EXPOSURE_INT);
        break;
      case OB_STREAM_COLOR:
      case OB_STREAM_COLOR_LEFT:
      case OB_STREAM_COLOR_RIGHT:
        device_->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, request->data);
        color_exposure_ = device_->getIntProperty(OB_PROP_COLOR_EXPOSURE_INT);
        break;
      default:
        RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
      case OB_STREAM_COLOR_LEFT:
      case OB_STREAM_COLOR_RIGHT:
        response->data = device_->getIntProperty(OB_PROP_COLOR_GAIN_INT);
        break;
      default:
        RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
      case OB_STREAM_COLOR_LEFT:
      case OB_STREAM_COLOR_RIGHT:
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
      RCLCPP_WARN_STREAM(logger_, "Gain value is out of range");
      response->message = "value out of range";
      return;
    }
    device_->setIntProperty(prop_id, request->data);
    const auto current_gain = device_->getIntProperty(prop_id);
    if (stream == OB_STREAM_IR_LEFT || stream == OB_STREAM_IR_RIGHT || stream == OB_STREAM_IR) {
      ir_gain_ = current_gain;
    } else if (stream == OB_STREAM_DEPTH) {
      depth_gain_ = current_gain;
    } else if (stream == OB_STREAM_COLOR || stream == OB_STREAM_COLOR_LEFT ||
               stream == OB_STREAM_COLOR_RIGHT) {
      color_gain_ = current_gain;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
  if (isGemini305SeriesPID(device_->getDeviceInfo()->getPid()) &&
      (stream != OB_STREAM_COLOR && ae_reference_stream_ == "color")) {
    response->success = false;
    response->message = "AE Reference Stream is color, other sensors setting is not supported";
    return;
  }
  if (isGemini305SeriesPID(device_->getDeviceInfo()->getPid()) &&
      (stream != OB_STREAM_DEPTH && ae_reference_stream_ == "depth")) {
    response->success = false;
    response->message =
        "AE Reference Stream is depth, other sensors sensor setting is not supported";
    return;
  }
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
        RCLCPP_INFO_STREAM(
            logger_, "Set depth AE ROI to "
                         << "[Left: " << config.x0_left << ", Right: " << config.x1_right
                         << ", Top: " << config.y0_top << ", Bottom: " << config.y1_bottom << "]");
        depth_ae_roi_left_ = config.x0_left;
        depth_ae_roi_top_ = config.y0_top;
        depth_ae_roi_right_ = config.x1_right;
        depth_ae_roi_bottom_ = config.y1_bottom;
        break;
      case OB_STREAM_COLOR:
      case OB_STREAM_COLOR_LEFT:
      case OB_STREAM_COLOR_RIGHT:
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
        RCLCPP_INFO_STREAM(
            logger_, "Set color AE ROI to "
                         << "[Left: " << config.x0_left << ", Right: " << config.x1_right
                         << ", Top: " << config.y0_top << ", Bottom: " << config.y1_bottom << "]");
        color_ae_roi_left_ = config.x0_left;
        color_ae_roi_top_ = config.y0_top;
        color_ae_roi_right_ = config.x1_right;
        color_ae_roi_bottom_ = config.y1_bottom;
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
      RCLCPP_WARN_STREAM(logger_, "White balance value is out of range");
      response->message = "value out of range";
      return;
    }
    bool auto_white_balance = device_->getBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
    if (auto_white_balance) {
      RCLCPP_WARN(logger_, "Auto white balance is enabled, set white balance will be ignored");
      response->success = false;
      response->message = "auto white balance is enabled";
      return;
    }
    device_->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, request->data);
    color_white_balance_ = device_->getIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT);
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    enable_color_auto_white_balance_ =
        device_->getBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
      case OB_STREAM_COLOR_LEFT:
      case OB_STREAM_COLOR_RIGHT:
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
      RCLCPP_WARN_STREAM(logger_, "Auto exposure value is out of range");
      response->message = "value out of range";
      return;
    }
    device_->setIntProperty(prop_id, request->data);
    const bool current_auto_exposure = device_->getBoolProperty(prop_id);
    if (stream == OB_STREAM_IR_LEFT || stream == OB_STREAM_IR_RIGHT || stream == OB_STREAM_IR) {
      enable_ir_auto_exposure_ = current_auto_exposure;
    } else if (stream == OB_STREAM_DEPTH) {
      enable_ir_auto_exposure_ = current_auto_exposure;
    } else if (stream == OB_STREAM_COLOR || stream == OB_STREAM_COLOR_LEFT ||
               stream == OB_STREAM_COLOR_RIGHT) {
      enable_color_auto_exposure_ = current_auto_exposure;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    bool property_modified = false;
    if (isPropertyWritable(device_, OB_PROP_LASER_CONTROL_INT)) {
      device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, laser_enable);
      property_modified = true;
    } else if (isPropertyWritable(device_, OB_PROP_LASER_BOOL)) {
      device_->setIntProperty(OB_PROP_LASER_BOOL, laser_enable);
      property_modified = true;
    }
    if (property_modified) {
      enable_laser_ = request->data;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    bool property_modified = false;
    if (!isPropertyWritable(device_, OB_PROP_LDP_BOOL)) {
      response->success = false;
      response->message = "LDP property is not supported";
      return;
    }
    if (isPropertyReadable(device_, OB_PROP_LASER_CONTROL_INT) &&
        isPropertyWritable(device_, OB_PROP_LASER_CONTROL_INT)) {
      auto laser_enable = device_->getIntProperty(OB_PROP_LASER_CONTROL_INT);
      device_->setBoolProperty(OB_PROP_LDP_BOOL, ldp_enable);
      device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, laser_enable);
      property_modified = true;
    } else if (isPropertyReadable(device_, OB_PROP_LASER_BOOL) &&
               isPropertyWritable(device_, OB_PROP_LASER_BOOL)) {
      if (!ldp_enable) {
        auto laser_enable = device_->getIntProperty(OB_PROP_LASER_BOOL);
        device_->setBoolProperty(OB_PROP_LDP_BOOL, ldp_enable);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        device_->setIntProperty(OB_PROP_LASER_BOOL, laser_enable);
      } else {
        device_->setBoolProperty(OB_PROP_LDP_BOOL, ldp_enable);
      }
      property_modified = true;
    }
    if (property_modified) {
      enable_ldp_ = ldp_enable;
    } else {
      response->success = false;
      response->message = "Laser property is not supported";
      return;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
      case OB_STREAM_COLOR_LEFT:
      case OB_STREAM_COLOR_RIGHT:
        response->data = device_->getIntProperty(OB_PROP_COLOR_EXPOSURE_INT);
        break;
      default:
        RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    response->info.current_sdk_version = getObSDKVersion();
    response->info.hardware_version = device_info->getHardwareVersion();
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::getDeviceConfigCallback(const std::shared_ptr<GetDeviceConfig::Request>& request,
                                           std::shared_ptr<GetDeviceConfig::Response>& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  response->schema_version = "1";
  response->color_preset = color_preset_;
  response->align_mode = align_mode_;
  response->align_target_stream = alignTargetStreamToString(align_target_stream_);
  response->time_domain = time_domain_;
  response->frame_aggregate_mode = frame_aggregate_mode_;
  response->disparity_to_depth_mode = disparity_to_depth_mode_;
  response->sync_mode = OBSyncModeToString(sync_mode_);
  response->depth_precision = depth_precision_str_;
  response->enable_frame_sync = enable_frame_sync_;
  response->depth_registration = depth_registration_;
  response->exposure_range_mode = exposure_range_mode_;
  response->intra_camera_sync_reference = intra_camera_sync_reference_;
  response->data_json = "";

  auto can_read = [this](OBPropertyID property_id) {
    return device_->isPropertySupported(property_id, OB_PERMISSION_READ) ||
           device_->isPropertySupported(property_id, OB_PERMISSION_READ_WRITE);
  };

  try {
    if (can_read(OB_PROP_DISPARITY_TO_DEPTH_BOOL) &&
        can_read(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL)) {
      response->disparity_to_depth_mode = disparityToDepthModeToString(
          device_->getBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL),
          device_->getBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL));
    }
  } catch (const ob::Error& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get disparity to depth mode: "
                                     << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get disparity to depth mode: " << e.what());
  } catch (...) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get disparity to depth mode");
  }

  try {
    if (can_read(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL)) {
      const bool hardware_align_enabled =
          device_->getBoolProperty(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL);
      if (hardware_align_enabled) {
        response->align_mode = "HW";
        response->align_target_stream = "COLOR";
        response->depth_registration = true;
      } else if (align_mode_ == "HW") {
        response->depth_registration = false;
      }
    }
  } catch (const ob::Error& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get hardware depth alignment status: "
                                     << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get hardware depth alignment status: " << e.what());
  } catch (...) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get hardware depth alignment status");
  }

  try {
    response->sync_mode = OBSyncModeToString(device_->getMultiDeviceSyncConfig().syncMode);
  } catch (const ob::Error& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get multi-device sync mode: "
                                     << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get multi-device sync mode: " << e.what());
  } catch (...) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get multi-device sync mode");
  }

  try {
    if (can_read(OB_PROP_DEPTH_PRECISION_LEVEL_INT)) {
      response->depth_precision =
          depthPrecisionLevelToString(device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT));
    } else if (can_read(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT)) {
      response->depth_precision =
          std::to_string(device_->getFloatProperty(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT)) +
          "mm";
    }
  } catch (const ob::Error& e) {
    RCLCPP_DEBUG_STREAM(
        logger_, "Failed to get depth precision: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get depth precision: " << e.what());
  } catch (...) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get depth precision");
  }

  try {
    if (can_read(OB_PROP_DEVICE_PERFORMANCE_MODE_INT)) {
      response->exposure_range_mode =
          exposureRangeModeToString(device_->getIntProperty(OB_PROP_DEVICE_PERFORMANCE_MODE_INT));
    }
  } catch (const ob::Error& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get exposure range mode: "
                                     << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get exposure range mode: " << e.what());
  } catch (...) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get exposure range mode");
  }

  try {
    if (can_read(OB_PROP_INTRA_CAMERA_SYNC_REFERENCE_INT)) {
      response->intra_camera_sync_reference = intraCameraSyncReferenceToString(
          device_->getIntProperty(OB_PROP_INTRA_CAMERA_SYNC_REFERENCE_INT));
    }
  } catch (const ob::Error& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get intra-camera sync reference: "
                                     << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get intra-camera sync reference: " << e.what());
  } catch (...) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get intra-camera sync reference");
  }

  try {
    response->device_preset = device_->getCurrentPresetName();
    if (response->device_preset == "Custom") {
      try {
        const char* current_depth_mode_name = device_->getCurrentDepthModeName();
        if (current_depth_mode_name != nullptr) {
          std::string current_depth_mode(current_depth_mode_name);
          if (!current_depth_mode.empty()) {
            response->device_preset = "Custom(from " + current_depth_mode + ")";
          }
        }
      } catch (const ob::Error& e) {
        RCLCPP_DEBUG_STREAM(logger_, "Failed to get current depth mode name: "
                                         << orbbec_camera::formatObErrorWithStatus(e));
      } catch (const std::exception& e) {
        RCLCPP_DEBUG_STREAM(logger_, "Failed to get current depth mode name: " << e.what());
      } catch (...) {
        RCLCPP_DEBUG_STREAM(logger_, "Failed to get current depth mode name");
      }
    }
  } catch (const ob::Error& e) {
    RCLCPP_DEBUG_STREAM(
        logger_, "Failed to get current preset: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get current preset: " << e.what());
  } catch (...) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get current preset");
  }

  try {
    if (device_->isColorPresetSupported()) {
      const char* color_preset_name = device_->getCurrentColorPresetName();
      if (color_preset_name != nullptr && color_preset_name[0] != '\0') {
        response->color_preset = color_preset_name;
      }
    }
  } catch (const ob::Error& e) {
    RCLCPP_DEBUG_STREAM(
        logger_, "Failed to get color preset: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get color preset: " << e.what());
  } catch (...) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get color preset");
  }

  try {
    response->preset_version = device_->getExtensionInfo("PresetVer");
  } catch (const ob::Error& e) {
    RCLCPP_DEBUG_STREAM(
        logger_, "Failed to get preset version: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get preset version: " << e.what());
  } catch (...) {
    RCLCPP_DEBUG_STREAM(logger_, "Failed to get preset version");
  }

  response->success = true;
  response->message = "OK";
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
        mirror_stream_[stream_index] = device_->getBoolProperty(OB_PROP_IR_RIGHT_MIRROR_BOOL);
        break;
      case OB_STREAM_IR_LEFT:
      case OB_STREAM_IR:
        device_->setBoolProperty(OB_PROP_IR_MIRROR_BOOL, request->data);
        mirror_stream_[stream_index] = device_->getBoolProperty(OB_PROP_IR_MIRROR_BOOL);
        break;
      case OB_STREAM_DEPTH:
        device_->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, request->data);
        mirror_stream_[stream_index] = device_->getBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL);
        break;
      case OB_STREAM_COLOR:
        device_->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, request->data);
        mirror_stream_[stream_index] = device_->getBoolProperty(OB_PROP_COLOR_MIRROR_BOOL);
        break;
      case OB_STREAM_COLOR_LEFT:
        device_->setBoolProperty(OB_PROP_COLOR_LEFT_MIRROR_BOOL, request->data);
        mirror_stream_[stream_index] = device_->getBoolProperty(OB_PROP_COLOR_LEFT_MIRROR_BOOL);
        break;
      case OB_STREAM_COLOR_RIGHT:
        device_->setBoolProperty(OB_PROP_COLOR_RIGHT_MIRROR_BOOL, request->data);
        mirror_stream_[stream_index] = device_->getBoolProperty(OB_PROP_COLOR_RIGHT_MIRROR_BOOL);
        break;
      default:
        RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
        flip_stream_[stream_index] = device_->getBoolProperty(OB_PROP_IR_RIGHT_FLIP_BOOL);
        break;
      case OB_STREAM_IR_LEFT:
        device_->setBoolProperty(OB_PROP_IR_FLIP_BOOL, request->data);
        flip_stream_[stream_index] = device_->getBoolProperty(OB_PROP_IR_FLIP_BOOL);
        break;
      case OB_STREAM_IR:
        device_->setBoolProperty(OB_PROP_IR_FLIP_BOOL, request->data);
        flip_stream_[stream_index] = device_->getBoolProperty(OB_PROP_IR_FLIP_BOOL);
        break;
      case OB_STREAM_DEPTH:
        device_->setBoolProperty(OB_PROP_DEPTH_FLIP_BOOL, request->data);
        flip_stream_[stream_index] = device_->getBoolProperty(OB_PROP_DEPTH_FLIP_BOOL);
        break;
      case OB_STREAM_COLOR:
        device_->setBoolProperty(OB_PROP_COLOR_FLIP_BOOL, request->data);
        flip_stream_[stream_index] = device_->getBoolProperty(OB_PROP_COLOR_FLIP_BOOL);
        break;
      case OB_STREAM_COLOR_LEFT:
        device_->setBoolProperty(OB_PROP_COLOR_LEFT_FLIP_BOOL, request->data);
        flip_stream_[stream_index] = device_->getBoolProperty(OB_PROP_COLOR_LEFT_FLIP_BOOL);
        break;
      case OB_STREAM_COLOR_RIGHT:
        device_->setBoolProperty(OB_PROP_COLOR_RIGHT_FLIP_BOOL, request->data);
        flip_stream_[stream_index] = device_->getBoolProperty(OB_PROP_COLOR_RIGHT_FLIP_BOOL);
        break;
      default:
        RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
        rotation_stream_[stream_index] = device_->getIntProperty(OB_PROP_IR_RIGHT_ROTATE_INT);
        break;
      case OB_STREAM_IR_LEFT:
        device_->setIntProperty(OB_PROP_IR_ROTATE_INT, request->data);
        rotation_stream_[stream_index] = device_->getIntProperty(OB_PROP_IR_ROTATE_INT);
        break;
      case OB_STREAM_IR:
        device_->setIntProperty(OB_PROP_IR_ROTATE_INT, request->data);
        rotation_stream_[stream_index] = device_->getIntProperty(OB_PROP_IR_ROTATE_INT);
        break;
      case OB_STREAM_DEPTH:
        device_->setIntProperty(OB_PROP_DEPTH_ROTATE_INT, request->data);
        rotation_stream_[stream_index] = device_->getIntProperty(OB_PROP_DEPTH_ROTATE_INT);
        break;
      case OB_STREAM_COLOR:
        device_->setIntProperty(OB_PROP_COLOR_ROTATE_INT, request->data);
        rotation_stream_[stream_index] = device_->getIntProperty(OB_PROP_COLOR_ROTATE_INT);
        break;
      case OB_STREAM_COLOR_LEFT:
        device_->setIntProperty(OB_PROP_COLOR_LEFT_ROTATE_INT, request->data);
        rotation_stream_[stream_index] = device_->getIntProperty(OB_PROP_COLOR_LEFT_ROTATE_INT);
        break;
      case OB_STREAM_COLOR_RIGHT:
        device_->setIntProperty(OB_PROP_COLOR_RIGHT_ROTATE_INT, request->data);
        rotation_stream_[stream_index] = device_->getIntProperty(OB_PROP_COLOR_RIGHT_ROTATE_INT);
        break;
      default:
        RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
        break;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    if (!device_->isPropertySupported(OB_PROP_LDP_BOOL, OB_PERMISSION_READ) ||
        !device_->isPropertySupported(OB_PROP_LDP_STATUS_BOOL, OB_PERMISSION_READ)) {
      response->message = "LDP property is not supported";
      response->success = false;
      return;
    }
    response->data = device_->getBoolProperty(OB_PROP_LDP_BOOL) &&
                     device_->getBoolProperty(OB_PROP_LDP_STATUS_BOOL);
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::getLaserStatusCallback(const std::shared_ptr<GetBool::Request>& request,
                                          std::shared_ptr<GetBool::Response>& response) {
  (void)request;
  try {
    if (isPropertyReadable(device_, OB_PROP_LASER_CONTROL_INT)) {
      response->data = device_->getBoolProperty(OB_PROP_LASER_CONTROL_INT);
    } else if (isPropertyReadable(device_, OB_PROP_LASER_BOOL)) {
      response->data = device_->getBoolProperty(OB_PROP_LASER_BOOL);
    } else {
      response->success = false;
      response->message = "Laser property is not supported";
      return;
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::setPtpConfigCallback(
    const std::shared_ptr<rmw_request_id_t>& request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  (void)request_header;

  try {
    if (!isPropertyReadable(device_, OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL) ||
        !isPropertyWritable(device_, OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL)) {
      response->success = false;
      RCLCPP_ERROR(logger_, "PTP clock sync property is not supported or not writable");
      return;
    }
    device_->setBoolProperty(OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL, request->data);
    enable_ptp_config_ = device_->getBoolProperty(OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL);
    response->success = true;
  } catch (const ob::Error& e) {
    response->success = false;
    response->message = orbbec_camera::formatObErrorWithStatus(e);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  } catch (...) {
    response->success = false;
    response->message = "unknown error";
  }
}

void OBCameraNode::getPtpConfigCallback(const std::shared_ptr<GetBool::Request>& request,
                                        std::shared_ptr<GetBool::Response>& response) {
  (void)request;
  try {
    response->data = device_->getBoolProperty(OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL);
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
      msg = stream_name_[stream_index] + " is already enabled";
    }
    RCLCPP_INFO_STREAM(logger_, "Request to set sensor " << stream_name_[stream_index] << " to ON");

  } else {
    if (!enable_stream_[stream_index]) {
      msg = stream_name_[stream_index] + " is already disabled";
    }
    RCLCPP_INFO_STREAM(logger_,
                       "Request to set sensor " << stream_name_[stream_index] << " to OFF");
  }
  if (!msg.empty()) {
    RCLCPP_WARN_STREAM(logger_, msg);
    response->success = true;
    response->message = msg;
    return;
  }
  response->success = toggleSensor(stream_index, request->data, response->message);
}

bool OBCameraNode::toggleSensor(const stream_index_pair& stream_index, bool enabled,
                                std::string& msg) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    const bool interleave_frame_enable = interleave_frame_enable_;
    stopStreams();
    interleave_frame_enable_ = interleave_frame_enable;
    stopColorFrameThreads();
    clearColorFrameQueues();
    enable_stream_[stream_index] = enabled;
    setupProfiles();
    startStreams();
    return true;
  } catch (const ob::Error& e) {
    msg = orbbec_camera::formatObErrorWithStatus(e);
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::exportConfigJsonCallback(const std::shared_ptr<SetString::Request>& request,
                                            std::shared_ptr<SetString::Response>& response) {
  response->success = exportConfigJsonToFile(request->data, response->message);
}

void OBCameraNode::setIRLongExposureCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  try {
    device_->setBoolProperty(OB_PROP_IR_LONG_EXPOSURE_BOOL, request->data);
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
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
    response->message = orbbec_camera::formatObErrorWithStatus(e);
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

void OBCameraNode::sendSoftwareTriggerCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
  try {
    if (request->data) {
      device_->triggerCapture();
    }
    response->success = true;
  } catch (const ob::Error& e) {
    response->message = orbbec_camera::formatObErrorWithStatus(e);
    response->success = false;
  } catch (const std::exception& e) {
    response->message = e.what();
    response->success = false;
  } catch (...) {
    response->message = "unknown error";
    response->success = false;
  }
}

bool OBCameraNode::writeCustomerData(const std::string& data) {
  if (data.empty()) return false;

  std::string md5_value = calcMD5(data);
  uint32_t data_len_net = htonl(static_cast<uint32_t>(data.size()));
  std::string len_bytes(reinterpret_cast<char*>(&data_len_net), sizeof(data_len_net));

  std::string write_buffer = len_bytes + md5_value + data;
  device_->writeCustomerData(write_buffer.c_str(), write_buffer.size());

  std::vector<uint8_t> read_buffer(write_buffer.size() + 8);
  uint32_t read_len = 0;
  device_->readCustomerData(read_buffer.data(), &read_len);

  if (read_len < sizeof(uint32_t) + 32) return false;

  uint32_t read_data_len_net = 0;
  memcpy(&read_data_len_net, read_buffer.data(), sizeof(uint32_t));
  uint32_t read_data_len = ntohl(read_data_len_net);

  std::string md5_read(reinterpret_cast<char*>(read_buffer.data() + sizeof(uint32_t)), 32);
  std::string data_read(reinterpret_cast<char*>(read_buffer.data() + sizeof(uint32_t) + 32),
                        read_data_len);

  return calcMD5(data_read) == md5_read;
}

bool OBCameraNode::readCustomerData(std::string& out_data) {
  std::vector<uint8_t> read_buffer(40960);
  uint32_t read_len = 0;
  device_->readCustomerData(read_buffer.data(), &read_len);
  if (read_len < sizeof(uint32_t) + 32) {
    return false;
  }
  uint32_t read_data_len_net = 0;
  memcpy(&read_data_len_net, read_buffer.data(), sizeof(uint32_t));
  uint32_t read_data_len = ntohl(read_data_len_net);
  std::string md5_read(reinterpret_cast<char*>(read_buffer.data() + sizeof(uint32_t)), 32);
  std::string data_read(reinterpret_cast<char*>(read_buffer.data() + sizeof(uint32_t) + 32),
                        read_data_len);
  if (calcMD5(data_read) == md5_read) {
    out_data = std::move(data_read);
    return true;
  } else {
    return false;
  }
}

void OBCameraNode::writeCustomerDataCallback(const std::shared_ptr<SetString::Request>& request,
                                             std::shared_ptr<SetString::Response>& response) {
  if (request->data.empty()) {
    response->success = false;
    response->message = "data empty";
    return;
  }

  try {
    if (writeCustomerData(request->data)) {
      write_customer_data_success_ = true;
      response->success = true;
      response->message = "write and verify success";
      user_calibration_ready_ = true;
    } else {
      write_customer_data_success_ = false;
      response->success = false;
      response->message = "write failed: MD5 mismatch or read too short";
      user_calibration_ready_ = false;
    }
  } catch (...) {
    response->success = false;
    response->message = "write exception";
  }
}

void OBCameraNode::readCustomerDataCallback(const std::shared_ptr<GetString::Request>& request,
                                            std::shared_ptr<GetString::Response>& response) {
  (void)request;
  try {
    std::string data;
    if (readCustomerData(data)) {
      response->success = true;
      response->data = std::move(data);
      response->message = "read success";
    } else {
      response->success = false;
      response->message = "read failed: MD5 mismatch or data too short";
    }
  } catch (...) {
    response->success = false;
    response->message = "read exception";
  }
}
void OBCameraNode::setUserCalibParamsCallback(
    const std::shared_ptr<SetUserCalibParams::Request>& request,
    std::shared_ptr<SetUserCalibParams::Response>& response) {
  try {
    std::ostringstream ss;
    for (const auto& v : request->k) ss << v << " ";
    for (const auto& v : request->d) ss << v << " ";
    for (const auto& v : request->rotation) ss << v << " ";
    for (const auto& v : request->translation) ss << v << " ";
    std::string serialized_data = ss.str();
    if (writeCustomerData(serialized_data)) {
      response->success = true;
      response->message = "write and verify success";
    } else {
      response->success = false;
      response->message = "write failed";
    }
  } catch (...) {
    response->success = false;
    response->message = "exception occurred";
  }
}
void OBCameraNode::getUserCalibParamsCallback(
    const std::shared_ptr<GetUserCalibParams::Request>& request,
    std::shared_ptr<GetUserCalibParams::Response>& response) {
  (void)request;
  try {
    std::string data_read;
    if (!readCustomerData(data_read)) {
      response->success = false;
      response->message = "read failed";
      return;
    }
    std::istringstream ss(data_read);
    for (size_t i = 0; i < 9; ++i) ss >> response->k[i];
    for (size_t i = 0; i < 8; ++i) ss >> response->d[i];
    for (size_t i = 0; i < 9; ++i) ss >> response->rotation[i];
    for (size_t i = 0; i < 3; ++i) ss >> response->translation[i];
    response->success = true;
    response->message = "read success";
  } catch (...) {
    response->success = false;
    response->message = "exception occurred";
  }
}
void OBCameraNode::setAEReferenceStreamCallback(const std::shared_ptr<SetString::Request>& request,
                                                std::shared_ptr<SetString::Response>& response) {
  try {
    if (device_->isPropertySupported(OB_PROP_DEVICE_AE_REFERENCE_INT, OB_PERMISSION_WRITE) &&
        (request->data == "depth" || request->data == "color")) {
      device_->setIntProperty(OB_PROP_DEVICE_AE_REFERENCE_INT, request->data == "depth" ? 0 : 1);
      ae_reference_stream_ = request->data;
      response->success = true;
      response->message = "set AE reference stream success";
    } else {
      response->success = false;
      response->message = "set AE reference stream failed";
    }
  } catch (...) {
    response->success = false;
    response->message = "exception occurred";
  }
}
void OBCameraNode::setAEStrategyCallback(const std::shared_ptr<SetString::Request>& request,
                                         std::shared_ptr<SetString::Response>& response) {
  try {
    if (device_->isPropertySupported(OB_PROP_DEVICE_AE_STRATEGY_INT, OB_PERMISSION_WRITE) &&
        (request->data == "default" || request->data == "motion")) {
      device_->setIntProperty(OB_PROP_DEVICE_AE_STRATEGY_INT, request->data == "motion" ? 1 : 0);
      ae_strategy_ = request->data;
      response->success = true;
      response->message = "set AE strategy success";
    } else {
      response->success = false;
      response->message = "set AE strategy failed";
    }
  } catch (...) {
    response->success = false;
    response->message = "exception occurred";
  }
}
}  // namespace orbbec_camera
