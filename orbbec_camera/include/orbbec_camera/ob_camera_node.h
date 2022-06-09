/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2022 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#pragma once

#include <glog/logging.h>
#include <nlohmann/json.hpp>
#include <magic_enum.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_publisher.h>

#include "libobsensor/ObSensor.hpp"

#include "orbbec_camera_msgs/msg/device_info.hpp"
#include "orbbec_camera_msgs/srv/get_device_info.hpp"
#include "orbbec_camera_msgs/msg/extrinsics.hpp"
#include "orbbec_camera_msgs/msg/metadata.hpp"
#include "orbbec_camera_msgs/srv/get_int32.hpp"
#include "orbbec_camera_msgs/srv/get_string.hpp"
#include "orbbec_camera_msgs/srv/set_int32.hpp"

#include "orbbec_camera/constants.h"
#include "orbbec_camera/dynamic_params.h"

#define STREAM_NAME(sip)                                                                       \
  (static_cast<std::ostringstream&&>(std::ostringstream()                                      \
                                     << _stream_name[sip.first]                                \
                                     << ((sip.second > 0) ? std::to_string(sip.second) : ""))) \
      .str()
#define FRAME_ID(sip)                                                                              \
  (static_cast<std::ostringstream&&>(std::ostringstream()                                          \
                                     << getNamespaceStr() << "_" << STREAM_NAME(sip) << "_frame")) \
      .str()
#define OPTICAL_FRAME_ID(sip)                                                                     \
  (static_cast<std::ostringstream&&>(                                                             \
       std::ostringstream() << getNamespaceStr() << "_" << STREAM_NAME(sip) << "_optical_frame")) \
      .str()
#define ALIGNED_DEPTH_TO_FRAME_ID(sip)                                            \
  (static_cast<std::ostringstream&&>(std::ostringstream()                         \
                                     << getNamespaceStr() << "_aligned_depth_to_" \
                                     << STREAM_NAME(sip) << "_frame"))            \
      .str()
#define BASE_FRAME_ID() \
  (static_cast<std::ostringstream&&>(std::ostringstream() << getNamespaceStr() << "_link")).str()
#define ODOM_FRAME_ID()                                                                           \
  (static_cast<std::ostringstream&&>(std::ostringstream() << getNamespaceStr() << "_odom_frame")) \
      .str()

namespace orbbec_camera {
using GetDeviceInfo = orbbec_camera_msgs::srv::GetDeviceInfo;
using Extrinsics = orbbec_camera_msgs::msg::Extrinsics;
using SetInt32 = orbbec_camera_msgs::srv::SetInt32;
using GetInt32 = orbbec_camera_msgs::srv::GetInt32;
using GetString = orbbec_camera_msgs::srv::GetString;

typedef std::pair<ob_stream_type, int> stream_index_pair;

const stream_index_pair COLOR{OB_STREAM_COLOR, 0};
const stream_index_pair DEPTH{OB_STREAM_DEPTH, 0};
const stream_index_pair INFRA0{OB_STREAM_IR, 0};
const stream_index_pair INFRA1{OB_STREAM_IR, 1};
const stream_index_pair INFRA2{OB_STREAM_IR, 2};

const stream_index_pair GYRO{OB_STREAM_GYRO, 0};
const stream_index_pair ACCEL{OB_STREAM_ACCEL, 0};

const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA0, COLOR};

const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL};

class OBCameraNode {
 public:
  OBCameraNode(rclcpp::Node* node, std::shared_ptr<ob::Device> device,
               std::shared_ptr<Parameters> parameters);

  template <class T>
  void setAndGetNodeParameter(
      T& param, const std::string& param_name, const T& default_value,
      const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
          rcl_interfaces::msg::ParameterDescriptor());  // set and get parameter

  ~OBCameraNode();

  void clean();

 private:
  void setupDevices();

  void setupProfiles();

  void getParameters();

  void setupTopics();

  void setupCameraCtrlServices();

  void startPipeline();

  void setupPublishers();

  void updateStreamCalibData();

  void publishStaticTF(const rclcpp::Time& t, const std::vector<float>& trans,
                       const tf2::Quaternion& q, const std::string& from, const std::string& to);

  void calcAndPublishStaticTransform();

  void publishDynamicTransforms();

  void publishStaticTransforms();

  std::optional<OBCameraParam> findDefaultCameraParam();

  std::optional<OBCameraParam> findStreamDefaultCameraParam(const stream_index_pair& stream);

  std::optional<OBCameraParam> findStreamCameraParam(const stream_index_pair& stream,
                                                     uint32_t width, uint32_t height);

  std::optional<OBCameraParam> findCameraParam(uint32_t color_width, uint32_t color_height,
                                               uint32_t depth_width, uint32_t depth_height);

  void getExposureCallback(const std::shared_ptr<GetInt32::Request>& request,
                           std::shared_ptr<GetInt32::Response>& response,
                           const stream_index_pair& stream_index);

  void setExposureCallback(const std::shared_ptr<SetInt32::Request>& request,
                           std::shared_ptr<SetInt32::Response>& response,
                           const stream_index_pair& stream_index);

  void getGainCallback(const std::shared_ptr<GetInt32::Request>& request,
                       std::shared_ptr<GetInt32::Response>& response,
                       const stream_index_pair& stream_index);

  void setGainCallback(const std::shared_ptr<SetInt32::Request>& request,
                       std::shared_ptr<SetInt32::Response>& response,
                       const stream_index_pair& stream_index);

  void getWhiteBalanceCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                               const std::shared_ptr<GetInt32::Request>& request,
                               std::shared_ptr<GetInt32::Response>& response);

  void setWhiteBalanceCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                               const std::shared_ptr<SetInt32 ::Request>& request,
                               std::shared_ptr<SetInt32 ::Response>& response);

  void setAutoExposureCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response>& response,
                               const stream_index_pair& stream_index);

  void setLaserEnableCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                              const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

  void setFloorEnableCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                              const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

  void setLdpEnableCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                            const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

  void setFanModeCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                          const std::shared_ptr<SetInt32::Request>& request,
                          std::shared_ptr<SetInt32::Response>& response);

  void getDeviceInfoCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                             const std::shared_ptr<GetDeviceInfo::Request>& request,
                             std::shared_ptr<GetDeviceInfo::Response>& response);

  void getApiVersion(const std::shared_ptr<rmw_request_id_t>& request_header,
                     const std::shared_ptr<GetString::Request>& request,
                     std::shared_ptr<GetString::Response>& response);

  void publishPointCloud(std::shared_ptr<ob::FrameSet> frame_set);

  void publishDepthPointCloud(std::shared_ptr<ob::FrameSet> frame_set);

  void publishColorPointCloud(std::shared_ptr<ob::FrameSet> frame_set);

  void frameSetCallback(std::shared_ptr<ob::FrameSet> frame_set);

  void publishColorFrame(std::shared_ptr<ob::ColorFrame> frame);

  bool rbgFormatConvertRGB888(std::shared_ptr<ob::ColorFrame> frame);

  void publishDepthFrame(std::shared_ptr<ob::DepthFrame> frame);

  void publishIRFrame(std::shared_ptr<ob::IRFrame> frame);

 private:
  rclcpp::Node* node_;
  std::shared_ptr<ob::Device> device_;
  std::shared_ptr<Parameters> parameters_;
  rclcpp::Logger logger_;
  std::atomic_bool is_running_{false};
  std::unique_ptr<ob::Pipeline> pipeline_;
  std::shared_ptr<ob::Config> config_;
  std::map<stream_index_pair, std::shared_ptr<ob::Sensor>> sensors_;
  std::map<stream_index_pair, ob_camera_intrinsic> stream_intrinsics_;
  std::map<stream_index_pair, sensor_msgs::msg::CameraInfo> camera_infos_;
  std::map<stream_index_pair, OBCameraParam> ob_camera_param_;
  std::map<stream_index_pair, int> width_;
  std::map<stream_index_pair, int> height_;
  std::map<stream_index_pair, double> fps_;
  std::map<stream_index_pair, std::string> frame_id_;
  std::map<stream_index_pair, std::string> optical_frame_id_;
  std::map<stream_index_pair, std::string> depth_aligned_frame_id_;
  std::string base_frame_id_;
  bool align_depth_;
  bool publish_rgb_point_cloud_;
  std::string d2c_mode_;  // sw, hw, none
  std::map<stream_index_pair, std::string> qos_;
  std::map<stream_index_pair, std::string> info_qos_;
  std::map<stream_index_pair, ob_format> format_;
  std::map<ob_stream_type, int> image_format_;
  std::map<stream_index_pair, std::vector<std::shared_ptr<ob::VideoStreamProfile>>>
      enabled_profiles_;
  std::map<stream_index_pair, uint32_t> seq_;
  std::map<stream_index_pair, cv::Mat> images_;
  std::map<stream_index_pair, std::string> encoding_;
  std::map<stream_index_pair, int> unit_step_size_;
  std::vector<int> compression_params_;
  ob::FormatConvertFilter format_convert_filter_;

  std::map<ob_frame_type, bool> is_first_frame_;

  std::map<stream_index_pair, bool> enable_;
  std::map<ob_stream_type, std::string> stream_name_;
  std::map<stream_index_pair, image_transport::Publisher> image_publishers_;
  std::map<stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr>
      camera_info_publishers_;

  std::map<stream_index_pair, rclcpp::Service<GetInt32>::SharedPtr> get_exposure_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetInt32>::SharedPtr> set_exposure_srv_;
  std::map<stream_index_pair, rclcpp::Service<GetInt32>::SharedPtr> get_gain_srv_;
  std::map<stream_index_pair, rclcpp::Service<SetInt32>::SharedPtr> set_gain_srv_;
  rclcpp::Service<GetInt32>::SharedPtr get_white_balance_srv_;  // only rgb
  rclcpp::Service<SetInt32>::SharedPtr set_white_balance_srv_;
  rclcpp::Service<GetString>::SharedPtr get_api_version_srv_;
  std::map<stream_index_pair, rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr>
      set_auto_exposure_srv_;  // only rgb color

  bool publish_tf_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
  ob::PointCloudFilter point_cloud_filter_;
  sensor_msgs::msg::PointCloud2 point_cloud_msg_;

  rclcpp::Publisher<Extrinsics>::SharedPtr extrinsics_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_sensors_srv_;
  orbbec_camera_msgs::msg::DeviceInfo device_info_;
  rclcpp::Service<GetDeviceInfo>::SharedPtr get_device_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_laser_enable_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_ldp_enable_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_floor_enable_srv_;
  rclcpp::Service<SetInt32>::SharedPtr set_fan_mode_srv_;
  std::vector<geometry_msgs::msg::TransformStamped> static_tf_msgs_;
  std::shared_ptr<std::thread> tf_thread_;
  std::condition_variable tf_cv_;
  double tf_publish_rate_;
};
}  // namespace orbbec_camera
