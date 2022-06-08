/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#pragma once
#include <ostream>
#include <glog/logging.h>
#include <magic_enum.hpp>
#include "libobsensor/ObSensor.hpp"
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <orbbec_camera_msgs/msg/extrinsics.hpp>

namespace orbbec_camera {
sensor_msgs::msg::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                                 OBCameraDistortion distortion);

void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName);

void savePointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName);

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]);

std::ostream& operator<<(std::ostream& os, const OBCameraParam& rhs);

orbbec_camera_msgs::msg::Extrinsics obExtrinsicsToMsg(const OBD2CTransform& extrinsics,
                                                      const std::string& frame_id);

rclcpp::Time frameTimeStampToROSTime(uint64_t ms);

}  // namespace orbbec_camera
