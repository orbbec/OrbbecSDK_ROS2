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

#include "orbbec_camera/ros_param_backend.h"
namespace orbbec_camera {
ParametersBackend::ParametersBackend(rclcpp::Node *node)
    : node_(node), logger_(node_->get_logger()) {}

ParametersBackend::~ParametersBackend() {
  if (ros_callback_) {
    node_->remove_on_set_parameters_callback(
        (rclcpp::node_interfaces::OnSetParametersCallbackHandle *)(ros_callback_.get()));
    ros_callback_.reset();
  }
}

void ParametersBackend::addOnSetParametersCallback(
    rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType callback) {
  ros_callback_ = node_->add_on_set_parameters_callback(callback);
}

}  // namespace orbbec_camera
