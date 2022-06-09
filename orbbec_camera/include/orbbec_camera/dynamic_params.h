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
#include <rclcpp/rclcpp.hpp>

#include "ros_param_backend.h"

namespace orbbec_camera {
class Parameters {
 public:
  explicit Parameters(rclcpp::Node* node);
  ~Parameters() noexcept;
  rclcpp::ParameterValue setParam(const std::string& param_name,
                                  rclcpp::ParameterValue initial_value,
                                  const std::function<void(const rclcpp::Parameter&)>& func =
                                      std::function<void(const rclcpp::Parameter&)>(),
                                  const rcl_interfaces::msg::ParameterDescriptor& descriptor =
                                      rcl_interfaces::msg::ParameterDescriptor());

  template <class T>
  void setParamT(std::string param_name, rclcpp::ParameterValue initial_value, T& param,
                 std::function<void(const rclcpp::Parameter&)> func =
                     std::function<void(const rclcpp::Parameter&)>(),
                 rcl_interfaces::msg::ParameterDescriptor descriptor =
                     rcl_interfaces::msg::ParameterDescriptor());
  template <class T>
  void setParamValue(T& param, const T& value);  // function updates the parameter value both
                                                 // locally and in the parameters server
  void removeParam(const std::string& param_name);

 private:
 private:
  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  std::map<std::string, std::vector<std::function<void(const rclcpp::Parameter&)> > >
      param_functions_;
  std::map<void*, std::string> param_names_;
  ParametersBackend params_backend_;
};
}  // namespace orbbec_camera
