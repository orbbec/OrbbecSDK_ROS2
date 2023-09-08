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
