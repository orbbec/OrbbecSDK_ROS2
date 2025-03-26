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

namespace orbbec_camera {
class ParametersBackend {
 public:
  explicit ParametersBackend(rclcpp::Node* node);
  ~ParametersBackend();

  template <typename T>
  void addOnSetParametersCallback(T callback) {
    ros_callback_ = node_->add_on_set_parameters_callback(callback);
  }

 private:
  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  std::shared_ptr<void> ros_callback_;
};
}  // namespace orbbec_camera
