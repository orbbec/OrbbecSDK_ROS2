/*******************************************************************************
 * Copyright (c) 2026 Orbbec 3D Technology, Inc
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

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "libobsensor/ObSensor.hpp"
#include "orbbec_camera_msgs/srv/send_action_command.hpp"

namespace orbbec_camera {

class ActionCommandNode : public rclcpp::Node {
 public:
  explicit ActionCommandNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

 private:
  void sendActionCommandCallback(
      const std::shared_ptr<orbbec_camera_msgs::srv::SendActionCommand::Request> request,
      std::shared_ptr<orbbec_camera_msgs::srv::SendActionCommand::Response> response);

  std::unique_ptr<ob::Context> context_;
  rclcpp::Service<orbbec_camera_msgs::srv::SendActionCommand>::SharedPtr
      send_action_command_service_;
};

}  // namespace orbbec_camera
