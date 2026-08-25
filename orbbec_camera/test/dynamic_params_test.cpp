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

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "orbbec_camera/dynamic_params.h"

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("dynamic parameter contract failed");
  }
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dynamic_params_test");

  {
    orbbec_camera::Parameters parameters(node.get());
    int64_t applied = 0;
    parameters.setParam("runtime_value", rclcpp::ParameterValue(int64_t{0}),
                        [&applied](const rclcpp::Parameter &parameter) {
                          const auto requested = parameter.as_int();
                          if (requested < 0) {
                            throw std::runtime_error("negative values are unsupported");
                          }
                          applied = requested;
                        });

    const auto accepted = node->set_parameter(rclcpp::Parameter("runtime_value", 7));
    require(accepted.successful);
    require(applied == 7);
    require(node->get_parameter("runtime_value").as_int() == 7);

    int64_t other_applied = 0;
    parameters.setParam("other_runtime_value", rclcpp::ParameterValue(int64_t{0}),
                        [&other_applied](const rclcpp::Parameter &parameter) {
                          other_applied = parameter.as_int();
                        });
    const auto atomic_rejected = node->set_parameters_atomically(
        {rclcpp::Parameter("runtime_value", 8), rclcpp::Parameter("other_runtime_value", 9)});
    require(!atomic_rejected.successful);
    require(atomic_rejected.reason.find("one at a time") != std::string::npos);
    require(applied == 7);
    require(other_applied == 0);
    require(node->get_parameter("runtime_value").as_int() == 7);
    require(node->get_parameter("other_runtime_value").as_int() == 0);

    const auto rejected = node->set_parameter(rclcpp::Parameter("runtime_value", -1));
    require(!rejected.successful);
    require(rejected.reason.find("negative values are unsupported") != std::string::npos);
    require(applied == 7);
    require(node->get_parameter("runtime_value").as_int() == 7);
  }

  rclcpp::shutdown();
  return 0;
}
