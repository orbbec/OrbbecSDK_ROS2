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

#include "orbbec_camera/action_command_node.h"

#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <sstream>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

namespace orbbec_camera {
namespace {

constexpr uint64_t kNanosecondsPerSecond = 1000000000ULL;
constexpr uint64_t kMillisecondsPerSecond = 1000ULL;
constexpr uint64_t kNanosecondsPerMillisecond = 1000000ULL;

std::string formatObError(const ob::Error& error) {
  std::ostringstream stream;
  stream << (error.getMessage() ? error.getMessage() : "Unknown OB error")
         << " status:" << static_cast<int>(error.getStatus());
  return stream.str();
}

bool getSystemTimeMilliseconds(uint64_t* milliseconds, std::string* error_message) {
  const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  if (now < 0) {
    *error_message = "System clock returned a time before its epoch";
    return false;
  }
  *milliseconds = static_cast<uint64_t>(now);
  return true;
}

}  // namespace

ActionCommandNode::ActionCommandNode(const rclcpp::NodeOptions& node_options)
    : Node("action_command_node", node_options), context_(std::make_unique<ob::Context>()) {
  context_->enableNetDeviceEnumeration(true);
  send_action_command_service_ = create_service<orbbec_camera_msgs::srv::SendActionCommand>(
      "~/send_action_command", std::bind(&ActionCommandNode::sendActionCommandCallback, this,
                                         std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(get_logger(), "Action Command service is ready");
}

void ActionCommandNode::sendActionCommandCallback(
    const std::shared_ptr<orbbec_camera_msgs::srv::SendActionCommand::Request> request,
    std::shared_ptr<orbbec_camera_msgs::srv::SendActionCommand::Response> response) {
  if (!request) {
    response->success = false;
    response->message = "Invalid request";
    return;
  }

  const std::string broadcast_ip =
      request->broadcast_ip.empty() ? "255.255.255.255" : request->broadcast_ip;
  using Request = orbbec_camera_msgs::srv::SendActionCommand::Request;
  uint64_t action_time = 0;
  std::string validation_error;

  switch (request->trigger_mode) {
    case Request::TRIGGER_MODE_IMMEDIATE:
      if (request->delay_ms != 0 || request->scheduled_time != 0) {
        validation_error = "Immediate trigger requires zero delay and zero scheduled time";
      }
      break;

    case Request::TRIGGER_MODE_DELAYED: {
      if (request->delay_ms == 0) {
        validation_error = "Action Command delay must be greater than zero";
        break;
      }
      if (request->scheduled_time != 0) {
        validation_error = "Delayed trigger requires zero scheduled time";
        break;
      }

      uint64_t now_milliseconds = 0;
      if (!getSystemTimeMilliseconds(&now_milliseconds, &validation_error)) {
        break;
      }
      const uint64_t delay_milliseconds = request->delay_ms;
      if (delay_milliseconds > (std::numeric_limits<uint64_t>::max)() - now_milliseconds) {
        validation_error = "Action Command target time overflows the timestamp range";
        break;
      }
      const uint64_t target_milliseconds = now_milliseconds + delay_milliseconds;
      const uint64_t seconds = target_milliseconds / kMillisecondsPerSecond;
      if (seconds > (std::numeric_limits<uint32_t>::max)()) {
        validation_error = "PTP seconds exceed the 32-bit GVCP timestamp range";
        break;
      }
      const uint64_t nanoseconds =
          (target_milliseconds % kMillisecondsPerSecond) * kNanosecondsPerMillisecond;
      action_time = (seconds << 32) | nanoseconds;
      break;
    }

    case Request::TRIGGER_MODE_TIMESTAMP: {
      if (request->delay_ms != 0) {
        validation_error = "PTP timestamp trigger requires zero delay";
        break;
      }
      if (request->scheduled_time == 0) {
        validation_error = "Action Command scheduled time must be greater than zero";
        break;
      }

      const uint64_t nanoseconds = request->scheduled_time & 0xFFFFFFFFULL;
      if (nanoseconds >= kNanosecondsPerSecond) {
        validation_error = "PTP nanoseconds must be less than 1000000000";
        break;
      }

      action_time = request->scheduled_time;
      break;
    }

    default:
      validation_error = "Unsupported Action Command trigger mode";
      break;
  }

  if (!validation_error.empty()) {
    response->success = false;
    response->encoded_scheduled_time = 0;
    response->message = validation_error;
    return;
  }

  response->encoded_scheduled_time = action_time;
  try {
    response->success =
        context_->sendActionCommand(request->device_key, request->group_key, request->group_mask,
                                    broadcast_ip.c_str(), action_time);
    response->message =
        response->success ? "Action Command dispatched" : "SDK failed to send Action Command";
  } catch (const ob::Error& error) {
    response->success = false;
    response->message = formatObError(error);
  } catch (const std::exception& error) {
    response->success = false;
    response->message = error.what();
  } catch (...) {
    response->success = false;
    response->message = "Unknown error";
  }
}

}  // namespace orbbec_camera

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::ActionCommandNode)
