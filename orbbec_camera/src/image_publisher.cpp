// Copyright 2023 Intel Corporation. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "orbbec_camera/image_publisher.h"

#include <map>
#include <mutex>
#include <utility>

namespace orbbec_camera {

namespace {
using ImageTransportPublisherCacheKey = std::pair<std::string, std::string>;

struct CachedImageTransportPublisher {
  rmw_qos_profile_t qos;
  std::shared_ptr<image_publisher> publisher;
};

using ImageTransportPublisherCache =
    std::map<ImageTransportPublisherCacheKey, CachedImageTransportPublisher>;

std::mutex& imageTransportPublisherCacheMutex() {
  static std::mutex mutex;
  return mutex;
}

ImageTransportPublisherCache& imageTransportPublisherCache() {
  static ImageTransportPublisherCache cache;
  return cache;
}

bool rmwTimeEqual(const rmw_time_t& lhs, const rmw_time_t& rhs) {
  return lhs.sec == rhs.sec && lhs.nsec == rhs.nsec;
}

bool qosProfilesEqual(const rmw_qos_profile_t& lhs, const rmw_qos_profile_t& rhs) {
  return lhs.history == rhs.history && lhs.depth == rhs.depth &&
         lhs.reliability == rhs.reliability && lhs.durability == rhs.durability &&
         rmwTimeEqual(lhs.deadline, rhs.deadline) && rmwTimeEqual(lhs.lifespan, rhs.lifespan) &&
         lhs.liveliness == rhs.liveliness &&
         rmwTimeEqual(lhs.liveliness_lease_duration, rhs.liveliness_lease_duration) &&
         lhs.avoid_ros_namespace_conventions == rhs.avoid_ros_namespace_conventions;
}
}  // namespace

// --- image_rcl_publisher implementation ---
image_rcl_publisher::image_rcl_publisher(rclcpp::Node& node, const std::string& topic_name,
                                         const rmw_qos_profile_t& qos) {
  image_publisher_impl = node.create_publisher<sensor_msgs::msg::Image>(
      topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
}

void image_rcl_publisher::publish(sensor_msgs::msg::Image::UniquePtr image_ptr) {
  image_publisher_impl->publish(std::move(image_ptr));
}

size_t image_rcl_publisher::get_subscription_count() const {
  return image_publisher_impl->get_subscription_count();
}

// --- image_transport_publisher implementation ---
#include <algorithm>
#include <string>

image_transport_publisher::image_transport_publisher(
    rclcpp::Node& node,
    const std::string& topic_name,
    const rmw_qos_profile_t& qos,
    const std::string& format)
{
  std::string param_name = topic_name;

  // convert topic separators to parameter-name separators.
  std::replace(param_name.begin(), param_name.end(), '/', '.');

  // declare the "format" parameter with our defaults for the image_transport::Publisher
  param_name = "." + param_name + ".compressed.format";
  node.declare_parameter(param_name, format);

  image_publisher_impl =
      std::make_shared<image_transport::Publisher>(image_transport::create_publisher(
#ifdef ORBBEC_IMAGE_TRANSPORT_USES_REQUIRED_INTERFACES
          image_transport::RequiredInterfaces{node},
#else
          &node,
#endif
          topic_name,
#ifdef ORBBEC_IMAGE_TRANSPORT_USES_REQUIRED_INTERFACES
          rclcpp::QoS{rclcpp::QoSInitialization::from_rmw(qos), qos}
#else
          qos
#endif
          ));
}

void image_transport_publisher::publish(sensor_msgs::msg::Image::UniquePtr image_ptr) {
  image_publisher_impl->publish(*image_ptr);
}

size_t image_transport_publisher::get_subscription_count() const {
  return image_publisher_impl->getNumSubscribers();
}

std::shared_ptr<image_publisher> getGlobalImageTransportPublisher(rclcpp::Node& node,
                                                                  const std::string& topic_name,
                                                                  const rmw_qos_profile_t& qos,
                                                                  const std::string& format) {
  const ImageTransportPublisherCacheKey key{node.get_fully_qualified_name(), topic_name};
  std::lock_guard<std::mutex> lock(imageTransportPublisherCacheMutex());
  auto& cache = imageTransportPublisherCache();
  auto cached = cache.find(key);
  if (cached != cache.end() && qosProfilesEqual(cached->second.qos, qos)) {
    return cached->second.publisher;
  }

  auto publisher = std::make_shared<image_transport_publisher>(node, topic_name, qos, format);
  cache[key] = CachedImageTransportPublisher{qos, publisher};
  return publisher;
}

void releaseGlobalImageTransportPublisher(rclcpp::Node& node, const std::string& topic_name) {
  const ImageTransportPublisherCacheKey key{node.get_fully_qualified_name(), topic_name};
  std::lock_guard<std::mutex> lock(imageTransportPublisherCacheMutex());
  imageTransportPublisherCache().erase(key);
}

void clearGlobalImageTransportPublishers(rclcpp::Node& node) {
  const std::string node_name = node.get_fully_qualified_name();
  std::lock_guard<std::mutex> lock(imageTransportPublisherCacheMutex());
  auto& cache = imageTransportPublisherCache();
  for (auto publisher = cache.begin(); publisher != cache.end();) {
    if (publisher->first.first == node_name) {
      publisher = cache.erase(publisher);
    } else {
      ++publisher;
    }
  }
}
}  // namespace orbbec_camera
