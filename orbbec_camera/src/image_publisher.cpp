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

#include <algorithm>
#include <map>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <image_transport/publisher_plugin.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>

namespace orbbec_camera {

namespace {
using ImageTransportPublisherCacheKey = std::pair<std::string, std::string>;

struct CachedImageTransportPublisher {
  rmw_qos_profile_t qos;
  std::vector<std::string> excluded_plugins;
  std::shared_ptr<image_publisher> publisher;
};

// Use the standard PublisherPlugin API when one transport is supplied separately.
// The node's configured allowlist remains intact; only this publisher excludes it.
class FilteredImageTransportPublisher : public image_publisher {
 public:
  FilteredImageTransportPublisher(rclcpp::Node& node, const std::string& topic_name,
                                  const rmw_qos_profile_t& qos,
                                  const std::vector<std::string>& excluded_plugins)
      : loader_(std::make_shared<pluginlib::ClassLoader<image_transport::PublisherPlugin>>(
            "image_transport", "image_transport::PublisherPlugin")) {
#ifdef ORBBEC_IMAGE_TRANSPORT_USES_REQUIRED_INTERFACES
    const auto topic = node.get_node_topics_interface()->resolve_topic_name(topic_name);
#else
    const auto topic =
        rclcpp::expand_topic_or_service_name(topic_name, node.get_name(), node.get_namespace());
#endif
    std::string parameter_base = topic.substr(std::string(node.get_effective_namespace()).size());
    std::replace(parameter_base.begin(), parameter_base.end(), '/', '.');
    if (!parameter_base.empty() && parameter_base.front() == '.') {
      parameter_base.erase(0, 1);
    }
    const auto parameter_name = parameter_base + ".enable_pub_plugins";
    std::vector<std::string> plugins;
    for (const auto& lookup_name : loader_->getDeclaredClasses()) {
      const auto suffix = lookup_name.rfind("_pub");
      plugins.push_back(suffix == std::string::npos ? lookup_name : lookup_name.substr(0, suffix));
    }
    if (node.has_parameter(parameter_name)) {
      plugins = node.get_parameter(parameter_name).as_string_array();
    } else {
      plugins = node.declare_parameter<std::vector<std::string>>(parameter_name, plugins);
    }
    // Match image_transport's deduplication of the allowlist.
    std::sort(plugins.begin(), plugins.end());
    plugins.erase(std::unique(plugins.begin(), plugins.end()), plugins.end());
    const bool exclude_compressed =
        std::find(excluded_plugins.begin(), excluded_plugins.end(), "image_transport/compressed") !=
        excluded_plugins.end();
    for (const auto& name : plugins) {
      if (std::find(excluded_plugins.begin(), excluded_plugins.end(), name) !=
          excluded_plugins.end()) {
        continue;
      }
      try {
        auto plugin = loader_->createSharedInstance(name + "_pub");
        // Also exclude aliases that advertise the reserved passthrough topic.
        if (exclude_compressed && plugin->getTransportName() == "compressed") {
          continue;
        }
#ifdef ORBBEC_IMAGE_TRANSPORT_USES_REQUIRED_INTERFACES
        plugin->advertise(image_transport::RequiredInterfaces{node}, topic,
                          rclcpp::QoS{rclcpp::QoSInitialization::from_rmw(qos), qos});
#else
        plugin->advertise(&node, topic, qos);
#endif
        plugins_.push_back(std::move(plugin));
      } catch (const std::runtime_error& error) {
        RCLCPP_ERROR(node.get_logger(), "Failed to load image transport plugin %s: %s",
                     name.c_str(), error.what());
      }
    }
    // An empty decoded path is valid when only JPEG passthrough was requested.
  }

  ~FilteredImageTransportPublisher() override {
    for (const auto& plugin : plugins_) {
      plugin->shutdown();
    }
  }

  void publish(sensor_msgs::msg::Image::UniquePtr image) override {
    for (const auto& plugin : plugins_) {
      if (plugin->getNumSubscribers() > 0) {
        plugin->publish(*image);
      }
    }
  }

  size_t get_subscription_count() const override {
    size_t count = 0;
    for (const auto& plugin : plugins_) {
      count += plugin->getNumSubscribers();
    }
    return count;
  }

 private:
  // Plugins must be destroyed before the loader that owns their libraries.
  std::shared_ptr<pluginlib::ClassLoader<image_transport::PublisherPlugin>> loader_;
  std::vector<std::shared_ptr<image_transport::PublisherPlugin>> plugins_;
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
image_transport_publisher::image_transport_publisher(rclcpp::Node& node,
                                                     const std::string& topic_name,
                                                     const rmw_qos_profile_t& qos) {
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

std::shared_ptr<image_publisher> getGlobalImageTransportPublisher(
    rclcpp::Node& node, const std::string& topic_name, const rmw_qos_profile_t& qos,
    const std::vector<std::string>& excluded_plugins) {
  const ImageTransportPublisherCacheKey key{node.get_fully_qualified_name(), topic_name};
  std::lock_guard<std::mutex> lock(imageTransportPublisherCacheMutex());
  auto& cache = imageTransportPublisherCache();
  auto cached = cache.find(key);
  if (cached != cache.end() && qosProfilesEqual(cached->second.qos, qos) &&
      cached->second.excluded_plugins == excluded_plugins) {
    return cached->second.publisher;
  }

  // Release the previous plugin endpoints before changing transport policy.
  cache.erase(key);
  std::shared_ptr<image_publisher> publisher;
  if (excluded_plugins.empty()) {
    publisher = std::make_shared<image_transport_publisher>(node, topic_name, qos);
  } else {
    publisher =
        std::make_shared<FilteredImageTransportPublisher>(node, topic_name, qos, excluded_plugins);
  }
  cache[key] = CachedImageTransportPublisher{qos, excluded_plugins, publisher};
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
