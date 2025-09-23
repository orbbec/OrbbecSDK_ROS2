/**
 * @file service_benchmark_node.cpp
 * @brief A ROS2 node to benchmark Orbbec camera service calls.
 *
 * Features:
 *   - Benchmark a single service call (latency, success rate)
 *   - Benchmark multiple services defined in a YAML configuration file
 *   - Optionally save results to a CSV file
 *
 * Usage:
 *  1.Benchmark a single service:
 *    ros2 run orbbec_camera service_benchmark_node \
          --ros-args \
          -p service_name:=/camera/get_depth_gain \
          -p service_type:=orbbec_camera_msgs/srv/GetInt32 \
          -p count:=10
 *
    2.Benchmark multiple services from a YAML config file:
 *   ros2 run orbbec_camera service_benchmark_node \
          --ros-args \
          -p yaml_file:=/path/to/default_service_cpp.yaml
 */
#include "orbbec_camera/ob_camera_node_driver.h"
#include <algorithm>
#include <chrono>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <fstream>
#include <functional>
#include <map>

class SingleServiceBenchmark {
 public:
  SingleServiceBenchmark(rclcpp::Node::SharedPtr nh, const std::string &service_name,
                         const std::string &service_type, int count, const YAML::Node &request_data)
      : nh_(nh),
        service_name_(service_name),
        service_type_(service_type),
        count_(count),
        request_data_(request_data) {
    service_map_["orbbec_camera_msgs/srv/GetDeviceInfo"] = [this](std::vector<double> &durations,
                                                                  int &success) {
      runTyped<orbbec_camera::GetDeviceInfo>(durations, success);
    };
    service_map_["orbbec_camera_msgs/srv/GetString"] = [this](std::vector<double> &durations,
                                                              int &success) {
      runTyped<orbbec_camera::GetString>(durations, success);
    };
    service_map_["orbbec_camera_msgs/srv/GetBool"] = [this](std::vector<double> &durations,
                                                            int &success) {
      runTyped<orbbec_camera::GetBool>(durations, success);
    };
    service_map_["orbbec_camera_msgs/srv/GetInt32"] = [this](std::vector<double> &durations,
                                                             int &success) {
      runTyped<orbbec_camera::GetInt32>(durations, success);
    };
    service_map_["std_srvs/srv/Empty"] = [this](std::vector<double> &durations, int &success) {
      runTyped<std_srvs::srv::Empty>(durations, success);
    };
    service_map_["orbbec_camera_msgs/srv/SetInt32"] = [this](std::vector<double> &durations,
                                                             int &success) {
      runTyped<orbbec_camera::SetInt32>(durations, success);
    };
    service_map_["orbbec_camera_msgs/srv/SetArrays"] = [this](std::vector<double> &durations,
                                                              int &success) {
      runTyped<orbbec_camera::SetArrays>(durations, success);
    };
    service_map_["std_srvs/srv/SetBool"] = [this](std::vector<double> &durations, int &success) {
      runTyped<std_srvs::srv::SetBool>(durations, success);
    };
    service_map_["orbbec_camera_msgs/srv/SetFilter"] = [this](std::vector<double> &durations,
                                                              int &success) {
      runTyped<orbbec_camera::SetFilter>(durations, success);
    };
    service_map_["orbbec_camera_msgs/srv/SetString"] = [this](std::vector<double> &durations,
                                                              int &success) {
      runTyped<orbbec_camera::SetString>(durations, success);
    };
  }

  template <typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr getClient() {
    auto it = client_cache_.find(service_name_);
    if (it != client_cache_.end()) {
      return std::dynamic_pointer_cast<rclcpp::Client<ServiceT>>(it->second);
    }
    auto client = nh_->create_client<ServiceT>(service_name_);
    client_cache_[service_name_] = client;
    return client;
  }

  int run(std::vector<double> &durations_out) {
    int success = 0;
    auto it = service_map_.find(service_type_);
    if (it != service_map_.end()) {
      it->second(durations_out, success);
    } else {
      RCLCPP_ERROR(nh_->get_logger(), "Unsupported service type: %s", service_type_.c_str());
    }
    return success;
  }

  void printSummary(const std::vector<double> &durations, int success) {
    if (durations.empty()) {
      RCLCPP_WARN(nh_->get_logger(), "No successful calls!");
      return;
    }

    double avg = std::accumulate(durations.begin(), durations.end(), 0.0) / durations.size();
    double minv = *std::min_element(durations.begin(), durations.end());
    double maxv = *std::max_element(durations.begin(), durations.end());
    double success_rate = 100.0 * success / count_;

    std::cout << std::string(64, '=') << std::endl;
    std::cout << std::setw(7) << "Calls" << std::setw(10) << "Success" << std::setw(11) << "Rate(%)"
              << std::setw(12) << "Avg(ms)" << std::setw(12) << "Min(ms)" << std::setw(12)
              << "Max(ms)" << std::endl;
    std::cout << std::string(64, '-') << std::endl;

    std::cout << std::setw(5) << count_ << std::setw(10) << success << std::setw(12) << std::fixed
              << std::setprecision(2) << success_rate << std::setw(11) << std::fixed
              << std::setprecision(2) << avg << std::setw(12) << std::fixed << std::setprecision(2)
              << minv << std::setw(12) << std::fixed << std::setprecision(2) << maxv << std::endl;

    std::cout << std::string(64, '=') << std::endl;
  }

 private:
  rclcpp::Node::SharedPtr nh_;
  std::string service_name_;
  std::string service_type_;
  int count_;
  YAML::Node request_data_;
  std::map<std::string, std::function<void(std::vector<double> &, int &)>> service_map_;
  std::map<std::string, rclcpp::ClientBase::SharedPtr> client_cache_;

  template <typename T, typename = void>
  struct has_success : std::false_type {};
  template <typename T>
  struct has_success<T, std::void_t<decltype(std::declval<typename T::Response>().success)>>
      : std::true_type {};

  template <typename ServiceT>
  void fillRequest(typename ServiceT::Request &request);

  template <typename ServiceT>
  void runTyped(std::vector<double> &durations_out, int &success_out) {
    auto client = getClient<ServiceT>();
    std::vector<double> durations;
    int success = 0;

    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(nh_->get_logger(), "Service %s not available", service_name_.c_str());
      durations_out.clear();
      success_out = 0;
      return;
    }

    for (int i = 0; i < count_; ++i) {
      auto Request = std::make_shared<typename ServiceT::Request>();
      fillRequest<ServiceT>(*Request);

      auto start = std::chrono::steady_clock::now();
      bool call_ok = false;
      try {
        auto result_future = client->async_send_request(Request);
        auto status = rclcpp::spin_until_future_complete(nh_, result_future);
        call_ok = (status == rclcpp::FutureReturnCode::SUCCESS);
        if (call_ok) {
          auto end = std::chrono::steady_clock::now();
          double dt =
              std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
          durations.push_back(dt);

          if constexpr (has_success<ServiceT>::value) {
            auto response = result_future.get();
            if (response->success) {
              success++;
              RCLCPP_INFO(nh_->get_logger(), "Call %s %d/%d succeeded (cost: %.2f ms)",
                      service_name_.c_str(), i + 1, count_, dt);
            } else {
              RCLCPP_WARN(nh_->get_logger(), "Call %s %d/%d (cost: %.2f ms) responded with success=false, message='%s'",
                          service_name_.c_str(), i + 1, count_, dt,response->message.c_str());
            }
          } else {
            success++;
          }
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(nh_->get_logger(), "Exception calling service %s: %s", service_name_.c_str(),
                     e.what());
      }

      if (!call_ok) {
        std::string request_str;
        try {
          if (request_data_ && !request_data_.IsNull()) {
            request_str = YAML::Dump(request_data_);
          } else {
            request_str = "{error 6}";
          }
        } catch (...) {
          request_str = "{error null}";
        }
        RCLCPP_WARN(nh_->get_logger(),
                    "Call %d/%d failed for service '%s' (type: '%s') with request: %s", i + 1,
                    count_, service_name_.c_str(), service_type_.c_str(), request_str.c_str());
      }
    }

    if (success == 0 || durations.empty()) {
      RCLCPP_WARN(nh_->get_logger(), "No successful calls for service %s (%s)",
                  service_name_.c_str(), service_type_.c_str());
      durations_out.clear();
      success_out = 0;
      return;
    }

    durations_out = durations;
    success_out = success;
  }
};

template <typename ServiceT>
void SingleServiceBenchmark::fillRequest(typename ServiceT::Request &request) {
  (void)request;
}

// SetInt32
template <>
void SingleServiceBenchmark::fillRequest<orbbec_camera::SetInt32>(
    orbbec_camera::SetInt32::Request &request) {
  if (request_data_ && request_data_["data"] && request_data_["data"].IsScalar()) {
    request.data = request_data_["data"].as<int>();
  }
}

// SetBool
template <>
void SingleServiceBenchmark::fillRequest<std_srvs::srv::SetBool>(
    std_srvs::srv::SetBool::Request &request) {
  if (request_data_ && request_data_["data"] && request_data_["data"].IsScalar()) {
    request.data = request_data_["data"].as<bool>();
  }
}

// SetString
template <>
void SingleServiceBenchmark::fillRequest<orbbec_camera_msgs::srv::SetString>(
    orbbec_camera_msgs::srv::SetString::Request &request) {
  if (!request_data_) {
    return;
  }
  if (request_data_.IsScalar()) {
    request.data = request_data_.as<std::string>();
  } else if (request_data_["data"]) {
    request.data = request_data_["data"].as<std::string>();
  }
}

// SetArrays
template <>
void SingleServiceBenchmark::fillRequest<orbbec_camera::SetArrays>(
    orbbec_camera::SetArrays::Request &request) {
  if (request_data_ && request_data_["data_param"] && request_data_["data_param"].IsSequence()) {
    const YAML::Node &arr = request_data_["data_param"];
    request.data_param.clear();
    for (const auto &v : arr) request.data_param.push_back(v.as<int>());
  }
}

// SetFilter
template <>
void SingleServiceBenchmark::fillRequest<orbbec_camera::SetFilter>(
    orbbec_camera::SetFilter::Request &request) {
  if (!request_data_ || request_data_.IsNull()) return;

  if (request_data_["filter_name"] && request_data_["filter_name"].IsScalar())
    request.filter_name = request_data_["filter_name"].as<std::string>();
  if (request_data_["filter_enable"] && request_data_["filter_enable"].IsScalar())
    request.filter_enable = request_data_["filter_enable"].as<bool>();
  if (request_data_["filter_param"] && request_data_["filter_param"].IsSequence()) {
    request.filter_param.clear();
    for (const auto &v : request_data_["filter_param"]) request.filter_param.push_back(v.as<int>());
  }
}

class MultiServiceBenchmark {
 public:
  MultiServiceBenchmark(rclcpp::Node::SharedPtr nh, const YAML::Node &services_config,
                        int default_count, const std::string &csv_file)
      : nh_(nh),
        services_config_(services_config),
        default_count_(default_count),
        csv_file_(csv_file) {
    if (!csv_file_.empty()) {
      csv_stream_.open(csv_file_, std::ios::out);
      if (csv_stream_.is_open()) {
        csv_stream_ << "Service,Type,Calls,Success,Rate,Avg(ms),Min(ms),Max(ms)\n";
      } else {
        RCLCPP_WARN(nh_->get_logger(), "Failed to open CSV file: %s", csv_file_.c_str());
      }
    }
  }

  ~MultiServiceBenchmark() {
    if (csv_stream_.is_open()) csv_stream_.close();
    RCLCPP_INFO(nh_->get_logger(), "Benchmark results saved to CSV file: %s", csv_file_.c_str());
  }

  void run() {
    if (!services_config_ || !services_config_.IsSequence()) {
      RCLCPP_ERROR(nh_->get_logger(), "No services found in YAML config.");
      return;
    }

    for (size_t i = 0; i < services_config_.size(); ++i) {
      YAML::Node svc = services_config_[i];
      std::string service_name = svc["name"] ? svc["name"].as<std::string>() : "";
      std::string service_type = svc["type"] ? svc["type"].as<std::string>() : "";
      YAML::Node request_data = svc["request"];
      int count = svc["count"] ? svc["count"].as<int>() : default_count_;

      RCLCPP_INFO(nh_->get_logger(), "Running benchmark for service %s (%s)", service_name.c_str(),
                  service_type.c_str());

      SingleServiceBenchmark bench(nh_, service_name, service_type, count, request_data);

      std::vector<double> durations;
      int success = bench.run(durations);

      if (csv_stream_.is_open()) {
        if (durations.empty() || success == 0) {
          csv_stream_ << service_name << "," << service_type << "," << count << "," << success
                      << ",0.00%,0.00,0.00,0.00\n";
        } else {
          double avg = std::accumulate(durations.begin(), durations.end(), 0.0) / durations.size();
          double minv = *std::min_element(durations.begin(), durations.end());
          double maxv = *std::max_element(durations.begin(), durations.end());
          double success_rate = 100.0 * success / count;

          csv_stream_ << service_name << "," << service_type << "," << count << "," << success
                      << "," << std::fixed << std::setprecision(2) << success_rate << "%"
                      << "," << avg << "," << minv << "," << maxv << "\n";
        }
      }
    }
  }

 private:
  rclcpp::Node::SharedPtr nh_;
  YAML::Node services_config_;
  int default_count_;
  std::string csv_file_;
  std::ofstream csv_stream_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("service_benchmark_node");

  std::string service_name, service_type, request_str;
  YAML::Node request_data;
  int count = 10;
  std::string yaml_file, csv_file;

  nh->declare_parameter("yaml_file", "");
  nh->declare_parameter("csv_file", "multi_service_results_log_cpp.csv");
  nh->declare_parameter("service_name", "");
  nh->declare_parameter("service_type", "");
  nh->declare_parameter("request_data", "");
  nh->declare_parameter("count", 10);

  nh->get_parameter("yaml_file", yaml_file);
  nh->get_parameter("csv_file", csv_file);
  nh->get_parameter("service_name", service_name);
  nh->get_parameter("service_type", service_type);
  nh->get_parameter("request_data", request_str);
  nh->get_parameter("count", count);

  if (yaml_file.empty()) {
    if (service_name.empty() || service_type.empty()) {
      RCLCPP_ERROR(nh->get_logger(),
                   "service_name or service_type is empty! Please set parameters.");
      rclcpp::shutdown();
      return 1;
    }
    if (!request_str.empty()) {
      try {
        request_data = YAML::Load(request_str);
      } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(nh->get_logger(), "Failed to parse request_data: %s", e.what());
        rclcpp::shutdown();
        return 1;
      }
    }
    SingleServiceBenchmark bench(nh, service_name, service_type, count, request_data);
    std::vector<double> durations_out;
    int success = bench.run(durations_out);
    bench.printSummary(durations_out, success);
  } else {
    YAML::Node config = YAML::LoadFile(yaml_file);
    YAML::Node services = config["services"];
    int global_count = config["default_count"] ? config["default_count"].as<int>() : 1;

    MultiServiceBenchmark multi_bench(nh, services, global_count, csv_file);
    multi_bench.run();
  }

  rclcpp::shutdown();
  return 0;
}
