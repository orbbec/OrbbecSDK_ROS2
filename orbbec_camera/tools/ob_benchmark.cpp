#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>
#include "orbbec_camera_msgs/msg/metadata.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <filesystem>

namespace orbbec_camera {
namespace tools {

class ObBenchmark : public rclcpp::Node {
 public:
  ObBenchmark() : Node("ObBenchmark") {
    params_init();
    stat_process();
    function_ = this->create_wall_timer(std::chrono::seconds(test_cycle_),
                                        std::bind(&ObBenchmark::functionCallback, this));
    config_ = this->create_wall_timer(std::chrono::seconds(switch_cycle_),
                                      std::bind(&ObBenchmark::configCallback, this));
  }
  ~ObBenchmark() { kill_process(process_name_); }

 private:
  rclcpp::TimerBase::SharedPtr function_;
  rclcpp::TimerBase::SharedPtr config_;

  std::vector<float> cpu_usage_;
  std::vector<float> memory_usage_;
  std::vector<std::string> current_time_;

  int usage_count_ = 0;
  int launch_count_ = 0;
  long prevIdle = 0;
  long prevTotal = 0;
  std::string process_pid_;

  std::string process_name_;
  int switch_cycle_;
  int test_cycle_;
  int skip_number_;

  std::mutex image_mutex_;

  void params_init() {
    std::ifstream file(
        "install/orbbec_camera/share/orbbec_camera/config/tools/startbenchmark/"
        "start_benchmark_params.json");
    if (!file.is_open()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open JSON file.");
      return;
    }
    nlohmann::json json_data;
    file >> json_data;
    process_name_ = json_data["start_benchmark_params"]["process_name"].get<std::string>();
    switch_cycle_ = json_data["start_benchmark_params"]["switch_cycle"].get<int>();
    test_cycle_ = json_data["start_benchmark_params"]["test_cycle"].get<int>();
    skip_number_ = json_data["start_benchmark_params"]["skip_number"].get<int>();
  }
  std::string exec_command(const std::string& command) {
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
      std::cerr << "Failed to run command" << std::endl;
      return "";
    }

    char buffer[128];
    std::string result = "";
    while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
      result += buffer;
    }

    return result;
  }
  float get_ps_cpu_usage_by_process(const std::string& process_name) {
    std::string command = "ps aux | grep '" + process_name + "' | grep -v grep";
    std::string result = exec_command(command);
    std::istringstream stream(result);
    std::string line;
    float total_cpu_usage = 0.0;

    while (std::getline(stream, line)) {
      std::istringstream line_stream(line);
      std::string user, pid, cpu, mem, command;
      line_stream >> user >> pid >> cpu >> mem;

      if (cpu.empty() || line.find("grep") != std::string::npos) {
        continue;
      }

      try {
        process_pid_ = pid;
        float cpu_usage = std::stof(cpu);
        total_cpu_usage += cpu_usage;
      } catch (const std::invalid_argument& e) {
        continue;
      }
    }

    return total_cpu_usage;
  }

  float get_top_cpu_usage_by_process() {
    std::string command =
        "top -bn1 -p " + process_pid_ + " | grep '" + process_pid_ + "' | awk '{print $9}'";
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
      std::cerr << "Failed to run command" << std::endl;
      return -1.0f;
    }
    char buffer[128];
    float total_cpu_usage = 0.0f;
    bool found_process = false;
    while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
      std::stringstream ss(buffer);
      float cpu_usage = 0.0f;
      ss >> cpu_usage;
      if (ss) {
        total_cpu_usage += cpu_usage;
        found_process = true;
      }
    }
    if (!found_process) {
      std::cerr << "No matching processes found for PID: " << process_pid_ << std::endl;
      return -1.0f;
    }
    return total_cpu_usage;
  }
  float get_memory_usage_by_process() {
    std::string command =
        "top -b -n 1 -p " + process_pid_ + " | grep '" + process_pid_ + "' | awk '{print $6}'";
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
      std::cerr << "Failed to run command" << std::endl;
      return -1.0f;
    }
    char buffer[128];
    std::string result = "";

    while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
      result += buffer;
    }
    std::stringstream ss(result);
    long memory_usage_kb;
    ss >> memory_usage_kb;
    float memory_usage_mb = static_cast<float>(memory_usage_kb) / 1024.0f;
    std::stringstream formatted_result;
    formatted_result << std::fixed << std::setprecision(1) << memory_usage_mb;
    return std::stof(formatted_result.str());
  }
  std::string getCurrentTimes() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_time_t);
    std::ostringstream date_stream;
    date_stream << std::put_time(&tm, "%H:%M:%S");
    std::string date_str = date_stream.str();
    return date_str;
  }
  void ensure_directory_exists(const std::string& path) {
    size_t found = path.find_last_of("/\\");
    std::string dir_path = path.substr(0, found);

    struct stat info;
    if (stat(dir_path.c_str(), &info) != 0) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ObBenchmark"),
                         "Directory does not exist, creating it: " << dir_path);
      if (mkdir(dir_path.c_str(), 0777) != 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to create directory: " << dir_path.c_str());
      }
    }
  }
  void save_to_csv(const std::string& filename) {
    ensure_directory_exists(filename);
    std::ofstream file;
    file.open(filename, std::ios_base::app);
    if (file.is_open()) {
      file << "Time,CPU,Memory(MB)" << "\n";
      for (int i = skip_number_; i < usage_count_ - 1; i++) {
        file << current_time_[i] << "," << cpu_usage_[i] << "%" << "," << memory_usage_[i] << "\n";
      }
      float cpu_average =
          std::accumulate(cpu_usage_.begin() + skip_number_, cpu_usage_.begin() + usage_count_ - 1, 0.0f) /
          (usage_count_ - (skip_number_+1));
      float memory_average = std::accumulate(memory_usage_.begin() + skip_number_,
                                             memory_usage_.begin() + usage_count_ - 1, 0.0f) /
                             (usage_count_ - (skip_number_+1));
      file << "Average: ," << cpu_average << "%, " << memory_average << "\n";
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ObBenchmark"),
                         "Average: " << cpu_average << "%" << memory_average << "MB");
      file.close();
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open file for writing.");
    }
    cpu_usage_.clear();
    memory_usage_.clear();
    current_time_.clear();
    usage_count_ = 0;
    ++launch_count_;
  }
void kill_process(const std::string& process_name) {
    std::string command = "ps aux | grep " + process_name + " | grep -v grep | awk '{print $2}'";

    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to run command to find PID.");
      return;
    }

    std::vector<int> pids;
    char path[1035];
    while (fgets(path, sizeof(path), pipe.get()) != nullptr) {
      int pid = std::stoi(path);
      pids.push_back(pid);
      std::cout << "----------------------------------------------------------" << std::endl;
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ObBenchmark"), "Found PID: " << pid);
    }

    if (pids.empty()) {
      RCLCPP_WARN(this->get_logger(), "No matching processes found.");
      return;
    }

    for (int pid : pids) {
        if (kill(pid, SIGINT) == 0) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("ObBenchmark"),
                             "Successfully sent SIGINT (Ctrl+C) to process with PID " << pid);
        } else {
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "Failed to send SIGINT to process with PID " << pid);
        }
    }
}
  void stat_process() {
    const char* ros2_launch = "/opt/ros/humble/bin/ros2";
    std::string start_launch = "ob_benchmark_" + std::to_string(launch_count_) + ".launch.py";
    std::vector<const char*> args = {ros2_launch, "launch", "orbbec_camera", start_launch.c_str(),
                                     nullptr};

    pid_t pid = fork();

    if (pid == 0) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ObBenchmark"),
                         "Child process: launching ros2 launch.");
      if (execvp(ros2_launch, (char* const*)args.data()) == -1) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Failed to execute ros2 launch: " << strerror(errno));
        exit(1);
      }
    } else if (pid > 0) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ObBenchmark"),
                         "Parent process: continuing execution.");
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("ObBenchmark"), "Fork failed!");
    }
  }

  void functionCallback() {
    std::lock_guard<std::mutex> lock(image_mutex_);
    // RCLCPP_INFO_STREAM(
    //     rclcpp::get_logger("ObBenchmark"),
    //     "get_ps_cpu_usage_by_process: " << get_ps_cpu_usage_by_process(process_name_));
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("ObBenchmark"),
    //                    "get_top_cpu_usage_by_process: " << get_top_cpu_usage_by_process());
    float cpu_usage =
        get_ps_cpu_usage_by_process(process_name_) * 0.4 + get_top_cpu_usage_by_process() * 0.6;
    float memory_usage = get_memory_usage_by_process();
    cpu_usage_.push_back(cpu_usage);
    memory_usage_.push_back(memory_usage);
    current_time_.push_back(getCurrentTimes());
    RCLCPP_INFO_STREAM(rclcpp::get_logger("ObBenchmark"),
                       "CurrentTimes: " << current_time_[usage_count_]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("ObBenchmark"), "CPU Usage of "
                                                              << process_name_ << ":"
                                                              << cpu_usage_[usage_count_] << "%");
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("ObBenchmark"),
        "Memory Usage of " << process_name_ << ":" << memory_usage_[usage_count_] << "MB");
    usage_count_++;
  }
  void configCallback() {
    std::lock_guard<std::mutex> lock(image_mutex_);
    std::string csv = std::string("ob_benchmark/") + std::to_string(launch_count_) + ".csv";

    save_to_csv(csv);
    kill_process(process_name_);
    stat_process();
  }
};

}  // namespace tools
}  // namespace orbbec_camera
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orbbec_camera::tools::ObBenchmark>());
  rclcpp::shutdown();
  return 0;
}
