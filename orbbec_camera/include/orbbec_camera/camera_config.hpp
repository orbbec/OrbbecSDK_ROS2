#ifndef ORBBEC_CAMERA_CONFIG_HPP
#define ORBBEC_CAMERA_CONFIG_HPP

#include <string>
// default setting?
struct DefaultConfig
{
	DefaultConfig() = default;

	// qos
	std::string point_cloud_qos_ = "default";

  bool publish_tf_ = false;
	double tf_publish_rate_ = 0.0;
	std::string ir_info_url_ = "";
	std::string color_info_url_ = "";
	bool enable_d2c_viewer_ = false;
	bool enable_hardware_d2d_ = true;
	bool enable_soft_filter_ = true;
	std::string depth_filter_config_ = "";

	bool enable_color_auto_exposure_ = true;
	bool enable_color_auto_white_balance_ = true;

	int color_rotation_ = -1;
  int color_exposure_ = -1;
	int color_gain_ = -1;
	int color_white_balance_ = -1;
	int color_ae_max_exposure_ = -1;
	int color_brightness_ = -1;
	int color_sharpness_ = -1;
	int color_saturation_ = -1;
	int color_contrast_ = -1;
	int color_gamma_ = -1;
	int color_hue_ = -1;
	int depth_rotation_ = -1;
	int left_ir_rotation_ = -1;
	int right_ir_rotation_ = -1;
	bool enable_ir_auto_exposure_ = true;
	int ir_exposure_ = -1;
	int ir_gain_ = -1;
	int ir_ae_max_exposure_ = -1;
	int ir_brightness_ = -1;
	bool enable_ir_long_exposure_ = true;

	std::string depth_work_mode_ = "";
	std::string sync_mode_str_ = "";
	int depth_delay_us_ = 0;
	int color_delay_us_ = 0;

	int trigger2image_delay_us_ = 0;
	int trigger_out_delay_us_ = 0;
	bool trigger_out_enabled_ = true;
	std::string depth_precision_str_ = "";
	std::string cloud_frame_id = "/sensor/camera/gemini/pointcloud";

	bool enable_ldp_ = true;
	int soft_filter_max_diff_ = -1;
  int soft_filter_speckle_size_ = -1;
	double linear_accel_cov_ = 0.0003;
	double angular_vel_cov_ = 0.02;
	bool ordered_pc_ = false;

	int max_save_images_count_ = 10;
	bool enable_depth_scale_ = true;
	std::string device_preset_ = "HD";

	bool enable_decimation_filter_ = false;
	bool enable_hdr_merge_ = false;
	bool enable_sequence_id_filter_ = false;
	bool enable_threshold_filter_ = false;
	bool enable_noise_removal_filter_ = true;
	bool enable_spatial_filter_ = true;
	bool enable_temporal_filter_ = false;
	bool enable_hole_filling_filter_ = false;
	int decimation_filter_scale_ = -1;
	int sequence_id_filter_id_ = -1;
	int threshold_filter_max_ = -1;
	int threshold_filter_min_ = -1;
	int noise_removal_filter_min_diff_ = 256;
	int noise_removal_filter_max_size_ = 80;
	float spatial_filter_alpha_ = -1.0;
	int spatial_filter_diff_threshold_ = -1;
	int spatial_filter_magnitude_ = -1;
	int spatial_filter_radius_ = -1;

	float temporal_filter_diff_threshold_ = -1.0;
	float temporal_filter_weight_ = -1.0;
	std::string hole_filling_filter_mode_ = "";

	int hdr_merge_exposure_1_ = -1;
  int hdr_merge_gain_1_ = -1;
	int hdr_merge_exposure_2_ = -1;
	int hdr_merge_gain_2_ = -1;

	double diagnostic_period_ = 1.0;
	bool enable_laser_ = true;
	int laser_on_off_mode_ = 0;
	std::string align_target_stream_str_ = "COLOR";

	bool retry_on_usb3_detection_failure_ = false;
	int laser_energy_level_ = -1;

	bool enable_3d_reconstruction_mode_ = false;
	int min_depth_limit_ = 0;
	int max_depth_limit_ = 0;
	bool enable_heartbeat_ = false;

	std::string industry_mode_ = "";
	bool enable_color_undistortion_ = false;

	int color_ae_roi_left_ = -1;
	int color_ae_roi_top_ = -1;
	int color_ae_roi_right_ = -1;
	int color_ae_roi_bottom_ = -1;
	int depth_ae_roi_left_ = -1;
	int depth_ae_roi_top_ = -1;
	int depth_ae_roi_right_ = -1;
	int depth_ae_roi_bottom_ = -1;

	std::string time_domain_ = "device";

	int frames_per_trigger_ = 2;
	long software_trigger_period_ = 33;

	std::string frame_aggregate_mode_ = "ANY";

};

struct GeminiConfig
{
  GeminiConfig() = default;

  // Basic camera identification
  std::string camera_name{"camera"};
  std::string position{};
  std::string usb_port{""};
  std::string serial_number{""};
  int device_num{3};
	std::string net_device_ip{""};
	int net_device_port{0};

  // Connection and timing settings
  int connection_delay{10};
  std::string sync_mode{"standalone"};
  bool enable_frame_sync{true};
  bool use_hardware_time{true};
  bool enable_sync_host_time{true};
  int fps{15};

  // Color camera parameters
  bool enable_color{true};
  int color_width{640};
  int color_height{480};
  int color_fps{0};
  std::string color_format{"MJPG"};
  std::string color_info_url{""};

  // Depth camera parameters
  bool enable_depth{true};
  bool depth_registration{true};
  int depth_width{640};
  int depth_height{480};
  int depth_fps{0};
  std::string depth_format{"Y16"};
  std::string depth_precision{""};
  bool enable_depth_filter = true;

  // Left infrared camera parameters
  bool enable_left_ir{false};
  int left_ir_width{640};
  int left_ir_height{480};
  int left_ir_fps{0};
  std::string left_ir_format{"Y8"};

  // Right infrared camera parameters
  bool enable_right_ir{false};
  int right_ir_width{640};
  int right_ir_height{480};
  int right_ir_fps{0};
  std::string right_ir_format{"Y8"};

  // Infrared camera shared info
  std::string ir_info_url{""};

  // Point cloud settings
  bool enable_point_cloud{false};
  bool enable_colored_point_cloud{false};

  // IMU (Inertial Measurement Unit) settings
  bool enable_sync_output_accel_gyro{false};
  bool enable_accel{false};
  std::string accel_rate{"200hz"};
  std::string accel_range{"4g"};
  bool enable_gyro{false};
  std::string gyro_rate{"200hz"};
  std::string gyro_range{"1000dps"};
  double liner_accel_cov{0.01};
  double angular_vel_cov{0.01};

  // Advanced settings
  std::string log_level{"none"};
  bool enable_publish_extrinsic{false};
  std::string align_mode{"HW"};
  std::string config_file_path{""};
  bool enable_hardware_reset{false};

  bool enumerate_net_device{false};
};

#endif // ORBBEC_CAMERA_CONFIG_HPP