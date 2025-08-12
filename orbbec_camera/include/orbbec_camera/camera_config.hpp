#include <string>


class CameraConfig
{
public:
	CameraConfig() = default;

	// qos
	std::string point_cloud_qos_ = "default";

  bool publish_tf_ = false;
	double tf_publish_rate_ = 0.0;
	std::string ir_info_url_ = "";
	std::string color_info_url_ = "";
	bool enable_d2c_viewer_ = false;
	bool enable_hardware_d2d_ = true;
	bool enable_soft_filter_ = false;
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
	bool trigger_out_enabled_ = false;
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
	float spatial_filter_alpha_ = -1;
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
};