std_msgs/Header header

# --- Color stream ---
float64 color_frame_rate_cur
float64 color_frame_rate_avg
float64 color_frame_rate_min
float64 color_frame_rate_max

float64 color_delay_ms_cur
float64 color_delay_ms_avg
float64 color_delay_ms_min
float64 color_delay_ms_max

# --- Depth stream ---
float64 depth_frame_rate_cur
float64 depth_frame_rate_avg
float64 depth_frame_rate_min
float64 depth_frame_rate_max

float64 depth_delay_ms_cur
float64 depth_delay_ms_avg
float64 depth_delay_ms_min
float64 depth_delay_ms_max

# --- Device info ---
bool device_online
string connection_type   # e.g. "USB2.0", "USB3.0", "GigE"

# --- Calibration status ---
bool customer_calibration_ready
bool calibration_from_factory
bool calibration_from_launch_param
