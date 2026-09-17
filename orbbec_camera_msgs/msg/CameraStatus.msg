bool initialized
bool connected
bool color_stream_enabled
bool depth_stream_enabled
bool imu_stream_enabled

# Experimental feature:
# Timeout fields, set parameter enable_timeouts_in_camera_status to true
# in the orbbec_camera_node to enable these fields

# An asynchronous status message with the timeout fields is sent for each frame timeout event
# If the next frame is received before the timeout, the field is cleared 
# No asynchronous message is sent when the timeout field is cleared
bool color_frame_timeout
bool depth_frame_timeout
bool imu_frame_timeout

# StatsMsg fields are only valid if stats_valid is true
# Stats may not be valid for asynchronous messages
# For example, for frame timeout events
bool stats_valid

StatsMsg color_frame_period
StatsMsg depth_frame_period
StatsMsg imu_frame_period
StatsMsg color_frame_latency
StatsMsg depth_frame_latency
StatsMsg imu_frame_latency

int32 color_frame_rate
int32 depth_frame_rate
int32 imu_frame_rate

int32 color_expected_frame_rate
int32 depth_expected_frame_rate
int32 imu_expected_frame_rate
