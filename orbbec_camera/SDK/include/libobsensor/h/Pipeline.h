/**
 * \if English
 * @file Pipeline.h
 * @brief The SDK's advanced API can quickly implement functions such as switching streaming, frame synchronization, software filtering, etc., suitable for
 * applications, and the algorithm focuses on rgbd data stream scenarios. If you are on real-time or need to handle synchronization separately, align the scene.
 * Please use the interface of Device's Lower API. \else
 * @file Pipeline.h
 * @brief SDK的高级API，可以快速实现开关流，帧同步，软件滤波等功能，适用于应用，算法重点关于rgbd数据流场景。如果对实时性或需要单独
 * 处理的同步，对齐的场景。请使用Device的Lower API的接口。
 * \endif
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * \if English
 * @brief Create a pipeline object
 *
 * @param[out] error Log error messages
 * @return ob_pipeline* returns the pipeline object
 * \else
 * @brief 创建pipeline对象
 *
 * @param[out] error 记录错误信息
 * @return ob_pipeline* 返回pipeline对象
 * \endif
 */
ob_pipeline *ob_create_pipeline(ob_error **error);

/**
 * \if English
 * @brief Using device objects to create pipeline objects
 *
 * @param[in] dev Device object used to create pipeline
 * @param[out] error  Log error messages
 * @return ob_pipeline* returns the pipeline object
 * \else
 * @brief 使用设备对象来创建pipeline对象
 *
 * @param[in] dev 用于创建pipeline的设备对象
 * @param[out] error 记录错误信息
 * @return ob_pipeline* 返回pipeline对象
 * \endif
 */
ob_pipeline *ob_create_pipeline_with_device(ob_device *dev, ob_error **error);

/**
 * \if English
 * @brief Use the playback file to create a pipeline object
 *
 * @param[in] file_name The playback file path used to create the pipeline
 * @param[out] error Log error messages
 * @return ob_pipeline* returns the pipeline object
 * \else
 * @brief 使用回放文件来创建pipeline对象
 *
 * @param[in] file_name 用于创建pipeline的回放文件路径
 * @param[out] error 记录错误信息
 * @return ob_pipeline* 返回pipeline对象
 * \endif
 */
ob_pipeline *ob_create_pipeline_with_playback_file(const char *file_name, ob_error **error);

/**
 * \if English
 * @brief Delete pipeline objects
 *
 * @param[in] pipeline  The pipeline object to be deleted
 * @param[out] error Log error messages
 * \else
 * @brief 删除pipeline对象
 *
 * @param[in] pipeline 要删除的pipeline对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_pipeline(ob_pipeline *pipeline, ob_error **error);

/**
 * \if English
 * @brief Start the pipeline with default parameters
 *
 * @param[in] pipeline  pipeline object
 * @param[out] error Log error messages
 * \else
 * @brief 以默认参数启动pipeline
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pipeline_start(ob_pipeline *pipeline, ob_error **error);

/**
 * \if English
 * @brief Start the pipeline with configuration parameters
 *
 * @param[in] pipeline  pipeline object
 * @param[in] config Parameters to be configured
 * @param[out] error Log error messages
 * \else
 * @brief 启动pipeline并配置参数
 *
 * @param[in] pipeline pipeline对象
 * @param[in] config 要配置的参数
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pipeline_start_with_config(ob_pipeline *pipeline, ob_config *config, ob_error **error);

/**
 * \if English
 * @brief Start the pipeline and set the frame collection data callback
 *
 * @param[in] pipeline pipeline object
 * @param[in] config  Parameters to be configured
 * @param[in] callback Trigger a callback when all frame data in the frame set arrives
 * @param[in] user_data  Pass in any user data and get it from the callback
 * @param[out] error  Log error messages
 * \else
 * @brief 启动pipeline并设置帧集合数据回调
 *
 * @param[in] pipeline pipeline对象
 * @param[in] config 要配置的参数
 * @param[in] callback 帧集合中的所有帧数据都到达时触发回调
 * @param[in] user_data 可以传入任意用户数据，并从回调中获取
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pipeline_start_with_callback(ob_pipeline *pipeline, ob_config *config, ob_frameset_callback callback, void *user_data, ob_error **error);

/**
 * \if English
 * @brief Stop pipeline
 *
 * @param[in] pipeline pipeline object
 * @param[out] error  Log error messages
 * \else
 * @brief 停止pipeline
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pipeline_stop(ob_pipeline *pipeline, ob_error **error);

/**
 * \if English
 * @brief Get current pipeline parameters
 *
 * @param[in] pipeline pipeline object
 * @param[out] error Log error messages
 * @return ob_config*  returns pipeline parameters
 * \else
 * @brief 获取pipeline当前参数
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * @return ob_config* 返回pipeline参数
 * \endif
 */
ob_config *ob_pipeline_get_config(ob_pipeline *pipeline, ob_error **error);

/**
 * \if English
 * @brief Waiting for the return of a set of frames in the form of synchronous blocking
 *
 * @param[in] pipeline pipeline object
 * @param[in] timeout_ms Waiting timeout (ms)
 * @param[out] error Log error messages
 * @return ob_frame*  returns the waiting frameset, frameset is a special frame, you can use the frameset related interface to obtain the independent frame in
 * the set \else
 * @brief 以同步阻塞的形式等待返回一组帧集合
 *
 * @param[in] pipeline pipeline对象
 * @param[in] timeout_ms 等待超时时间(毫秒)
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回等待的帧集合frameset,frameset是特殊的frame，可以使用frameset相关接口获取集合内的独立frame
 * \endif
 */
ob_frame *ob_pipeline_wait_for_frameset(ob_pipeline *pipeline, uint32_t timeout_ms, ob_error **error);

/**
 * \if English
 * @brief Get device from the pipeline
 *
 * @param[in] pipeline pipeline object
 * @param[out] error Log error messages
 * @return ob_device* returns the device object
 * \else
 * @brief 从pipeline中获取设备
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * @return ob_device* 返回设备对象
 * \endif
 */
ob_device *ob_pipeline_get_device(ob_pipeline *pipeline, ob_error **error);

/**
 * \if English
 * @brief Get playback object from pipeline
 *
 * @param[in] pipeline pipeline objet
 * @param[out] error Log error messages
 * @return ob_playback* returns the playback object
 * \else
 * @brief 从pipeline中获取回放对象
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * @return ob_playback* 返回回放对象
 * \endif
 */
ob_playback *ob_pipeline_get_playback(ob_pipeline *pipeline, ob_error **error);

/**
 * \if English
 * @brief Get stream profile from the pipeline
 *
 * @param[in] pipeline pipeline object
 * @param[in] sensor_type sensor type, the sensor_type type supported by the camera can be obtained through the ob_device_get_sensor_list() interface
 * @param[out] profile_count The number of stream configurations obtained
 * @param[out] error Log error messages
 * @return ob_stream_profile_list* returns to the stream profile list
 * \else
 * @brief 获取传感器相应的流配置
 *
 * @param[in] pipeline pipeline对象
 * @param[in] sensor_type 传感器类型 ，相机支持的sensor_type类型可通过ob_device_get_sensor_list()接口获知
 * @param[out] profile_count 获取的流配置数量
 * @param[out] error 记录错误信息
 * @return ob_stream_profile_list* 返回流配置列表
 * \endif
 */
ob_stream_profile_list *ob_pipeline_get_stream_profile_list(ob_pipeline *pipeline, ob_sensor_type sensorType, ob_error **error);

/**
 * \if English
 * @brief Enable frame synchronization
 *
 * @param[in] pipeline pipeline object
 * @param[out] error Log error messages
 * \else
 * @brief 打开帧同步功能
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pipeline_enable_frame_sync(ob_pipeline *pipeline, ob_error **error);

/**
 * \if English
 * @brief Disable frame synchronization
 *
 * @param[in] pipeline pipeline object
 * @param[out] error Log error messages
 * \else
 * @brief 关闭帧同步功能
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pipeline_disable_frame_sync(ob_pipeline *pipeline, ob_error **error);

/**
 * \if English
 * @brief Dynamically switch the corresponding config configuration
 *
 * @param[in] pipeline pipeline object
 * @param[in] config pipeline configuration
 * @param[out] error Log error messages
 * \else
 * @brief 动态切换对应的config配置
 *
 * @param[in] pipeline pipeline对象
 * @param[in] config pipeline的配置
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pipeline_switch_config(ob_pipeline *pipeline, ob_config *config, ob_error **error);

/**
 * \if English
 * @brief Get current camera parameters
 * @attention If D2C is enabled, it will return the camera parameters after D2C, if not, it will return to the default parameters
 *
 * @param[in] pipeline pipeline object
 * @param[out] error Log error messages
 * @return ob_camera_param returns camera internal parameters
 * \else
 * @brief 获取当前相机参数
 * @attention 如果开启了D2C将返回D2C后的相机参数，如果没有将返回默认参数
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * @return ob_camera_param 返回相机内参
 * \endif
 */
ob_camera_param ob_pipeline_get_camera_param(ob_pipeline *pipeline, ob_error **error);

/**
 * \if English
 * @brief Returns a list of D2C-enabled depth sensor resolutions corresponding to the input color sensor resolution
 *
 * @param[in] pipeline pipeline object
 * @param[in] color_profile Input resolution of Color Sensor
 * @param[in] align_mode Input align mode
 * @param[out] error Log error messages
 * @return ob_stream_profile_list*
 * \else
 * @brief 返回与输入的彩色传感器分辨率对应的支持D2C的深度传感器分辨率列表
 *
 * @param[in] pipeline pipeline对象
 * @param[in] color_profile 输入的Color Sensor的分辨率
 * @param[in] align_mode 输入的对齐模式
 * @param[out] error 记录错误信息
 * @return ob_stream_profile_list*
 * \endif
 */
ob_stream_profile_list *ob_get_d2c_depth_profile_list(ob_pipeline *pipeline, ob_stream_profile *color_profile, ob_align_mode align_mode, ob_error **error);

/**
 * \if English
 * @brief Get valid area after D2C (DEPRECATED)
 *
 * @param[in] pipeline pipeline object
 * @param[in] distance working distance
 * @param[out] error Log error messages
 * @return ob_rect returns the area information valid after D2C at working distance
 * \else
 * @brief 获取D2C后有效的区域 (废弃接口)
 *
 * @param[in] pipeline pipeline对象
 * @param[in] distance 工作距离
 * @param[out] error 记录错误信息
 * @return ob_rect 返回在工作距离下D2C后有效的区域信息
 * \endif
 */
ob_rect ob_get_d2c_valid_area(ob_pipeline *pipeline, uint32_t distance, ob_error **error);

/**
 * \if English
 * @brief Get valid area between minimum distance and maximum distance after D2C
 *
 * @param[in] pipeline pipeline object
 * @param[in] minimum_distance minimum working distance
 * @param[in] maximum_distance maximum working distance
 * @param[out] error Log error messages
 * @return ob_rect returns the area information valid after D2C at working distance
 * \else
 * @brief 获取D2C后有效的区域。
 * 如果需要获取指定距离D2C后的ROI区域，将minimum_distance与maximum_distance设置成一样或者将maximum_distance设置成0
 *
 * @param[in] pipeline pipeline对象
 * @param[in] minimum_distance 最小工作距离
 * @param[in] maximum_distance 最大工作距离
 * @param[out] error 记录错误信息
 * @return ob_rect 返回在工作距离下D2C后有效的区域信息
 * \endif
 */
ob_rect ob_get_d2c_range_valid_area(ob_pipeline *pipeline, uint32_t minimum_distance, uint32_t maximum_distance, ob_error **error);

/**
 * \if English
 * @brief Start recording
 *
 * @param[in] pipeline pipeline object
 * @param[in] file_name Recorded file path
 * @param[out] error Log error messages
 * \else
 * @brief 开启录制
 *
 * @param[in] pipeline pipeline对象
 * @param[in] file_name 录制的文件路径
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pipeline_start_record(ob_pipeline *pipeline, const char *file_name, ob_error **error);

/**
 * \if English
 * @brief Turn off recording
 *
 * @param[in] pipeline pipeline object
 * @param[out] error Log error messages
 * \else
 * @brief 关闭录制
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * \endif
 */

void ob_pipeline_stop_record(ob_pipeline *pipeline, ob_error **error);

/**
 * \if English
 * @brief Create the pipeline configuration
 *
 * @param[out] error  Log error messages
 * @return ob_config* returns the configuration object
 * \else
 * @brief 创建pipeline的配置
 *
 * @param[out] error 记录错误信息
 * @return ob_config* 返回配置对象
 * \endif
 */
ob_config *ob_create_config(ob_error **error);

/**
 * \if English
 * @brief Delete the pipeline configuration
 *
 * @param[in] config Configuration to be deleted
 * @param[out] error  Log error messages
 * \else
 * @brief 删除pipeline的配置
 *
 * @param[in] config 要删除的配置
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_config(ob_config *config, ob_error **error);

/**
 * \if English
 * @brief Configure the stream to be enabled
 *
 * @param[in] config  Configuration of pipeline
 * @param[in] profile The configuration of the stream to be opened
 * @param[out] error Log error messages
 * \else
 * @brief 配置要打开的流
 *
 * @param[in] config pipeline的配置
 * @param[in] profile 要打开的流的配置
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_config_enable_stream(ob_config *config, ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Configure all streams to be enabled
 *
 * @param[in] config Configuration of pipeline
 * @param[out] error  Log error messages
 * \else
 * @brief 配置打开所有的流
 *
 * @param[in] config pipeline的配置
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_config_enable_all_stream(ob_config *config, ob_error **error);

/**
 * \if English
 * @brief Configure the stream to be disabled
 *
 * @param[in] config Configuration of pipeline
 * @param[in] profile  The configuration of the stream to be closed
 * @param[out] error Log error messages
 * \else
 * @brief 配置要关闭的流
 *
 * @param[in] config pipeline的配置
 * @param[in] profile 要关闭的流的配置
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_config_disable_stream(ob_config *config, ob_stream_type type, ob_error **error);

/**
 * \if English
 * @brief Configure all streams to be disabled
 *
 * @param[in] config Configuration of pipeline
 * @param[out] error Log error messages
 * \else
 * @brief 配置关闭所有的流
 *
 * @param[in] config pipeline的配置
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_config_disable_all_stream(ob_config *config, ob_error **error);

/**
 * \if English
 * @brief Set the alignment mode
 *
 * @param[in] config config object
 * @param[in] mode alignment mode
 * @param[out] error Log error messages
 * \else
 * @brief 设置对齐模式
 *
 * @param[in] config config对象
 * @param[in] mode 对齐模式
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_config_set_align_mode(ob_config *config, ob_align_mode mode, ob_error **error);

/**
 * \if English
 * @brief Whether the depth needs to be scaled after setting D2C
 *
 * @param[in] config config object
 * @param[in] enable Whether scaling is required
 * @param[out] error Log error messages
 * \else
 * @brief 设置D2C后是否需要缩放深度
 *
 * @param[in] config config对象
 * @param[in] enable 是否需要缩放
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_config_set_depth_scale_require(ob_config *config, bool enable, ob_error **error);

/**
 * \if English
 * @brief  Set the D2C target resolution, which is applicable to cases where the Color stream is not enabled using the OrbbecSDK and the depth needs to be D2C
 * Note: When you use OrbbecSDK to enable the Color stream, you also use this interface to set the D2C target resolution. The configuration of the enabled Color
 * stream is preferred for D2C
 *
 * @param[in] config config object
 * @param[in] d2c_target_width The D2C target has a wide resolution
 * @param[in] d2c_target_height The D2C targets has a high resolution
 * @param[out] error Log error messages
 * \else
 * @brief 设置D2C目标分辨率，适用于未使用OrbbecSDK开启Color流，且需要对深度进行D2C的情况
 * 注意:当使用OrbbecSDK开启Color流时，同时使用了此接口设置了D2C目标分辨率时。优先使用开启的Color流的配置进行D2C。
 *
 * @param[in] config config对象
 * @param[in] d2c_target_width D2C目标分辨率宽
 * @param[in] d2c_target_height D2C目标分辨率高
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_config_set_d2c_target_resolution(ob_config *config, uint32_t d2c_target_width, uint32_t d2c_target_height, ob_error **error);

/**
 * \if English
 * @brief Frame aggregation output mode
 * @brief The processing strategy when the FrameSet generated by the frame aggregation function does not contain the frames of all opened streams (which
 * can be caused by different frame rates of each stream, or by the loss of frames of one stream): drop directly or output to the user
 *
 * @param config config object
 * @param mode frame aggregation output mode（default mode is @ref OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION）
 * @param error  Log error messages
 * \else
 * @brief 数据帧汇聚输出模式
 * @brief 用于控制帧汇聚功能生成的FrameSet不包含所有已开启数据流的数据帧时（各路流帧率不同、或者某路流丢帧会导致这样的情况）的处理策略：直接丢弃或者输出给用户
 *
 * @param config config对象
 * @param mode 数据帧汇聚输出模式（默认模式为 @ref OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION）
 * @param error 记录错误信息
 * \endif
 */
void ob_config_set_frame_aggregate_output_mode(ob_config *config, ob_frame_aggregate_output_mode mode, ob_error **error);

#ifdef __cplusplus
}
#endif
