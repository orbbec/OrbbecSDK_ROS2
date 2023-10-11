/**
 * @file Pipeline.h
 * @brief The SDK's advanced API can quickly implement functions such as switching streaming, frame synchronization, software filtering, etc., suitable for
 * applications, and the algorithm focuses on rgbd data stream scenarios. If you are on real-time or need to handle synchronization separately, align the scene.
 * Please use the interface of Device's Lower API.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Create a pipeline object
 *
 * @param[out] error Log error messages
 * @return ob_pipeline* return the pipeline object
 */
ob_pipeline *ob_create_pipeline(ob_error **error);

/**
 * @brief Using device objects to create pipeline objects
 *
 * @param[in] dev Device object used to create pipeline
 * @param[out] error  Log error messages
 * @return ob_pipeline* return the pipeline object
 */
ob_pipeline *ob_create_pipeline_with_device(ob_device *dev, ob_error **error);

/**
 * @brief Use the playback file to create a pipeline object
 *
 * @param[in] file_name The playback file path used to create the pipeline
 * @param[out] error Log error messages
 * @return ob_pipeline* return the pipeline object
 */
ob_pipeline *ob_create_pipeline_with_playback_file(const char *file_name, ob_error **error);

/**
 * @brief Delete pipeline objects
 *
 * @param[in] pipeline  The pipeline object to be deleted
 * @param[out] error Log error messages
 */
void ob_delete_pipeline(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Start the pipeline with default parameters
 *
 * @param[in] pipeline  pipeline object
 * @param[out] error Log error messages
 */
void ob_pipeline_start(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Start the pipeline with configuration parameters
 *
 * @param[in] pipeline  pipeline object
 * @param[in] config Parameters to be configured
 * @param[out] error Log error messages
 */
void ob_pipeline_start_with_config(ob_pipeline *pipeline, ob_config *config, ob_error **error);

/**
 * @brief Start the pipeline and set the frame collection data callback
 *
 * @param[in] pipeline pipeline object
 * @param[in] config  Parameters to be configured
 * @param[in] callback Trigger a callback when all frame data in the frameset arrives
 * @param[in] user_data  Pass in any user data and get it from the callback
 * @param[out] error  Log error messages
 */
void ob_pipeline_start_with_callback(ob_pipeline *pipeline, ob_config *config, ob_frameset_callback callback, void *user_data, ob_error **error);

/**
 * @brief Stop pipeline
 *
 * @param[in] pipeline pipeline object
 * @param[out] error  Log error messages
 */
void ob_pipeline_stop(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Get the configuration object associated with the pipeline
 *
 * @param[in] pipeline The pipeline object
 * @param[out] error Log error messages
 * @return ob_config* The configuration object
 */
ob_config *ob_pipeline_get_config(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Wait for a set of frames to be returned synchronously
 *
 * @param[in] pipeline The pipeline object
 * @param[in] timeout_ms The timeout for waiting (in milliseconds)
 * @param[out] error Log error messages
 * @return ob_frame* The frameset that was waited for. A frameset is a special frame that can be used to obtain independent frames from the set.
 */
ob_frame *ob_pipeline_wait_for_frameset(ob_pipeline *pipeline, uint32_t timeout_ms, ob_error **error);

/**
 * @brief Get the device object associated with the pipeline
 *
 * @param[in] pipeline The pipeline object
 * @param[out] error Log error messages
 * @return ob_device* The device object
 */
ob_device *ob_pipeline_get_device(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Get the playback object associated with the pipeline
 *
 * @param[in] pipeline The pipeline object
 * @param[out] error Log error messages
 * @return ob_playback* The playback object
 */
ob_playback *ob_pipeline_get_playback(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Get the stream profile list associated with the pipeline
 *
 * @param[in] pipeline The pipeline object
 * @param[in] sensorType The sensor type. The supported sensor types can be obtained through the ob_device_get_sensor_list() interface.
 * @param[out] error Log error messages
 * @return ob_stream_profile_list* The stream profile list
 */
ob_stream_profile_list *ob_pipeline_get_stream_profile_list(ob_pipeline *pipeline, ob_sensor_type sensorType, ob_error **error);

/**
 * @brief Enable frame synchronization
 *
 * @param[in] pipeline The pipeline object
 * @param[out] error Log error messages
 */
void ob_pipeline_enable_frame_sync(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Disable frame synchronization
 *
 * @param[in] pipeline The pipeline object
 * @param[out] error Log error messages
 */
void ob_pipeline_disable_frame_sync(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Dynamically switch the corresponding configuration
 *
 * @param[in] pipeline The pipeline object
 * @param[in] config The pipeline configuration
 * @param[out] error Log error messages
 */
void ob_pipeline_switch_config(ob_pipeline *pipeline, ob_config *config, ob_error **error);

/**
 * @brief Get the current camera parameters
 *
 * @param[in] pipeline pipeline object
 * @param[in] colorWidth color width
 * @param[in] colorHeight color height
 * @param[in] depthWidth depth width
 * @param[in] depthHeight depth height
 * @param[out] error Log error messages
 * @return ob_camera_param returns camera internal parameters
 */
ob_camera_param ob_pipeline_get_camera_param_with_profile(ob_pipeline *pipeline, uint32_t colorWidth, uint32_t colorHeight, uint32_t depthWidth,
                                                          uint32_t depthHeight, ob_error **error);

/**
 * \if English
 * @brief Get current camera parameters
 * @attention If D2C is enabled, it will return the camera parameters after D2C, if not, it will return to the default parameters
 *
 * @param[in] pipeline pipeline object
 * @param[out] error Log error messages
 * @return ob_camera_param The camera internal parameters
 */
ob_camera_param ob_pipeline_get_camera_param(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Return a list of D2C-enabled depth sensor resolutions corresponding to the input color sensor resolution
 *
 * @param[in] pipeline The pipeline object
 * @param[in] color_profile The input profile of the color sensor
 * @param[in] align_mode The input align mode
 * @param[out] error Log error messages
 * @return ob_stream_profile_list* The list of D2C-enabled depth sensor resolutions
 */
ob_stream_profile_list *ob_get_d2c_depth_profile_list(ob_pipeline *pipeline, ob_stream_profile *color_profile, ob_align_mode align_mode, ob_error **error);

/**
 * @brief Get the valid area after D2C (DEPRECATED)
 *
 * @param[in] pipeline The pipeline object
 * @param[in] distance The working distance
 * @param[out] error Log error messages
 * @return ob_rect The area information that is valid after D2C at the working distance
 */
ob_rect ob_get_d2c_valid_area(ob_pipeline *pipeline, uint32_t distance, ob_error **error);

/**
 * @brief Get the valid area between the minimum distance and maximum distance after D2C
 *
 * @param[in] pipeline The pipeline object
 * @param[in] minimum_distance The minimum working distance
 * @param[in] maximum_distance The maximum working distance
 * @param[out] error Log error messages
 * @return ob_rect The area information that is valid after D2C at the working distance
 */
ob_rect ob_get_d2c_range_valid_area(ob_pipeline *pipeline, uint32_t minimum_distance, uint32_t maximum_distance, ob_error **error);

/**
 * @brief Start recording
 *
 * @param[in] pipeline The pipeline object
 * @param[in] file_name The recorded file path
 * @param[out] error Log error messages
 */
void ob_pipeline_start_record(ob_pipeline *pipeline, const char *file_name, ob_error **error);

/**
 * @brief Stop recording
 *
 * @param[in] pipeline The pipeline object
 * @param[out] error Log error messages
 */
void ob_pipeline_stop_record(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Create the pipeline configuration
 *
 * @param[out] error Log error messages
 * @return ob_config* The configuration object
 */
ob_config *ob_create_config(ob_error **error);

/**
 * @brief Delete the pipeline configuration
 *
 * @param[in] config The configuration to be deleted
 * @param[out] error Log error messages
 */
void ob_delete_config(ob_config *config, ob_error **error);

/**
 * @brief Enable the specified stream in the pipeline configuration
 *
 * @param[in] config The pipeline configuration
 * @param[in] profile The stream configuration to be enabled
 * @param[out] error Log error messages
 */
void ob_config_enable_stream(ob_config *config, ob_stream_profile *profile, ob_error **error);

/**
 * @brief Enable all streams in the pipeline configuration
 *
 * @param[in] config The pipeline configuration
 * @param[out] error Log error messages
 */
void ob_config_enable_all_stream(ob_config *config, ob_error **error);

/**
 * @brief Disable a specific stream in the pipeline configuration
 *
 * @param[in] config The pipeline configuration
 * @param[in] type The type of stream to be disabled
 * @param[out] error Log error messages
 */
void ob_config_disable_stream(ob_config *config, ob_stream_type type, ob_error **error);

/**
 * @brief Disable all streams in the pipeline configuration
 *
 * @param[in] config The pipeline configuration
 * @param[out] error Log error messages
 */
void ob_config_disable_all_stream(ob_config *config, ob_error **error);

/**
 * @brief Set the alignment mode for the pipeline configuration
 *
 * @param[in] config The pipeline configuration
 * @param[in] mode The alignment mode to be set
 * @param[out] error Log error messages
 */
void ob_config_set_align_mode(ob_config *config, ob_align_mode mode, ob_error **error);

/**
 * @brief Set whether depth scaling is required after setting D2C
 *
 * @param[in] config The pipeline configuration
 * @param[in] enable Whether scaling is required
 * @param[out] error Log error messages
 */
void ob_config_set_depth_scale_require(ob_config *config, bool enable, ob_error **error);

/**
 * @brief Set the target resolution for D2C, which is applicable when the color stream is not enabled using the OrbbecSDK and the depth needs to be D2C
 * Note: When using the OrbbecSDK to enable the color stream, this interface should also be used to set the D2C target resolution. The configuration of the
 * enabled color stream is preferred for D2C.
 *
 * @param[in] config The pipeline configuration
 * @param[in] d2c_target_width The target width for D2C
 * @param[in] d2c_target_height The target height for D2C
 * @param[out] error Log error messages
 */
void ob_config_set_d2c_target_resolution(ob_config *config, uint32_t d2c_target_width, uint32_t d2c_target_height, ob_error **error);

/**
 * @brief Set the frame aggregation output mode for the pipeline configuration
 * @brief The processing strategy when the FrameSet generated by the frame aggregation function does not contain the frames of all opened streams (which
 * can be caused by different frame rates of each stream, or by the loss of frames of one stream): drop directly or output to the user.
 *
 * @param[in] config The pipeline configuration
 * @param[in] mode The frame aggregation output mode to be set (default mode is @ref OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION)
 * @param[out] error Log error messages
 */
void ob_config_set_frame_aggregate_output_mode(ob_config *config, ob_frame_aggregate_output_mode mode, ob_error **error);

#ifdef __cplusplus
}
#endif
