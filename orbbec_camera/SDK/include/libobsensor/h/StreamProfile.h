/**
 * @file StreamProfile.h
 * @brief The stream profile related type is used to get information such as the width, height, frame rate, and format of the stream.
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Get stream profile format
 *
 * @param[in] profile  Stream configuration object
 * @param[out] error  Log error messages
 * @return ob_format return the format of the stream
 */
ob_format ob_stream_profile_format(ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get stream profile type
 *
 * @param[in] profile Stream configuration object
 * @param[out] error Log error messages
 * @return ob_stream_type stream type
 */
ob_stream_type ob_stream_profile_type(ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the frame rate of the video stream configuration
 *
 * @param[in] profile Stream configuration object
 * @param[out] error Log error messages
 * @return uint32_t return the frame rate of the stream
 */
uint32_t ob_video_stream_profile_fps(ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the width of the video stream configuration
 *
 * @param[in] profile Stream configuration object, if the configuration is not a video stream configuration, an error will be returned
 * @param[out] error Log error messages
 * @return uint32_t return the width of the stream
 */
uint32_t ob_video_stream_profile_width(ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the height of the video stream configuration
 *
 * @param[in] profile Stream configuration object, if the configuration is not a video stream configuration, an error will be returned
 * @param[out] error  Log error messages
 * @return uint32_t return the height of the stream
 */
uint32_t ob_video_stream_profile_height(ob_stream_profile *profile, ob_error **error);
/**
 * @brief Get the full-scale range of the accelerometer stream.
 *
 * @param[in] profile Stream configuration object. If the configuration is not for the accelerometer stream, an error will be returned.
 * @param[out] error Log error messages.
 * @return The full-scale range of the accelerometer stream.
 */
ob_accel_full_scale_range ob_accel_stream_profile_full_scale_range(ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the sampling frequency of the accelerometer frame.
 *
 * @param[in] profile Stream configuration object. If the configuration is not for the accelerometer stream, an error will be returned.
 * @param[out] error Log error messages.
 * @return The sampling frequency of the accelerometer frame.
 */
ob_accel_sample_rate ob_accel_stream_profile_sample_rate(ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the full-scale range of the gyroscope stream.
 *
 * @param[in] profile Stream configuration object. If the configuration is not for the gyroscope stream, an error will be returned.
 * @param[out] error Log error messages.
 * @return The full-scale range of the gyroscope stream.
 */
ob_gyro_full_scale_range ob_gyro_stream_profile_full_scale_range(ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the sampling frequency of the gyroscope stream.
 *
 * @param[in] profile Stream configuration object. If the configuration is not for the gyroscope stream, an error will be returned.
 * @param[out] error Log error messages.
 * @return The sampling frequency of the gyroscope stream.
 */
ob_gyro_sample_rate ob_gyro_stream_profile_sample_rate(ob_stream_profile *profile, ob_error **error);

/**
 * @brief Match the corresponding ob_stream_profile through the passed parameters. If there are multiple matches,
 * the first one in the list will be returned by default. If no matched profile is found, an error will be returned.
 *
 * @param[in] profile_list Resolution list.
 * @param[in] width Width. If you don't need to add matching conditions, you can pass OB_WIDTH_ANY.
 * @param[in] height Height. If you don't need to add matching conditions, you can pass OB_HEIGHT_ANY.
 * @param[in] format Format. If you don't need to add matching conditions, you can pass OB_FORMAT_ANY.
 * @param[in] fps Frame rate. If you don't need to add matching conditions, you can pass OB_FPS_ANY.
 * @param[out] error Log error messages.
 * @return The matching profile.
 */
ob_stream_profile *ob_stream_profile_list_get_video_stream_profile(ob_stream_profile_list *profile_list, int width, int height, ob_format format, int fps,
                                                                   ob_error **error);

/**
 * @brief Match the corresponding ob_stream_profile through the passed parameters. If there are multiple matches,
 * the first one in the list will be returned by default. If no matched profile is found, an error will be returned.
 *
 * @param[in] profile_list Resolution list.
 * @param[in] fullScaleRange Full-scale range. If you don't need to add matching conditions, you can pass 0.
 * @param[in] sampleRate Sample rate. If you don't need to add matching conditions, you can pass 0.
 * @param[out] error Log error messages.
 * @return The matching profile.
 */
ob_stream_profile *ob_stream_profile_list_get_accel_stream_profile(ob_stream_profile_list *profile_list, ob_accel_full_scale_range fullScaleRange,
                                                                   ob_accel_sample_rate sampleRate, ob_error **error);

/**
 * @brief Match the corresponding ob_stream_profile through the passed parameters. If there are multiple matches,
 * the first one in the list will be returned by default. If no matched profile is found, an error will be returned.
 *
 * @param[in] profile_list Resolution list.
 * @param[in] fullScaleRange Full-scale range. If you don't need to add matching conditions, you can pass 0.
 * @param[in] sampleRate Sample rate. If you don't need to add matching conditions, you can pass 0.
 * @param[out] error Log error messages.
 * @return The matching profile.
 */
ob_stream_profile *ob_stream_profile_list_get_gyro_stream_profile(ob_stream_profile_list *profile_list, ob_gyro_full_scale_range fullScaleRange,
                                                                  ob_gyro_sample_rate sampleRate, ob_error **error);

/**
 * @brief Get the corresponding StreamProfile by subscripting.
 *
 * @param[in] profile_list StreamProfile lists.
 * @param[in] index Index.
 * @param[out] error Log error messages.
 * @return The matching profile.
 */
ob_stream_profile *ob_stream_profile_list_get_profile(ob_stream_profile_list *profile_list, int index, ob_error **error);

/**
 * @brief Get the number of StreamProfile lists.
 *
 * @param[in] profile_list StreamProfile list.
 * @param[out] error Log error messages.
 * @return The number of StreamProfile lists.
 */
uint32_t ob_stream_profile_list_count(ob_stream_profile_list *profile_list, ob_error **error);

/**
 * @brief Delete the stream profile list.
 *
 * @param[in] profile_list Stream configuration list.
 * @param[out] error Log error messages.
 */
void ob_delete_stream_profile_list(ob_stream_profile_list *profile_list, ob_error **error);

/**
 * @brief Delete the stream configuration.
 *
 * @param[in] profile Stream configuration object.
 * @param[out] error Log error messages.
 */
void ob_delete_stream_profile(ob_stream_profile *profile, ob_error **error);

#ifdef __cplusplus
}
#endif