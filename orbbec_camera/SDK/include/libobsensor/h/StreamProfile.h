/**
 * \if English
 * @file StreamProfile.h
 * @brief The stream configuration related type is used to get information such as the width, height, frame rate, and format of the stream.
 * \else
 * @file StreamProfile.h
 * @brief 流配置相关函数，用于获取流的宽、高、帧率及格式等信息
 * \endif
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * \if English
 * @brief Get stream profile format
 *
 * @param[in] profile  Stream configuration object
 * @param[out] error  Log error messages
 * @return ob_format returns the format of the stream
 * \else
 * @brief 获取流配置的格式
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 * @return ob_format 返回流的格式
 * \endif
 */
ob_format ob_stream_profile_format(ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Get stream profile type
 *
 * @param[in] profile Stream configuration object
 * @param[out] error Log error messages
 * @return ob_stream_type stream type
 * \else
 * @brief 获取流的类型
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 * @return ob_stream_type 流的类型
 * \endif
 */
ob_stream_type ob_stream_profile_type(ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Get the frame rate of the video stream configuration
 *
 * @param[in] profile Stream configuration object
 * @param[out] error Log error messages
 * @return uint32_t returns the frame rate of the stream
 * \else
 * @brief 获取视频流配置的帧率
 *
 * @param[in] profile 流配置对象, 如果该配置不是视频流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的帧率
 * \endif
 */
uint32_t ob_video_stream_profile_fps(ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Get the width of the video stream configuration
 *
 * @param[in] profile Stream configuration object, if the configuration is not a video stream configuration, an error will be returned
 * @param[out] error Log error messages
 * @return uint32_t returns the width of the stream
 * \else
 * @brief 获取视频流配置的宽
 *
 * @param[in] profile 流配置对象, 如果该配置不是视频流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的宽
 * \endif
 */
uint32_t ob_video_stream_profile_width(ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Get the height of the video stream configuration
 *
 * @param[in] profile Stream configuration object, if the configuration is not a video stream configuration, an error will be returned
 * @param[out] error  Log error messages
 * @return uint32_t returns the height of the stream
 * \else
 * @brief 获取视频流配置的高
 *
 * @param[in] profile 流配置对象, 如果该配置不是视频流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的高
 * \endif
 */
uint32_t ob_video_stream_profile_height(ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Get scale range of accelerometer stream
 *
 * @param[in] profile Stream configuration object, if the configuration is not the accelerometer stream configuration, an error will be returned
 * @param[out] error Log error messages
 * @return ob_accel_full_scale_range range
 * \else
 * @brief 获取加速度计流配置的量程范围
 *
 * @param[in] profile 流配置对象, 如果该配置不是加速度计流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return ob_accel_full_scale_range 量程范围
 * \endif
 */
ob_accel_full_scale_range ob_accel_stream_profile_full_scale_range(ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Get sampling frequency of the accelerometer frame
 *
 * @param[in] profile Stream configuration object, if the configuration is not the accelerometer stream configuration, an error will be returned
 * @param[out] error Log error messages
 * @return ob_accel_sample_rate sampling frequency
 * \else
 * @brief 获取加速度计流配置的采样频率
 *
 * @param[in] profile 流配置对象, 如果该配置不是加速度计流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return ob_accel_sample_rate 采样频率
 * \endif
 */
ob_accel_sample_rate ob_accel_stream_profile_sample_rate(ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Get scale range of gyroscope stream
 *
 * @param[in] profile  Stream configuration object, if the configuration is not a gyroscope stream configuration, an error will be returned
 * @param[out] error Log error messages
 * @return ob_gyro_full_scale_range range
 * \else
 * @brief 获取陀螺仪流配置的量程范围
 *
 * @param[in] profile 流配置对象, 如果该配置不是陀螺仪流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return ob_gyro_full_scale_range 量程范围
 * \endif
 */
ob_gyro_full_scale_range ob_gyro_stream_profile_full_scale_range(ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Get the sampling frequency of the gyro flow configuration
 *
 * @param[in] profile Stream configuration object, if the configuration is not a gyroscope stream configuration, an error will be returned
 * @param[out] error Log error messages
 * @return ob_gyro_sample_rate sampling frequency
 * \else
 * @brief 获取陀螺仪流配置的采样频率
 *
 * @param[in] profile 流配置对象, 如果该配置不是陀螺仪流配置，将会返错误
 * @param[out] error 记录错误信息
 * @return ob_gyro_sample_rate 采样频率
 * \endif
 */
ob_gyro_sample_rate ob_gyro_stream_profile_sample_rate(ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Match the corresponding ob_stream_profile through the passed parameters. If there are multiple matches,
 * the first one in the list will be returned by default. If no matched profile found, will return error!
 *
 * @param profile_list Resolution list
 * @param width Width, if you don't need to add matching conditions, you can pass 0
 * @param height Height, if you don't need to add matching conditions, you can pass 0
 * @param format Format, if you don't need to add matching conditions, you can pass OB_FORMAT_UNKNOWN
 * @param fps 	Frame rate, if you don't need to add matching conditions, you can pass 0
 * @param[out] error Log error messages
 * @return ob_stream_profile* returns the matching profile
 * \else
 * @brief 通过传入的参数进行匹配对应的ob_stream_profile。若有多个匹配项默认返回列表中的第一个, 若未找打匹配的项，则返回错误。
 *
 * @param profile_list 分辨率列表
 * @param width 宽度, 如不要求加入匹配条件，可传0
 * @param height 高度, 如不要求加入匹配条件，可传0
 * @param format 格式, 如不要求加入匹配条件，可传OB_FORMAT_UNKNOWN
 * @param fps 帧率, 如不要求加入匹配条件，可传0
 * @param[out] error 记录错误信息
 * @return ob_stream_profile* 返回匹配的profile
 * \endif
 */
ob_stream_profile *ob_stream_profile_list_get_video_stream_profile(ob_stream_profile_list *profile_list, int width, int height, ob_format format, int fps,
                                                                   ob_error **error);

/**
 * \if English
 * @brief Get the corresponding StreamProfile by subscripting
 *
 * @param[in] profile_list StreamProfile lists
 * @param[in] index
 * @param[out] error Log error messages
 * @return ob_stream_profile* returns the matching profile
 * \else
 * @brief 通过下标获取对应的StreamProfile
 *
 * @param[in] profile_list StreamProfile列表
 * @param[in] index 索引
 * @param[out] error 记录错误信息
 * @return ob_stream_profile* 返回匹配的profile
 *\endif
 */
ob_stream_profile *ob_stream_profile_list_get_profile(ob_stream_profile_list *profile_list, int index, ob_error **error);

/**
 * \if English
 * @brief Get the number of StreamProfile lists
 *
 * @param[in] profile_list StreamProfile list
 * @param[out] error Log error messages
 * @return uint32_t returns the number of StreamProfile lists
 * \else
 * @brief 获取StreamProfile列表的数量
 *
 * @param[in] profile_list StreamProfile列表
 * @param[out] error 记录错误信息
 * @return uint32_t 返回StreamProfile列表的数量
 * \endif
 */
uint32_t ob_stream_profile_list_count(ob_stream_profile_list *profile_list, ob_error **error);

/**
 * \if English
 * @brief Delete stream configuration list
 *
 * @param[in] profiles Stream configuration list
 * @param[out] error Log error messages
 * \else
 * @brief 删除流配置列表
 *
 * @param[in] profiles 流配置列表
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_stream_profile_list(ob_stream_profile_list *profile_list, ob_error **error);

/**
 *\if English
 * @brief Delete stream configuration
 *
 * @param[in] profile Stream configuration object
 * @param[out] error Log error messages
 * \else
 * @brief 删除流配置
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_stream_profile(ob_stream_profile *profile, ob_error **error);

#ifdef __cplusplus
}
#endif