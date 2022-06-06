/**
 * \if English
 * @file RecordPlayback.h
 * @brief For recording and playback functions
 * \else
 * @file RecordPlayback.h
 * @brief 用于录制和回放功能
 * \endif
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * \if English
 * @brief Create a recorder for data recording
 *
 * @param[out] error log error messages
 * @return ob_recorder* returns the recorder object
 * \else
 * @brief 创建录制器用于数据的录制
 *
 * @param[out] error 记录错误信息
 * @return ob_recorder* 返回录制器对象
 * \endif
 */
ob_recorder *ob_create_recorder(ob_error **error);

/**
 * \if English
 * @brief Create a recorder for data recording
 *
 * @param dev The device object used to create the recorder
 * @param[out] error log error messages
 * @return ob_recorder* returns the recorder object
 * \else
 * @brief 创建录制器用于数据的录制
 *
 * @param dev 用于创建录制器的设备对象
 * @param[out] error 记录错误信息
 * @return ob_recorder* 返回录制器对象
 * \endif
 */
ob_recorder *ob_create_recorder_with_device(ob_device *dev, ob_error **error);

/**
 * \if English
 * @brief delete recorder object
 *
 * @param recorder recorder object
 * @param[out] error log error messages
 * \else
 * @brief 删除录制器对象
 *
 * @param recorder 录制器对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_recorder(ob_recorder *recorder, ob_error **error);

/**
 * \if English
 * @brief start recording
 *
 * @param[in] recorder recorder object
 * @param[in] filename Recorded file name
 * @param[in] async Whether to record asynchronously
 * @param[out] error log error messages
 * \else
 * @brief 开始录制
 *
 * @param[in] recorder 录制器对象
 * @param[in] filename 录制的文件名称
 * @param[in] async 是否异步录制
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_recorder_start(ob_recorder *recorder, const char *filename, bool async, ob_error **error);

/**
 * \if English
 * @brief stop recording
 *
 * @param[in] recorder recorder object
 * @param[out] error log error messages
 * \else
 * @brief 停止录制
 *
 * @param[in] recorder 录制器对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_recorder_stop(ob_recorder *recorder, ob_error **error);

/**
 * \if English
 * @brief Write frame data to the recorder
 *
 * @param[in] recorder recorder object
 * @param[in] frame Write frame data
 * @param[out] error log error messages
 * \else
 * @brief 向录制器内写入帧数据
 *
 * @param[in] recorder 录制器对象
 * @param[in] frame 写入的帧数据
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_recorder_write_frame(ob_recorder *recorder, ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Create playback object
 * @param[in] filename Playback filename
 * @param[out] error log error messages
 * @return ob_playback* returns the playback object
 * \else
 * @brief 创建回放对象
 * @param[in] filename 回放的文件名
 * @param[out] error 记录错误信息
 * @return ob_playback* 返回回放的对象
 * \endif
 */
ob_playback *ob_create_playback(const char *filename, ob_error **error);

/**
 * \if English
 * @brief delete playback object
 *
 * @param[in] playback playback object
 * @param[out] error log error messages
 * \else
 * @brief 删除回放对象
 *
 * @param[in] playback 回放对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_playback(ob_playback *playback, ob_error **error);

/**
 * \if English
 * @brief Start playback, playback data is returned from the callback
 *
 * @param[in] playback 	playback object
 * @param[in] callback Callback for playback data
 * @param[in] user_data User data
 * @param[in] type Type of playback data
 * @param[out] error log error messages
 * \else
 * @brief 开启回放，回放数据从回调中返回
 *
 * @param[in] playback 回放对象
 * @param[in] callback 回放数据的回调
 * @param[in] user_data 用户数据
 * @param[in] type 回放数据的类型
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_playback_start(ob_playback *playback, ob_playback_callback callback, void *user_data, ob_media_type type, ob_error **error);

/**
 * \if English
 * @brief stop playback
 *
 * @param[in] playback 	playback object
 * @param[out] error log error messages
 * \else
 * @brief 停止回放
 *
 * @param[in] playback 回放对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_playback_stop(ob_playback *playback, ob_error **error);

/**
 * \if English
 * @brief Set playback state
 * @param[in] playback 	playback object
 * @param[in] callback playback status callback
 * @param[in] user_data User data
 * @param[out] error log error messages
 * \else
 * @brief 设置回放状态
 * @param[in] playback 回放对象
 * @param[in] callback 回放状态回调
 * @param[in] user_data 用户数据
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_set_playback_state_callback(ob_playback *playback, ob_media_state_callback callback, void *user_data, ob_error **error);

/**
 * \if English
 * @brief Get the device information in the recording file
 *
 * @param[in] playback 	playback object
 * @param[out] error log error messages
 * @return ob_device_info* returns device information
 * \else
 * @brief 获取录制文件内的设备信息
 *
 * @param[in] playback 回放对象
 * @param[out] error 记录错误信息
 * @return ob_device_info* returns device information
 * \endif
 */
ob_device_info *ob_playback_get_device_info(ob_playback *playback, ob_error **error);

/**
 * \if English
 * @brief Get the intrinsic and extrinsic parameter information in the recording file
 *
 * @param[in] playback playback object
 * @param[out] error log error messages
 * @return ob_camera_param returns internal and external parameter information
 * \else
 * @brief 获取录制文件内的内外参信息
 *
 * @param[in] playback 回放对象
 * @param[out] error 记录错误信息
 * @return ob_camera_param 返回的内外参信息
 * \endif
 */
ob_camera_param ob_playback_get_camera_param(ob_playback *playback, ob_error **error);

#ifdef __cplusplus
}
#endif
