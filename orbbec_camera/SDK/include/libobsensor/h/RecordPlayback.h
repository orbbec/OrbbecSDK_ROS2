/**
 * @file RecordPlayback.h
 * @brief Header file for recording and playback functions.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Create a recorder for data recording.
 *
 * @param[out] error Pointer to log error messages.
 * @return Pointer to the recorder object.
 */
ob_recorder *ob_create_recorder(ob_error **error);

/**
 * @brief Create a recorder for data recording.
 *
 * @param dev The device object used to create the recorder.
 * @param[out] error Pointer to log error messages.
 * @return Pointer to the recorder object.
 */
ob_recorder *ob_create_recorder_with_device(ob_device *dev, ob_error **error);

/**
 * @brief Delete the recorder object.
 *
 * @param recorder Pointer to the recorder object.
 * @param[out] error Pointer to log error messages.
 */
void ob_delete_recorder(ob_recorder *recorder, ob_error **error);

/**
 * @brief Start recording.
 *
 * @param[in] recorder Pointer to the recorder object.
 * @param[in] filename Recorded file name.
 * @param[in] async Whether to record asynchronously.
 * @param[out] error Pointer to log error messages.
 */
void ob_recorder_start(ob_recorder *recorder, const char *filename, bool async, ob_error **error);

/**
 * @brief Stop recording.
 *
 * @param[in] recorder Pointer to the recorder object.
 * @param[out] error Pointer to log error messages.
 */
void ob_recorder_stop(ob_recorder *recorder, ob_error **error);

/**
 * @brief Write frame data to the recorder.
 *
 * @param[in] recorder Pointer to the recorder object.
 * @param[in] frame Pointer to the frame data to write.
 * @param[out] error Pointer to log error messages.
 */
void ob_recorder_write_frame(ob_recorder *recorder, ob_frame *frame, ob_error **error);

/**
 * @brief Create a playback object.
 *
 * @param[in] filename Playback filename.
 * @param[out] error Pointer to log error messages.
 * @return Pointer to the playback object.
 */
ob_playback *ob_create_playback(const char *filename, ob_error **error);

/**
 * @brief Delete the playback object.
 *
 * @param[in] playback Pointer to the playback object.
 * @param[out] error Pointer to log error messages.
 */
void ob_delete_playback(ob_playback *playback, ob_error **error);

/**
 * @brief Start playback, with data returned from the callback.
 *
 * @param[in] playback Pointer to the playback object.
 * @param[in] callback Callback function for playback data.
 * @param[in] user_data User data.
 * @param[in] type Type of playback data.
 * @param[out] error Pointer to log error messages.
 */
void ob_playback_start(ob_playback *playback, ob_playback_callback callback, void *user_data, ob_media_type type, ob_error **error);

/**
 * @brief Stop playback.
 *
 * @param[in] playback Pointer to the playback object.
 * @param[out] error Pointer to log error messages.
 */
void ob_playback_stop(ob_playback *playback, ob_error **error);

/**
 * @brief Set the playback state.
 *
 * @param[in] playback Pointer to the playback object.
 * @param[in] callback Playback status callback function.
 * @param[in] user_data User data.
 * @param[out] error Pointer to log error messages.
 */
void ob_set_playback_state_callback(ob_playback *playback, ob_media_state_callback callback, void *user_data, ob_error **error);

/**
 * @brief Get the device information in the recording file.
 *
 * @param[in] playback Pointer to the playback object.
 * @param[out] error Pointer to log error messages.
 * @return Pointer to the device information.
 */
ob_device_info *ob_playback_get_device_info(ob_playback *playback, ob_error **error);

/**
 * @brief Get the intrinsic and extrinsic parameter information in the recording file.
 *
 * @param[in] playback Pointer to the playback object.
 * @param[out] error Pointer to log error messages.
 * @return  Camera intrinsic and extrinsic parameter.
 */
ob_camera_param ob_playback_get_camera_param(ob_playback *playback, ob_error **error);

#ifdef __cplusplus
}
#endif
