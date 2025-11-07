// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file RecordPlayback.hpp
 * @brief Record and playback device-related types, including interfaces to create recording and playback devices,
 record and playback streaming data, etc.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "Device.h"

/**
 * @brief Create a recording device for the specified device with a specified file path and compression enabled.
 *
 * @param[in] device The device to record.
 * @param[in] file_path The file path to record to.
 * @param[in] compression_enabled Whether to enable compression for the recording.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return A pointer to the newly created recording device, or NULL if an error occurred.
 */
OB_EXPORT ob_record_device *ob_create_record_device(ob_device *device, const char *file_path, bool compression_enabled, ob_error **error);

/**
 * @brief Delete a recording device.
 *
 * @param[in] recorder The recording device to delete.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_record_device(ob_record_device *recorder, ob_error **error);

/**
 * @brief Pause recording on the specified recording device.
 *
 * @param[in] recorder The recording device to pause.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_record_device_pause(ob_record_device *recorder, ob_error **error);

/**
 * @brief Resume recording on the specified recording device.
 *
 * @param[in] recorder The recording device to resume.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_record_device_resume(ob_record_device *recorder, ob_error **error);

/**
 * @brief Create a playback device for the specified file path.
 *
 * @param[in] file_path The file path to playback from.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return A pointer to the newly created playback device, or NULL if an error occurred.
 */
OB_EXPORT ob_device *ob_create_playback_device(const char *file_path, ob_error **error);

/**
 * @brief Pause playback on the specified playback device.
 *
 * @param[in] player The playback device to pause.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_playback_device_pause(ob_device *player, ob_error **error);

/**
 * @brief Resume playback on the specified playback device.
 *
 * @param[in] player The playback device to resume.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_playback_device_resume(ob_device *player, ob_error **error);

/**
 * @brief Set the playback to a specified time point of the played data.
 *
 * @param[in] player The playback device to set the position for.
 * @param[in] timestamp The position to set the playback to, in milliseconds.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_playback_device_seek(ob_device *player, const uint64_t timestamp, ob_error **error);

/**
 * @brief Set the playback to a specified time point of the played data.
 *
 * @param[in] player The playback device to set the position for.
 * @param[in] rate The playback rate to set.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_playback_device_set_playback_rate(ob_device *player, const float rate, ob_error **error);

/**
 * @brief Get the current playback status of the played data.
 *
 * @param[in] player The playback device to get the status for.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return The current playback status of the played data.
 */
OB_EXPORT ob_playback_status ob_playback_device_get_current_playback_status(ob_device *player, ob_error **error);

/**
 * @brief Set a callback function to receive playback status updates.
 *
 * @param[in] player The playback device to set the callback for.
 * @param[in] callback The callback function to receive playback status updates.
 * @param[in] user_data Pointer to user data that can be passed to and retrieved from the callback function
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_playback_device_set_playback_status_changed_callback(ob_device *player, ob_playback_status_changed_callback callback, void *user_data,
                                                                       ob_error **error);

/**
 * @brief Get the current playback position of the played data.
 *
 * @param[in] player The playback device to get the position for.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return The current playback position of the played data, in milliseconds.
 */
OB_EXPORT uint64_t ob_playback_device_get_position(ob_device *player, ob_error **error);

/**
 * @brief Get the duration of the played data.
 *
 * @param[in] player The playback device to get the duration for.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return The duration of the played data, in milliseconds.
 */
OB_EXPORT uint64_t ob_playback_device_get_duration(ob_device *player, ob_error **error);

#ifdef __cplusplus
}  // extern "C"
#endif