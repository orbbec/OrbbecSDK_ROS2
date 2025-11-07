// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file MultipleDevices.h
 * @brief This file contains the multiple devices related API witch is used to control the synchronization between multiple devices and the synchronization
 * between different sensor within single device.
 * @brief The synchronization between multiple devices is complex, and different models have different synchronization modes and limitations. please refer to
 * the product manual for details.
 * @brief As the Depth and Infrared are the same sensor physically, the behavior of the Infrared is same as the Depth in the synchronization mode.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"
#include "Device.h"

/**
 * @brief Get the supported multi device sync mode bitmap of the device.
 * @brief For example, if the return value is 0b00001100, it means the device supports @ref OB_MULTI_DEVICE_SYNC_MODE_PRIMARY and @ref
 * OB_MULTI_DEVICE_SYNC_MODE_SECONDARY. User can check the supported mode by the code:
 * ```c
 *   if(supported_mode_bitmap & OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN){
 *      //support OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN
 *   }
 *   if(supported_mode_bitmap & OB_MULTI_DEVICE_SYNC_MODE_STANDALONE){
 *     //support OB_MULTI_DEVICE_SYNC_MODE_STANDALONE
 *   }
 *   // and so on
 * ```
 * @param[in] device The device handle.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint16_t return the supported multi device sync mode bitmap of the device.
 */
OB_EXPORT uint16_t ob_device_get_supported_multi_device_sync_mode_bitmap(const ob_device *device, ob_error **error);

/**
 * @brief set the multi device sync configuration of the device.
 *
 * @param[in] device The device handle.
 * @param[in] config The multi device sync configuration.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_set_multi_device_sync_config(ob_device *device, const ob_multi_device_sync_config *config, ob_error **error);

/**
 * @brief get the current multi device sync configuration of the device.
 *
 * @param[in] device The device handle.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_multi_device_sync_config return the multi device sync configuration of the device.
 */
OB_EXPORT ob_multi_device_sync_config ob_device_get_multi_device_sync_config(const ob_device *device, ob_error **error);

/**
 * @brief send the capture command to the device to trigger the capture.
 * @brief The device will start one time capture after receiving the capture command when it is in the @ref OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING
 *
 * @attention The frequency of the user call this function multiplied by the number of frames per trigger should be less than the frame rate of the stream. The
 * number of frames per trigger can be set by @ref ob_multi_device_sync_config::framesPerTrigger.
 * @attention For some models, receive and execute the capture command will have a certain delay and performance consumption, so the frequency of calling this
 * function should not be too high, please refer to the product manual for the specific supported frequency.
 * @attention If the device is not in the @ref OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING mode, device will ignore the capture command.
 *
 * @param[in] device The device handle.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_trigger_capture(ob_device *device, ob_error **error);

/**
 * @brief set the timestamp reset configuration of the device.
 *
 * @param[in] device The device handle.
 * @param[in] config The timestamp reset configuration.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_set_timestamp_reset_config(ob_device *device, const ob_device_timestamp_reset_config *config, ob_error **error);

/**
 * @brief get the timestamp reset configuration of the device.
 *
 * @param[in] device The device handle.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_device_timestamp_reset_config return the timestamp reset configuration of the device.
 */
OB_EXPORT ob_device_timestamp_reset_config ob_device_get_timestamp_reset_config(ob_device *device, ob_error **error);

/**
 * @brief send the timestamp reset command to the device.
 * @brief The device will reset the timer for calculating the timestamp for output frames to 0 after receiving the timestamp reset command when the timestamp
 * reset function is enabled. The timestamp reset function can be enabled by call @ref ob_device_set_timestamp_reset_config.
 *
 * @attention If the stream of the device is started, the timestamp of the continuous frames output by the stream will jump once after the timestamp reset.
 * @attention Due to the timer of device is not high-accuracy, the timestamp of the continuous frames output by the stream will drift after a long time. User
 * can call this function periodically to reset the timer to avoid the timestamp drift, the recommended interval time is 60 minutes.
 *
 * @param[in] device The device handle.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_timestamp_reset(ob_device *device, ob_error **error);

/**
 * @brief Alias for @ref ob_device_timestamp_reset since it is more accurate.
 */
#define ob_device_timer_reset ob_device_timestamp_reset

/**
 * @brief synchronize the timer of the device with the host.
 * @brief After calling this function, the timer of the device will be synchronized with the host. User can call this function to multiple devices to
 * synchronize all timers of the devices.
 *
 * @attention If the stream of the device is started, the timestamp of the continuous frames output by the stream will may jump once after the timer sync.
 * @attention Due to the timer of device is not high-accuracy, the timestamp of the continuous frames output by the stream will drift after a long time. User
 * can call this function periodically to synchronize the timer to avoid the timestamp drift, the recommended interval time is 60 minutes.
 *
 * @param[in] device The device handle.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_timer_sync_with_host(ob_device *device, ob_error **error);

#ifdef __cplusplus
}  // extern "C"
#endif

