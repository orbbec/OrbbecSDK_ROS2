/**
 * @file Types.hpp
 * @brief Provides SDK structure and enumeration constant definitions (depending on libobsensor/h/ObTypes.h).
 */

#pragma once

#include "libobsensor/h/ObTypes.h"

#include <functional>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback function for file transfer status updates.
 *
 * @param state The file transfer status.
 * @param message Status information.
 * @param percent The percentage of the file that has been transferred.
 */
using SendFileCallback = std::function<void(OBFileTranState state, const char *message, uint8_t percent)>;

/**
 * @brief Callback function for device upgrade status updates.
 *
 * @param state The device upgrade status.
 * @param message Status information.
 * @param percent The percentage of the upgrade that has been completed.
 */
using DeviceUpgradeCallback = std::function<void(OBUpgradeState state, const char *message, uint8_t percent)>;

/**
 * @brief Callback function for device status updates.
 *
 * @param state The device status.
 * @param message Status information.
 */
using DeviceStateChangedCallback = std::function<void(OBDeviceState state, const char *message)>;

/**
 * @brief Callback function for getting raw data property data when data and progress callbacks are made.
 *
 * @param dataChunk The data chunk.
 * @param state The status of getting the data.
 */
using GetDataCallback = std::function<void(OBDataTranState state, OBDataChunk *dataChunk)>;

/**
 * @brief Callback function for setting the raw data property when progress callbacks are made.
 *
 * @param percent The progress percentage.
 * @param state The status of setting the data.
 */
using SetDataCallback = std::function<void(OBDataTranState state, uint8_t percent)>;

#ifdef __cplusplus
}
#endif
