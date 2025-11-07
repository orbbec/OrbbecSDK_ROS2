// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Convert OBFormat to " char* " type and then return.
 *
 * @param[in] type OBFormat type.
 *
 * @return OBFormat of "char*" type.
 */
OB_EXPORT const char* ob_format_type_to_string(OBFormat type);

/**
 * @brief Convert OBFrameType to " char* " type and then return.
 *
 * @param[in] type OBFrameType type.
 *
 * @return OBFrameType of "char*" type.
 */
OB_EXPORT const char* ob_frame_type_to_string(OBFrameType type);

/**
 * @brief Convert OBStreamType to " char* " type and then return.
 *
 * @param[in] type OBStreamType type.
 *
 * @return OBStreamType of "char*" type.
 */
OB_EXPORT const char* ob_stream_type_to_string(OBStreamType type);

/**
 * @brief Convert OBSensorType to " char* " type and then return.
 *
 * @param[in] type OBSensorType type.
 *
 * @return OBSensorType of "char*" type.
 */
OB_EXPORT const char* ob_sensor_type_to_string(OBSensorType type);

/**
 * @brief Convert OBIMUSampleRate to " char* " type and then return.
 *
 * @param[in] type OBIMUSampleRate type.
 *
 * @return OBIMUSampleRate of "char*" type.
 */
OB_EXPORT const char* ob_imu_rate_type_to_string(OBIMUSampleRate type);

/**
 * @brief Convert OBGyroFullScaleRange to " char* " type and then return.
 *
 * @param[in] type OBGyroFullScaleRange type.
 *
 * @return OBGyroFullScaleRange of "char*" type.
 */
OB_EXPORT const char* ob_gyro_range_type_to_string(OBGyroFullScaleRange type);

/**
 * @brief Convert OBAccelFullScaleRange to " char* " type and then return.
 *
 * @param[in] type OBAccelFullScaleRange type.
 *
 * @return OBAccelFullScaleRange of "char*" type.
 */
OB_EXPORT const char* ob_accel_range_type_to_string(OBAccelFullScaleRange type);

/**
 * @brief Convert OBLiDARScanRate to " char* " type and then return.
 *
 * @param[in] type OBLiDARScanRate type.
 * @return OBLiDARScanRate of "char*" type.
 */
OB_EXPORT const char *ob_lidar_scan_rate_type_to_string(OBLiDARScanRate type);

/**
 * @brief Convert OBFrameMetadataType to " char* " type and then return.
 *
 * @param[in] type OBFrameMetadataType type.
 *
 * @return OBFrameMetadataType of "char*" type.
 */
OB_EXPORT const char* ob_meta_data_type_to_string(OBFrameMetadataType type);

/**
 * @brief Convert OBStreamType to OBSensorType.
 *
 * @param[in] type The sensor type to convert.
 *
 * @return OBStreamType The corresponding stream type.
 */
OB_EXPORT OBStreamType ob_sensor_type_to_stream_type(OBSensorType type);

/**
 * @brief Convert OBFormat to " char* " type and then return.
 *
 * @param[in] format The OBFormat to convert.
 *
 * @return The string.
 */
OB_EXPORT const char *ob_format_to_string(OBFormat format);
#ifdef __cplusplus
}
#endif
