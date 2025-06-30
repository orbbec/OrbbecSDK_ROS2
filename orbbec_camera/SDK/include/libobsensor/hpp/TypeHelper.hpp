// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once

#include <string>
#include <iostream>
#include "libobsensor/h/ObTypes.h"
#include "libobsensor/h/TypeHelper.h"

#include <functional>

namespace ob {
class TypeHelper {
public:
    /**
     * @brief Convert OBFormat to " string " type and then return.
     *
     * @param[in] type OBFormat type.
     * @return OBFormat of "string" type.
     */
    static std::string convertOBFormatTypeToString(const OBFormat &type) {
        return ob_format_type_to_string(type);
    }

    /**
     * @brief Convert OBFrameType to " string " type and then return.
     *
     * @param[in] type OBFrameType type.
     * @return OBFrameType of "string" type.
     */
    static std::string convertOBFrameTypeToString(const OBFrameType &type) {
        return ob_frame_type_to_string(type);
    }

    /**
     * @brief Convert OBStreamType to " string " type and then return.
     *
     * @param[in] type OBStreamType type.
     * @return OBStreamType of "string" type.
     */
    static std::string convertOBStreamTypeToString(const OBStreamType &type) {
        return ob_stream_type_to_string(type);
    }

    /**
     * @brief Convert OBSensorType to " string " type and then return.
     *
     * @param[in] type OBSensorType type.
     * @return OBSensorType of "string" type.
     */
    static std::string convertOBSensorTypeToString(const OBSensorType &type) {
        return ob_sensor_type_to_string(type);
    }

    /**
     * @brief Convert OBIMUSampleRate to " string " type and then return.
     *
     * @param[in] type OBIMUSampleRate type.
     * @return OBIMUSampleRate of "string" type.
     */
    static std::string convertOBIMUSampleRateTypeToString(const OBIMUSampleRate &type) {
        return ob_imu_rate_type_to_string(type);
    }

    /**
     * @brief Convert OBGyroFullScaleRange to " string " type and then return.
     *
     * @param[in] type OBGyroFullScaleRange type.
     * @return OBGyroFullScaleRange of "string" type.
     */
    static std::string convertOBGyroFullScaleRangeTypeToString(const OBGyroFullScaleRange &type) {
        return ob_gyro_range_type_to_string(type);
    }

    /**
     * @brief Convert OBAccelFullScaleRange to " string " type and then return.
     *
     * @param[in] type OBAccelFullScaleRange type.
     * @return OBAccelFullScaleRange of "string" type.
     */
    static std::string convertOBAccelFullScaleRangeTypeToString(const OBAccelFullScaleRange &type) {
        return ob_accel_range_type_to_string(type);
    }

    /**
     * @brief Convert OBFrameMetadataType to " string " type and then return.
     *
     * @param[in] type OBFrameMetadataType type.
     * @return OBFrameMetadataType of "string" type.
     */
    static std::string convertOBFrameMetadataTypeToString(const OBFrameMetadataType &type) {
        return ob_meta_data_type_to_string(type);
    }

    /**
     * @brief Convert OBSensorType to OBStreamType type and then return.
     *
     * @param[in] type OBSensorType type.
     * @return OBStreamType type.
     */
    static OBStreamType convertSensorTypeToStreamType(OBSensorType type) {
        return ob_sensor_type_to_stream_type(type);
    }

    /**
     * @brief Check if the given sensor type is a video sensor.
     * @brief Video sensors are sensors that produce video frames.
     * @brief The following sensor types are considered video sensors:
     *      OB_SENSOR_COLOR,
     *      OB_SENSOR_DEPTH,
     *      OB_SENSOR_IR,
     *      OB_SENSOR_IR_LEFT,
     *      OB_SENSOR_IR_RIGHT,
     *      OB_SENSOR_CONFIDENCE,
     *
     * @param type The sensor type
     * @return true
     * @return false
     */
    static bool isVideoSensorType(OBSensorType type) {
        return ob_is_video_sensor_type(type);
    }

    /**
     * @brief Check if the given stream type is a video stream.
     * @brief Video streams are streams that contain video frames.
     * @brief The following stream types are considered video streams:
     *      OB_STREAM_VIDEO,
     *      OB_STREAM_DEPTH,
     *      OB_STREAM_COLOR,
     *      OB_STREAM_IR,
     *      OB_STREAM_IR_LEFT,
     *      OB_STREAM_IR_RIGHT,
     *      OB_STREAM_CONFIDENCE,
     *
     * @param type The stream type to check.
     * @return true if the given stream type is a video stream, false otherwise.
     */
    static bool isVideoStreamType(OBStreamType type) {
        return ob_is_video_stream_type(type);
    }
};
}  // namespace ob

