// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

/**
 * @file ObTypes.h
 * @brief Provide structs commonly used in the SDK, enumerating constant definitions.
 */

#pragma once
#if(defined(WIN32) || defined(_WIN32) || defined(WINCE))
#ifdef OB_EXPORTS
#define OB_EXTENSION_API __declspec(dllexport)
#define OB_EXTENSION_INTERNAL_API __declspec(dllexport)
#elif defined(OB2_EXPORTS)
#define OB_EXTENSION_API __declspec(dllexport)
#define OB_EXTENSION_INTERNAL_API
#else
#ifndef OB_STATIC
#define OB_EXTENSION_API __declspec(dllimport)
#define OB_EXTENSION_INTERNAL_API __declspec(dllimport)
#else
#define OB_EXTENSION_API
#define OB_EXTENSION_INTERNAL_API
#endif
#endif
#else
#ifndef OB_STATIC
#define OB_EXTENSION_API __attribute__((visibility("default")))
#define OB_EXTENSION_INTERNAL_API __attribute__((visibility("default")))
#else
#define OB_EXTENSION_API
#define OB_EXTENSION_INTERNAL_API
#endif
#endif

#if defined(__GNUC__) || defined(__clang__)
#define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED
#endif

#pragma pack(push, 1)  // struct 1-byte align

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct ContextImpl             ob_context;
typedef struct DeviceImpl              ob_device;
typedef struct DeviceInfoImpl          ob_device_info;
typedef struct DeviceListImpl          ob_device_list;
typedef struct CameraParamListImpl     ob_camera_param_list;
typedef struct SensorImpl              ob_sensor;
typedef struct SensorListImpl          ob_sensor_list;
typedef struct StreamProfileImpl       ob_stream_profile;
typedef struct StreamProfileListImpl   ob_stream_profile_list;
typedef struct CFrameImpl              ob_frame;
typedef struct FilterImpl              ob_filter;
typedef struct PipelineImpl            ob_pipeline;
typedef struct ConfigImpl              ob_config;
typedef struct RecorderImpl            ob_recorder;
typedef struct PlaybackImpl            ob_playback;
typedef struct OBDepthWorkModeListImpl ob_depth_work_mode_list;

#define OB_WIDTH_ANY 0
#define OB_HEIGHT_ANY 0
#define OB_FPS_ANY 0
#define OB_FORMAT_ANY OB_FORMAT_UNKNOWN
#define OB_PROFILE_DEFAULT 0

/**
 * @brief send data or receive data return status type
 */
typedef enum {
    HP_STATUS_OK                      = 0,      /**< success*/
    HP_STATUS_NO_DEVICE_FOUND         = 1,      /**< No device found*/
    HP_STATUS_CONTROL_TRANSFER_FAILED = 2,      /**< Transfer failed*/
    HP_STATUS_UNKNOWN_ERROR           = 0xffff, /**< Unknown error*/
} OBHPStatusCode,
    ob_hp_status_code;

/**
 * @brief the permission type of api or property
 */
typedef enum {
    OB_PERMISSION_DENY       = 0,   /**< no permission */
    OB_PERMISSION_READ       = 1,   /**< can read */
    OB_PERMISSION_WRITE      = 2,   /**< can write */
    OB_PERMISSION_READ_WRITE = 3,   /**< can read and write */
    OB_PERMISSION_ANY        = 255, /**< any situation above */
} OBPermissionType,
    ob_permission_type;

/**
 * @brief error code
 */
typedef enum {
    OB_STATUS_OK    = 0, /**< status ok */
    OB_STATUS_ERROR = 1, /**< status error */
} OBStatus,
    ob_status;

/**
 * @brief log level, the higher the level, the stronger the log filter
 */
typedef enum {
    OB_LOG_SEVERITY_DEBUG, /**< debug */
    OB_LOG_SEVERITY_INFO,  /**< information */
    OB_LOG_SEVERITY_WARN,  /**< warning */
    OB_LOG_SEVERITY_ERROR, /**< error */
    OB_LOG_SEVERITY_FATAL, /**< fatal error */
    OB_LOG_SEVERITY_OFF    /**< off (close LOG) */
} OBLogSeverity,
    ob_log_severity, DEVICE_LOG_SEVERITY_LEVEL, OBDeviceLogSeverityLevel, ob_device_log_severity_level;
#define OB_LOG_SEVERITY_NONE OB_LOG_SEVERITY_OFF
/**
 * @brief The exception types in the SDK, through the exception type, you can easily determine the specific type of error.
 * For detailed error API interface functions and error logs, please refer to the information of ob_error
 */
typedef enum {
    OB_EXCEPTION_TYPE_UNKNOWN,                 /**< Unknown error, an error not clearly defined by the SDK */
    OB_EXCEPTION_TYPE_CAMERA_DISCONNECTED,     /**< SDK device disconnection exception */
    OB_EXCEPTION_TYPE_PLATFORM,                /**< An error in the SDK adaptation platform layer means an error in the implementation of a specific system
                                                  platform */
    OB_EXCEPTION_TYPE_INVALID_VALUE,           /**< Invalid parameter type exception, need to check input parameter */
    OB_EXCEPTION_TYPE_WRONG_API_CALL_SEQUENCE, /**< Exception caused by API version mismatch */
    OB_EXCEPTION_TYPE_NOT_IMPLEMENTED,         /**< SDK and firmware have not yet implemented functions */
    OB_EXCEPTION_TYPE_IO,                      /**< SDK access IO exception error */
    OB_EXCEPTION_TYPE_MEMORY,                  /**< SDK access and use memory errors, which means that the frame fails to allocate memory */
    OB_EXCEPTION_TYPE_UNSUPPORTED_OPERATION,   /**< Unsupported operation type error by SDK or RGBD device */
} OBExceptionType,
    ob_exception_type;

/**
 * @brief The error class exposed by the SDK, users can get detailed error information according to the error
 */
typedef struct ob_error {
    ob_status         status;          ///< Describe the status code of the error, as compatible with previous customer status code requirements
    char              message[256];    ///< Describe the detailed error log
    char              function[256];   ///< Describe the name of the function where the error occurred
    char              args[256];       ///< Describes the parameters passed to the function when an error occurs. Used to check whether the parameter is wrong
    ob_exception_type exception_type;  ///< The description is the specific error type of the SDK
} ob_error;

/**
 * @brief Enumeration value describing the sensor type
 */
typedef enum {
    OB_SENSOR_UNKNOWN   = 0, /**< Unknown type sensor */
    OB_SENSOR_IR        = 1, /**< IR */
    OB_SENSOR_COLOR     = 2, /**< Color */
    OB_SENSOR_DEPTH     = 3, /**< Depth */
    OB_SENSOR_ACCEL     = 4, /**< Accel */
    OB_SENSOR_GYRO      = 5, /**< Gyro */
    OB_SENSOR_IR_LEFT   = 6, /**< left IR */
    OB_SENSOR_IR_RIGHT  = 7, /**< Right IR */
    OB_SENSOR_RAW_PHASE = 8, /**< Raw Phase */
} OBSensorType,
    ob_sensor_type;

/**
 * @brief Enumeration value describing the type of data stream
 */
typedef enum {
    OB_STREAM_UNKNOWN   = -1, /**< Unknown type stream */
    OB_STREAM_VIDEO     = 0,  /**< Video stream (infrared, color, depth streams are all video streams) */
    OB_STREAM_IR        = 1,  /**< IR stream */
    OB_STREAM_COLOR     = 2,  /**< color stream */
    OB_STREAM_DEPTH     = 3,  /**< depth stream */
    OB_STREAM_ACCEL     = 4,  /**< Accelerometer data stream */
    OB_STREAM_GYRO      = 5,  /**< Gyroscope data stream */
    OB_STREAM_IR_LEFT   = 6,  /**< Left IR stream */
    OB_STREAM_IR_RIGHT  = 7,  /**< Right IR stream */
    OB_STREAM_RAW_PHASE = 8,  /**< RawPhase Stream */
} OBStreamType,
    ob_stream_type;

/**
 * @brief Enumeration value describing the type of frame
 */
typedef enum {
    OB_FRAME_UNKNOWN   = -1, /**< Unknown frame type */
    OB_FRAME_VIDEO     = 0,  /**< Video frame */
    OB_FRAME_IR        = 1,  /**< IR frame */
    OB_FRAME_COLOR     = 2,  /**< Color frame */
    OB_FRAME_DEPTH     = 3,  /**< Depth frame */
    OB_FRAME_ACCEL     = 4,  /**< Accelerometer data frame */
    OB_FRAME_SET       = 5,  /**< Frame collection (internally contains a variety of data frames) */
    OB_FRAME_POINTS    = 6,  /**< Point cloud frame */
    OB_FRAME_GYRO      = 7,  /**< Gyroscope data frame */
    OB_FRAME_IR_LEFT   = 8,  /**< Left IR frame */
    OB_FRAME_IR_RIGHT  = 9,  /**< Right IR frame */
    OB_FRAME_RAW_PHASE = 10, /**< Rawphase frame*/
} OBFrameType,
    ob_frame_type;

/**
 * @brief Enumeration value describing the pixel format
 */
typedef enum {
    OB_FORMAT_YUYV       = 0,    /**< YUYV format */
    OB_FORMAT_YUY2       = 1,    /**< YUY2 format (the actual format is the same as YUYV) */
    OB_FORMAT_UYVY       = 2,    /**< UYVY format */
    OB_FORMAT_NV12       = 3,    /**< NV12 format */
    OB_FORMAT_NV21       = 4,    /**< NV21 format */
    OB_FORMAT_MJPG       = 5,    /**< MJPEG encoding format */
    OB_FORMAT_H264       = 6,    /**< H.264 encoding format */
    OB_FORMAT_H265       = 7,    /**< H.265 encoding format */
    OB_FORMAT_Y16        = 8,    /**< Y16 format, single channel 16-bit depth */
    OB_FORMAT_Y8         = 9,    /**< Y8 format, single channel 8-bit depth */
    OB_FORMAT_Y10        = 10,   /**< Y10 format, single channel 10-bit depth (SDK will unpack into Y16 by default) */
    OB_FORMAT_Y11        = 11,   /**< Y11 format, single channel 11-bit depth (SDK will unpack into Y16 by default) */
    OB_FORMAT_Y12        = 12,   /**< Y12 format, single channel 12-bit depth (SDK will unpack into Y16 by default) */
    OB_FORMAT_GRAY       = 13,   /**< GRAY (the actual format is the same as YUYV) */
    OB_FORMAT_HEVC       = 14,   /**< HEVC encoding format (the actual format is the same as H265) */
    OB_FORMAT_I420       = 15,   /**< I420 format */
    OB_FORMAT_ACCEL      = 16,   /**< Acceleration data format */
    OB_FORMAT_GYRO       = 17,   /**< Gyroscope data format */
    OB_FORMAT_POINT      = 19,   /**< XYZ 3D coordinate point format */
    OB_FORMAT_RGB_POINT  = 20,   /**< XYZ 3D coordinate point format with RGB information */
    OB_FORMAT_RLE        = 21,   /**< RLE pressure test format (SDK will be unpacked into Y16 by default) */
    OB_FORMAT_RGB        = 22,   /**< RGB format (actual RGB888)  */
    OB_FORMAT_BGR        = 23,   /**< BGR format (actual BGR888) */
    OB_FORMAT_Y14        = 24,   /**< Y14 format, single channel 14-bit depth (SDK will unpack into Y16 by default) */
    OB_FORMAT_BGRA       = 25,   /**< BGRA format */
    OB_FORMAT_COMPRESSED = 26,   /**< Compression format */
    OB_FORMAT_RVL        = 27,   /**< RVL pressure test format (SDK will be unpacked into Y16 by default) */
    OB_FORMAT_UNKNOWN    = 0xff, /**< unknown format */
} OBFormat,
    ob_format;

#define OB_FORMAT_RGB888 OB_FORMAT_RGB  // Alias of OB_FORMAT_RGB for compatibility
#define OB_FORMAT_MJPEG OB_FORMAT_MJPG  // Alias of OB_FORMAT_MJPG for compatibility

/**
 * @brief Enumeration value describing the firmware upgrade status
 */
typedef enum {
    STAT_VERIFY_SUCCESS = 5,  /**< Image file verifify success */
    STAT_FILE_TRANSFER  = 4,  /**< file transfer */
    STAT_DONE           = 3,  /**< update completed */
    STAT_IN_PROGRESS    = 2,  /**< upgrade in process */
    STAT_START          = 1,  /**< start the upgrade */
    STAT_VERIFY_IMAGE   = 0,  /**< Image file verification */
    ERR_VERIFY          = -1, /**< Verification failed */
    ERR_PROGRAM         = -2, /**< Program execution failed */
    ERR_ERASE           = -3, /**< Flash parameter failed */
    ERR_FLASH_TYPE      = -4, /**< Flash type error */
    ERR_IMAGE_SIZE      = -5, /**< Image file size error */
    ERR_OTHER           = -6, /**< other errors */
    ERR_DDR             = -7, /**< DDR access error */
    ERR_TIMEOUT         = -8  /**< timeout error */
} OBUpgradeState,
    ob_upgrade_state;

/**
 * @brief Enumeration value describing the file transfer status
 */
typedef enum {
    FILE_TRAN_STAT_TRANSFER         = 2,  /**< File transfer */
    FILE_TRAN_STAT_DONE             = 1,  /**< File transfer succeeded */
    FILE_TRAN_STAT_PREPAR           = 0,  /**< Preparing */
    FILE_TRAN_ERR_DDR               = -1, /**< DDR access failed */
    FILE_TRAN_ERR_NOT_ENOUGH_SPACE  = -2, /**< Insufficient target space error */
    FILE_TRAN_ERR_PATH_NOT_WRITABLE = -3, /**< Destination path is not writable */
    FILE_TRAN_ERR_MD5_ERROR         = -4, /**< MD5 checksum error */
    FILE_TRAN_ERR_WRITE_FLASH_ERROR = -5, /**< Write flash error */
    FILE_TRAN_ERR_TIMEOUT           = -6  /**< Timeout error */
} OBFileTranState,
    ob_file_tran_state;

/**
 * @brief Enumeration value describing the data transfer status
 */
typedef enum {
    DATA_TRAN_STAT_VERIFY_DONE  = 4,  /**< data verify done */
    DATA_TRAN_STAT_STOPPED      = 3,  /**< data transfer stoped */
    DATA_TRAN_STAT_DONE         = 2,  /**< data transfer completed */
    DATA_TRAN_STAT_VERIFYING    = 1,  /**< data verifying */
    DATA_TRAN_STAT_TRANSFERRING = 0,  /**< data transferring */
    DATA_TRAN_ERR_BUSY          = -1, /**< Transmission is busy */
    DATA_TRAN_ERR_UNSUPPORTED   = -2, /**< Not supported */
    DATA_TRAN_ERR_TRAN_FAILED   = -3, /**< Transfer failed */
    DATA_TRAN_ERR_VERIFY_FAILED = -4, /**< Test failed */
    DATA_TRAN_ERR_OTHER         = -5  /**< Other errors */
} OBDataTranState,
    ob_data_tran_state;

/**
 * @brief Structure for transmitting data blocks
 */
typedef struct {
    uint8_t *data;          ///< Pointer to current block data
    uint32_t size;          ///< Length of current block data
    uint32_t offset;        ///< Offset of current data block relative to complete data
    uint32_t fullDataSize;  ///< Size of full data
} OBDataChunk, ob_data_chunk;

/**
 * @brief Structure for integer range
 */
typedef struct {
    int32_t cur;   ///< Current value
    int32_t max;   ///< Maximum value
    int32_t min;   ///< Minimum value
    int32_t step;  ///< Step value
    int32_t def;   ///< Default value
} OBIntPropertyRange, ob_int_property_range;

/**
 * @brief Structure for float range
 */
typedef struct {
    float cur;   ///< Current value
    float max;   ///< Maximum value
    float min;   ///< Minimum value
    float step;  ///< Step value
    float def;   ///< Default value
} OBFloatPropertyRange, ob_float_property_range;

/**
 * @brief Structure for boolean range
 */
typedef struct {
    bool cur;   ///< Current value
    bool max;   ///< Maximum value
    bool min;   ///< Minimum value
    bool step;  ///< Step value
    bool def;   ///< Default value
} OBBoolPropertyRange, ob_bool_property_range;

/**
 * @brief Structure for camera internal parameters
 */
typedef struct {
    float   fx;      ///< Focal length in x direction
    float   fy;      ///< Focal length in y direction
    float   cx;      ///< Optical center abscissa
    float   cy;      ///< Optical center ordinate
    int16_t width;   ///< Image width
    int16_t height;  ///< Image height
} OBCameraIntrinsic, ob_camera_intrinsic;

/**
 * @brief Structure for distortion parameters
 */
typedef struct {
    float k1;  ///< Radial distortion factor 1
    float k2;  ///< Radial distortion factor 2
    float k3;  ///< Radial distortion factor 3
    float k4;  ///< Radial distortion factor 4
    float k5;  ///< Radial distortion factor 5
    float k6;  ///< Radial distortion factor 6
    float p1;  ///< Tangential distortion factor 1
    float p2;  ///< Tangential distortion factor 2
} OBCameraDistortion, ob_camera_distortion;

/**
 * @brief Structure for rotation/transformation
 */
typedef struct {
    float rot[9];    ///< Rotation matrix
    float trans[3];  ///< Transformation matrix
} OBD2CTransform, ob_d2c_transform;

/**
 * @brief Structure for camera parameters
 */
typedef struct {
    OBCameraIntrinsic  depthIntrinsic;   ///< Depth camera internal parameters
    OBCameraIntrinsic  rgbIntrinsic;     ///< Color camera internal parameters
    OBCameraDistortion depthDistortion;  ///< Depth camera distortion parameters
    OBCameraDistortion rgbDistortion;    ///< Color camera distortion parameters
    OBD2CTransform     transform;        ///< Rotation/transformation matrix
    bool               isMirrored;       ///< Whether the image frame corresponding to this group of parameters is mirrored
} OBCameraParam, ob_camera_param;
/**
 * @brief Camera parameters
 */
typedef struct {
    OBCameraIntrinsic  depthIntrinsic;   ///< Depth camera internal parameters
    OBCameraIntrinsic  rgbIntrinsic;     ///< Color camera internal parameters
    OBCameraDistortion depthDistortion;  ///< Depth camera distortion parameters

    OBCameraDistortion rgbDistortion;  ///< Distortion parameters for color camera
    OBD2CTransform     transform;      ///< Rotation/transformation matrix
} OBCameraParam_V0, ob_camera_param_v0;

/**
 * @brief Configuration for depth margin filter
 */
typedef struct {
    int      margin_x_th;       ///< Horizontal threshold settings
    int      margin_y_th;       ///< Vertical threshold settings
    int      limit_x_th;        ///< Maximum horizontal threshold
    int      limit_y_th;        ///< Maximum vertical threshold
    uint32_t width;             ///< Image width
    uint32_t height;            ///< Image height
    bool     enable_direction;  ///< Set to true for horizontal and vertical, false for horizontal only
} ob_margin_filter_config, OBMarginFilterConfig;

/**
 * @brief Alignment mode
 */
typedef enum {
    ALIGN_DISABLE,     /**< Turn off alignment */
    ALIGN_D2C_HW_MODE, /**< Hardware D2C alignment mode */
    ALIGN_D2C_SW_MODE, /**< Software D2C alignment mode */
} OBAlignMode,
    ob_align_mode;

/**
 * @brief Rectangle
 */
typedef struct {
    uint32_t x;       ///< Origin coordinate x
    uint32_t y;       ///< Origin coordinate y
    uint32_t width;   ///< Rectangle width
    uint32_t height;  ///< Rectangle height
} OBRect, ob_rect;

/**
 * @brief Enumeration of format conversion types
 */
typedef enum {
    FORMAT_YUYV_TO_RGB888 = 0, /**< YUYV to RGB888 */
    FORMAT_I420_TO_RGB888,     /**< I420 to RGB888 */
    FORMAT_NV21_TO_RGB888,     /**< NV21 to RGB888 */
    FORMAT_NV12_TO_RGB888,     /**< NV12 to RGB888 */
    FORMAT_MJPG_TO_I420,       /**< MJPG to I420 */
    FORMAT_RGB888_TO_BGR,      /**< RGB888 to BGR */
    FORMAT_MJPG_TO_NV21,       /**< MJPG to NV21 */
    FORMAT_MJPG_TO_RGB888,     /**< MJPG to RGB888 */
    FORMAT_MJPG_TO_BGR888,     /**< MJPG to BGR888 */
    FORMAT_MJPG_TO_BGRA,       /**< MJPG to BGRA */
    FORMAT_UYVY_TO_RGB888,     /**< UYVY to RGB888 */
    FORMAT_BGR_TO_RGB,         /**< BGR to RGB */
} OBConvertFormat,
    ob_convert_format;

// DEPRECATED: Only used for old version program compatibility, will be completely deleted in subsequent iterative versions
#define FORMAT_MJPEG_TO_I420 FORMAT_MJPG_TO_I420
#define FORMAT_MJPEG_TO_NV21 FORMAT_MJPG_TO_NV21
#define FORMAT_MJPEG_TO_RGB888 FORMAT_MJPG_TO_RGB888
#define FORMAT_MJPEG_TO_BGR888 FORMAT_MJPG_TO_BGR888
#define FORMAT_MJPEG_TO_BGRA FORMAT_MJPG_TO_BGRA

/**
 * @brief Enumeration of IMU sample rate values (gyroscope or accelerometer)
 */
typedef enum {
    OB_SAMPLE_RATE_1_5625_HZ = 1, /**< 1.5625Hz */
    OB_SAMPLE_RATE_3_125_HZ,      /**< 3.125Hz */
    OB_SAMPLE_RATE_6_25_HZ,       /**< 6.25Hz */
    OB_SAMPLE_RATE_12_5_HZ,       /**< 12.5Hz */
    OB_SAMPLE_RATE_25_HZ,         /**< 25Hz */
    OB_SAMPLE_RATE_50_HZ,         /**< 50Hz */
    OB_SAMPLE_RATE_100_HZ,        /**< 100Hz */
    OB_SAMPLE_RATE_200_HZ,        /**< 200Hz */
    OB_SAMPLE_RATE_500_HZ,        /**< 500Hz */
    OB_SAMPLE_RATE_1_KHZ,         /**< 1KHz */
    OB_SAMPLE_RATE_2_KHZ,         /**< 2KHz */
    OB_SAMPLE_RATE_4_KHZ,         /**< 4KHz */
    OB_SAMPLE_RATE_8_KHZ,         /**< 8KHz */
    OB_SAMPLE_RATE_16_KHZ,        /**< 16KHz */
    OB_SAMPLE_RATE_32_KHZ,        /**< 32Hz */
} OBGyroSampleRate,
    ob_gyro_sample_rate, OBAccelSampleRate, ob_accel_sample_rate, OB_SAMPLE_RATE;

/**
 * @brief Enumeration of gyroscope ranges
 */
typedef enum {
    OB_GYRO_FS_16dps = 1, /**< 16 degrees per second */
    OB_GYRO_FS_31dps,     /**< 31 degrees per second */
    OB_GYRO_FS_62dps,     /**< 62 degrees per second */
    OB_GYRO_FS_125dps,    /**< 125 degrees per second */
    OB_GYRO_FS_250dps,    /**< 250 degrees per second */
    OB_GYRO_FS_500dps,    /**< 500 degrees per second */
    OB_GYRO_FS_1000dps,   /**< 1000 degrees per second */
    OB_GYRO_FS_2000dps,   /**< 2000 degrees per second */
} OBGyroFullScaleRange,
    ob_gyro_full_scale_range, OB_GYRO_FULL_SCALE_RANGE;

/**
 * @brief Enumeration of accelerometer ranges
 */
typedef enum {
    OB_ACCEL_FS_2g = 1, /**< 1x the acceleration of gravity */
    OB_ACCEL_FS_4g,     /**< 4x the acceleration of gravity */
    OB_ACCEL_FS_8g,     /**< 8x the acceleration of gravity */
    OB_ACCEL_FS_16g,    /**< 16x the acceleration of gravity */
} OBAccelFullScaleRange,
    ob_accel_full_scale_range, OB_ACCEL_FULL_SCALE_RANGE;

/**
 * @brief Data structures for accelerometers and gyroscopes
 */
typedef struct {
    float x;  ///< X-direction component
    float y;  ///< Y-direction component
    float z;  ///< Z-direction component
} OBAccelValue, OBGyroValue, ob_accel_value, ob_gyro_value;

/**
 * @brief Device state
 */
typedef uint64_t OBDeviceState, ob_device_state;

/**
 * @brief Temperature parameters of the device (unit: Celsius)
 */
typedef struct {
    float cpuTemp;         ///< CPU temperature
    float irTemp;          ///< IR temperature
    float ldmTemp;         ///< Laser temperature
    float mainBoardTemp;   ///< Motherboard temperature
    float tecTemp;         ///< TEC temperature
    float imuTemp;         ///< IMU temperature
    float rgbTemp;         ///< RGB temperature
    float irLeftTemp;      ///< Left IR temperature
    float irRightTemp;     ///< Right IR temperature
    float chipTopTemp;     ///< MX6600 top temperature
    float chipBottomTemp;  ///< MX6600 bottom temperature
} OBDeviceTemperature, ob_device_temperature, DEVICE_TEMPERATURE;

/**
 * @brief Enumeration for depth crop modes
 */
typedef enum {
    DEPTH_CROPPING_MODE_AUTO  = 0, /**< Automatic mode */
    DEPTH_CROPPING_MODE_CLOSE = 1, /**< Close crop */
    DEPTH_CROPPING_MODE_OPEN  = 2, /**< Open crop */
} OBDepthCroppingMode,
    ob_depth_cropping_mode, OB_DEPTH_CROPPING_MODE;

/**
 * @brief Enumeration for device types
 */
typedef enum {
    OB_STRUCTURED_LIGHT_MONOCULAR_CAMERA = 0, /**< Monocular structured light camera */
    OB_STRUCTURED_LIGHT_BINOCULAR_CAMERA = 1, /**< Binocular structured light camera */
    OB_TOF_CAMERA                        = 2, /**< Time-of-flight camera */
} OBDeviceType,
    ob_device_type, OB_DEVICE_TYPE;

/**
 * @brief Enumeration for types of media to record or playback
 */
typedef enum {
    OB_MEDIA_COLOR_STREAM    = 1,   /**< Color stream */
    OB_MEDIA_DEPTH_STREAM    = 2,   /**< Depth stream */
    OB_MEDIA_IR_STREAM       = 4,   /**< Infrared stream */
    OB_MEDIA_GYRO_STREAM     = 8,   /**< Gyroscope stream */
    OB_MEDIA_ACCEL_STREAM    = 16,  /**< Accelerometer stream */
    OB_MEDIA_CAMERA_PARAM    = 32,  /**< Camera parameter */
    OB_MEDIA_DEVICE_INFO     = 64,  /**< Device information */
    OB_MEDIA_STREAM_INFO     = 128, /**< Stream information */
    OB_MEDIA_IR_LEFT_STREAM  = 256, /**< Left infrared stream */
    OB_MEDIA_IR_RIGHT_STREAM = 512, /**< Right infrared stream */

    OB_MEDIA_ALL = OB_MEDIA_COLOR_STREAM | OB_MEDIA_DEPTH_STREAM | OB_MEDIA_IR_STREAM | OB_MEDIA_GYRO_STREAM | OB_MEDIA_ACCEL_STREAM | OB_MEDIA_CAMERA_PARAM
                   | OB_MEDIA_DEVICE_INFO | OB_MEDIA_STREAM_INFO | OB_MEDIA_IR_LEFT_STREAM | OB_MEDIA_IR_RIGHT_STREAM, /**< All media data types */
} OBMediaType,
    ob_media_type, OB_MEDIA_TYPE;

/**
 * @brief Enumeration for record playback status
 */
typedef enum {
    OB_MEDIA_BEGIN = 0, /**< Begin */
    OB_MEDIA_PAUSE,     /**< Pause */
    OB_MEDIA_RESUME,    /**< Resume */
    OB_MEDIA_END,       /**< End */
} OBMediaState,
    ob_media_state, OB_MEDIA_STATE_EM;

/**
 * @brief Enumeration for depth precision levels
 * @attention The depth precision level does not completely determine the depth unit and real precision, and the influence of the data packaging format needs to
 * be considered. The specific unit can be obtained through getValueScale() of DepthFrame
 */
typedef enum {
    OB_PRECISION_1MM,   /**< 1mm */
    OB_PRECISION_0MM8,  /**< 0.8mm */
    OB_PRECISION_0MM4,  /**< 0.4mm */
    OB_PRECISION_0MM1,  /**< 0.1mm */
    OB_PRECISION_0MM2,  /**< 0.2mm */
    OB_PRECISION_0MM5,  /**< 0.5mm */
    OB_PRECISION_0MM05, /**< 0.05mm */
    OB_PRECISION_UNKNOWN,
    OB_PRECISION_COUNT,
} OBDepthPrecisionLevel,
    ob_depth_precision_level, OB_DEPTH_PRECISION_LEVEL;

/**
 * @brief Enumeration for TOF filter scene ranges
 */
typedef enum {
    OB_TOF_FILTER_RANGE_CLOSE  = 0,   /**< Close range */
    OB_TOF_FILTER_RANGE_MIDDLE = 1,   /**< Middle range */
    OB_TOF_FILTER_RANGE_LONG   = 2,   /**< Long range */
    OB_TOF_FILTER_RANGE_DEBUG  = 100, /**< Debug range */
} OBTofFilterRange,
    ob_tof_filter_range, TOF_FILTER_RANGE;

/**
 * @brief 3D point structure in the SDK
 */
typedef struct {
    float x;  ///< X coordinate
    float y;  ///< Y coordinate
    float z;  ///< Z coordinate
} OBPoint, ob_point;

/**
 * @brief 3D point structure with color information
 */
typedef struct {
    float x;  ///< X coordinate
    float y;  ///< Y coordinate
    float z;  ///< Z coordinate
    float r;  ///< Red channel component
    float g;  ///< Green channel component
    float b;  ///< Blue channel component
} OBColorPoint, ob_color_point;

/**
 * @brief Compression mode
 */
typedef enum {
    OB_COMPRESSION_LOSSLESS = 0, /**< Lossless compression mode */
    OB_COMPRESSION_LOSSY    = 1, /**< Lossy compression mode */
} OBCompressionMode,
    ob_compression_mode, OB_COMPRESSION_MODE;

/**
 * Compression Params
 */
typedef struct {
    /**
     * Lossy compression threshold, range [0~255], recommended value is 9, the higher the threshold, the higher the compression ratio.
     */
    int threshold;
} OBCompressionParams, ob_compression_params, OB_COMPRESSION_PARAMS;

/**
 * @brief TOF Exposure Threshold
 */
typedef struct {
    int32_t upper;  ///< Upper threshold, unit: ms
    int32_t lower;  ///< Lower threshold, unit: ms
} OBTofExposureThresholdControl, ob_tof_exposure_threshold_control, TOF_EXPOSURE_THRESHOLD_CONTROL;

/**
 * @brief Sync mode
 * @deprecated This define is deprecated, please use @ref ob_multi_device_sync_mode instead
 */
typedef enum {
    /**
     * @brief Close synchronize mode
     * @brief Single device, neither process input trigger signal nor output trigger signal
     * @brief Each Sensor in a single device automatically triggers
     */
    OB_SYNC_MODE_CLOSE = 0x00,

    /**
     * @brief Standalone synchronize mode
     * @brief Single device, neither process input trigger signal nor output trigger signal
     * @brief Inside single device, RGB as Major sensor: RGB -> IR/Depth/TOF
     */
    OB_SYNC_MODE_STANDALONE = 0x01,

    /**
     * @brief Primary synchronize mode
     * @brief Primary device. Ignore process input trigger signal, only output trigger signal to secondary devices.
     * @brief Inside single device, RGB as Major sensor: RGB -> IR/Depth/TOF
     */
    OB_SYNC_MODE_PRIMARY = 0x02,

    /**
     * @brief Secondary synchronize mode
     * @brief Secondary device. Both process input trigger signal and output trigger signal to other devices.
     * @brief Different sensors in a single devices receive trigger signals respectively：ext trigger -> RGB && ext trigger -> IR/Depth/TOF
     *
     * @attention With the current Gemini 2 device set to this mode, each Sensor receives the first external trigger signal
     *     after the stream is turned on and starts timing self-triggering at the set frame rate until the stream is turned off
     */
    OB_SYNC_MODE_SECONDARY = 0x03,

    /**
     * @brief MCU Primary synchronize mode
     * @brief Primary device. Ignore process input trigger signal, only output trigger signal to secondary devices.
     * @brief Inside device, MCU is the primary signal source:  MCU -> RGB && MCU -> IR/Depth/TOF
     */
    OB_SYNC_MODE_PRIMARY_MCU_TRIGGER = 0x04,

    /**
     * @brief IR Primary synchronize mode
     * @brief Primary device. Ignore process input trigger signal, only output trigger signal to secondary devices.
     * @brief Inside device, IR is the primary signal source: IR/Depth/TOF -> RGB
     */
    OB_SYNC_MODE_PRIMARY_IR_TRIGGER = 0x05,

    /**
     * @brief Software trigger synchronize mode
     * @brief Host, triggered by software control (receive the upper computer command trigger), at the same time to the trunk output trigger signal
     * @brief Different sensors in a single machine receive trigger signals respectively: soft trigger -> RGB && soft trigger -> IR/Depth/TOF
     *
     * @attention Support product: Gemini2
     */
    OB_SYNC_MODE_PRIMARY_SOFT_TRIGGER = 0x06,

    /**
     * @brief Software trigger synchronize mode as secondary device
     * @brief The slave receives the external trigger signal (the external trigger signal comes from the soft trigger host) and outputs the trigger signal to
     * the external relay.
     * @brief Different sensors in a single machine receive trigger signals respectively：ext trigger -> RGB && ext  trigger -> IR/Depth/TOF
     */
    OB_SYNC_MODE_SECONDARY_SOFT_TRIGGER = 0x07,

    /**
     * @brief Unknown type
     */
    OB_SYNC_MODE_UNKNOWN = 0xff,

} OBSyncMode,
    ob_sync_mode, OB_SYNC_MODE;

/**
 * @brief Device synchronization configuration
 * @deprecated This structure is deprecated, please use @ref ob_multi_device_sync_config instead
 */
typedef struct {
    /**
     * @brief Device synchronize mode
     */
    OBSyncMode syncMode;

    /**
     * @brief IR Trigger signal input delay: Used to configure the delay between the IR/Depth/TOF Sensor receiving the trigger signal and starting exposure,
     * Unit: microsecond
     *
     * @attention This parameter is invalid when the synchronization MODE is set to @ref OB_SYNC_MODE_PRIMARY_IR_TRIGGER
     */
    uint16_t irTriggerSignalInDelay;

    /**
     * @brief RGB trigger signal input delay is used to configure the delay from the time when an RGB Sensor receives the trigger signal to the time when the
     * exposure starts. Unit: microsecond
     *
     * @attention This parameter is invalid when the synchronization MODE is set to @ref OB_SYNC_MODE_PRIMARY
     */
    uint16_t rgbTriggerSignalInDelay;

    /**
     * @brief Device trigger signal output delay, used to control the delay configuration of the host device to output trigger signals or the slave device to
     * output trigger signals. Unit: microsecond
     *
     * @attention This parameter is invalid when the synchronization MODE is set to @ref OB_SYNC_MODE_CLOSE or @ref OB_SYNC_MODE_STANDALONE
     */
    uint16_t deviceTriggerSignalOutDelay;

    /**
     * @brief The device trigger signal output polarity is used to control the polarity configuration of the trigger signal output from the host device or the
     * trigger signal output from the slave device
     * @brief 0: forward pulse; 1: negative pulse
     *
     * @attention This parameter is invalid when the synchronization MODE is set to @ref OB_SYNC_MODE_CLOSE or @ref OB_SYNC_MODE_STANDALONE
     */
    uint16_t deviceTriggerSignalOutPolarity;

    /**
     * @brief MCU trigger frequency, used to configure the output frequency of MCU trigger signal in MCU master mode, unit: Hz
     * @brief This configuration will directly affect the image output frame rate of the Sensor. Unit: FPS (frames per second)
     *
     * @attention This parameter is invalid only when the synchronization MODE is set to @ref OB_SYNC_MODE_PRIMARY_MCU_TRIGGER
     */
    uint16_t mcuTriggerFrequency;

    /**
     * @brief Device number. Users can mark the device with this number
     */
    uint16_t deviceId;

} OBDeviceSyncConfig, ob_device_sync_config, OB_DEVICE_SYNC_CONFIG;
/**
 * @brief Depth work mode
 */
typedef struct {
    /**
     * @brief Checksum of work mode
     */
    uint8_t checksum[16];

    /**
     * @brief Name of work mode
     */
    char name[32];
} OBDepthWorkMode, ob_depth_work_mode;

/**
 * @brief Control command protocol version number
 */
typedef struct {
    /**
     * @brief Major version number
     */
    uint8_t major;

    /**
     * @brief Minor version number
     */
    uint8_t minor;

    /**
     * @brief Patch version number
     */
    uint8_t patch;
} OBProtocolVersion, ob_protocol_version;

/**
 * @brief Command version associated with property id
 */
typedef enum {
    OB_CMD_VERSION_V0 = (uint16_t)0,  ///< Version 1.0
    OB_CMD_VERSION_V1 = (uint16_t)1,  ///< Version 2.0
    OB_CMD_VERSION_V2 = (uint16_t)2,  ///< Version 3.0
    OB_CMD_VERSION_V3 = (uint16_t)3,  ///< Version 4.0

    OB_CMD_VERSION_NOVERSION = (uint16_t)0xfffe,
    OB_CMD_VERSION_INVALID   = (uint16_t)0xffff,  ///< Invalid version
} OB_CMD_VERSION,
    OBCmdVersion, ob_cmd_version;

/**
 * @brief Internal API for future publication
 *
 * @note This data type matches OBCmdVersion of one propertyId. PropertyId has multiple OBCmdVersion, and different OBCmdVersion of this propertyId has
 * different data types. PropertyId and OBCmdVersion match only one data type. itemCount is the number of data types contained in data bytes. C language and C++
 * have differences.
 *
 * C language:
 * data's type is a uint8_t pointer, and the user parses data to the destination type.
 * itemTypeSize == 1, dataSize == itemCount;
 *
 * C++:
 * data's type is the propertyId and OBCmdVersion's data type.
 * itemTypeSize = sizeof(T), itemCount = dataSize / itemTypeSize;
 */
typedef struct OBDataBundle {
    /**
     * @brief OBCmdVersion of propertyId
     */
    OBCmdVersion cmdVersion;

    /**
     * @brief Data containing itemCount of elements
     *
     * @note void *data = new T[itemCount];
     */
    void *data;

    /**
     * @brief Data size in bytes
     *
     * @note dataSize == itemTypeSize * itemCount
     */
    uint32_t dataSize;

    /**
     * @brief Size of data item
     *
     * @note C language: itemTypeSize = 1, C++: itemTypeSize = sizeof(T)
     */
    uint32_t itemTypeSize;

    /**
     * @brief Count of data item
     *
     * @note itemCount = dataSize / itemTypeSize; 0 == dataSize % itemTypeSize;
     */
    uint32_t itemCount;
} OBDataBundle, ob_data_bundle;

/**
 * @brief IP address configuration for network devices (IPv4)
 */
typedef struct {
    /**
     * @brief DHCP status
     *
     * @note 0: static IP; 1: DHCP
     */
    uint16_t dhcp;

    /**
     * @brief IP address (IPv4, big endian: 192.168.1.10, address[0] = 192, address[1] = 168, address[2] = 1, address[3] = 10)
     */
    uint8_t address[4];

    /**
     * @brief Subnet mask (big endian)
     */
    uint8_t mask[4];

    /**
     * @brief Gateway (big endian)
     */
    uint8_t gateway[4];
} OBNetIpConfig, ob_net_ip_config, DEVICE_IP_ADDR_CONFIG;

#define OBDeviceIpAddrConfig OBNetIpConfig
#define ob_device_ip_addr_config OBNetIpConfig

/**
 * @brief Device communication mode
 */
typedef enum {
    OB_COMM_USB = 0x00,  ///< USB
    OB_COMM_NET = 0x01,  ///< Ethernet
} OBCommunicationType,
    ob_communication_type, OB_COMMUNICATION_TYPE;

/**
 * @brief USB power status
 */
typedef enum {
    OB_USB_POWER_NO_PLUGIN = 0,  ///< No plugin
    OB_USB_POWER_5V_0A9    = 1,  ///< 5V/0.9A
    OB_USB_POWER_5V_1A5    = 2,  ///< 5V/1.5A
    OB_USB_POWER_5V_3A0    = 3,  ///< 5V/3.0A
} OBUSBPowerState,
    ob_usb_power_state;

/**
 * @brief DC power status
 */
typedef enum {
    OB_DC_POWER_NO_PLUGIN = 0,  ///< No plugin
    OB_DC_POWER_PLUGIN    = 1,  ///< Plugin
} OBDCPowerState,
    ob_dc_power_state;

/**
 * @brief Rotate degree
 */
typedef enum {
    OB_ROTATE_DEGREE_0   = 0,    ///< Rotate 0
    OB_ROTATE_DEGREE_90  = 90,   ///< Rotate 90
    OB_ROTATE_DEGREE_180 = 180,  ///< Rotate 180
    OB_ROTATE_DEGREE_270 = 270,  ///< Rotate 270
} ob_rotate_degree_type,
    OBRotateDegreeType;

/**
 * @brief Power line frequency mode, for color camera anti-flicker configuration
 */
typedef enum {
    OB_POWER_LINE_FREQ_MODE_CLOSE = 0,  ///< Close
    OB_POWER_LINE_FREQ_MODE_50HZ  = 1,  ///< 50Hz
    OB_POWER_LINE_FREQ_MODE_60HZ  = 2,  ///< 60Hz
} ob_power_line_freq_mode,
    OBPowerLineFreqMode;

/**
 * @brief Frame aggregate output mode
 */
typedef enum {
    /**
     * @brief Only FrameSet that contains all types of data frames will be output
     */
    OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE = 0,

    /**
     * @brief Color Frame Require output mode
     * @brief Suitable for Color using H264, H265 and other inter-frame encoding format open stream
     *
     * @attention In this mode, the user may return null when getting a non-Color type data frame from the acquired FrameSet
     */
    OB_FRAME_AGGREGATE_OUTPUT_COLOR_FRAME_REQUIRE,

    /**
     * @brief FrameSet for any case will be output
     *
     * @attention In this mode, the user may return null when getting the specified type of data frame from the acquired FrameSet
     */
    OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION,
} OB_FRAME_AGGREGATE_OUTPUT_MODE,
    OBFrameAggregateOutputMode, ob_frame_aggregate_output_mode;

/**
 * @brief Enumeration of point cloud coordinate system types
 */
typedef enum {
    OB_LEFT_HAND_COORDINATE_SYSTEM  = 0,
    OB_RIGHT_HAND_COORDINATE_SYSTEM = 1,
} OB_COORDINATE_SYSTEM_TYPE,
    OBCoordinateSystemType, ob_coordinate_system_type;

/**
 * @brief Enumeration of device development modes
 */
typedef enum {
    /**
     * @brief User mode (default mode), which provides full camera device functionality
     */
    OB_USER_MODE = 0,

    /**
     * @brief Developer mode, which allows developers to access the operating system and software/hardware resources on the device directly
     */
    OB_DEVELOPER_MODE = 1,
} OB_DEVICE_DEVELOPMENT_MODE,
    OBDeviceDevelopmentMode, ob_device_development_mode;

/**
 * @brief The synchronization mode of the device.
 */
typedef enum {

    /**
     * @brief free run mode
     * @brief The device does not synchronize with other devices,
     * @brief The Color and Depth can be set to different frame rates.
     */
    OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN = 1 << 0,

    /**
     * @brief standalone mode
     * @brief The device does not synchronize with other devices.
     * @brief The Color and Depth should be set to same frame rates, the Color and Depth will be synchronized.
     */
    OB_MULTI_DEVICE_SYNC_MODE_STANDALONE = 1 << 1,

    /**
     * @brief primary mode
     * @brief The device is the primary device in the multi-device system, it will output the trigger signal via VSYNC_OUT pin on synchronization port by
     * default.
     * @brief The Color and Depth should be set to same frame rates, the Color and Depth will be synchronized and can be adjusted by @ref colorDelayUs, @ref
     * depthDelayUs or @ref trigger2ImageDelayUs.
     */
    OB_MULTI_DEVICE_SYNC_MODE_PRIMARY = 1 << 2,

    /**
     * @brief secondary mode
     * @brief The device is the secondary device in the multi-device system, it will receive the trigger signal via VSYNC_IN pin on synchronization port. It
     * will out the trigger signal via VSYNC_OUT pin on synchronization port by default.
     * @brief The Color and Depth should be set to same frame rates, the Color and Depth will be synchronized and can be adjusted by @ref colorDelayUs, @ref
     * depthDelayUs or @ref trigger2ImageDelayUs.
     * @brief After starting the stream, the device will wait for the trigger signal to start capturing images, and will stop capturing images when the trigger
     * signal is stopped.
     *
     * @attention The frequency of the trigger signal should be same as the frame rate of the stream profile which is set when starting the stream.
     */
    OB_MULTI_DEVICE_SYNC_MODE_SECONDARY = 1 << 3,

    /**
     * @brief secondary synced mode
     * @brief The device is the secondary device in the multi-device system, it will receive the trigger signal via VSYNC_IN pin on synchronization port. It
     * will out the trigger signal via VSYNC_OUT pin on synchronization port by default.
     * @brief The Color and Depth should be set to same frame rates, the Color and Depth will be synchronized and can be adjusted by @ref colorDelayUs, @ref
     * depthDelayUs or @ref trigger2ImageDelayUs.
     * @brief After starting the stream, the device will be immediately start capturing images, and will adjust the capture time when the trigger signal is
     * received to synchronize with the primary device. If the trigger signal is stopped, the device will still capture images.
     *
     * @attention The frequency of the trigger signal should be same as the frame rate of the stream profile which is set when starting the stream.
     */
    OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED = 1 << 4,

    /**
     * @brief software triggering mode
     * @brief The device will start one time image capture after receiving the capture command and will output the trigger signal via VSYNC_OUT pin by default.
     * The capture command can be sent form host by call @ref ob_device_trigger_capture. The number of images captured each time can be set by @ref
     * framesPerTrigger.
     * @brief The Color and Depth should be set to same frame rates, the Color and Depth will be synchronized and can be adjusted by @ref colorDelayUs, @ref
     * depthDelayUs or @ref trigger2ImageDelayUs.
     *
     * @brief The frequency of the user call @ref ob_device_trigger_capture to send the capture command multiplied by the number of frames per trigger should be
     * less than the frame rate of the stream profile which is set when starting the stream.
     */
    OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING = 1 << 5,

    /**
     * @brief hardware triggering mode
     * @brief The device will start one time image capture after receiving the trigger signal via VSYNC_IN pin on synchronization port and will output the
     * trigger signal via VSYNC_OUT pin by default. The number of images captured each time can be set by @ref framesPerTrigger.
     * @brief The Color and Depth should be set to same frame rates, the Color and Depth will be synchronized and can be adjusted by @ref colorDelayUs, @ref
     * depthDelayUs or @ref trigger2ImageDelayUs.
     *
     * @attention The frequency of the trigger signal multiplied by the number of frames per trigger should be less than the frame rate of the stream profile
     * which is set when starting the stream.
     * @attention The trigger signal input via VSYNC_IN pin on synchronization port should be ouput by other device via VSYNC_OUT pin in hardware triggering
     * mode or software triggering mode.
     * @attention Due to different models may have different signal input requirements, please do not use different models to output trigger
     * signal as input-trigger signal.
     */
    OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING = 1 << 6,

} ob_multi_device_sync_mode,
    OBMultiDeviceSyncMode;

/**
 * @brief The synchronization configuration of the device.
 */
typedef struct {
    /**
     * @brief The sync mode of the device.
     */
    ob_multi_device_sync_mode syncMode;

    /**
     * @brief The delay time of the depth image capture after receiving the capture command or trigger signal in microseconds.
     *
     * @attention This parameter is only valid for some models， please refer to the product manual for details.
     */
    int depthDelayUs;

    /**
     * @brief The delay time of the color image capture after receiving the capture command or trigger signal in microseconds.
     *
     * @attention This parameter is only valid for some models， please refer to the product manual for details.
     */
    int colorDelayUs;

    /**
     * @brief The delay time of the image capture after receiving the capture command or trigger signal in microseconds.
     * @brief The depth and color images are captured synchronously as the product design and can not change the delay between the depth and color images.
     *
     * @attention For Orbbec Astra 2 device, this parameter is valid only when the @ref triggerOutDelayUs is set to 0.
     * @attention This parameter is only valid for some models to replace @ref depthDelayUs and @ref colorDelayUs, please refer to the product manual for
     * details.
     */
    int trigger2ImageDelayUs;

    /**
     * @brief Trigger signal output enable flag.
     * @brief After the trigger signal output is enabled, the trigger signal will be output when the capture command or trigger signal is received. User can
     * adjust the delay time of the trigger signal output by @ref triggerOutDelayUs.
     *
     * @attention For some models, the trigger signal output is always enabled and cannot be disabled.
     * @attention If device is in the @ref OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN or @ref OB_MULTI_DEVICE_SYNC_MODE_STANDALONE mode, the trigger signal output is
     * always disabled. Set this parameter to true will not take effect.
     */
    bool triggerOutEnable;

    /**
     * @brief The delay time of the trigger signal output after receiving the capture command or trigger signal in microseconds.
     *
     * @attention For Orbbec Astra 2 device, only supported -1 and 0. -1 means the trigger signal output delay is automatically adjusted by the device, 0 means
     * the trigger signal output is disabled.
     */
    int triggerOutDelayUs;

    /**
     * @brief The frame number of each stream after each trigger in triggering mode.
     *
     * @attention This parameter is only valid when the triggering mode is set to @ref OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING or @ref
     * OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING.
     * @attention The trigger frequency multiplied by the number of frames per trigger cannot exceed the maximum frame rate of the stream profile which is set
     * when starting the stream.
     */
    int framesPerTrigger;
} ob_multi_device_sync_config, OBMultiDeviceSyncConfig;

/**
 * @brief The timestamp reset configuration of the device.
 *
 */
typedef struct {
    /**
     * @brief Whether to enable the timestamp reset function.
     * @brief If the timestamp reset function is enabled, the timer for calculating the timestamp for output frames will be reset to 0 when the timestamp reset
     * command or timestamp reset signal is received, and one timestamp reset signal will be output via TIMER_SYNC_OUT pin on synchronization port by default.
     * The timestamp reset signal is input via TIMER_SYNC_IN pin on the synchronization port.
     *
     * @attention For some models, the timestamp reset function is always enabled and cannot be disabled.
     */
    bool enable;

    /**
     * @brief The delay time of executing the timestamp reset function after receiving the command or signal in microseconds.
     */
    int timestamp_reset_delay_us;

    /**
     * @brief the timestamp reset signal output enable flag.
     *
     * @attention For some models, the timestamp reset signal output is always enabled and cannot be disabled.
     */
    bool timestamp_reset_signal_output_enable;
} ob_device_timestamp_reset_config, OBDeviceTimestampResetConfig;

/**
 * @brief Baseline calibration parameters
 */
typedef struct {
    /**
     * @brief Baseline length
     */
    float baseline;
    /**
     * @brief Calibration distance
     */
    float zpd;
} BASELINE_CALIBRATION_PARAM, ob_baseline_calibration_param, OBBaselineCalibrationParam;

/**
 * @brief Callback for file transfer
 *
 * @param state Transmission status
 * @param message Transfer status information
 * @param percent Transfer progress percentage
 * @param user_data User-defined data
 */
typedef void (*ob_file_send_callback)(ob_file_tran_state state, const char *message, uint8_t percent, void *user_data);

/**
 * @brief Callback for firmware upgrade
 *
 * @param state Upgrade status
 * @param message Upgrade status information
 * @param percent Upgrade progress percentage
 * @param user_data User-defined data
 */
typedef void (*ob_device_upgrade_callback)(ob_upgrade_state state, const char *message, uint8_t percent, void *user_data);

/**
 * @brief Callback for device status
 *
 * @param state Device status
 * @param message Device status information
 * @param user_data User-defined data
 */
typedef void (*ob_device_state_callback)(ob_device_state state, const char *message, void *user_data);

/**
 * @brief Callback for writing data
 *
 * @param state Write data status
 * @param percent Write data percentage
 * @param user_data User-defined data
 */
typedef void (*ob_set_data_callback)(ob_data_tran_state state, uint8_t percent, void *user_data);

/**
 * @brief Callback for reading data
 *
 * @param state Read data status
 * @param dataChunk Read the returned data block
 * @param user_data User-defined data
 */
typedef void (*ob_get_data_callback)(ob_data_tran_state state, ob_data_chunk *dataChunk, void *user_data);

/**
 * @brief Callback for media status (recording and playback)
 *
 * @param state Condition
 * @param user_data User-defined data
 */
typedef void (*ob_media_state_callback)(ob_media_state state, void *user_data);

/**
 * @brief Callback for device change
 *
 * @param removed List of deleted (dropped) devices
 * @param added List of added (online) devices
 * @param user_data User-defined data
 */
typedef void (*ob_device_changed_callback)(ob_device_list *removed, ob_device_list *added, void *user_data);

// typedef void (*ob_net_device_added_callback)(const char *added, void *user_data);
// typedef void (*ob_net_device_removed_callback)(const char *removed, void *user_data);

/**
 * @brief Callback for frame
 *
 * @param frame Frame object
 * @param user_data User-defined data
 */
typedef void (*ob_frame_callback)(ob_frame *frame, void *user_data);
#define ob_filter_callback ob_frame_callback
#define ob_playback_callback ob_frame_callback

/**
 * @brief Callback for frameset
 *
 * @param frameset Frameset object
 * @param user_data User-defined data
 */
typedef void (*ob_frameset_callback)(ob_frame *frameset, void *user_data);

/**
 * @brief Customize the delete callback
 *
 * @param buffer Data that needs to be deleted
 * @param user_data User-defined data
 */
typedef void(ob_frame_destroy_callback)(void *buffer, void *user_data);

/**
 * @brief Callback for receiving log
 *
 * @param severity Current log level
 * @param message Log message
 * @param user_data User-defined data
 */
typedef void(ob_log_callback)(ob_log_severity severity, const char *message, void *user_data);

typedef void(ob_get_imu_data_callback)(const uint8_t *data, uint32_t dataLen);

/**
 * @brief Check if sensor_type is an IR sensor
 *
 * @param sensor_type Sensor type to check
 * @return True if sensor_type is an IR sensor, false otherwise
 */
#define is_ir_sensor(sensor_type) (sensor_type == OB_SENSOR_IR || sensor_type == OB_SENSOR_IR_LEFT || sensor_type == OB_SENSOR_IR_RIGHT)
#define isIRSensor is_ir_sensor

/**
 * @brief Check if stream_type is an IR stream
 *
 * @param stream_type Stream type to check
 * @return True if stream_type is an IR stream, false otherwise
 */
#define is_ir_stream(stream_type) (stream_type == OB_STREAM_IR || stream_type == OB_STREAM_IR_LEFT || stream_type == OB_STREAM_IR_RIGHT)
#define isIRStream is_ir_stream

/**
 * @brief Check if frame_type is an IR frame
 *
 * @param frame_type Frame type to check
 * @return True if frame_type is an IR frame, false otherwise
 */
#define is_ir_frame(frame_type) (frame_type == OB_FRAME_IR || frame_type == OB_FRAME_IR_LEFT || frame_type == OB_FRAME_IR_RIGHT)
#define isIRFrame is_ir_frame

/**
 * @brief The default Decrypt Key
 */
#define OB_DEFAULT_DECRYPT_KEY (nullptr)

#ifdef __cplusplus
}
#endif

#pragma pack(pop)
