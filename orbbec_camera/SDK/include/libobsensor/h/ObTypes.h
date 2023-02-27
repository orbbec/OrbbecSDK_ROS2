// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

/**
 * \if English
 * @file ObTypes.h
 * @brief Provide structs commonly used in the SDK, enumerating constant definitions.
 * \else
 * @file ObTypes.h
 * @brief 提供 SDK 常用的结构体、枚举常量定义。
 * \endif
 */

#pragma once

#if(defined(WIN32) || defined(_WIN32) || defined(WINCE))
#ifdef OB_EXPORTS
#define OB_EXTENSION_API __declspec(dllexport)
#define OB_EXTENSION_INTERNAL_API __declspec(dllexport)
#else
#define OB_EXTENSION_API __declspec(dllimport)
#define OB_EXTENSION_INTERNAL_API
#endif
#else
#define OB_EXTENSION_API __attribute__((visibility("default")))
#define OB_EXTENSION_INTERNAL_API
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

/**
 * \if English
 * @brief the permission type of api or property
 * \else
 * @brief 接口 / 属性的访问权限类型
 * \endif
 */
typedef enum {
    OB_PERMISSION_DENY       = 0, /**< \if English no permission \else 无访问权限 \endif */
    OB_PERMISSION_READ       = 1, /**< \if English can read \else 可读 \endif */
    OB_PERMISSION_WRITE      = 2, /**< \if English can write \else 可写 \endif */
    OB_PERMISSION_READ_WRITE = 3, /**< \if English can read and write \else 可读写 \endif */
} OBPermissionType,
    ob_permission_type;

/**
 * \if English
 * @brief error code
 * \else
 * @brief 错误码
 * \endif
 */
typedef enum {
    OB_STATUS_OK    = 0, /**< \if English status ok \else 状态正常 \endif */
    OB_STATUS_ERROR = 1, /**< \if English status error \else 状态异常 \endif */
} OBStatus,
    ob_status;

/**
 * \if English
 * @brief log level, the higher the level, the stronger the log filter
 * \else
 * @brief log 等级, 等级越高 Log 过滤力度越大
 * \endif
 */
typedef enum {
    OB_LOG_SEVERITY_DEBUG, /**< \if English debug \else 调试 \endif */
    OB_LOG_SEVERITY_INFO,  /**< \if English information \else 信息 \endif */
    OB_LOG_SEVERITY_WARN,  /**< \if English warning \else 警告 \endif */
    OB_LOG_SEVERITY_ERROR, /**< \if English error \else 错误 \endif */
    OB_LOG_SEVERITY_FATAL, /**< \if English fatal error \else 致命错误 \endif */
    OB_LOG_SEVERITY_NONE   /**< \if English none (close LOG) \else 无 (关闭 LOG) \endif */
} OBLogSeverity,
    ob_log_severity, DEVICE_LOG_SEVERITY_LEVEL, OBDeviceLogSeverityLevel, ob_device_log_severity_level;

/**
 * \if English
 * @brief The exception types in the SDK, through the exception type, you can easily determine the specific type of error.
 * For detailed error API interface functions and error logs, please refer to the information of ob_error
 * \else
 * @brief SDK 内部的异常类型，通过异常类型，可以简单判断具体哪个类型的错误
 * 详细的错误 API 接口函数、错误日志请参考 ob_error 的信息
 * \endif
 */
typedef enum {
    OB_EXCEPTION_TYPE_UNKNOWN, /**< \if English Unknown error, an error not clearly defined by the SDK \else 未知错误，SDK 未明确定义的错误 \endif */
    OB_EXCEPTION_TYPE_CAMERA_DISCONNECTED, /**< \if English SDK device disconnection exception \else SDK 的设备断开的异常 \endif */
    OB_EXCEPTION_TYPE_PLATFORM, /**< \if English An error in the SDK adaptation platform layer means an error in the implementation of a specific system
                                   platform \else 在 SDK 适配平台层错误，代表是具体一个系统平台实现上错误 \endif */
    OB_EXCEPTION_TYPE_INVALID_VALUE, /**< \if English Invalid parameter type exception, need to check input parameter \else 无效的参数类型异常，需要检查输入参数
                                        \endif */
    OB_EXCEPTION_TYPE_WRONG_API_CALL_SEQUENCE, /**< \if English Exception caused by API version mismatch\else API 版本不匹配带来的异常 \endif */
    OB_EXCEPTION_TYPE_NOT_IMPLEMENTED, /**< \if English SDK and firmware have not yet implemented functions \else SDK 及固件还未实现功能 \endif */
    OB_EXCEPTION_TYPE_IO,              /**< \if English SDK access IO exception error \else SDK 访问 IO 异常错误 \endif */
    OB_EXCEPTION_TYPE_MEMORY,          /**< \if English SDK access and use memory errors, which means that the frame fails to allocate memory \else
                                          SDK 的访问和使用内存错误，代表桢分配内存失败 \ endif */
    OB_EXCEPTION_TYPE_UNSUPPORTED_OPERATION, /**< \if English Unsupported operation type error by SDK or RGBD device \else SDK 或 RGBD 设备不支持的操作类型错误
                                                \endif */
} OBExceptionType,
    ob_exception_type;

/**
 * \if English
 * @brief The error class exposed by the SDK, users can get detailed error information according to the erro
 * \else
 * @brief SDK 的对外暴露的错误类，用户可以根据该错误类，获取详细的错误信息
 * \endif
 */
typedef struct ob_error {
    ob_status status;    ///< \if English Describe the status code of the error, as compatible with previous customer status code requirements \else
                         ///< 描述错误的状态码，作为兼容之前客户状态码需求 \endif
    char message[256];   ///< \if English Describe the detailed error log \else 描述详细的错误日志 \endif
    char function[256];  ///< \if English Describe the name of the function where the error occurred \else 描述出现错误的函数名称 \endif
    char args[256];  ///< \if English Describes the parameters passed to the function when an error occurs. Used to check whether the parameter is wrong \else
                     ///< 描述出错时，函数传入的参数。用来检查是不是参数错 \endif
    ob_exception_type exception_type;  ///< \if English The description is the specific error type of the SDK \else 描述是 SDK 的具体错误类型 \endif
} ob_error;

/**
 * \if English
 * @brief Enumeration value describing the sensor type
 * \else
 * @brief 描述传感器类型的枚举值
 * \endif
 */
typedef enum {
    OB_SENSOR_UNKNOWN  = 0, /**< \if English Unknown type sensor \else 未知类型传感器 \endif */
    OB_SENSOR_IR       = 1, /**< \if English IR \else 红外 \endif */
    OB_SENSOR_COLOR    = 2, /**< \if English Color \else 彩色 \endif */
    OB_SENSOR_DEPTH    = 3, /**< \if English Depth \else 深度 \endif */
    OB_SENSOR_ACCEL    = 4, /**< \if English Accel \else 加速度计 \endif */
    OB_SENSOR_GYRO     = 5, /**< \if English Gyro \else 陀螺仪 \endif */
    OB_SENSOR_IR_LEFT  = 6, /**< \if English left IR \else 左红外 \endif */
    OB_SENSOR_IR_RIGHT = 7, /**< \if English Right IR \else 右红外 \endif */
} OBSensorType,
    ob_sensor_type;

/**
 * \if English
 * @brief Enumeration value describing the type of data stream
 * \else
 * @brief 描述数据流类型的枚举值
 * \endif
 */
typedef enum {
    OB_STREAM_UNKNOWN = -1, /**< \if English Unknown type stream \else 未知类型数据流 \endif */
    OB_STREAM_VIDEO =
        0, /**< \if English Video stream (infrared, color, depth streams are all video streams) \else 视频流 (红外、彩色、深度流都属于视频流) \endif */
    OB_STREAM_IR       = 1, /**< \if English IR stream \else 红外流 \endif */
    OB_STREAM_COLOR    = 2, /**< \if English color stream \else 彩色流 \endif */
    OB_STREAM_DEPTH    = 3, /**< \if English depth stream \else 深度流 \endif */
    OB_STREAM_ACCEL    = 4, /**< \if English Accelerometer data stream \else 加速度计数据流 \endif */
    OB_STREAM_GYRO     = 5, /**< \if English Gyroscope data stream \else 陀螺仪数据流 \endif */
    OB_STREAM_IR_LEFT  = 6, /**< \if English Left IR stream \else 左路红外流 \endif */
    OB_STREAM_IR_RIGHT = 7, /**< \if English Right IR stream \else 右路红外流 \endif */
} OBStreamType,
    ob_stream_type;

/**
 * \if English
 * @brief Describe the Frame type enumeration value
 * \else
 * @brief 描述 Frame 类型枚举值
 * \endif
 */
typedef enum {
    OB_FRAME_UNKNOWN = -1, /**< \if English Unknown type frame \else 未知类型数据帧 \endif */
    OB_FRAME_VIDEO   = 0,  /**< \if English Describes the Frame type enumeration value \else 视频帧 (红外、彩色、深度帧都属于视频帧) \endif */
    OB_FRAME_IR      = 1,  /**< \if English IR frame \else 红外帧 \endif */
    OB_FRAME_COLOR   = 2,  /**< \if English color frame \else 彩色帧 \endif */
    OB_FRAME_DEPTH   = 3,  /**< \if English depth frame \else 深度帧 \endif */
    OB_FRAME_ACCEL   = 4,  /**< \if English Accelerometer data frame \else 加速度计数据帧 \endif */
    OB_FRAME_SET    = 5, /**< \if English Frame collection (internally contains a variety of data frames) \else 帧集合(内部包含多种数据帧) \endif */
    OB_FRAME_POINTS = 6, /**< \if English point cloud frame \else 点云帧 \endif */
    OB_FRAME_GYRO   = 7, /**< \if English Gyroscope data frame \else 陀螺仪数据帧 \endif */
    OB_FRAME_IR_LEFT  = 8, /**< \if English Left IR frame \else 左路红外帧 \endif */
    OB_FRAME_IR_RIGHT = 9, /**< \if English Right IR frame \else 右路红外帧 \endif */
} OBFrameType,
    ob_frame_type;

/**
 * \if English
 * @brief Enumeration value describing the pixel format
 * \else
 * @brief 描述像素格式的枚举值
 * \endif
 */
typedef enum {
    OB_FORMAT_YUYV = 0,  /**< \if English YUYV format \else YUYV 格式 \endif */
    OB_FORMAT_YUY2 = 1,  /**< \if English YUY2 format (the actual format is the same as YUYV) \else YUY2 格式 (实际格式与 YUYV 相同) \endif */
    OB_FORMAT_UYVY = 2,  /**< \if English UYVY format \else UYVY 格式 \endif */
    OB_FORMAT_NV12 = 3,  /**< \if English NV12 format \else NV12 格式 \endif */
    OB_FORMAT_NV21 = 4,  /**< \if English NV21 format \else NV21 格式 \endif */
    OB_FORMAT_MJPG = 5,  /**< \if English MJPEG encoding format \else MJPEG 编码格式 \endif */
    OB_FORMAT_H264 = 6,  /**< \if English H.264 encoding format \else H.264 编码格式 \endif */
    OB_FORMAT_H265 = 7,  /**< \if English H.265 encoding format \else H.265 编码格式 \endif */
    OB_FORMAT_Y16  = 8,  /**< \if English Y16 format, single channel 16bit depth \else Y16 格式，单通道 16bit 深度 \endif */
    OB_FORMAT_Y8   = 9,  /**< \if English Y8 format, single channel 8bit depth \else Y8 格式，单通道 8bit 深度 \endif */
    OB_FORMAT_Y10  = 10, /**< \if English Y10 format, single channel 10bit depth (SDK will unpack into Y16 by default) \else
                            Y10 格式，单通道 10bit 深度 (SDK 默认会解包成 Y16) \endif */
    OB_FORMAT_Y11 = 11,  /**< \if English Y11 format, single channel 11bit depth (SDK will unpack into Y16 by default)\else
                            Y11 格式，单通道 11bit 深度 (SDK 默认会解包成 Y16) \endif */
    OB_FORMAT_Y12 = 12,  /**< \if English Y12 format, single channel 12bit depth (SDK will unpack into Y16 by default) \else
                            Y12 格式，单通道 12bit 深度 (SDK 默认会解包成 Y16) \endif */
    OB_FORMAT_GRAY = 13, /**< \if English GRAY (the actual format is the same as YUYV) \else GRAY 灰度 (实际格式与 YUYV 相同) \endif */
    OB_FORMAT_HEVC = 14, /**< \if English HEVC encoding format (the actual format is the same as H265) \else HEVC 编码格式 (实际格式与 H265 相同) \endif */
    OB_FORMAT_I420      = 15, /**< \if English I420 format \else I420 格式  \endif */
    OB_FORMAT_ACCEL     = 16, /**< \if English Acceleration data format \else 加速度数据格式  \endif */
    OB_FORMAT_GYRO      = 17, /**< \if English Gyroscope Data Format \else 陀螺仪数据格式  \endif */
    OB_FORMAT_POINT     = 19, /**< \if English xyz 3D coordinate point format \else 纯 x-y-z 三维坐标点格式  \endif */
    OB_FORMAT_RGB_POINT = 20, /**< \if English xyz 3D coordinate point format with RGB information \else 带 RGB 信息的 x-y-z 三维坐标点格式 \endif */
    OB_FORMAT_RLE = 21, /**< \if English RLE pressure test format (SDK will be unpacked into Y16 by default) \else RLE 压测格式 (SDK 默认会解包成 Y16) \endif */
    OB_FORMAT_RGB = 22,          /**< \if English RGB format (actual BRG888)  \else RGB888 格式 (实际 RGB888)  \endif */
    OB_FORMAT_BGR = 23,          /**< \if English BGR format (actual BRG888) \else BGR 格式 (实际 BRG888) \endif */
    OB_FORMAT_Y14 = 24,          /**< \if English Y14 format, single channel 14bit depth (SDK will unpack into Y16 by default) \else
                                    Y14 格式，单通道 14bit 深度 (SDK 默认会解包成 Y16) \endif */
    OB_FORMAT_BGRA       = 25,   /**< BGRA */
    OB_FORMAT_COMPRESSED = 26,   /**< \if English compressed format \else 压缩格式 \endif */
    OB_FORMAT_UNKNOWN    = 0xff, /**< \if English unknown format \else 未知格式 \endif */
} OBFormat,
    ob_format;

#define OB_FORMAT_RGB888 OB_FORMAT_RGB  // \if English Alias of OB_FORMAT_RGB for compatible \else 别名，用于兼容旧版本命名 \endif
#define OB_FORMAT_MJPEG OB_FORMAT_MJPG  // \if English Alias of OB_FORMAT_MJPG for compatible \else 别名，用于兼容旧版本命名 \endif

/**
 * \if English
 * @brief Firmware upgrade status
 * \else
 * @brief 固件升级状态
 * \endif
 */
typedef enum {
    STAT_FILE_TRANSFER = 4,  /**< \if English file transfer \else 文件传输中 \endif */
    STAT_DONE          = 3,  /**< \if English update completed \else 升级完成 \endif */
    STAT_IN_PROGRESS   = 2,  /**< \if English upgrade in process \else 升级中 \endif */
    STAT_START         = 1,  /**< \if English start the upgrade \else 开始升级 \endif */
    STAT_VERIFY_IMAGE  = 0,  /**< \if English Image file verification \else 镜像文件校验中 \endif */
    ERR_VERIFY         = -1, /**< \if English Verification failed \else 校验失败 \endif */
    ERR_PROGRAM        = -2, /**< \if English Program execution failed \else 程序执行失败 \endif */
    ERR_ERASE          = -3, /**< \if English Flash parameter failed \else Flash 参数失败 \endif */
    ERR_FLASH_TYPE     = -4, /**< \if English Flash type error \else Flash 类型错误 \endif */
    ERR_IMAGE_SIZE     = -5, /**< \if English Image file size error \else 镜像文件大小错误 \endif */
    ERR_OTHER          = -6, /**< \if English other errors \else 其他错误 \endif */
    ERR_DDR            = -7, /**< \if English DDR access error \else DDR 访问错误 \endif */
    ERR_TIMEOUT        = -8  /**< \if English timeout error \else 超时错误 \endif */
} OBUpgradeState,
    ob_upgrade_state;
/**
 * \if English
 * @brief file transfer status
 * \else
 * @brief 文件传输状态
 * \endif
 */
typedef enum {
    FILE_TRAN_STAT_TRANSFER         = 2,  /**< \if English file transfer \else 文件传输中 \endif */
    FILE_TRAN_STAT_DONE             = 1,  /**< \if English file transfer succeeded\else 文件传输成功 \endif */
    FILE_TRAN_STAT_PREPAR           = 0,  /**< \if English preparing \else 准备中 \endif */
    FILE_TRAN_ERR_DDR               = -1, /**< \if English DDR access failed \else DDR 访问失败 \endif */
    FILE_TRAN_ERR_NOT_ENOUGH_SPACE  = -2, /**< \if English Insufficient target space error \else 目标空间不足错误 \endif */
    FILE_TRAN_ERR_PATH_NOT_WRITABLE = -3, /**< \if English Destination path is not writable \else 目标路径不可写 \endif */
    FILE_TRAN_ERR_MD5_ERROR         = -4, /**< \if English MD5 checksum error \else MD5 校验错误 \endif */
    FILE_TRAN_ERR_WRITE_FLASH_ERROR = -5, /**< \if English write flash error \else 写 Flash 错误 \endif */
    FILE_TRAN_ERR_TIMEOUT           = -6  /**< \if English timeout error \else 超时错误 \endif */
} OBFileTranState,
    ob_file_tran_state;

/**
 * \if English
 * @brief data transfer status
 * \else
 * @brief 数据传输状态
 * \endif
 */
typedef enum {
    DATA_TRAN_STAT_STOPPED      = 3,  /**< \if English data transfer stoped \else 数据传输终止 \endif */
    DATA_TRAN_STAT_DONE         = 2,  /**< \if English data transfer completed \else 数据传输完成 \endif */
    DATA_TRAN_STAT_VERIFYING    = 1,  /**< \if English data verifying \else 数据校验中 \endif */
    DATA_TRAN_STAT_TRANSFERRING = 0,  /**< \if English data transferring \else 数据传输中 \endif */
    DATA_TRAN_ERR_BUSY          = -1, /**< \if English Transmission is busy \else 传输忙 \endif */
    DATA_TRAN_ERR_UNSUPPORTED   = -2, /**< \if English not support \else 不支持 \endif */
    DATA_TRAN_ERR_TRAN_FAILED   = -3, /**< \if English transfer failed \else 传输失败 \endif */
    DATA_TRAN_ERR_VERIFY_FAILED = -4, /**< \if English Test failed \else 检验失败 \endif */
    DATA_TRAN_ERR_OTHER         = -5  /**< \if English other errors \else 其他错误 \ endif */
} OBDataTranState,
    ob_data_tran_state;

/**
 * \if English
 * @brief Data block structure for data block transmission
 * \else
 * @brief 数据块结构体，用于数据分块传输
 * \endif
 */
typedef struct {
    void *   data;    ///< \if English current block data pointer \else 当前块数据指针 \endif
    uint32_t size;    ///< \if English Current block data length \else 当前块数据长度 \endif
    uint32_t offset;  ///< \if English The offset of the current data block relative to the complete data \else 当前数据块相对完整数据的偏移 \endif
    uint32_t fullDataSize;  ///< \if English full data size \else 完整数据大小 \endif
} OBDataChunk, ob_data_chunk;

/**
 * \if English
 * @brief Int range structure
 * \else
 * @brief 整形范围的结构体
 * \endif
 */
typedef struct {
    int32_t cur;   ///< \if English current value \else 当前值 \endif
    int32_t max;   ///< \if English maximum value \else 最大值 \endif
    int32_t min;   ///< \if English minimum value \else 最小值 \endif
    int32_t step;  ///< \if English step value \else 步进值 \endif
    int32_t def;   ///< \if English Default \else 默认值 \endif
} OBIntPropertyRange, ob_int_property_range;

/**
 * \if English
 * @brief Float range structure
 * \else
 * @brief 浮点型范围的结构体
 * \endif
 */
typedef struct {
    float cur;   ///< \if English current value \else 当前值 \endif
    float max;   ///< \if English maximum value \else 最大值 \endif
    float min;   ///< \if English minimum value \else 最小值 \endif
    float step;  ///< \if English step value \else 步进值 \endif
    float def;   ///< \if English default \else 默认值 \endif
} OBFloatPropertyRange, ob_float_property_range;

/**
 * \if English
 * @brief Boolean-scoped structure
 * \else
 * @brief 布尔型范围的结构体
 * \endif
 */
typedef struct {
    bool cur;   ///< \if English current value \else 当前值 \endif
    bool max;   ///< \if English maximum value \else 最大值 \endif
    bool min;   ///< \if English minimum value \else 最小值 \endif
    bool step;  ///< \if English step value \else 步进值 \endif
    bool def;   ///< \if English default \else 默认值 \endif
} OBBoolPropertyRange, ob_bool_property_range;

/**
 * \if English
 * @brief Camera internal parameters
 * \else
 * @brief 相机内参
 * \endif
 */
typedef struct {
    float   fx;      ///< \if English focal length in x direction \else x 方向焦距 \endif
    float   fy;      ///< \if English focal length in y direction \else y 方向焦距 \endif
    float   cx;      ///< \if English Optical center abscissa \else 光心横坐标 \endif
    float   cy;      ///< \if English Optical center ordinate \else 光心纵坐标 \endif
    int16_t width;   ///< \if English image width \else 图像宽度 \endif
    int16_t height;  ///< \if English image height \else 图像高度 \endif
} OBCameraIntrinsic, ob_camera_intrinsic;

/**
 * \if English
 * @brief Distortion Parameters
 * \else
 * @brief 畸变参数
 * \endif
 */
typedef struct {
    float k1;  ///< \if English Radial distortion factor 1 \else 径向畸变系数 1 \endif
    float k2;  ///< \if English Radial distortion factor 2 \else 径向畸变系数 2 \endif
    float k3;  ///< \if English Radial distortion factor 3 \else 径向畸变系数 3 \endif
    float k4;  ///< \if English Radial distortion factor 4 \else 径向畸变系数 4 \endif
    float k5;  ///< \if English Radial distortion factor 5 \else 径向畸变系数 5 \endif
    float k6;  ///< \if English Radial distortion factor 6 \else 径向畸变系数 6 \endif
    float p1;  ///< \if English Tangential distortion factor 1 \else 切向畸变系数 1 \endif
    float p2;  ///< \if English Tangential distortion factor 2 \else 切向畸变系数 2 \endif
} OBCameraDistortion, ob_camera_distortion;

/**
 * \if English
 * @brief Rotation/Transformation
 * \else
 * @brief 旋转 / 变换矩阵
 * \endif
 */
typedef struct {
    float rot[9];    ///< \if English Rotation matrix \else 旋转矩阵，行优先 \ endif
    float trans[3];  ///< \if English transformation matrix \else 变化矩阵 \endif
} OBD2CTransform, ob_d2c_transform;

/**
 * \if English
 * @brief Camera parameters
 * \else
 * @brief 相机参数
 * \endif
 */
typedef struct {
    OBCameraIntrinsic  depthIntrinsic;   ///< \if English Depth camera internal parameters \else 深度相机内参 \endif
    OBCameraIntrinsic  rgbIntrinsic;     ///< \if English Color camera internal parameters \else 彩色相机内参 \endif
    OBCameraDistortion depthDistortion;  ///< \if English Depth camera distortion parameters \else 深度相机畸变参数 \endif
    OBCameraDistortion rgbDistortion;    ///< \if English Color camera distortion parameters 1 \else 彩色相机畸变参数 \endif
    OBD2CTransform     transform;        ///< \if English rotation/transformation matrix \else 旋转 / 变换矩阵 \endif

    // TODO lumiaozi 2022 年 10 月 12 日 14:28:59 最新算法定义没有镜像参数
    bool isMirrored;  ///< \if English Whether the image frame corresponding to this group of parameters is mirrored \else 本组参数对应的图像帧是否被镜像 \endif
} OBCameraParam, ob_camera_param;

/**
 * \if English
 * @brief Camera parameters
 * \else
 * @brief 相机参数
 *      Gemini2  OBCmdVersion --> V0, 2022 年 10 月 13 日 09:09:30
 * \endif
 */
typedef struct {
    OBCameraIntrinsic  depthIntrinsic;   ///< \if English Depth camera internal parameters \else 深度相机内参 \endif
    OBCameraIntrinsic  rgbIntrinsic;     ///< \if English Color camera internal parameters \else 彩色相机内参 \endif
    OBCameraDistortion depthDistortion;  ///< \if English Depth camera distortion parameters \else 深度相机畸变参数 \endif
    OBCameraDistortion rgbDistortion;    ///< \if English Color camera distortion parameters 1 \else 彩色相机畸变参数 \endif
    OBD2CTransform     transform;        ///< \if English rotation/transformation matrix \else 旋转 / 变换矩阵 \endif
} OBCameraParam_V0, ob_camera_param_v0;

/**
 * @brief 深度对齐校验参数
 *
 */
typedef struct {
    OBCameraIntrinsic  leftIntrin;  // target ：单目结构光以及双目左 L
    OBCameraDistortion leftDisto;
    float              leftRot[9];

    OBCameraIntrinsic  rightIntrin;  // ref
    OBCameraDistortion rightDisto;
    float              rightRot[9];

    OBCameraIntrinsic leftVirtualIntrin;  // output intrinsics from rectification (and rotation)
    OBCameraIntrinsic rightVirtualIntrin;
} OBDERectifyMaskParams, OBDERectifyMaskParams_V0;

typedef struct MaskFilterConfig_S {
    float scale;
    int   margin_th_u;     // 对应在 src 图像的边缘宽度
    int   margin_th_v;     // 对应在 src 图像的边缘宽度
    int   mask_margin_th;  // 在 mask 图像的边缘宽度
    bool  enable_undisto;
} ob_mask_filter_config, OBMaskFilterConfig;

/**
 * \if English
 * @brief alignment mode
 * \else
 * @brief 对齐模式
 * \endif
 */
typedef enum {
    ALIGN_DISABLE,     /**< \if English turn off alignment \else 关闭对齐 \endif */
    ALIGN_D2C_HW_MODE, /**< \if English Hardware D2C alignment mode \else 硬件 D2C 对齐模式 \endif */
    ALIGN_D2C_SW_MODE, /**< \if English Software D2C alignment mode \else 软件 D2C 对齐模式 \endif */
} OBAlignMode,
    ob_align_mode;

/**
 * \if English
 * @brief rectangle
 * \else
 * @brief 矩形
 * \endif
 */
typedef struct {
    uint32_t x;       ///< \if English origin coordinate x \else 原点坐标 x \endif
    uint32_t y;       ///< \if English origin coordinate y \else 原点坐标 y \endif
    uint32_t width;   ///< \if English rectangle width \else 矩形宽度 \endif
    uint32_t height;  ///< \if English rectangle height \else 矩形高度 \endif
} OBRect, ob_rect;

/**
 * \if English
 * @brief format conversion type enumeration
 * \else
 * @brief 格式转换类型枚举
 * \endif
 */
typedef enum {
    FORMAT_YUYV_TO_RGB888 = 0, /**< \if English YUYV to RGB888 \else YUYV 转换为 RGB888 \endif */
    FORMAT_I420_TO_RGB888,     /**< \if English I420 to RGB888 \else I420 转换为 RGB888 \endif */
    FORMAT_NV21_TO_RGB888,     /**< \if English NV21 to RGB888 \else NV21 转换为 RGB888 \endif */
    FORMAT_NV12_TO_RGB888,     /**< \if English NV12 to RGB888 \else NV12 转换为 RGB888 \endif */
    FORMAT_MJPG_TO_I420,       /**< \if English MJPG to I420 \else MJPG 转换为 I420\endif */
    FORMAT_RGB888_TO_BGR,      /**< \if English RGB888 to BGR \else RGB888 转换为 BGR \endif */
    FORMAT_MJPG_TO_NV21,       /**< \if English MJPG to NV21 \else MJPG 转换为 NV21 \endif */
    FORMAT_MJPG_TO_RGB888,     /**< \if English MJPG to RGB888 \else MJPG 转换为 RGB888 \endif */
    FORMAT_MJPG_TO_BGR888,     /**< \if English MJPG to BGR888 \else MJPG 转换为 BGR888 \endif */
    FORMAT_MJPG_TO_BGRA,       /**< \if English MJPG to BGRA \else MJPG 转换为 BGRA \endif */
    FORMAT_UYVY_TO_RGB888,     /**< \if English UYVY to RGB888 \else MJPG 转换为 RGB888 \endif */
    FORMAT_BGR_TO_RGB,         /**< \if English BGR to RGB \else BGR 转换为 RGB \endif */
} OBConvertFormat,
    ob_convert_format;

// DEPRECATED: 仅用于旧版本程序兼容，会在后续迭代版本彻底删除
#define FORMAT_MJPEG_TO_I420 FORMAT_MJPG_TO_I420
#define FORMAT_MJPEG_TO_NV21 FORMAT_MJPG_TO_NV21
#define FORMAT_MJPEG_TO_RGB888 FORMAT_MJPG_TO_RGB888
#define FORMAT_MJPEG_TO_BGR888 FORMAT_MJPG_TO_BGR888
#define FORMAT_MJPEG_TO_BGRA FORMAT_MJPG_TO_BGRA

/**
 * \if English
 * @brief Enumeration of IMU sample rate values ​​(gyroscope or accelerometer)
 * \else
 * @brief IMU 采样率值的枚举 (陀螺仪或加速度计)
 * \endif
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
 * \if English
 * @brief Enumeration of gyroscope ranges
 * \else
 * @brief 陀螺仪量程的枚举
 * \endif
 */
typedef enum {
    OB_GYRO_FS_16dps = 1, /**< \if English 16 degrees per second \else 16 度每秒 \endif */
    OB_GYRO_FS_31dps,     /**< \if English 31 degrees per second \else 31 度每秒 \endif */
    OB_GYRO_FS_62dps,     /**< \if English 62 degrees per second \else 62 度每秒 \endif */
    OB_GYRO_FS_125dps,    /**< \if English 125 degrees per second \else 125 度每秒 \endif */
    OB_GYRO_FS_250dps,    /**< \if English 250 degrees per second \else 250 度每秒 \endif */
    OB_GYRO_FS_500dps,    /**< \if English 500 degrees per second \else 500 度每秒 \endif */
    OB_GYRO_FS_1000dps,   /**< \if English 1000 degrees per second \else 1000 度每秒 \endif */
    OB_GYRO_FS_2000dps,   /**< \if English 2000 degrees per second \else 2000 度每秒 \endif */
} OBGyroFullScaleRange,
    ob_gyro_full_scale_range, OB_GYRO_FULL_SCALE_RANGE;

/**
 * \if English
 * @brief Accelerometer range enumeration
 * \else
 * @brief 加速度计量程枚举
 * \endif
 */
typedef enum {
    OB_ACCEL_FS_2g = 1, /**< \if English 1x the acceleration of gravity \else 1 倍重力加速度 \endif */
    OB_ACCEL_FS_4g,     /**< \if English 4x the acceleration of gravity \else 4 倍重力加速度 \endif */
    OB_ACCEL_FS_8g,     /**< \if English 8x the acceleration of gravity \else 8 倍重力加速度 \endif */
    OB_ACCEL_FS_16g,    /**< \if English 16x the acceleration of gravity \else 16 倍重力加速度 \ endif */
} OBAccelFullScaleRange,
    ob_accel_full_scale_range, OB_ACCEL_FULL_SCALE_RANGE;

/**
 * \if English
 * @brief Data structures for accelerometers and gyroscopes
 *\else
 * @brief 加速度计及陀螺仪的数据结构体
 * \endif
 */
typedef struct {
    float x;  ///< \if English x-direction component \else x 方向分量 \endif
    float y;  ///< \if English y-direction component \else y 方向分量 \endif
    float z;  ///< \if English z-direction component \else z 方向分量 \endif
} OBAccelValue, OBGyroValue, ob_accel_value, ob_gyro_value;

/**
 * \if English
 * @brief Device state
 * \else
 * @brief 设备状态码
 * \endif
 */
typedef uint64_t OBDeviceState, ob_device_state;

/**
 * \if English
 * @brief Get the temperature parameters of the device (unit: Celsius)
 * \else
 * @brief 获取设备的温度参数（单位：摄氏度）
 * \endif
 */
typedef struct {
    float cpuTemp;         ///< \if English CPU temperature \else cpu 温度 \endif
    float irTemp;          ///< \if English IR temperature \else IR 温度 \endif
    float ldmTemp;         ///< \if English laser temperature \else 激光温度 \endif
    float mainBoardTemp;   ///< \if English motherboard temperature \else 主板温度 \endif
    float tecTemp;         ///< \if English TEC temperature \else TEC温度 \endif
    float imuTemp;         ///< \if English IMU temperature \else IMU温度 \endif
    float rgbTemp;         ///< \if English RGB temperature \else RGB温度 \endif
    float irLeftTemp;      ///< if English Left IR temperature \else 左IR温度 \endif
    float irRightTemp;     ///< if English Right IR temperature \else 右IR温度 \endif
    float chipTopTemp;     ///<  if English MX6600 top temperature \else MX6600 top 温度 \endif
    float chipBottomTemp;  ///<  if English MX6600 bottom temperature\else MX6600 bottom 温度 \endif
} OBDeviceTemperature, ob_device_temperature, DEVICE_TEMPERATURE;

/**
 * \if English
 * @brief Depth crop mode enumeration
 * \else
 * @brief 深度裁切模式枚举
 * \endif
 */
typedef enum {
    DEPTH_CROPPING_MODE_AUTO  = 0, /**< \if English automatic mode \else 自动模式 \endif */
    DEPTH_CROPPING_MODE_CLOSE = 1, /**< \if English close crop \else 关闭裁切 \endif */
    DEPTH_CROPPING_MODE_OPEN  = 2, /**< \if English open crop \else 打开裁切 \endif */
} OBDepthCroppingMode,
    ob_depth_cropping_mode, OB_DEPTH_CROPPING_MODE;

/**
 * \if English
 * @brief device type enumeration
 * \else
 * @brief 设备类型枚举
 * \endif
 */
typedef enum {
    OB_STRUCTURED_LIGHT_MONOCULAR_CAMERA = 0, /**< \if English Monocular structured light camera \else 单目结构光相机  \endif */
    OB_STRUCTURED_LIGHT_BINOCULAR_CAMERA = 1, /**< \if English Binocular structured light camera \else 双目结构光相机  \endif */
    OB_TOF_CAMERA                        = 2, /**< \if English TOF camera \else TOF 相机  \endif */
} OBDeviceType,
    ob_device_type, OB_DEVICE_TYPE;

/**
 * \if English
 * @brief record playback of the type of interest
 * \else
 * @brief 录制回放感兴趣数据类型
 * \endif
 */
typedef enum {
    OB_MEDIA_COLOR_STREAM    = 1,   /**< \if English color stream \else 彩色流 \ endif */
    OB_MEDIA_DEPTH_STREAM    = 2,   /**< \if English depth stream \else 深度流 \endif */
    OB_MEDIA_IR_STREAM       = 4,   /**< \if English IR stream \else 红外流 \endif */
    OB_MEDIA_GYRO_STREAM     = 8,   /**< \if English gyro stream \else 陀螺仪数据流 \endif */
    OB_MEDIA_ACCEL_STREAM    = 16,  /**< \if English accel stream \else 加速度计数据流 \endif */
    OB_MEDIA_CAMERA_PARAM    = 32,  /**< \if English camera parameter \else 相机参数 \endif */
    OB_MEDIA_DEVICE_INFO     = 64,  /**< \if English device information \else 设备信息  \endif */
    OB_MEDIA_STREAM_INFO     = 128, /**< \if English stream information \else 流信息 \endif */
    OB_MEDIA_IR_LEFT_STREAM  = 256, /**< \if English Left IR stream \else 左 IR 流 \endif */
    OB_MEDIA_IR_RIGHT_STREAM = 512, /**< \if English Right Left IR stream \else 右 IR 流 \endif */

    OB_MEDIA_ALL = OB_MEDIA_COLOR_STREAM | OB_MEDIA_DEPTH_STREAM | OB_MEDIA_IR_STREAM | OB_MEDIA_GYRO_STREAM | OB_MEDIA_ACCEL_STREAM | OB_MEDIA_CAMERA_PARAM
                   | OB_MEDIA_DEVICE_INFO | OB_MEDIA_STREAM_INFO | OB_MEDIA_IR_LEFT_STREAM
                   | OB_MEDIA_IR_RIGHT_STREAM, /**< \if English All media data types \else 所有媒体数据类型 \endif */
} OBMediaType,
    ob_media_type, OB_MEDIA_TYPE;

/**
 * \if English
 * @brief Record playback status
 * \else
 * @brief 录制回放状态
 * \endif
 */
typedef enum {
    OB_MEDIA_BEGIN = 0, /**< \if English begin \else 开始 \endif */
    OB_MEDIA_PAUSE,     /**< \if English pause \else 暂停 \endif */
    OB_MEDIA_RESUME,    /**< \if English resume \else 继续 \endif */
    OB_MEDIA_END,       /**< \if English end \else 终止  \endif */
} OBMediaState,
    ob_media_state, OB_MEDIA_STATE_EM;

/**
 * \if English
 * @brief depth accuracy class
 * @attention The depth accuracy level does not completely determine the depth unit and real accuracy, and the influence of the data packaging format needs to
 * be considered. The specific unit can be obtained through getValueScale() of DepthFrame \else
 * @brief 深度精度等级
 * @attention 深度精度等级并不完全决定深度的单位和真实精度，需要考虑数据打包格式的影响，
 * 具体单位可通过 DepthFrame 的 getValueScale() 获取
 * \endif
 */
typedef enum {
    OB_PRECISION_1MM,  /**< 1mm */
    OB_PRECISION_0MM8, /**< 0.8mm */
    OB_PRECISION_0MM4, /**< 0.4mm */
    OB_PRECISION_0MM1, /**< 0.1mm */
    OB_PRECISION_0MM2, /**< 0.2mm */
    OB_PRECISION_COUNT,
} OBDepthPrecisionLevel,
    ob_depth_precision_level, OB_DEPTH_PRECISION_LEVEL;

/**
 * \if English
 * @brief tof filter scene range
 * \else
 * @brief tof 滤波场景范围
 * \endif
 */
typedef enum {
    OB_TOF_FILTER_RANGE_CLOSE  = 0,   /**< \if English close range \else 近距离范围 \endif */
    OB_TOF_FILTER_RANGE_MIDDLE = 1,   /**< \if English middle range \else 中距离范围  \endif */
    OB_TOF_FILTER_RANGE_LONG   = 2,   /**< \if English long range \else 远距离范围 \endif */
    OB_TOF_FILTER_RANGE_DEBUG  = 100, /**< \if English debug range \else Debug 模式  \endif */
} OBTofFilterRange,
    ob_tof_filter_range, TOF_FILTER_RANGE;

/**
 * \if English
 * @brief 3D point structure in SDK
 * \else
 * @brief SDK 中 3D 点结构体
 * \endif
 */
typedef struct {
    float x;  ///< \if English x coordinate \else X 坐标 \endif
    float y;  ///< \if English y coordinate \else Y 坐标 \endif
    float z;  ///< \if English z coordinate \else Z 坐标 \endif
} OBPoint, ob_point;

/**
 * \if English
 * @brief 3D point structure with color information
 * \else
 * @brief 带有颜色信息的 3D 点结构体
 * \endif
 */
typedef struct {
    float x;  ///< \if English x coordinate \else X 坐标 \endif
    float y;  ///< \if English y coordinate \else Y 坐标 \endif
    float z;  ///< \if English z coordinate \else Z 坐标 \endif
    float r;  ///< \if English red channel component \else 红色通道分量 \endif
    float g;  ///< \if English green channel component \else 绿色通道分量 \endif
    float b;  ///< \if English blue channel component\else 蓝色通道分量 \endif
} OBColorPoint, ob_color_point;

typedef enum {
    OB_COMPRESSION_LOSSLESS = 0,
    OB_COMPRESSION_LOSSY    = 1,
} OBCompressionMode,
    ob_compression_mode, OB_COMPRESSION_MODE;

typedef struct {
    int threshold;
} OBCompressionParams, ob_compression_params, OB_COMPRESSION_PARAMS;

/**
 * \if English
 * @brief TOF Exposure Threshold
 * \else
 * @brief TOF 曝光阈值
 *\endif
 */
typedef struct {
    int32_t upper;  ///< \if English Upper threshold, unit: ms \else 阈值上限， 单位：ms \endif
    int32_t lower;  ///< \if English Lower threshold, unit: ms \else 阈值下限， 单位：ms \endif
} OBTofExposureThresholdControl, ob_tof_exposure_threshold_control, TOF_EXPOSURE_THRESHOLD_CONTROL;

/**
 * \if English
 * @brief Sync mode
 * \else
 * @brief 同步模式
 * \endif
 */
typedef enum {
    /**
     * \if English
     * @brief Close synchronize mode
     * @brief Single device, neither process input trigger signal nor output trigger signal
     * @brief Each Sensor in a single device automatically triggers
     * \else
     * @brief 同步关闭
     * @brief 单机，不接收外部触发信号，不输出触发信号
     * @brief 单机内各 Sensor 自触发
     * \endif
     *
     */
    OB_SYNC_MODE_CLOSE = 0x00,

    /**
     * \if English
     * @brief Standalone synchronize mode
     * @brief Single device, neither process input trigger signal nor output trigger signal
     * @brief Inside single device, RGB as Major sensor: RGB -> IR/Depth/TOF
     * \else
     * @brief 单机模式
     * @brief 单机，不接收外部触发信号，不输出触发信号
     * @brief 单机内 RGB 做主： RGB -> IR/Depth/TOF
     * \endif
     */
    OB_SYNC_MODE_STANDALONE = 0x01,

    /**
     * \if English
     * @brief Primary synchronize mode
     * @brief Primary device. Ignore process input trigger signal, only output trigger signal to secondary devices.
     * @brief Inside single device, RGB as Major sensor: RGB -> IR/Depth/TOF
     * \else
     * @brief 主机模式
     * @brief 主机，不接收外部触发信号，向外输出触发信号
     * @brief 单机内 RGB 做主：RGB -> IR/Depth/TOF
     *
     * @attention 部分设备型号不支持该模式： Gemini 2 设备设置该模式会自动变更为 OB_SYNC_MODE_PRIMARY_MCU_TRIGGER 模式
     *
     */
    OB_SYNC_MODE_PRIMARY = 0x02,

    /**
     * \if English
     * @brief Secondary synchronize mode
     * @brief Secondary device. Both process input trigger signal and output trigger signal to other devices.
     * @brief Different sensors in a single devices receive trigger signals respectively：ext trigger -> RGB && ext trigger -> IR/Depth/TOF
     *
     * @attention With the current Gemini 2 device set to this mode, each Sensor receives the first external trigger signal
     *     after the stream is turned on and starts timing self-triggering at the set frame rate until the stream is turned off
     * \else
     * @brief 从机模式
     * @brief 从机，接收外部触发信号，同时向外中继输出触发信号
     * @brief 单机内不同 Sensor 各自接收触发信号：ext trigger -> RGB && ext trigger -> IR/Depth/TOF
     *
     * @attention 当前 Gemini 2 设备设置为该模式后，各Sensor在开流后，接收到第一次外部触发信号即开始按照设置的帧率进行定时自触发，直到流关闭
     * \endif
     *
     */
    OB_SYNC_MODE_SECONDARY = 0x03,

    /**
     * \if English
     * @brief MCU Primary synchronize mode
     * @brief Primary device. Ignore process input trigger signal, only output trigger signal to secondary devices.
     * @brief Inside device, MCU is the primary signal source:  MCU -> RGB && MCU -> IR/Depth/TOF
     * \else
     * @brief MCU 主模式
     * @brief 主机，不接收外部触发信号，向外输出触发信号
     * @brief 单机内 MCU 做主： MCU -> RGB && MCU -> IR/Depth/TOF
     * \endif
     */
    OB_SYNC_MODE_PRIMARY_MCU_TRIGGER = 0x04,

    /**
     * \if English
     * @brief IR Primary synchronize mode
     * @brief Primary device. Ignore process input trigger signal, only output trigger signal to secondary devices.
     * @brief Inside device, IR is the primary signal source: IR/Depth/TOF -> RGB
     *
     * \else
     * @brief IR 主模式
     * @brief 主机，不接收外部触发信号，向外输出触发信号
     * @brief 单机内 IR 做主：IR/Depth/TOF -> RGB
     * \endif
     */
    OB_SYNC_MODE_PRIMARY_IR_TRIGGER = 0x05,

    /**
     * \if English
     * @brief Software trigger synchronize mode
     * @brief Host, triggered by software control (receive the upper computer command trigger), at the same time to the trunk output trigger signal
     * @brief Different sensors in a single machine receive trigger signals respectively: soft trigger -> RGB && soft trigger -> IR/Depth/TOF
     *
     * @attention Support product: Gemini2
     * \else
     * @brief 软触发模式
     * @brief 主机，由软件控制触发（接收上位机命令触发），同时向外中继输出触发信号
     * @brief 单机内不同 Sensor 各自接收触发信号：soft trigger -> RGB && soft trigger -> IR/Depth/TOF
     *
     * @attention 当前仅 Gemini2 支持该模式
     * \endif
     */
    OB_SYNC_MODE_PRIMARY_SOFT_TRIGGER = 0x06,

    /**
     * \if English
     * @brief Software trigger synchronize mode as secondary device
     * @brief The slave receives the external trigger signal (the external trigger signal comes from the soft trigger host) and outputs the trigger signal to
     * the external relay.
     * @brief Different sensors in a single machine receive trigger signals respectively：ext trigger -> RGB && ext  trigger -> IR/Depth/TOF
     * \else
     * @brief 软触发从机模式
     * @brief 从机，接收外部触发信号（外部触发信号来自软触发的主机），同时向外中继输出触发信号。
     * @brief 单机内不同 Sensor 各自接收触发信号：ext trigger -> RGB && ext  trigger -> IR/Depth/TOF
     *
     * @attention 当前仅 Gemini2 支持该模式
     * \endif
     */
    OB_SYNC_MODE_SECONDARY_SOFT_TRIGGER = 0x07,

    /**
     * \if English
     * @brief Unknown type
     * \else
     * @brief 未知类型
     * \endif
     */
    OB_SYNC_MODE_UNKNOWN = 0xff,

} OBSyncMode,
    ob_sync_mode, OB_SYNC_MODE;

// 别名，用于兼容旧版本命名
#define OB_SYNC_STOP OB_SYNC_MODE_CLOSE
#define OB_SYNC_SINGLE_MODE OB_SYNC_MODE_STANDALONE
#define OB_SYNC_ONLINE_HOST_MODE OB_SYNC_MODE_PRIMARY
#define OB_SYNC_ONLINE_SLAVE_MODE OB_SYNC_MODE_SECONDARY
#define OB_SYNC_ONLY_MCU_MODE OB_SYNC_MODE_PRIMARY_MCU_TRIGGER
#define OB_SYNC_ONLY_IR_MODE OB_SYNC_MODE_PRIMARY_IR_TRIGGER

/**
 * \if English
 * @brief Device synchronization configuration
 * \else
 * @brief 设备同步配置
 *
 * @brief 单机内不同 Sensor 的同步 及 多机间同步 配置
 * \endif
 */
typedef struct {
    /**
     * \if English
     * \else
     * @brief 同步模式
     * \endif
     *
     */
    OBSyncMode syncMode;

    /**
     * \if English
     * @brief IR Trigger signal input delay: Used to configure the delay between the IR/Depth/TOF Sensor receiving the trigger signal and starting exposure,
     * Unit: microsecond
     *
     * @attention This parameter is invalid when the synchronization MODE is set to @ref OB_SYNC_MODE_PRIMARY_IR_TRIGGER
     * \else
     * @brief IR 触发信号输入延时，用于 IR/Depth/TOF Sensor 接收到触发信号后到开始曝光的延时配置，单位为微秒
     *
     * @attention 同步模式配置为  @ref OB_SYNC_MODE_PRIMARY_IR_TRIGGER 时无效
     * \endif
     */
    uint16_t irTriggerSignalInDelay;

    /**
     * \if English
     * @brief RGB trigger signal input delay is used to configure the delay from the time when an RGB Sensor receives the trigger signal to the time when the
     * exposure starts. Unit: microsecond
     *
     * @attention This parameter is invalid when the synchronization MODE is set to @ref OB_SYNC_MODE_PRIMARY
     * \else
     * @brief RGB 触发信号输入延时，用于 RGB Sensor 接收到触发信号后到开始曝光的延时配置，单位为微秒
     *
     * @attention 同步模式配置为  @ref OB_SYNC_MODE_PRIMARY 时无效
     * \endif
     */
    uint16_t rgbTriggerSignalInDelay;

    /**
     * \if English
     * @brief Device trigger signal output delay, used to control the delay configuration of the host device to output trigger signals or the slave device to
     * output trigger signals. Unit: microsecond
     *
     * @attention This parameter is invalid when the synchronization MODE is set to @ref OB_SYNC_MODE_CLOSE or @ref OB_SYNC_MODE_STANDALONE
     * \else
     * @brief 设备触发信号输出延时，用于控制主机设备向外输 或 从机设备向外中继输出 触发信号的延时配置，单位：微秒
     *
     * @attention 同步模式配置为 @ref OB_SYNC_MODE_CLOSE 和  @ref OB_SYNC_MODE_STANDALONE 时无效
     * \endif
     */
    uint16_t deviceTriggerSignalOutDelay;

    /**
     * \if English
     * @brief The device trigger signal output polarity
     * @brief 0: forward pulse; 1: negative pulse
     *
     * @attention This parameter is invalid when the synchronization MODE is set to @ref OB_SYNC_MODE_CLOSE or @ref OB_SYNC_MODE_STANDALONE
     * \else
     * @brief 设备触发信号输出极性
     * @brief 0: 正向脉冲；1: 负向脉冲
     *
     * @attention 同步模式配置为 @ref OB_SYNC_MODE_CLOSE 和  @ref OB_SYNC_MODE_STANDALONE 时无效
     * \endif
     */
    uint16_t deviceTriggerSignalOutPolarity;

    /**
     * \if English
     * @brief MCU trigger frequency, used to configure the output frequency of MCU trigger signal in MCU master mode, unit: Hz
     * @brief This configuration will directly affect the image output frame rate of the Sensor. Unit: FPS （frame pre second）
     *
     * @attention This parameter is invalid only when the synchronization MODE is set to @ref OB_SYNC_MODE_PRIMARY_MCU_TRIGGER
     * \else
     * @brief MCU 触发频率，用于 MCU 主模式下，MCU触发信号输出频率配置，单位：Hz
     * @brief 该配置会直接影响 Sensor 的图像输出帧率，即也可以认为单位为：FPS （frame pre second）
     *
     * @attention 仅当同步模式配置为 @ref OB_SYNC_MODE_PRIMARY_MCU_TRIGGER 时无效
     * \endif
     */
    uint16_t mcuTriggerFrequency;

    /**
     * \if English
     * @brief Device number. Users can mark the device with this number
     * \else
     * @brief 设备编号，用户可用该编号对设备进行标记
     * \endif
     */
    uint16_t deviceId;
} OBDeviceSyncConfig, ob_device_sync_config, OB_DEVICE_SYNC_CONFIG;

// 别名，用于兼容旧版本命名
#define OBMultiDeviceSyncConfig OBDeviceSyncConfig
#define ob_multi_device_sync_config ob_device_sync_config
#define OB_MULTI_DEVICE_SYNC_CONFIG OB_DEVICE_SYNC_CONFIG
// #define tofPhaseDelay irTriggerSignalInDelay
// #define rgbPhaseDelay rgbTriggerSignalInDelay
// #define outPhaseDelay deviceTriggerSignalOutDelay
// #define outOCPolarity deviceTriggerSignalOutPolarity
// #define mcuHostFps mcuTriggerFrequency
// #define curDevId deviceId

/**
 * \if English
 * @brief Depth work mode
 * \else
 * @brief 相机深度工作模式
 * \endif
 *
 */
typedef struct {
    uint8_t checksum[16];  ///< \if English Checksum of work mode \else 相机深度模式对应哈希二进制数组 \endif
    char    name[32];      ///< \if English 名称 \else Name of work mode \endif
} OBDepthWorkMode, ob_depth_work_mode;

/**
 * \if English
 * @brief Command version of device property
 * \else
 * @brief 控制命令协议版本号
 * \endif
 *
 */
typedef struct {
    uint8_t major;  ///< \if English Major version number \else 主版本号 \endif
    uint8_t minor;  ///< \if English Minor version number \else 次版本号 \endif
    uint8_t patch;  ///< \if English Patch version number \else 补丁版本 \endif
} OBProtocolVersion, ob_protocol_version;

/**
 * \if English
 * Command version associate with property id
 * \else
 * 与属性ID关联的协议版本
 * \endif
 *
 */
typedef enum {
    OB_CMD_VERSION_V0 = (uint16_t)0,  ///< \if English version 1.0 \else 版本1.0 \endif
    OB_CMD_VERSION_V1 = (uint16_t)1,  ///< \if English version 2.0 \else 版本2.0 \endif
    OB_CMD_VERSION_V2 = (uint16_t)2,  ///< \if English version 3.0 \else 版本3.0 \endif
    OB_CMD_VERSION_V3 = (uint16_t)3,  ///< \if English version 4.0 \else 版本4.0 \endif

    OB_CMD_VERSION_NOVERSION = (uint16_t)0xfffe,
    OB_CMD_VERSION_INVALID   = (uint16_t)0xffff,  ///< \if English Invalid version \else 无效版本 \endif
} OB_CMD_VERSION,
    OBCmdVersion, ob_cmd_version;

/**
 * \if English
 * Internal api, publish future.
 * Description: Data type match OBCmdVersion of one propertyId
 * 1. propertyId has multiple OBCmdVersion; different OBCmdVersion of this propertyId has different data type;
 *    propertyId && OBCmdVersion match only one data type.
 * 2. itemCount is the number of data type contain in data bytes.
 * 3. C language and C++ is difference.
 *
 * C language:
 * data's type is an uint8_t pointer, user parse data to destionation type.
 * itemTypeSize == 1，dataSize == itemCount；
 *
 * C++:
 * data's type is the propertyId && OBCmdVersion's data type.
 * itemTypeSize = sizeof(T)，itemCount = dataSize / itemTypeSize;
 * \else
 * @brief OrbbecSDK私有接口, 未来对外开放
 * 功能：保存多版本数据结构的数据内容；
 * 1. 通过propertyId和cmdVersion可以决定具体的数据类型；
 * 2. 通过itemCount可以决定data内容映射的对象数量；
 *
 * 3. 由于 C 语言和 C++ 支持的特性差异，OBDataBundle 的 data 需要释放资源，导致实现上的差异
 * C 语言：
 * 由于 C 语言没有模板，因此 data 是 uint8_t 数组，itemTypeSize == 1，dataSize == itemCount；
 * C 语言需要开发者自行将 data 的 byte 数组转换为目标数据结构
 *
 * C++:
 * C++支持模板，因此data是目标数据结构T的数组，itemTypeSize = sizeof(T)，itemCount = dataSize / itemTypeSize;
 * \endif
 */
typedef struct OBDataBundle {
    OBCmdVersion cmdVersion;  ///< \if English propertyId's OBCmdVersion \else 与控制命令关联的数据结构的版本号 \endif

    void *data;  ///< \if English Data contain which contain itemCount of element. void *data = new T[itemCount];\else 数据内容，是一个数组，长度为 itemCount;
                 ///< void *data = new T[itemCount]; 数组元素类型为 T，类型 T 由 propertyId 和 cmdVersion 决定 \endif
    uint32_t dataSize;  ///< \if English Data size. unit: byte. dataSize == itemTypeSize * itemCount \else data 的大小，单位：byte，dataSize == itemTypeSize *
                        ///< itemCount \endif

    uint32_t itemTypeSize;  ///< \if English Size of data item. C language, itemTypeSize = 1, C++: itemTypeSize = sizeof(T) \else data
                            ///< 内容映射的目标对象的数据类型大小，C 语言：1，C++：sizeof(T) \endif
    uint32_t itemCount;     ///< \if English Count of data item. itemCount = dataSize / itemTypeSize; 0 == dataSize % itemTypeSize; \else data
                            ///< 内容映射的目标对象数量，满足条件：itemCount = dataSize / itemTypeSize; 0 == dataSize % itemTypeSize; \endif
} OBDataBundle, ob_data_bundle;

/**
 * @brief 网络设备的 IP 地址配置（ipv4）
 *
 */
typedef struct {
    uint16_t dhcp;        ///< dhcp 动态 ip 配置开关; 0: 关; 1: 开
    uint8_t  address[4];  ///< ip 地址 (大端模式, 如 192.168.1.1，则 address[0]==192)
    uint8_t  mask[4];     ///< 子网掩码 (大端模式)
    uint8_t  gateway[4];  ///< 网关 (大端模式)
} OBDeviceIpAddrConfig, ob_device_ip_addr_config, DEVICE_IP_ADDR_CONFIG;

/**
 * \if English
 * @brief Device communication mode
 * \else
 * @brief 设备通信模式
 * \endif
 */
typedef enum {
    OB_COMM_USB = 0x00,  ///< \if English USB \else USB \endif
    OB_COMM_NET = 0x01,  ///< \if English Ethernet \else 以太网 \endif
} OBCommunicationType,
    ob_communication_type, OB_COMMUNICATION_TYPE;

/**
 * \if English
 * @brief USB power status
 * \else
 * @brief USB电源连接状态
 * \endif
 */
typedef enum {
    OB_USB_POWER_NO_PLUGIN = 0,  ///< \if English no plugin \else 未插入 \endif
    OB_USB_POWER_5V_0A9    = 1,  ///< \if English 5V/0.9A \else 5V/0.9A \endif
    OB_USB_POWER_5V_1A5    = 2,  ///< \if English 5V/1.5A \else 5V/1.5A \endif
    OB_USB_POWER_5V_3A0    = 3,  ///< \if English 5V/3.0A \else 5V/3.0A \endif
} OBUSBPowerState,
    ob_usb_power_state;

/**
 * \if English
 * @brief DC power status
 * \else
 * @brief DC电源连接状态
 * \endif
 */
typedef enum {
    OB_DC_POWER_NO_PLUGIN = 0,  ///< \if English no plugin \else 未插入 \endif
    OB_DC_POWER_PLUGIN    = 1,  ///< \if English plugin \else 已插入 \endif
} OBDCPowerState,
    ob_dc_power_state;

/**
 * \if English
 * @brief Rotate degree
 * \else
 * @brief 旋转角度
 * \endif
 */
typedef enum {
    OB_ROTATE_DEGREE_0   = 0,    ///< \if English Rotate 0 \else 旋转0度 \endif
    OB_ROTATE_DEGREE_90  = 90,   ///< \if English Rotate 90 \else 旋转90度 \endif
    OB_ROTATE_DEGREE_180 = 180,  ///< \if English Rotate 180 \else 旋转180度 \endif
    OB_ROTATE_DEGREE_270 = 270,  ///< \if English Rotate 270 \else 旋转270度 \endif
} ob_rotate_degree_type,
    OBRotateDegreeType;

/**
 * \if English
 * @brief Power line frequency mode，for Color camera anti-flicker configuration
 * \else
 * @brief 电力线频率模式，用于Color相机防闪烁功能配置
 * \endif
 */
typedef enum {
    OB_POWER_LINE_FREQ_MODE_CLOSE = 0,  ///< \if English close \else 关闭 \endif
    OB_POWER_LINE_FREQ_MODE_50HZ  = 1,  ///< \if English 50Hz \else 50Hz \endif
    OB_POWER_LINE_FREQ_MODE_60HZ  = 2,  ///< \if English 60Hz \else 60Hz \endif
} ob_power_line_freq_mode,
    OBPowerLineFreqMode;

/**
 * \if English
 * @brief Frame aggregate output mode
 * \else
 * @brief 帧汇聚输出模式
 * \endif
 */
typedef enum {
    /**
     * \if English
     * @brief Only FrameSet that contains all types of data frames will be output
     * \else
     * @brief 只有包含所有类型数据帧的FrameSet才会被输出
     * \endif
     */
    OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE = 0,

    /**
     * \if English
     *
     * @brief Frame aggregate output mode
     * @brief Suitable for Color using H264, H265 and other inter-frame encoding format open stream
     *
     * @attention In this mode, the user may return null when getting a non-Color type data frame from the acquired FrameSet
     *
     * \else
     *
     * @brief 必须包含Color类似数据帧的FrameSet才会被输出
     * @brief 适用于Color使用H264、H265等帧间编码格式开流的情况
     *
     * @attention 该模式下，用户从获取到的FrameSet上获取非Color类型数据帧可能会返回空
     *
     * \endif
     */
    OB_FRAME_AGGREGATE_OUTPUT_COLOR_FRAME_REQUIRE,

    /**
     * \if English
     *
     * @brief FrameSet for any case will be output
     *
     * @attention In this mode, the user may return null when getting the specified type of data frame from the acquired FrameSet
     *
     * \else
     *
     * @brief 任何情况的FrameSet都会被输出
     *
     * @attention 该模式下，用户从获取到的FrameSet上获取指定类型数据帧可能会返回空
     *
     * \endif
     */
    OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION,
} OB_FRAME_AGGREGATE_OUTPUT_MODE,
    OBFrameAggregateOutputMode, ob_frame_aggregate_output_mode;

/**
 * \if English
 * @brief file transfer callback
 * @param state Transmission status
 * @param message Transfer status information
 * @param state Transfer progress percentage
 * @param user_data user-defined data
 * \else
 * @brief 文件传输回调
 * @param state 传输状态
 * @param message 传输状态信息
 * @param state 传输进度百分比
 * @param user_data 用户自定义数据
 * \endif
 */
typedef void (*ob_file_send_callback)(ob_file_tran_state state, const char *message, uint8_t percent, void *user_data);

/**
 * \if English
 * @brief Firmware upgrade callback
 * @param state upgrade status
 * @param message upgrade status information
 * @param state upgrade progress percentage
 * @param user_data user-defined data
 * @brief 固件升级回调
 * @param state 升级状态
 * @param message 升级状态信息
 * @param state 升级进度百分比
 * @param user_data 用户自定义数据
 * \endif
 */
typedef void (*ob_device_upgrade_callback)(ob_upgrade_state state, const char *message, uint8_t percent, void *user_data);

/**
 * \if English
 * @brief device status callback
 * @param state device status
 * @param message Device Status Information
 * @param user_data user-defined data
 * \else
 * @brief 设备状态回调
 * @param state 设备状态
 * @param message 设备状态信息
 * @param user_data 用户自定义数据
 * \endif
 */
typedef void (*ob_device_state_callback)(ob_device_state state, const char *message, void *user_data);

/**
 * \if English
 * @brief Callback for writing data
 * @param state write data status
 * @param percent Write data percentage
 * @param user_data user-defined data
 * \else
 * @brief 写数据的回调
 * @param state 写数据状态
 * @param percent 写数据百分比
 * @param user_data 用户自定义数据
 * \endif
 */
typedef void (*ob_set_data_callback)(ob_data_tran_state state, uint8_t percent, void *user_data);

/**
 * \if English
 * @brief read data callback
 * @param state read data status
 * @param dataChunk read the returned data block
 * @param user_data user-defined data
 * \else
 * @brief 读数据回调
 * @param state 读数据状态
 * @param dataChunk 读取返回的数据块
 * @param user_data 用户自定义数据
 * \endif
 */
typedef void (*ob_get_data_callback)(ob_data_tran_state state, ob_data_chunk *dataChunk, void *user_data);

/**
 * \if English
 * @brief Media status callbacks (recording and playback)
 * @param state condition
 * @param user_data user-defined data
 * \else
 * @brief 媒体状态回调（录制和回放）
 * @param state 状态
 * @param user_data 用户自定义数据
 * \endif
 */
typedef void (*ob_media_state_callback)(ob_media_state state, void *user_data);

/**
 * \if English
 * @brief Device change (up and down) callback
 * @param removed List of deleted (dropped) devices
 * @param added List of added (online) devices
 * @param user_data user-defined data
 * \else
 * @brief 设备变化（上下线）回调
 * @param removed 已删除（掉线）的设备列表
 * @param added 已添加（上线）的设备列表
 * @param user_data 用户自定义数据
 * \endif
 */
typedef void (*ob_device_changed_callback)(ob_device_list *removed, ob_device_list *added, void *user_data);

/**
 * \if English
 * @brief dataframe callback
 * @param frame Data Frame
 * @param user_data user-defined data
 * \else
 * @brief 数据帧回调
 * @param frame 数据帧
 * @param user_data 用户自定义数据
 * \endif
 */
typedef void (*ob_frame_callback)(ob_frame *frame, void *user_data);
#define ob_filter_callback ob_frame_callback
#define ob_playback_callback ob_frame_callback

/**
 * \if English
 * @brief dataframe collection callback
 * @param frameset collection of dataframes
 * @param user_data user-defined data
 * \else
 * @brief 数据帧集合回调
 * @param frameset 数据帧集合
 * @param user_data 用户自定义数据
 * \endif
 */
typedef void (*ob_frameset_callback)(ob_frame *frameset, void *user_data);

/**
 * \if English
 * @brief Customize the delete callback, 当引用计数为 0 时主动调用
 * @param buffer Data that needs to be deleted
 * @param context user-defined data
 * \else
 * @brief 自定义删除回调，当引用计数为 0 时主动调用
 * @param buffer 需要被删除的数据
 * @param context 用户传入的数据
 * \endif
 */
typedef void(ob_frame_destroy_callback)(void *buffer, void *context);

/**
 * \if English
 * @brief Check sensor_type is IR sensor, true: IR sensor, false: no IR sensor
 * \else
 * @brief 判断是否为 IR Sensor
 * \endif
 *
 */
#define is_ir_sensor(sensor_type) (sensor_type == OB_SENSOR_IR || sensor_type == OB_SENSOR_IR_LEFT || sensor_type == OB_SENSOR_IR_RIGHT)
#define isIRSensor is_ir_sensor

/**
 * \if English
 * @brief Check stream_type is IR stream, true: IR stream, false: no IR stream
 * \else
 * @brief 判断是否为 IR 数据流
 * \endif
 *
 */
#define is_ir_stream(stream_type) (stream_type == OB_STREAM_IR || stream_type == OB_STREAM_IR_LEFT || stream_type == OB_STREAM_IR_RIGHT)
#define isIRStream is_ir_stream

/**
 * \if English
 * @brief Check frame_type is IR frame, true: IR frame, false: no IR frame
 * \else
 * @brief 判断是否为 IR 数据帧
 * \endif
 *
 */
#define is_ir_frame(frame_type) (frame_type == OB_FRAME_IR || frame_type == OB_FRAME_IR_LEFT || frame_type == OB_FRAME_IR_RIGHT)
#define isIRFrame is_ir_frame

#ifdef __cplusplus
}
#endif

#pragma pack(pop)
