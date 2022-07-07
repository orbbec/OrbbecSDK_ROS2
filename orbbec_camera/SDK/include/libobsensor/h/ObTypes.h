// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

/**
 * \if English
 * @file ObTypes.h
 * @brief Provide structs commonly used in the SDK, enumerating constant definitions.
 * \else
 * @file ObTypes.h
 * @brief 提供SDK常用的结构体、枚举常量定义。
 * \endif
 */

#pragma once

#if(defined(WIN32) || defined(_WIN32) || defined(WINCE))
#ifdef OB_EXPORTS
#define OB_EXTENSION_API __declspec(dllexport)
#else
#define OB_EXTENSION_API __declspec(dllimport)
#endif
#else
#define OB_EXTENSION_API __attribute__((visibility("default")))
#endif

#if defined(__GNUC__) || defined(__clang__)
#define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct ContextImpl           ob_context;
typedef struct DeviceImpl            ob_device;
typedef struct DeviceInfoImpl        ob_device_info;
typedef struct DeviceListImpl        ob_device_list;
typedef struct CameraParamListImpl   ob_camera_param_list;
typedef struct SensorImpl            ob_sensor;
typedef struct SensorListImpl        ob_sensor_list;
typedef struct StreamProfileImpl     ob_stream_profile;
typedef struct StreamProfileListImpl ob_stream_profile_list;
typedef struct FrameImpl             ob_frame;
typedef struct FilterImpl            ob_filter;
typedef struct PipelineImpl          ob_pipeline;
typedef struct ConfigImpl            ob_config;
typedef struct RecorderImpl          ob_recorder;
typedef struct PlaybackImpl          ob_playback;

/**
 * \if English
 * @brief the permission type of api or property
 * \else
 * @brief 接口/属性的访问权限类型
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
 * @brief log等级, 等级越高Log过滤力度越大
 * \endif
 */
typedef enum {
    OB_LOG_SEVERITY_DEBUG, /**< \if English debug \else 调试 \endif */
    OB_LOG_SEVERITY_INFO,  /**< \if English information \else 信息 \endif */
    OB_LOG_SEVERITY_WARN,  /**< \if English warning \else 警告 \endif */
    OB_LOG_SEVERITY_ERROR, /**< \if English error \else 错误 \endif */
    OB_LOG_SEVERITY_FATAL, /**< \if English fatal error \else 致命错误 \endif */
    OB_LOG_SEVERITY_NONE   /**< \if English none (close LOG) \else 无(关闭LOG) \endif */
} OBLogSeverity,
    ob_log_severity, DEVICE_LOG_SEVERITY_LEVEL, OBDeviceLogSeverityLevel, ob_device_log_severity_level;

/**
 * \if English
 * @brief The exception types in the SDK, through the exception type, you can easily determine the specific type of error.
 * For detailed error API interface functions and error logs, please refer to the information of ob_error
 * \else
 * @brief SDK内部的异常类型，通过异常类型，可以简单判断具体哪个类型的错误
 * 详细的错误API接口函数、错误日志请参考ob_error的信息
 * \endif
 */
typedef enum {
    OB_EXCEPTION_TYPE_UNKNOWN,                 /**< \if English Unknown error, an error not clearly defined by the SDK \else 未知错误，SDK未明确定义的错误 \endif */
    OB_EXCEPTION_TYPE_CAMERA_DISCONNECTED,     /**< \if English SDK device disconnection exception \else SDK的设备断开的异常 \endif */
    OB_EXCEPTION_TYPE_PLATFORM,                /**< \if English An error in the SDK adaptation platform layer means an error in the implementation of a specific system platform \else 在SDK适配平台层错误，代表是具体一个系统平台实现上错误 \endif */
    OB_EXCEPTION_TYPE_INVALID_VALUE,           /**< \if English Invalid parameter type exception, need to check input parameter \else 无效的参数类型异常，需要检查输入参数 \endif */
    OB_EXCEPTION_TYPE_WRONG_API_CALL_SEQUENCE, /**< \if English Exception caused by API version mismatch\else API版本不匹配带来的异常 \endif */
    OB_EXCEPTION_TYPE_NOT_IMPLEMENTED,         /**< \if English SDK and firmware have not yet implemented functions \else SDK及固件还未实现功能 \endif */
    OB_EXCEPTION_TYPE_IO,                      /**< \if English SDK access IO exception error \else SDK访问IO异常错误 \endif */
    OB_EXCEPTION_TYPE_MEMORY,                  /**< \if English SDK access and use memory errors, which means that the frame fails to allocate memory \else SDK的访问和使用内存错误，代表桢分配内存失败\endif */
    OB_EXCEPTION_TYPE_UNSUPPORTED_OPERATION,   /**< \if English Unsupported operation type error by SDK or RGBD device \else SDK或RGBD设备不支持的操作类型错误 \endif */
} OBExceptionType,
    ob_exception_type;

/**
 * \if English
 * @brief The error class exposed by the SDK, users can get detailed error information according to the erro
 * \else
 * @brief SDK的对外暴露的错误类，用户可以根据该错误类，获取详细的错误信息
 * \endif
 */
typedef struct ob_error {
    ob_status         status;          ///< \if English Describe the status code of the error, as compatible with previous customer status code requirements \else 描述错误的状态码，作为兼容之前客户状态码需求 \endif
    char              message[256];    ///< \if English Describe the detailed error log \else 描述详细的错误日志 \endif
    char              function[256];   ///< \if English Describe the name of the function where the error occurred \else 描述出现错误的函数名称 \endif
    char              args[256];       ///< \if English Describes the parameters passed to the function when an error occurs. Used to check whether the parameter is wrong \else 描述出错时，函数传入的参数。用来检查是不是参数错 \endif
    ob_exception_type exception_type;  ///< \if English The description is the specific error type of the SDK \else 描述是SDK的具体错误类型 \endif
} ob_error;

/**
 * \if English
 * @brief Enumeration value describing the sensor type
 * \else
 * @brief 描述传感器类型的枚举值
 * \endif
 */
typedef enum {
    OB_SENSOR_UNKNOWN = 0, /**< \if English Unknown type sensor \else 未知类型传感器 \endif */
    OB_SENSOR_IR      = 1, /**< \if English IR \else 红外 \endif */
    OB_SENSOR_COLOR   = 2, /**< \if English Color \else 彩色 \endif */
    OB_SENSOR_DEPTH   = 3, /**< \if English Depth \else 深度 \endif */
    OB_SENSOR_ACCEL   = 4, /**< \if English Accel \else 加速度计 \endif */
    OB_SENSOR_GYRO    = 5, /**< \if English Gyro \else 陀螺仪 \endif */
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
    OB_STREAM_VIDEO = 0, /**< \if English Video stream (infrared, color, depth streams are all video streams) \else 视频流(红外、彩色、深度流都属于视频流) \endif */
    OB_STREAM_IR    = 1, /**< \if English IR stream \else 红外流 \endif */
    OB_STREAM_COLOR = 2, /**< \if English color stream \else 彩色流 \endif */
    OB_STREAM_DEPTH = 3, /**< \if English depth stream \else 深度流 \endif */
    OB_STREAM_ACCEL = 4, /**< \if English Accelerometer data stream \else 加速度计数据流 \endif */
    OB_STREAM_GYRO  = 5, /**< \if English Gyroscope data stream \else 陀螺仪数据流 \endif */
} OBStreamType,
    ob_stream_type;

/**
 * \if English
 * @brief Describe the Frame type enumeration value
 * \else
 * @brief 描述Frame类型枚举值
 * \endif
 */
typedef enum {
    OB_FRAME_VIDEO  = 0, /**< \if English Describes the Frame type enumeration value \else 视频帧(红外、彩色、深度帧都属于视频帧) \endif */
    OB_FRAME_IR     = 1, /**< \if English IR frame \else 红外帧 \endif */
    OB_FRAME_COLOR  = 2, /**< \if English color stream \else 彩色帧 \endif */
    OB_FRAME_DEPTH  = 3, /**< \if English depth stream \else 深度帧 \endif */
    OB_FRAME_ACCEL  = 4, /**< \if English Accelerometer data frame \else 加速度计数据帧 \endif */
    OB_FRAME_SET    = 5, /**< \if English Frame collection (internally contains a variety of data frames) \else 帧集合(内部包含多种数据帧) \endif */
    OB_FRAME_POINTS = 6, /**< \if English point cloud frame \else 点云帧 \endif */
    OB_FRAME_GYRO   = 7, /**< \if English Gyroscope data frame \else 陀螺仪数据帧 \endif */
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
    OB_FORMAT_YUYV      = 0,    /**< \if English YUYV format \else YUYV格式 \endif */
    OB_FORMAT_YUY2      = 1,    /**< \if English YUY2 format (the actual format is the same as YUYV) \else YUY2格式(实际格式与YUYV相同) \endif */
    OB_FORMAT_UYVY      = 2,    /**< \if English UYVY format \else UYVY格式 \endif */
    OB_FORMAT_NV12      = 3,    /**< \if English NV12 format \else NV12格式 \endif */
    OB_FORMAT_NV21      = 4,    /**< \if English NV21 format \else NV21格式 \endif */
    OB_FORMAT_MJPG      = 5,    /**< \if English MJPG encoding format \else MJPG编码格式 \endif */
    OB_FORMAT_H264      = 6,    /**< \if English H.264 encoding format \else H.264编码格式 \endif */
    OB_FORMAT_H265      = 7,    /**< \if English H.265 encoding format \else H.265编码格式 \endif */
    OB_FORMAT_Y16       = 8,    /**< \if English Y16 format, single channel 16bit depth \else Y16格式，单通道16bit深度 \endif */
    OB_FORMAT_Y8        = 9,    /**< \if English Y8 format, single channel 8bit depth \else Y8格式，单通道8bit深度 \endif */
    OB_FORMAT_Y10       = 10,   /**< \if English Y10 format, single channel 10bit depth (SDK will unpack into Y16 by default) \else Y10格式，单通道10bit深度(SDK默认会解包成Y16) \endif */
    OB_FORMAT_Y11       = 11,   /**< \if English Y11 format, single channel 11bit depth (SDK will unpack into Y16 by default)\else Y11格式，单通道11bit深度(SDK默认会解包成Y16) \endif */
    OB_FORMAT_Y12       = 12,   /**< \if English Y12 format, single channel 12bit depth (SDK will unpack into Y16 by default) \else Y12格式，单通道12bit深度(SDK默认会解包成Y16) \endif */
    OB_FORMAT_GRAY      = 13,   /**< \if English GRAY (the actual format is the same as YUYV) \else GRAY灰度(实际格式与YUYV相同) \endif */
    OB_FORMAT_HEVC      = 14,   /**< \if English HEVC encoding format (the actual format is the same as H265) \else HEVC编码格式(实际格式与H265相同) \endif */
    OB_FORMAT_I420      = 15,   /**< \if English I420 format \else I420格式  \endif */
    OB_FORMAT_ACCEL     = 16,   /**< \if English Acceleration data format \else 加速度数据格式  \endif */
    OB_FORMAT_GYRO      = 17,   /**< \if English Gyroscope Data Format \else 陀螺仪数据格式  \endif */
    OB_FORMAT_POINT     = 19,   /**< \if English xyz 3D coordinate point format \else 纯x-y-z三维坐标点格式  \endif */
    OB_FORMAT_RGB_POINT = 20,   /**< \if English xyz 3D coordinate point format with RGB information \else 带RGB信息的x-y-z三维坐标点格式 \endif */ 
    OB_FORMAT_RLE       = 21,   /**< \if English RLE pressure test format (SDK will be unpacked into Y16 by default) \else RLE压测格式(SDK默认会解包成Y16) \endif */
    OB_FORMAT_RGB888    = 22,   /**< \if English RGB888 format \else RGB888格式  \endif */
    OB_FORMAT_BGR       = 23,   /**< \if English BGR format (actual BRG888) \else BGR格式(实际BRG888) \endif */
    OB_FORMAT_Y14       = 24,   /**< \if English Y14 format, single channel 14bit depth (SDK will unpack into Y16 by default) \else Y14格式，单通道14bit深度(SDK默认会解包成Y16) \endif */
    OB_FORMAT_UNKNOWN   = 0xff, /**< \if English unknown format \else 未知格式 \endif */
} OBFormat,
    ob_format;

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
    ERR_ERASE          = -3, /**< \if English Flash parameter failed \else Flash参数失败 \endif */
    ERR_FLASH_TYPE     = -4, /**< \if English Flash type error \else Flash类型错误 \endif */
    ERR_IMAGE_SIZE     = -5, /**< \if English Image file size error \else 镜像文件大小错误 \endif */
    ERR_OTHER          = -6, /**< \if English other errors \else 其他错误 \endif */
    ERR_DDR            = -7, /**< \if English DDR access error \else DDR访问错误 \endif */
    ERR_TIMEOUT = -8         /**< \if English timeout error \else 超时错误 \endif */
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
    FILE_TRAN_ERR_DDR               = -1, /**< \if English DDR access failed \else DDR访问失败 \endif */
    FILE_TRAN_ERR_NOT_ENOUGH_SPACE  = -2, /**< \if English Insufficient target space error \else 目标空间不足错误 \endif */
    FILE_TRAN_ERR_PATH_NOT_WRITABLE = -3, /**< \if English Destination path is not writable \else 目标路径不可写 \endif */
    FILE_TRAN_ERR_MD5_ERROR         = -4, /**< \if English MD5 checksum error \else MD5校验错误 \endif */
    FILE_TRAN_ERR_WRITE_FLASH_ERROR = -5, /**< \if English write flash error \else 写Flash错误 \endif */
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
    DATA_TRAN_ERR_OTHER         = -5  /**< \if English other errors \else 其他错误\endif */
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
    void *   data;          ///< \if English current block data pointer \else 当前块数据指针 \endif     
    uint32_t size;          ///< \if English Current block data length \else 当前块数据长度 \endif     
    uint32_t offset;        ///< \if English The offset of the current data block relative to the complete data \else 当前数据块相对完整数据的偏移 \endif    
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
    float   fx;      ///< \if English focal length in x direction \else x方向焦距 \endif  
    float   fy;      ///< \if English focal length in y direction \else y方向焦距 \endif   
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
    float k1;  ///< \if English Radial distortion factor 1 \else 径向畸变系数1 \endif
    float k2;  ///< \if English Radial distortion factor 2 \else 径向畸变系数2 \endif
    float k3;  ///< \if English Radial distortion factor 3 \else 径向畸变系数3 \endif
    float k4;  ///< \if English Radial distortion factor 4 \else 径向畸变系数4 \endif
    float k5;  ///< \if English Radial distortion factor 5 \else 径向畸变系数5 \endif
    float k6;  ///< \if English Radial distortion factor 6 \else 径向畸变系数6 \endif
    float p1;  ///< \if English Tangential distortion factor 1 \else 切向畸变系数1 \endif    
    float p2;  ///< \if English Tangential distortion factor 2 \else 切向畸变系数2 \endif  
} OBCameraDistortion, ob_camera_distortion;

/**
 * \if English
 * @brief Rotation/Transformation
 * \else
 * @brief 旋转/变换矩阵
 * \endif
 */
typedef struct {
    float rot[9];    ///< \if English Rotation matrix \else 旋转矩阵，行优先\endif   
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
    OBD2CTransform     transform;        ///< \if English rotation/transformation matrix \else 旋转/变换矩阵 \endif   
    bool               isMirrored;       ///< \if English Whether the image frame corresponding to this group of parameters is mirrored \else 本组参数对应的图像帧是否被镜像 \endif   
} OBCameraParam, ob_camera_param;

/**
 * \if English
 * @brief alignment mode
 * \else
 * @brief 对齐模式
 * \endif
 */
typedef enum {
    ALIGN_DISABLE,     /**< \if English turn off alignment \else 关闭对齐 \endif */
    ALIGN_D2C_HW_MODE, /**< \if English Hardware D2C alignment mode \else 硬件D2C对齐模式 \endif */
    ALIGN_D2C_SW_MODE, /**< \if English Software D2C alignment mode \else 软件D2C对齐模式 \endif */
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
    uint32_t x;       ///< \if English origin coordinate x \else 原点坐标x \endif     
    uint32_t y;       ///< \if English origin coordinate y \else 原点坐标y \endif     
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
    FORMAT_YUYV_TO_RGB888 = 0, /**< \if English YUYV to RGB888 \else YUYV转换为RGB888 \endif */
    FORMAT_I420_TO_RGB888,     /**< \if English I420 to RGB888 \else I420转换为RGB888 \endif */
    FORMAT_NV21_TO_RGB888,     /**< \if English NV21 to RGB888 \else NV21转换为RGB888 \endif */
    FORMAT_NV12_TO_RGB888,     /**< \if English NV12 to RGB888 \else NV12转换为RGB888 \endif */
    FORMAT_MJPEG_TO_I420,      /**< \if English MJPG to I420 \else MJPG转换为I420\endif */
    FORMAT_RGB888_TO_BGR,      /**< \if English RGB888 to BGR \else RGB888转换为BGR \endif */
    FORMAT_MJPEG_TO_NV21,      /**< \if English MJPG to NV21 \else MJPG转换为NV21 \endif */
    FORMAT_MJPEG_TO_RGB888,    /**< \if English MJPG to RGB888 \else MJPG转换为RGB888 \endif */
} OBConvertFormat,
    ob_convert_format;

/**
 * \if English
 * @brief Enumeration of IMU sample rate values ​​(gyroscope or accelerometer)
 * \else
 * @brief IMU采样率值的枚举(陀螺仪或加速度计)
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
    OB_GYRO_FS_16dps = 1, /**< \if English 16 degrees per second \else 16度每秒 \endif */
    OB_GYRO_FS_31dps,     /**< \if English 31 degrees per second \else 31度每秒 \endif */
    OB_GYRO_FS_62dps,     /**< \if English 62 degrees per second \else 62度每秒 \endif */
    OB_GYRO_FS_125dps,    /**< \if English 125 degrees per second \else 125度每秒 \endif */
    OB_GYRO_FS_245dps,    /**< \if English 245 degrees per second \else 245度每秒 \endif */
    OB_GYRO_FS_250dps,    /**< \if English 250 degrees per second \else 250度每秒 \endif */
    OB_GYRO_FS_500dps,    /**< \if English 500 degrees per second \else 500度每秒 \endif */
    OB_GYRO_FS_1000dps,   /**< \if English 1000 degrees per second \else 1000度每秒 \endif */
    OB_GYRO_FS_2000dps,   /**< \if English 2000 degrees per second \else 2000度每秒 \endif */
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
    OB_ACCEL_FS_2g = 1, /**< \if English 1x the acceleration of gravity \else 1倍重力加速度 \endif */
    OB_ACCEL_FS_4g,     /**< \if English 4x the acceleration of gravity \else 4倍重力加速度 \endif */
    OB_ACCEL_FS_8g,     /**< \if English 8x the acceleration of gravity \else 8倍重力加速度 \endif */
    OB_ACCEL_FS_16g,    /**< \if English 16x the acceleration of gravity \else 16倍重力加速度\endif */
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
    float x;  ///< \if English x-direction component \else x方向分量 \endif     
    float y;  ///< \if English y-direction component \else y方向分量 \endif    
    float z;  ///< \if English z-direction component \else z方向分量 \endif     
} OBAccelValue, OBGyroValue, ob_accel_value, ob_gyro_value;

/**
 * \if English
 * @brief Device state enumeration
 * \else
 * @brief 设备状态枚举
 * \endif
 */
typedef enum {
    OB_DEVICE_STATE_NULL        = 0x0000, /**< \if English No status or LOG information update \else 无状态或LOG信息更新 \endif */
    OPEN_STREAM_OPERATION_ERROR = 0x0001, /**< \if English Open current exception \else 开流异常 \endif */
    OB_DEVICE_STATE_INFO        = 0x7fff  /**< \if English LOG information update \else LOG信息更新 \endif */
} OBDeviceState,
    ob_device_state;

/**
 * \if English
 * @brief Get the temperature parameters of the device (unit: Celsius)
 * \else
 * @brief 获取设备的温度参数（单位：摄氏度）
 * \endif
 */
typedef struct {
    float cpuTemp;        ///< \if English CPU temperature \else cpu温度 \endif     
    float irTemp;         ///< \if English IR temperature \else IR温度 \endif     
    float ldmTemp;        ///< \if English laser temperature \else 激光温度 \endif     
    float mainBoardTemp;  ///< \if English motherboard temperature \else 主板温度 \endif     
    float tecTemp;        ///< \if English TEC temperature \else TEC温度 \endif     
    float imuTemp;        ///< \if English IMU temperature \else IMU温度 \endif     
    float rgbTemp;        ///< \if English RGB temperature \else RGB温度 \endif     
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
    OB_STRUCTURED_LIGHT_BINOCULAR_CAMERA,     /**< \if English Binocular structured light camera \else 双目结构光相机  \endif */
    OB_TOF_CAMERA,                            /**< \if English TOF camera \else TOF相机  \endif */
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
    OB_MEDIA_COLOR_STREAM = 1,   /**< \if English color stream \else 彩色流\endif */ 
    OB_MEDIA_DEPTH_STREAM = 2,   /**< \if English depth stream \else 深度流 \endif */
    OB_MEDIA_IR_STREAM    = 4,   /**< \if English IR stream \else 红外流 \endif */
    OB_MEDIA_GYRO_STREAM  = 8,   /**< \if English gyro stream \else 陀螺仪数据流 \endif */
    OB_MEDIA_ACCEL_STREAM = 16,  /**< \if English accel stream \else 加速度计数据流 \endif */ 
    OB_MEDIA_CAMERA_PARAM = 32,  /**< \if English camera parameter \else 相机参数 \endif */ 
    OB_MEDIA_DEVICE_INFO  = 64,  /**< \if English device information \else 设备信息  \endif */
    OB_MEDIA_STREAM_INFO  = 128, /**< \if English stream information \else 流信息 \endif */ 

    OB_MEDIA_ALL = OB_MEDIA_COLOR_STREAM | OB_MEDIA_DEPTH_STREAM | OB_MEDIA_IR_STREAM | OB_MEDIA_GYRO_STREAM | OB_MEDIA_ACCEL_STREAM | OB_MEDIA_CAMERA_PARAM
                   | OB_MEDIA_DEVICE_INFO | OB_MEDIA_STREAM_INFO, /**< \if English All media data types \else 所有媒体数据类型 \endif */ 
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
 * @attention The depth accuracy level does not completely determine the depth unit and real accuracy, and the influence of the data packaging format needs to be considered. 
 * The specific unit can be obtained through getValueScale() of DepthFrame
 * \else
 * @brief 深度精度等级
 * @attention 深度精度等级并不完全决定深度的单位和真实精度，需要考虑数据打包格式的影响，
 * 具体单位可通过DepthFrame的getValueScale()获取
 * \endif
 */
typedef enum {
    OB_PRECISION_1MM,  /**< 1mm */
    OB_PRECISION_0MM8, /**< 0.8mm */
    OB_PRECISION_0MM4, /**< 0.4mm */
    OB_PRECISION_0MM1, /**< 0.1mm */
    OB_PRECISION_COUNT,
} OBDepthPrecisionLevel,
    ob_depth_precision_level, OB_DEPTH_PRECISION_LEVEL;

/**
 * \if English
 * @brief tof filter scene range
 * \else
 * @brief tof滤波场景范围
 * \endif
 */
typedef enum {
    OB_TOF_FILTER_RANGE_CLOSE  = 0,   /**< \if English close range \else 近距离范围 \endif */ 
    OB_TOF_FILTER_RANGE_MIDDLE = 1,   /**< \if English middle range \else 中距离范围  \endif */
    OB_TOF_FILTER_RANGE_LONG   = 2,   /**< \if English long range \else 远距离范围 \endif */
    OB_TOF_FILTER_RANGE_DEBUG  = 100, /**< \if English debug range \else Debug模式  \endif */
} OBTofFilterRange,
    ob_tof_filter_range, TOF_FILTER_RANGE;

/**
 * \if English
 * @brief 3D point structure in SDK
 * \else
 * @brief SDK中3D点结构体
 * \endif
 */
typedef struct {
    float x;  ///< \if English x coordinate \else X坐标 \endif     
    float y;  ///< \if English y coordinate \else Y坐标 \endif     
    float z;  ///< \if English z coordinate \else Z坐标 \endif     
} OBPoint, ob_point;

/**
 * \if English
 * @brief 3D point structure with color information
 * \else
 * @brief 带有颜色信息的3D点结构体
 * \endif
 */
typedef struct {
    float x;  ///< \if English x coordinate \else X坐标 \endif    
    float y;  ///< \if English y coordinate \else Y坐标 \endif   
    float z;  ///< \if English z coordinate \else Z坐标 \endif    
    float r;  ///< \if English red channel component \else 红色通道分量 \endif    
    float g;  ///< \if English green channel component \else 绿色通道分量 \endif    
    float b;  ///< \if English blue channel component\else 蓝色通道分量 \endif    
} OBColorPoint, ob_color_point;

/**
 * \if English
 * @brief Multi-device sync mode
 * \else
 * @brief 多设备同步模式
 * \endif
 */
typedef enum {
    OB_SYNC_STOP              = 0x00, /**< \if English turn off sync \else 关闭同步 \endif */ 
    OB_SYNC_SINGLE_MODE       = 0x01, /**< \if English single device mode \else single device模式 \endif */
    OB_SYNC_ONLINE_HOST_MODE  = 0x02, /**< \if English The single device mode is also the host mode, which is dominated by ir \else single device模式，也是host模式，是ir作主的  \endif */
    OB_SYNC_ONLINE_SLAVE_MODE = 0x03, /**< \if English slave mode (ext_in --> rgb、tof、ext_out) \else slave模式 (ext_in --> rgb、tof、ext_out)  \endif */
    OB_SYNC_ONLY_MCU_MODE     = 0x04, /**< \if English MCU as host mode \else mcu作host的模式 \endif */ 
    OB_SYNC_ONLY_IR_MODE      = 0x05, /**< \if English IR as host mode\else ir作host的模式  \endif */
} OBSyncMode,
    ob_sync_mode, OB_SYNC_MODE;

/**
 * \if English
 * @brief TOF Exposure Threshold
 * \else
 * @brief TOF曝光阈值
 *\endif
 */
typedef struct {
    int32_t upper;  ///< \if English Upper threshold, unit: ms \else 阈值上限， 单位：ms \endif
    int32_t lower;  ///< \if English Lower threshold, unit: ms \else 阈值下限， 单位：ms \endif
} OBTofExposureThresholdControl, ob_tof_exposure_threshold_control, TOF_EXPOSURE_THRESHOLD_CONTROL;

/**
 * \if English
 * @brief Multi-device synchronization configuration
 * \else
 * @brief 多设备同步配置
 * \endif
 */
typedef struct {
    OBSyncMode syncMode;       ///< \if English Sync mode \else 同步模式 \endif  
    uint16_t   tofPhaseDelay;  ///< \if English Tof trigger signal delay us. \else Tof触发信号延时 us \endif  
    uint16_t   rgbPhaseDelay;  ///< \if English rgb trigger signal delay us \else rgb触发信号延时 us \endif  
    uint16_t   outPhaseDelay;  ///< \if English Output signal delay us \else 输出信号延时 us \endif  
    uint16_t   outOCPolarity;  ///< \if English 0: Positive pulse, 1: Negative pulse \else 0:正向脉冲, 1: 负向脉冲 \endif  
    uint16_t   mcuHostFps;     ///< \if English Trigger frame rate in mcu master mode \else mcu主模式时的触发帧率 \endif  
    uint16_t   curDevId;       ///< \if English Current device number \else 当前设备编号 \endif  
} OBMultiDeviceSyncConfig, ob_multi_device_sync_config, OB_MULTI_DEVICE_SYNC_CONFIG;

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

#ifdef __cplusplus
}
#endif
