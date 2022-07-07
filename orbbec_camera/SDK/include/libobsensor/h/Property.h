// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

/**
 * \if English
 * @file Property.h
 * @brief Control command property list maintenance
 * \else
 * @file Property.h
 * @brief 控制命令属性列表维护
 * \endif
 */

#ifndef _OB_PROPERTY_H_
#define _OB_PROPERTY_H_

#include "ObTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \if English
 * @brief Enumeration value describing all attribute control commands of the device
 * \else
 * @brief 描述设备所有的属性控制命令的枚举值
 * \endif
 */
typedef enum {
    OB_PROP_LDP_BOOL                              = 2,  /**< \if English LDP switch \else LDP开关 \endif */
    OB_PROP_LASER_BOOL                            = 3,  /**< \if English Laser switch \else 激光开关 \endif */
    OB_PROP_LASER_PULSE_WIDTH_INT                 = 4,  /**< \if English Read and write laser Time value (pulse width) \else 读写激光Time值（脉宽） \endif */
    OB_PROP_LASER_CURRENT_FLOAT                   = 5,  /**< \if English Laser current \else 激光电流 \endif */
    OB_PROP_FLOOD_BOOL                            = 6,  /**< \if English IR flood switch \else 泛光灯开关 \endif */
    OB_PROP_FLOOD_LEVEL_INT                       = 7,  /**< \if English IR flood level \else 泛光灯等级 \endif */
    OB_PROP_DEPTH_MIRROR_BOOL                     = 14, /**< \if English Depth mirror \else 深度镜像 \endif */
    OB_PROP_DEPTH_FLIP_BOOL                       = 15, /**< \if English Depth flip \else  深度翻转 \endif */
    OB_PROP_DEPTH_POSTFILTER_BOOL                 = 16, /**< \if English Depth Postfilter \else 深度Postfilter \endif */
    OB_PROP_DEPTH_HOLEFILTER_BOOL                 = 17, /**< \if English Depth Holefilter \else 深度Holefilter \endif */
    OB_PROP_IR_MIRROR_BOOL                        = 18, /**< \if English IR mirror \else  IR 镜像 \endif */
    OB_PROP_IR_FLIP_BOOL                          = 19, /**< \if English IR flip \else  IR 翻转 \endif */
    OB_PROP_MIN_DEPTH_INT                         = 22, /**< \if English Minimum depth threshold \else  最小深度阈值 \endif */
    OB_PROP_MAX_DEPTH_INT                         = 23, /**< \if English Maximum depth threshold \else  最大深度阈值 \endif */
    OB_PROP_DEPTH_SOFT_FILTER_BOOL                = 24, /**< \if English Software filter switch \else  软件滤波开关 \endif */
    OB_PROP_LDP_STATUS_BOOL                       = 32, /**< \if English LDP status \else  LDP状态 \endif */
    OB_PROP_DEPTH_MAX_DIFF_INT                    = 40, /**< soft filter maxdiff param */
    OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT            = 41, /**< soft filter maxSpeckleSize */
    OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL             = 42, /**< \if English Hardware d2c is on \else  硬件d2c开 \endif */
    OB_PROP_TIMESTAMP_OFFSET_INT                  = 43, /**< \if English Timestamp adjustment \else  时间戳调校 \endif */
    OB_PROP_HARDWARE_DISTORTION_SWITCH_BOOL       = 61, /**< \if English Hardware distortion switch Rectify \else  硬件去畸变开关 Rectify \endif */
    OB_PROP_FAN_WORK_MODE_INT                     = 62, /**< \if English Fan mode switch \else  风扇开关模式 \endif */
    OB_PROP_DEPTH_ALIGN_HARDWARE_MODE_INT         = 63, /**< \if English Multi-resolution D2C mode \else  多分辨率D2C模式 \endif */
    OB_PROP_ANTI_COLLUSION_ACTIVATION_STATUS_BOOL = 64, /**< \if English Anti_collusion activation status \else  防串货状态 \endif */
    OB_PROP_DEPTH_PRECISION_LEVEL_INT  = 75, /**< \if English The depth precision level, which may change the depth frame data unit, needs to be confirmed through the ValueScale interface of DepthFrame \else  深度精度等级，可能会改变深度帧数据单位，需要通过DepthFrame的ValueScale接口确认 \endif */
    OB_PROP_TOF_FILTER_RANGE_INT       = 76, /**< \if English tof filter range configuration \else  tof滤波场景范围配置 \endif */
    OB_PROP_LASER_MODE_INT             = 79, /**< \if English laser mode, the firmware terminal currently only returns 1: IR Drive, 2: Torch \else  激光点亮模式，固件端目前只返回 1: IR Drive, 2: Torch \endif */
    OB_PROP_RECTIFY2_BOOL              = 80, /**< \if English brt2r-rectify function switch (brt2r is a special module on mx6600), 0: Disable, 1: Rectify Enable \else  brt2r-rectify功能开关(brt2r是mx6600上的一个特殊模块)，0:关闭, 1:打开 \endif */
    OB_PROP_COLOR_MIRROR_BOOL          = 81, /**< \if English Color mirror \else  彩色镜像 \endif */
    OB_PROP_COLOR_FLIP_BOOL            = 82, /**< \if English Color flip \else  彩色翻转 \endif */
    OB_PROP_INDICATOR_LIGHT_BOOL       = 83, /**< \if English Indicator switch, 0: Disable, 1: Enable \else  指示灯开关，0:关闭， 1:打开 \endif */
    OB_PROP_DISPARITY_TO_DEPTH_BOOL    = 85, /**< \if English Disparity to depth switch, 0: off, the depth stream outputs the disparity map; 1. On, the depth stream outputs the depth map. \else  视差转深度开关， 0：关闭，深度流输出视差图； 1. 打开，深度流输出深度图 \endif */
    OB_PROP_BRT_BOOL                   = 86, /**< \if English BRT function switch (anti-background interference), 0: Disable, 1: Enable \else  BRT功能开关(抗背景干扰)，0:关闭， 1:打开 \endif */
    OB_PROP_WATCHDOG_BOOL              = 87, /**< \if English Watchdog function switch, 0: Disable, 1: Enable \else  看门狗功能开关，0:关闭， 1:打开 \endif */
    OB_PROP_EXTERNAL_SIGNAL_RESET_BOOL = 88, /**< \if English External signal trigger restart function switch, 0: Disable, 1: Enable \else  外部信号触发重启功能开关，0:关闭， 1:打开 \endif */
    OB_PROP_HEARTBEAT_BOOL             = 89, /**< \if English Heartbeat monitoring function switch, 0: Disable, 1: Enable \else  心跳监测功能开关，0:关闭， 1:打开 \endif */
    OB_PROP_DEPTH_CROPPING_MODE_INT    = 90, /**< \if English Depth cropping mode device: OB_DEPTH_CROPPING_MODE  \else  深度裁剪模式设备: OB_DEPTH_CROPPING_MODE  \endif */
    OB_PROP_D2C_PREPROCESS_BOOL        = 91, /**< \if English D2C preprocessing switch (such as RGB cropping), 0: off, 1: on \else  D2C前处理开关（如RGB裁剪），0:关闭，1:打开 \endif */
    OB_PROP_RGB_CUSTOM_CROP_BOOL       = 94, /**< \if English Custom RGB cropping switch, 0 is off, 1 is on custom cropping, and the ROI cropping area is issued \else  自定义RGB裁剪开关，0为关闭，1为开启自定义裁剪，下发ROI裁剪区域\endif */
    OB_PROP_DEVICE_WORK_MODE_INT       = 95, /**< \if English Device operating mode (power consumption) \else  设备工作模式（功耗） \endif */

    OB_STRUCT_BASELINE_CALIBRATION_PARAM     = 1002, /**< \if English Baseline calibration parameters \else  基线标定参数 \endif */
    OB_STRUCT_DEVICE_TEMPERATURE             = 1003, /**< \if English Device temperature information \else  设备温度信息 \endif */
    OB_STRUCT_TOF_EXPOSURE_THRESHOLD_CONTROL = 1024, /**< \if English TOF exposure threshold range \else  TOF曝光阈值范围 \endif */
    OB_STRUCT_DEVICE_SERIAL_NUMBER           = 1035, /**< \if English get/set serial number \else  获取/设置序列号 \endif */
    OB_STRUCT_DEVICE_TIME                    = 1037, /**< \if English get/set device time \else 获取/设置设备时间 \endif */ 
    OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG       = 1038, /**< \if English Multi-device synchronization mode and parameter configuration \else  多设备同步模式和参数配置 \endif */
    OB_STRUCT_RGB_CROP_ROI                   = 1040, /**< \if English RGB cropping ROI \else  RGB裁剪ROI \endif */

    OB_PROP_COLOR_AUTO_EXPOSURE_BOOL         = 2000, /**< \if English Color camera auto exposure \else  彩色相机自动曝光 \endif */
    OB_PROP_COLOR_EXPOSURE_INT               = 2001, /**< \if English Color camera exposure adjustment \else  彩色相机曝光调节 \endif */
    OB_PROP_COLOR_GAIN_INT                   = 2002, /**< \if English Color camera gain adjustment \else  彩色相机增益调节 \endif */
    OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL    = 2003, /**< \if English Color camera automatic white balance \else  彩色相机自动白平衡 \endif */
    OB_PROP_COLOR_WHITE_BALANCE_INT          = 2004, /**< \if English Color camera white balance adjustment \else  彩色相机白平衡调节 \endif */
    OB_PROP_COLOR_BRIGHTNESS_INT             = 2005, /**< \if English Color camera brightness adjustment \else  彩色相机亮度调节 \endif */
    OB_PROP_COLOR_SHARPNESS_INT              = 2006, /**< \if English Color camera sharpness adjustment \else  彩色相机锐度调节 \endif */
    OB_PROP_COLOR_SATURATION_INT             = 2008, /**< \if English Color camera saturation adjustment \else  彩色相机饱和度调节 \endif */
    OB_PROP_COLOR_CONTRAST_INT               = 2009, /**< \if English Color camera contrast adjustment \else  彩色相机对比度调节 \endif */
    OB_PROP_COLOR_GAMMA_INT                  = 2010, /**< \if English Color camera gamma adjustment \else  彩色相机伽马值调节 \endif */
    OB_PROP_COLOR_ROLL_INT                   = 2011, /**< \if English Color camera image rotation\else  彩色相机图像旋转 \endif */
    OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT = 2012, /**< \if English Color camera auto exposure priority \else  彩色相机自动曝光优先 \endif */
    OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT = 2013, /**< \if English Color camera brightness compensation \else  彩色相机亮度补偿 \endif */
    OB_PROP_COLOR_HUE_INT                    = 2014, /**< \if English color camera color tint \else  彩色相机彩色色调 \endif */
    OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT   = 2015, /**< \if English Color Camera Power Line Frequency \else  彩色相机电力线路频率 \endif */
    OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL         = 2016, /**< \if English Automatic exposure of depth camera (infrared camera will be set synchronously under some models of devices) \else  深度相机自动曝光（某些型号设备下会同步设置红外相机） \endif */
    OB_PROP_DEPTH_EXPOSURE_INT               = 2017, /**< \if English Depth camera exposure adjustment (infrared cameras will be set synchronously under some models of devices) \else  深度相机曝光调节（某些型号设备下会同步设置红外相机） \endif */
    OB_PROP_DEPTH_GAIN_INT                   = 2018, /**< \if English Depth camera gain adjustment (infrared cameras will be set synchronously under some models of devices) \else  深度相机增益调节（某些型号设备下会同步设置红外相机） \endif */
    OB_PROP_IR_AUTO_EXPOSURE_BOOL            = 2025, /**< \if English Infrared camera auto exposure (depth camera will be set synchronously under some models of devices) \else  红外相机自动曝光（某些型号设备下会同步设置深度相机） \endif */
    OB_PROP_IR_EXPOSURE_INT                  = 2026, /**< \if English Infrared camera exposure adjustment (some models of devices will set the depth camera synchronously) \else  红外相机曝光调节（某些型号设备下会同步设置深度相机） \endif */
    OB_PROP_IR_GAIN_INT                      = 2027, /**< \if English Infrared camera gain adjustment (the depth camera will be set synchronously under some models of devices) \else  红外相机增益调节（某些型号设备下会同步设置深度相机） \endif */

    OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL = 3004, /**< 视差转深度 */
    OB_PROP_SDK_DEPTH_FRAME_UNPACK_BOOL = 3007, /**< Depth数据解包功能开关(每次开流都会默认打开，支持RLE/Y10/Y11/Y12/Y14格式) */
    OB_PROP_SDK_IR_FRAME_UNPACK_BOOL    = 3008, /**< Ir数据解包功能开关(每次开流都会默认打开，支持RLE/Y10/Y11/Y12/Y14格式) */

    OB_STRUCT_DEBUG_SENSOR_EXPOSURE_TIME = 5504, /**< \if English LDP status \else  曝光时间读写 \endif */

} OBPropertyID,
    ob_property_id;

/**
 * \if English
 * @brief The data type used to describe all property settings
 * \else
 * @brief 用于描述所有属性设置的数据类型
 * \endif
 */
typedef enum OBPropertyType {
    OB_BOOL_PROPERTY   = 0, /**< \if English Bool property \else  bool类型数据的属性 \endif */
    OB_INT_PROPERTY    = 1, /**< \if English int property \else  int类型数据的属性\endif */
    OB_FLOAT_PROPERTY  = 2, /**< \if English float property \else  float类型数据的属性 \endif */
    OB_STRUCT_PROPERTY = 3, /**< \if English struct property \else  struct类型数据的属性 \endif */
} OBPropertyType,
    ob_property_type;

/**
 * \if English
 * @brief Used to describe the characteristics of each property
 * \else
 * @brief 用于描述每一个属性的特性
 * \endif
 */
typedef struct OBPropertyItem {
    OBPropertyID     id;          ///< \if English Property id \else 属性id \endif
    const char      *name;        ///< \if English Property name \else 属性名字 \endif
    OBPropertyType   type;        ///< \if English Property type \else 属性类型 \endif
    OBPermissionType permission;  ///< \if English Property read and write permission \else 属性读写权限 \endif
} OBPropertyItem, ob_property_item;

#ifdef __cplusplus
}
#endif

#endif  // _OB_PROPERTY_H_
