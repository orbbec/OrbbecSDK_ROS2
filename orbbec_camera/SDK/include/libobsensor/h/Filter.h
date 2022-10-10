/**
 * \if English
 * @file Filter.h
 * @brief The processing unit of the SDK can perform point cloud generation, format conversion and other functions.

 * \else
 * @file Filter.h
 * @brief SDK的处理单元，可以进行点云的生成，格式转换等功能
 * \endif
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * \if English
 * @brief Create PointCloud Filter
 *
 * @param[out] error Log error messages
 *
 * @return filter pointcloud_filter object
 * \else
 * @brief 创建PointCloud Filter
 *
 * @param[out] error 记录错误信息
 *
 * @return filter pointcloud_filter对象
 * \endif
 */
ob_filter *ob_create_pointcloud_filter(ob_error **error);

/**
 * \if English
 * @brief PointCloud Filter device camera parameters
 *
 * @param[in] filter pointcloud_filter object
 * @param[in] param Camera parameters
 * @param[out] error Log error messages
 * \else
 * @brief PointCloud Filter设备相机参数
 *
 * @param[in] filter pointcloud_filter对象
 * @param[in] param 相机参数
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pointcloud_filter_set_camera_param(ob_filter *filter, ob_camera_param param, ob_error **error);

/**
 * \if English
 * @brief Set point cloud type parameters
 *
 * @param[in] filter pointcloud_filter object
 * @param[in] type Point cloud type: depth point cloud or RGBD point cloud
 * @param[out] error Log error messages
 * \else
 * @brief 设置点云类型参数
 *
 * @param[in] filter pointcloud_filter对象
 * @param[in] type 点云类型 深度点云或RGBD点云
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pointcloud_filter_set_point_format(ob_filter *filter, ob_format type, ob_error **error);

/**
 * \if English
 * @brief  Set the alignment state of the frames that will be input to produce the point cloud
 * @param[in] filter pointcloud_filter object
 * @param[in] state Alignment status, True: aligned; False: unaligned
 * @param[out] error Log error messages
 * \else
 * @brief  设置将要输入用于生产点云的帧的对齐状态
 * @param[in] filter pointcloud_filter对象
 * @param[in] state 对齐状态，True：已对齐； False：未对齐
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pointcloud_filter_set_frame_align_state(ob_filter *filter, bool state, ob_error **error);

/**
 * \if English
 * @brief  Set point cloud position data scale ratio
 * @param[in] filter pointcloud_filter object
 * @param[in] scale scale ratio
 * @param[out] error Log error messages
 * \else
 * @brief  设置点云数据缩放比例
 * @param[in] filter pointcloud_filter对象
 * @param[in] scale 设置点云坐标数据缩放比例
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pointcloud_filter_set_position_data_scale(ob_filter *filter, float scale, ob_error **error);

/**
 * \if English
 * @brief  Set point cloud color data normalization
 * @param[in] filter pointcloud_filter object
 * @param[in] state Whether to normalize
 * @param[out] error Log error messages
 * \else
 * @brief  设置点云颜色数据归一化
 * @param[in] filter pointcloud_filter对象
 * @param[in] state 设置点云颜色数据是否归一化
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_pointcloud_filter_set_color_data_normalization(ob_filter *filter, bool state, ob_error **error);

/**
 * \if English
 * @brief Create FormatConvet Filter
 *
 * @param[out] error Log error messages
 *
 * @return filter format_convert object
 * \else
 * @brief 创建FormatConvet Filter
 *
 * @param[out] error 记录错误信息
 *
 * @return filter format_convert 对象
 * \endif
 */
ob_filter *ob_create_format_convert_filter(ob_error **error);

/**
 * \if English
 * @brief Set the type of format conversion
 *
 * @param[in] filter formatconvet_filter object
 * @param[in] type Format conversion type
 * @param[out] error Log error messages
 * \else
 * @brief 设置格式转化的类型
 *
 * @param[in] filter formatconvet_filter对象
 * @param[in] type 格式转化类型
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_format_convert_filter_set_format(ob_filter *filter, ob_convert_format type, ob_error **error);

/**
 * \if English
 * @brief  Filter reset, cache clear, state reset. If the asynchronous interface is used, the processing thread will also be stopped and the pending cache
 * frames will be cleared.
 *
 * @param[in] filter filter object
 * @param[out] error Log error messages
 * \else
 * @brief  Filter重置, 缓存清空，状态复位。如果是使用异步方式接口，还会停止处理线程，清空待处理的缓存帧
 *
 * @param[in] filter filter对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_filter_reset(ob_filter *filter, ob_error **error);

/**
 * \if English
 * @brief Filter processing (synchronous interface)
 *
 * @param[in] filter filter object
 * @param[in] frame pointer to the frame object to be processed
 * @param[out] error Log error messages
 *
 * @return ob_frame  The frame object processed by the filter
 * \else
 * @brief Filter 处理(同步接口)
 *
 * @param[in] filter filter对象
 * @param[in] frame 需要被处理的frame对象指针
 * @param[out] error 记录错误信息
 *
 * @return ob_frame  filter处理后的frame对象
 * \endif
 */
ob_frame *ob_filter_process(ob_filter *filter, ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Filter Set the processing result callback function (asynchronous callback interface)
 *
 * @param[in] filter filter object
 * @param[in] callback
 * @param[in] user_data Arbitrary user data pointer can be passed in and returned from the callback
 * @param[out] error Log error messages
 * \else
 * @brief Filter 设置处理结果回调函数(异步回调接口)
 *
 * @param[in] filter filter对象
 * @param[in] callback 回调函数
 * @param[in] user_data 可以传入任意用户数据指针，并从回调返回该数据指针
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_filter_set_callback(ob_filter *filter, ob_filter_callback callback, void *user_data, ob_error **error);

/**
 * \if English
 * @brief filter Push the frame into the pending cache (asynchronous callback interface)
 *
 * @param[in] filter filter object
 * @param[out] error Log error messages
 * \else
 * @brief filter 压入frame到待处理缓存(异步回调接口)
 *
 * @param[in] filter filter对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_filter_push_frame(ob_filter *filter, ob_frame *frame, ob_error **error);

/**
 * \if English
 * @brief Delete Filter
 *
 * @param[in] filter filter object
 * @param[out] error Log error messages
 * \else
 * @brief 删除Filter
 *
 * @param[in] filter filter 对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_filter(ob_filter *filter, ob_error **error);

#ifdef __cplusplus
}
#endif