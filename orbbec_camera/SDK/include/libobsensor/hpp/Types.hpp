/**
 * \if English
 * \file Types.hpp
 *    \brief  Provide SDK structure and enumeration constant definition (depending on libobsensor/h/ObTypes.h)
 * \else
 * \file Types.hpp
 *    \brief  提供SDK的结构体、枚举常量定义（依赖libobsensor/h/ObTypes.h）
 *
 * \endif
 *
 */
#pragma once

#include "libobsensor/h/ObTypes.h"

#include <functional>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \if English
 * @brief file transfer callbacks
 *
 * @param state file transfer status
 * @param message status information
 * \else
 * @brief 文件传输回调
 *
 * @param state 文件传输状态
 * @param message 状态信息
 * \endif
 */
using SendFileCallback = std::function<void(OBFileTranState state, const char *message, uint8_t percent)>;

/**
 * \if English
 * @brief device upgrade callback
 *
 * @param state upgrade status
 * @param message status information
 * \else
 * @brief 设备升级回调
 *
 * @param state 升级状态
 * @param message 状态信息
 * \endif
 */
using DeviceUpgradeCallback = std::function<void(OBUpgradeState state, const char *message, uint8_t percent)>;

/**
 * \if English
 * @brief device status callback
 *
 * @param state device status
 * @param message status information
 * \else
 * @brief 设备状态回调
 *
 * @param state 设备状态
 * @param message 状态信息
 * \endif
 */
using DeviceStateChangedCallback = std::function<void(OBDeviceState state, const char *message)>;

/**
 * \if English
 * @brief get raw data property data when data and progress callbacks
 *
 * @param dataChunk data chunk
 * @param state get the data status
 * \else
 * @brief 获取raw data属性数据时数据及进度回调
 *
 * @param dataChunk 数据块
 * @param state 获取数据状态
 * \endif
 */
using GetDataCallback = std::function<void(OBDataTranState state, OBDataChunk *dataChunk)>;

/**
 * \if English
 * @brief set the raw data property when the progress callback is made
 *
 * @param percent  progress percentage
 * @param state  set the data status
 * \else
 * @brief 设置raw data属性数据时进度回调
 *
 * @param percent  进度百分比
 * @param state  设置数据状态
 * \endif
 */
using SetDataCallback = std::function<void(OBDataTranState state, uint8_t percent)>;

#ifdef __cplusplus
}
#endif
