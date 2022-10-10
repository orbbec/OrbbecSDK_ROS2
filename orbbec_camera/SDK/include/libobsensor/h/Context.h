/**
 * \if English
 * @file Context.h
 * @brief Context is a management class that describes the runtime of the SDK and is responsible for resource application and release of the SDK.
 * Context has the ability to manage multiple devices. It is responsible for enumerating devices, monitoring device callbacks, and enabling multi device
 * synchronization. \else
 * @file Context.h
 * @brief context是描述SDK的runtime一个管理类，负责SDK的资源申请与释放
 * context具备多设备的管理能力，负责枚举设备，监听设备回调，启用多设备同步等功能
 * \endif
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * \if English
 * @brief create context api
 *
 * @param[out] error record the error information
 * @return ob_context* return the context that created
 * \else
 * @brief 创建context的接口函数
 *
 * @param[out] error 记录错误信息
 * @return ob_context* 返回上下文环境
 * \endif
 */
ob_context *ob_create_context(ob_error **error);

/**
 * \if English
 * @brief create context with config
 *
 * @param[in] config_path Configure the path of the file, and return null if the default path is used.
 * @param[out] error Log error messages
 * @return ob_context* returns the context
 * \else
 * @brief 创建context的接口函数
 *
 * @param[in] config_path 配置文件的路径，如果使用默认路径则传NULL
 * @param[out] error 记录错误信息
 * @return ob_context* 返回上下文环境
 * \endif
 */
ob_context *ob_create_context_with_config(const char *config_path, ob_error **error);

/**
 * \if English
 * @brief Delete context
 *
 * @param[in] context The context to delete
 * @param[out] error Log error messages
 * \else
 * @brief 删除上下文环境
 *
 * @param[in] context 要删除的上下文环境
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_context(ob_context *context, ob_error **error);

/**
 * \if English
 * @brief Get the list of enumerated devices
 *
 * @param[in] context Context
 * @param[out] error Log error messages
 * @return ob_device_list* return device list object
 * \else
 * @brief 枚举设备列表
 *
 * @param[in] context 上下文环境
 * @param[out] error 记录错误信息
 * @return ob_device_list* 返回设备列表对象
 * \endif
 */
ob_device_list *ob_query_device_list(ob_context *context, ob_error **error);

/**
 * @brief 创建网络设备
 *
 * @param[in] context 上下文环境
 * @param[in] address 设备ip地址
 * @param[in] port 设备端口
 * @param[out] error 记录错误信息
 * @return[out] ob_device* 返回设备对象
 */
ob_device *ob_create_net_device(ob_context *context, const char *address, uint16_t port, ob_error **error);

/**
 * \if English
 * @brief Set device plug-in callback function
 * @attention The added and removed device list returned through the callback interface need to be released manually
 *
 * @param[in] context Context
 * @param[in] callback Callback triggered when the device is plugged and unplugged
 * @param[in] user_data You can pass in any user data and get it from the callback
 * @param[out] error Log error messages
 * \else
 * @brief 设置设备插拔回调函数
 * @attention 通过回调接口返回的added和removed设备列表，需要手动释放设备列表
 *
 * @param[in] context 上下文环境
 * @param[in] callback 设备插拔时触发的回调
 * @param[in] user_data 可以传入任意用户数据，并从回调中获取
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_set_device_changed_callback(ob_context *context, ob_device_changed_callback callback, void *user_data, ob_error **error);

/**
 * \if English
 * @brief Activate the multi-device synchronization function to synchronize the clock of the created device(the device needs to support this function)
 *
 * @param[in]  context Context
 * @param[in]  repeatInterval synchronization time interval (unit: ms; if repeatInterval=0, itmeans that it will only be synchronized once and will not be
 * executedregularly)
 * @param[out] error Log error messages
 * \else
 * @brief 启动多设备同步功能，同步已创建设备的时钟(需要使用的设备支持该功能)
 *
 * @param[in]  context 上下文环境
 * @param[in]  repeatInterval 定时同步时间间隔（单位ms；如果repeatInterval=0，表示只同步一次，不再定时执行）
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_enable_multi_device_sync(ob_context *context, uint64_t repeatInterval, ob_error **error);

/**
 * \if English
 * @brief Set the global log level and this will affect both the log level output to the console and the log output to the file
 *
 * @param[in] severity Output log level
 * @param[out] error Log error messages
 * \else
 * @brief 设置全局日志的等级，会同时作用于输出到console和输出到文件的日志等级
 *
 * @param[in] severity 输出日志等级
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_set_logger_severity(ob_log_severity severity, ob_error **error);

/**
 * \if English
 * @brief Set output log to file
 *
 * @param[in] severity log level output to file
 * @param[in] directory The log file output path. If the path is empty, the existing settings will continue to be used (if the existing configuration is also
 * empty, the log will not be output to the file)
 * @param[out] error Log error messages
 * \else
 * @brief 设置输出日志到文件
 *
 * @param[in] severity 输出到文件的日志等级
 * @param[in] directory 日志文件输出路径，如果路径为空，则继续使用已有设置(已有配置也为空则不输出日志到文件)
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_set_logger_to_file(ob_log_severity severity, const char *directory, ob_error **error);

/**
 * \if English
 /**
 * \if English
 * @brief Set the output log to the console
 *
 * @param[in] log Log level
 * @param[out] error Log error messages
 * \else
 * @brief 设置输出日志到控制台
 *
 * @param[in] severity 日志的等级
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_set_logger_to_console(ob_log_severity severity, ob_error **error);

#ifdef __cplusplus
}
#endif