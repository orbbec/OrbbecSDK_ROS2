// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Context.h
 * @brief Context is a management class that describes the runtime of the SDK and is responsible for resource allocation and release of the SDK.
 * Context has the ability to manage multiple devices. It is responsible for enumerating devices, monitoring device callbacks, and enabling multi-device
 * synchronization.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Create a context object with the default configuration file
 *
 * @param[out] error Pointer to an error object that will be populated if an error occurs during context creation
 *
 * @return Pointer to the created context object
 */
OB_EXPORT ob_context *ob_create_context(ob_error **error);

/**
 * @brief Create a context object with a specified configuration file
 *
 * @param[in] config_file_path Path to the configuration file. If NULL, the default configuration file will be used.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during context creation
 *
 * @return Pointer to the created context object
 */
OB_EXPORT ob_context *ob_create_context_with_config(const char *config_file_path, ob_error **error);

/**
 * @brief Delete a context object
 *
 * @param[in] context Pointer to the context object to be deleted
 * @param[out] error Pointer to an error object that will be populated if an error occurs during context deletion
 */
OB_EXPORT void ob_delete_context(ob_context *context, ob_error **error);

/**
 * @brief Get a list of enumerated devices
 *
 * @param[in] context Pointer to the context object
 * @param[out] error Pointer to an error object that will be populated if an error occurs during device enumeration
 *
 * @return Pointer to the device list object
 */
OB_EXPORT ob_device_list *ob_query_device_list(ob_context *context, ob_error **error);

/**
 * @brief Enable or disable network device enumeration
 * @brief After enabling, the network device will be automatically discovered and can be retrieved through @ref ob_query_device_list. The default state can be
 * set in the configuration file.
 *
 * @attention Network device enumeration is performed through the GVCP protocol. If the device is not in the same subnet as the host, it will be discovered but
 * cannot be connected.
 *
 * @param[in] context Pointer to the context object
 * @param[in] enable true to enable, false to disable
 * @param[out] error Pointer to an error object that will be populated if an error occurs.
 */
OB_EXPORT void ob_enable_net_device_enumeration(ob_context *context, bool enable, ob_error **error);

/**
 * @brief "Force" a static IP address configuration in a device identified by its MAC Address.
 *
 * @param[in] macAddress MAC address of the network device.
 *                       You can obtain it from @ref ob_device_info_get_uid, or specify it manually
 *                       in the format xx:xx:xx:xx:xx:xx, where each xx is a two-digit hexadecimal value.
 * @param[in] config The new IP configuration.
 * @param[out] error Pointer to an error object that will be populated if an error occurs.
 *
 * @return bool true if the configuration command was processed successfully, false otherwise.
 *
 * @note This applies to all GigE Vision devices
 */
OB_EXPORT bool ob_force_ip_config(const char *macAddress, ob_net_ip_config config, ob_error **error);

/**
 * @brief Create a network device object
 *
 * @param[in] context Pointer to the context object
 * @param[in] address IP address of the device
 * @param[in] port Port number of the device
 * @param[out] error Pointer to an error object that will be populated if an error occurs during device creation
 *
 * @return Pointer to the created device object
 */
OB_EXPORT ob_device *ob_create_net_device(ob_context *context, const char *address, uint16_t port, ob_error **error);

/**
 * @brief Create a network device object
 *
 * @param[in] context Pointer to the context object
 * @param[in] address IP address of the device
 * @param[in] port Port number of the device
 * @param[in] accessMode Device access mode. @ref ob_device_access_mode.
 *                       If the device does not support setting the Access Mode, the default OB_DEVICE_DEFAULT_ACCESS is used.
 *                       Applies only on first device creation or after release and re-creation; subsequent calls ignore it.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during device creation
 *
 * @return Pointer to the created device object
 */
OB_EXPORT ob_device *ob_create_net_device_ex(ob_context *context, const char *address, uint16_t port, ob_device_access_mode accessMode, ob_error **error);

/**
 * @brief Set a device plug-in callback function
 * 
 * @deprecated This function is deprecated. Please use ob_register_device_changed_callback() instead.
 *
 * @attention The added and removed device lists returned through the callback interface need to be released manually
 * @attention This function supports multiple callbacks. Each call to this function adds a new callback to an internal list.
 *
 * @param[in] context Pointer to the context object
 * @param[in] callback Pointer to the callback function triggered when a device is plugged or unplugged
 * @param[in] user_data Pointer to user data that can be passed to and retrieved from the callback function
 * @param[out] error Pointer to an error object that will be populated if an error occurs during callback function setting
 */
OB_EXPORT void ob_set_device_changed_callback(ob_context *context, ob_device_changed_callback callback, void *user_data, ob_error **error);

/**
 * @brief Register a device plug-in callback function.
 *
 * This function registers a callback that will be triggered whenever a device is plugged in or unplugged.
 *
 * @attention The added and removed device lists returned through the callback interface need to be released manually.
 * @attention This function supports multiple callbacks. Each call to this function adds a new callback to an internal list.
 *
 * @param[in] context Pointer to the context object.
 * @param[in] callback Pointer to the callback function triggered when a device is plugged or unplugged.
 * @param[in] user_data Pointer to user data that can be passed to and retrieved from the callback function.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during callback function setting.
 *
 * @return A ob_callback_id that uniquely identifies the registered callback.
 *         This ID can be used later to unregister the callback with ob_unregister_device_changed_callback().
 *         Returns INVALID_DEVICE_CHANGED_CALLBACK_ID if the registration fails.
 */
OB_EXPORT ob_callback_id ob_register_device_changed_callback(ob_context *context, ob_device_changed_callback callback, void *user_data, ob_error **error);

/**
 * @brief Unregister a previously registered device plug-in callback function.
 *
 * This function removes a callback that was registered by ob_register_device_changed_callback().
 *
 * @param[in] context Pointer to the context object.
 * @param[in] callback_id The callback ID returned by ob_register_device_changed_callback().
 * @param[out] error Pointer to an error object that will be populated if an error occurs during callback function setting.
 */
OB_EXPORT void ob_unregister_device_changed_callback(ob_context *context, ob_callback_id callback_id, ob_error **error);

/**
 * @brief Activates device clock synchronization to synchronize the clock of the host and all created devices (if supported).
 *
 * @param[in] context Pointer to the context object
 * @param[in] repeat_interval_msec The interval for auto-repeated synchronization, in milliseconds. If the value is 0, synchronization is performed only once.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during execution
 */
OB_EXPORT void ob_enable_device_clock_sync(ob_context *context, uint64_t repeat_interval_msec, ob_error **error);

/**
 * @brief Free idle memory from the internal frame memory pool
 *
 * @param[in] context Pointer to the context object
 * @param[out] error Pointer to an error object that will be populated if an error occurs during memory freeing
 */
OB_EXPORT void ob_free_idle_memory(ob_context *context, ob_error **error);

/**
 * @brief For linux, there are two ways to enable the UVC backend: libuvc and v4l2. This function is used to set the backend type.
 * @brief It is effective when the new device is created.
 *
 * @attention This interface is only available for Linux.
 *
 * @param[in] context Pointer to the context object
 * @param[in] backend_type The backend type to be used.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during backend type setting
 */
OB_EXPORT void ob_set_uvc_backend_type(ob_context *context, ob_uvc_backend_type backend_type, ob_error **error);

/**
 * @brief Set the global log level
 *
 * @attention This interface setting will affect the output level of all logs (terminal, file, callback)
 *
 * @param[in] severity Log level to set
 * @param[out] error Pointer to an error object that will be populated if an error occurs during log level setting
 */
OB_EXPORT void ob_set_logger_severity(ob_log_severity severity, ob_error **error);

/**
 * @brief Set the log output to a file
 *
 * @param[in] severity Log level to output to file
 * @param[in] directory Path to the log file output directory.
 *                      If empty, the existing configuration is used;
 *                      if the existing configuration is also empty,
 *                      the value from the config file or the default configuration will be used.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during log output setting
 *
 * @note Other settings, such as log file name, remain unchanged.
 */
OB_EXPORT void ob_set_logger_to_file(ob_log_severity severity, const char *directory, ob_error **error);

/**
 * @brief Set the log file name for file output
 *
 * @param[in] file_name Log file name. Must not be empty.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @note Other settings, such as log level and output directory, remain unchanged.
 */
OB_EXPORT void ob_set_logger_file_name(const char *file_name, ob_error **error);

/**
 * @brief Set the log callback function
 *
 * @param[in] severity Log level to set for the callback function
 * @param[in] callback Pointer to the callback function
 * @param[in] user_data Pointer to user data that can be passed to and retrieved from the callback function
 * @param[out] error Pointer to an error object that will be populated if an error occurs during log callback function setting
 */
OB_EXPORT void ob_set_logger_to_callback(ob_log_severity severity, ob_log_callback callback, void *user_data, ob_error **error);

/**
 * @brief Set the log output to the console
 *
 * @param[in] severity Log level to output to the console
 * @param[out] error Pointer to an error object that will be populated if an error occurs during log output setting
 */
OB_EXPORT void ob_set_logger_to_console(ob_log_severity severity, ob_error **error);

/**
 * @brief Logs a message with severity, file, function, and line info.
 *
 * @param[in] severity Log level, see @ref ob_log_severity for details
 * @param[in] module The module or component the log belongs to
 * @param[in] message Message string to log
 * @param[in] file Source file name, e.g., __FILE__
 * @param[in] func Function name, e.g., __func__
 * @param[in] line Line number, e.g., __LINE__
 * @param[out] error Pointer to an error object that will be populated if an error occurs during log output setting
 */
OB_EXPORT void ob_log_external_message(ob_log_severity severity, const char *module, const char *message, const char *file, const char *func, int line,
                                       ob_error **error);

/**
 * @brief Set the extensions directory
 * @brief The extensions directory is used to search for dynamic libraries that provide additional functionality to the SDK, such as the Frame filters.
 *
 * @attention Should be called before creating the context and pipeline, otherwise the default extensions directory (./extensions) will be used.
 *
 * @param[in] directory Path to the extensions directory. If the path is empty, extensions path will be set to the current working directory.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during extensions directory setting
 */
OB_EXPORT void ob_set_extensions_directory(const char *directory, ob_error **error);

// The following interfaces are deprecated and are retained here for compatibility purposes.
#define ob_enable_multi_device_sync ob_enable_device_clock_sync
#define ob_set_logger_callback ob_set_logger_to_callback

#ifdef __cplusplus
}
#endif
