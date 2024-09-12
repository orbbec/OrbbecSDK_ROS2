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
 * @brief Create a context object
 *
 * @param[out] error Pointer to an error object that will be populated if an error occurs during context creation
 * @return Pointer to the created context object
 */
ob_context *ob_create_context(ob_error **error);

/**
 * @brief Create a context object with a specified configuration file
 *
 * @param[in] config_path Path to the configuration file. If NULL, the default configuration file will be used.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during context creation
 * @return Pointer to the created context object
 */
ob_context *ob_create_context_with_config(const char *config_path, ob_error **error);

/**
 * @brief Delete a context object
 *
 * @param[in] context Pointer to the context object to be deleted
 * @param[out] error Pointer to an error object that will be populated if an error occurs during context deletion
 */
void ob_delete_context(ob_context *context, ob_error **error);

/**
 * @brief Get a list of enumerated devices
 *
 * @param[in] context Pointer to the context object
 * @param[out] error Pointer to an error object that will be populated if an error occurs during device enumeration
 * @return Pointer to the device list object
 */
ob_device_list *ob_query_device_list(ob_context *context, ob_error **error);

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
void ob_enable_net_device_enumeration(ob_context *context, bool enable, ob_error **error);

/**
 * @brief Create a network device object
 *
 * @param[in] context Pointer to the context object
 * @param[in] address IP address of the device
 * @param[in] port Port number of the device
 * @param[out] error Pointer to an error object that will be populated if an error occurs during device creation
 * @return Pointer to the created device object
 */
ob_device *ob_create_net_device(ob_context *context, const char *address, uint16_t port, ob_error **error);

/**
 * @brief Set a device plug-in callback function
 * @attention The added and removed device lists returned through the callback interface need to be released manually
 *
 * @param[in] context Pointer to the context object
 * @param[in] callback Pointer to the callback function triggered when a device is plugged or unplugged
 * @param[in] user_data Pointer to user data that can be passed to and retrieved from the callback function
 * @param[out] error Pointer to an error object that will be populated if an error occurs during callback function setting
 */
void ob_set_device_changed_callback(ob_context *context, ob_device_changed_callback callback, void *user_data, ob_error **error);

/**
 * @brief Activates device clock synchronization to synchronize the clock of the host and all created devices (if supported).
 *
 * @param[in] context Pointer to the context object
 * @param[in] repeatInterval The interval for auto-repeated synchronization, in milliseconds. If the value is 0, synchronization is performed only once.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during execution
 */
void ob_enable_device_clock_sync(ob_context *context, uint64_t repeatInterval, ob_error **error);
#define ob_enable_multi_device_sync ob_enable_device_clock_sync

/**
 * @brief Free idle memory from the internal frame memory pool
 *
 * @param[in] context Pointer to the context object
 * @param[out] error Pointer to an error object that will be populated if an error occurs during memory freeing
 */
void ob_free_idle_memory(ob_context *context, ob_error **error);

/**
 * @brief Set the global log level
 *
 * @attention This interface setting will affect the output level of all logs (terminal, file, callback)
 *
 * @param[in] severity Log level to set
 * @param[out] error Pointer to an error object that will be populated if an error occurs during log level setting
 */
void ob_set_logger_severity(ob_log_severity severity, ob_error **error);

/**
 * @brief Set the log output to a file
 *
 * @param[in] severity Log level to output to file
 * @param[in] directory Path to the log file output directory. If the path is empty, the existing settings will continue to be used (if the existing
 * configuration is also empty, the log will not be output to the file)
 * @param[out] error Pointer to an error object that will be populated if an error occurs during log output setting
 */
void ob_set_logger_to_file(ob_log_severity severity, const char *directory, ob_error **error);

/**
 * @brief Set the log callback function
 *
 * @param[in] severity Log level to set for the callback function
 * @param[in] callback Pointer to the callback function
 * @param[in] user_data Pointer to user data that can be passed to and retrieved from the callback function
 * @param[out] error Pointer to an error object that will be populated if an error occurs during log callback function setting
 */
void ob_set_logger_callback(ob_log_severity severity, ob_log_callback callback, void *user_data, ob_error **error);

/**
 * @brief Set the log output to the console
 *
 * @param[in] severity Log level to output to the console
 * @param[out] error Pointer to an error object that will be populated if an error occurs during log output setting
 */
void ob_set_logger_to_console(ob_log_severity severity, ob_error **error);

/**
 * @brief Load a license file
 *
 * @param[in] filePath Path to the license file
 * @param[in] key Decryption key. "OB_DEFAULT_DECRYPT_KEY" can be used to represent the default key.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during license loading
 */
void ob_load_license(const char *filePath, const char *key, ob_error **error);

/**
 * @brief Load a license from data
 *
 * @param[in] data Pointer to the license data
 * @param[in] dataLen Length of the license data
 * @param[in] key Decryption key. "OB_DEFAULT_DECRYPT_KEY" can be used to represent the default key.
 * @param[out] error Pointer to an error object that will be populated if an error occurs during license loading
 */
void ob_load_license_from_data(const char *data, uint32_t dataLen, const char *key, ob_error **error);

/**
 * @brief Set the UVC backend for the specified context
 * This function configures the Universal Video Class (UVC) backend for the given context, allowing the selection of a specific backend for
 * video capture operations.
 *
 * @attention This function is only supported on Linux (ARM) platforms.
 * Some devices, like the Dabai series, do not support V4L2. Therefore, the default backend is LIBUVC. Ensure that the device
 * supports V4L2 before setting it as the backend.
 *
 * @param[in] context Pointer to the context object
 * @param[in] uvc_backend Specifies the UVC backend to use:
 *                       - `UVC_BACKEND_AUTO`: Automatically selects between
 *                         V4L2 or libuvc based on metadata support.
 *                       - `UVC_BACKEND_LIBUVC`: Forces the use of libuvc.
 *                       - `UVC_BACKEND_V4L2`: Forces the use of V4L2.
 * @param[out] error Pointer to an error object that will be populated if an error occurs.
 */
void ob_set_uvc_backend(ob_context *context, ob_uvc_backend uvc_backend, ob_error **error);

#ifdef __cplusplus
}
#endif