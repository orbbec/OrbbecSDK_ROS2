// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Filter.h
 * @brief The processing unit of the SDK can perform point cloud generation, format conversion and other functions.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Create a Filter object.
 *
 * @attention If the filter of the specified name is a private filter, and the creator of the filter have not been activated, the function will return NULL.
 *
 * @param[in] name The name of the filter.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT ob_filter *ob_create_filter(const char *name, ob_error **error);

/**
 * @brief Get the name of ob_filter
 *
 * @param[in] filter ob_filter object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return char The filter of name
 */
OB_EXPORT const char *ob_filter_get_name(const ob_filter *filter, ob_error **error);

/**
 * @brief Get the vendor specific code of a filter by filter name.
 * @brief A private filter can define its own vendor specific code for specific purposes.
 *
 * @param[in] name The name of the filter.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* Return the vendor specific code of the filter.
 */
OB_EXPORT const char *ob_filter_get_vendor_specific_code(const char *name, ob_error **error);

/**
 * @brief Create a private Filter object with activation key.
 * @brief Some private filters require an activation key to be activated, its depends on the vendor of the filter.
 *
 * @param[in] name The name of the filter.
 * @param[in] activation_key The activation key of the filter.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_filter* Return the private filter object.
 */
OB_EXPORT ob_filter *ob_create_private_filter(const char *name, const char *activation_key, ob_error **error);

/**
 * @brief Delete the filter.
 *
 * @param[in] filter The filter object to be deleted.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_filter(ob_filter *filter, ob_error **error);

/**
 * @brief Get config schema of the filter
 * @brief The returned string is a csv format string representing the configuration schema of the filter. The format of the string is:
 *  `<parameter_name>`, `<parameter_type: "int", "float", "bool">`, `<minimum_value>`, `<maximum_value>`, `<value_step>`, `<default_value>`,
 * `<parameter_description>`
 *
 * @param[in] filter The filter object to get the configuration schema for
 * @param[out] error Pointer to an error object that will be set if an error occurs
 *
 * @return A csv format string representing the configuration schema of the filter
 */
OB_EXPORT const char *ob_filter_get_config_schema(const ob_filter *filter, ob_error **error);

/**
 * @brief Get the filter config schema list of the filter
 * @brief The returned string is a list of ob_config_schema_item representing the configuration schema of the filter.
 *
 * @attention The returned list should be deleted by calling @ref ob_delete_filter_config_schema_list when it is no longer needed.
 *
 * @param[in] filter The filter object to get the configuration schema for
 * @param[out] error Pointer to an error object that will be set if an error occurs
 *
 * @return ob_filter_config_schema_list* Return the filter config schema list of the filter
 */
OB_EXPORT ob_filter_config_schema_list *ob_filter_get_config_schema_list(const ob_filter *filter, ob_error **error);

/**
 * @brief Delete a list of filter config schema items.
 *
 * @param[in] config_schema_list The list of filter config schema items to delete.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_filter_config_schema_list(ob_filter_config_schema_list *config_schema_list, ob_error **error);

/**
 * @brief Update config of the filter
 *
 * @attention The passed in argc and argv must match the configuration schema returned by the @ref ob_filter_get_config_schema function.
 *
 * @param[in] filter The filter object to update the configuration for
 * @param[in] argc The number of arguments in the argv array
 * @param[in] argv An array of strings representing the configuration values
 * @param[out] error Pointer to an error object that will be set if an error occurs
 */
OB_EXPORT void ob_filter_update_config(ob_filter *filter, uint8_t argc, const char **argv, ob_error **error);

/**
 * @brief Get the filter config value by name and cast to double.
 *
 * @attention The returned value is cast to double, the actual type of the value depends on the filter config schema returned by @ref
 * ob_filter_get_config_schema.
 *
 * @param[in] filter A filter object.
 * @param[in] config_name config name
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return double The value of the config.
 */
OB_EXPORT double ob_filter_get_config_value(const ob_filter *filter, const char *config_name, ob_error **error);

/**
 * @brief Set the filter config value by name.
 *
 * @attention The pass into value type is double, witch will be cast to the actual type inside the filter. The actual type can be queried by the filter config
 * schema returned by @ref ob_filter_get_config_schema.
 *
 * @param[in] filter A filter object.
 * @param[in] config_name config name
 * @param[in] value The value to set.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_filter_set_config_value(ob_filter *filter, const char *config_name, double value, ob_error **error);

/**
 * @brief Reset the filter, clears the cache, and resets the state. If the asynchronous interface is used, the processing thread will also be stopped and the
 * pending cache frames will be cleared.
 *
 * @param[in] filter A filter object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_filter_reset(ob_filter *filter, ob_error **error);

/**
 * @brief Enable the frame post processing
 * @brief The filter default is enable.
 *
 * @attention If the filter has been disabled by calling this function, processing will directly output a clone of the input frame.
 *
 * @param[in] filter A filter object.
 * @param[in] enable enable status, true: enable; false: disable.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_filter_enable(ob_filter *filter, bool enable, ob_error **error);

/**
 * @brief Get the enable status of the frame post processing
 *
 * @attention If the filter is disabled, the processing will directly output a clone of the input frame.
 *
 * @param[in] filter A filter object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return The post processing filter status. True: enable; False: disable.
 */
OB_EXPORT bool ob_filter_is_enabled(const ob_filter *filter, ob_error **error);

/**
 * @brief Process the frame (synchronous interface).
 *
 * @param[in] filter A filter object.
 * @param[in] frame Pointer to the frame object to be processed.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return The frame object processed by the filter.
 */
OB_EXPORT ob_frame *ob_filter_process(ob_filter *filter, const ob_frame *frame, ob_error **error);

/**
 * @brief Set the processing result callback function for the filter (asynchronous callback interface).
 *
 * @param[in] filter A filter object.
 * @param[in] callback Callback function.
 * @param[in] user_data Arbitrary user data pointer can be passed in and returned from the callback.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_filter_set_callback(ob_filter *filter, ob_filter_callback callback, void *user_data, ob_error **error);

/**
 * @brief Push the frame into the pending cache for the filter (asynchronous callback interface).
 * @brief The frame will be processed by the filter when the processing thread is available and return a new processed frame to the callback function.
 *
 * @attention The frame object will be add reference count, so the user still need call @ref ob_delete_frame to release the frame after calling this function.
 *
 * @param[in] filter A filter object.
 * @param[in] frame Pointer to the frame object to be processed.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_filter_push_frame(ob_filter *filter, const ob_frame *frame, ob_error **error);

/**
 * @brief Get the number of filter in the list
 *
 * @param[in] filter_list  filter list
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint32_t The number of list
 */
OB_EXPORT uint32_t ob_filter_list_get_count(const ob_filter_list *filter_list, ob_error **error);

/**
 * @brief Get the filter by index
 *
 * @param[in] filter_list Filter list
 * @param[in] index  Filter index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_filter The index of ob_filter
 */
OB_EXPORT ob_filter *ob_filter_list_get_filter(const ob_filter_list *filter_list, uint32_t index, ob_error **error);

/**
 * @brief Delete a list of ob_filter objects.
 *
 * @param[in] filter_list The list of ob_filter objects to delete.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_filter_list(ob_filter_list *filter_list, ob_error **error);

/**
 * @brief Get the number of config schema items in the config schema list
 *
 * @param[in] config_schema_list Filter config schema list
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint32_t The number of config schema items in the filter list
 */
OB_EXPORT uint32_t ob_filter_config_schema_list_get_count(const ob_filter_config_schema_list *config_schema_list, ob_error **error);

/**
 * @brief Get the config schema item by index
 *
 * @param[in] config_schema_list Filter config schema list
 * @param[in] index Config schema item index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_filter_config_schema_item* The config schema item by index
 */
OB_EXPORT ob_filter_config_schema_item ob_filter_config_schema_list_get_item(const ob_filter_config_schema_list *config_schema_list, uint32_t index,
                                                                             ob_error **error);

/**
 * @brief Set the align to stream profile for the align filter.
 * @brief It is useful when the align target stream dose not started (without any frame to get intrinsics and extrinsics).
 *
 * @param[in] filter A filter object.
 * @param[in] align_to_stream_profile The align target stream profile.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_align_filter_set_align_to_stream_profile(ob_filter *filter, const ob_stream_profile *align_to_stream_profile, ob_error **error);

// The following interfaces are deprecated and are retained here for compatibility purposes.
#define ob_get_filter ob_filter_list_get_filter
#define ob_get_filter_name ob_filter_get_name

#ifdef __cplusplus
}
#endif

