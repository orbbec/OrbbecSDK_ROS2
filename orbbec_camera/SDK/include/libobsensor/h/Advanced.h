// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Get the current depth work mode.
 *
 * @param[in] device The device object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_depth_work_mode The current depth work mode.
 */
OB_EXPORT ob_depth_work_mode ob_device_get_current_depth_work_mode(const ob_device *device, ob_error **error);

/**
 * @brief Get current depth mode name
 * @brief According the current preset name to return current depth mode name
 * @return const char* return the current depth mode name.
 */
OB_EXPORT const char *ob_device_get_current_depth_work_mode_name(const ob_device *device, ob_error **error);

/**
 * @brief Switch the depth work mode by ob_depth_work_mode.
 *        Prefer to use ob_device_switch_depth_work_mode_by_name to switch depth mode when the complete name of the depth work mode is known.
 *
 * @param[in] device The device object.
 * @param[in] work_mode The depth work mode from ob_depth_work_mode_list which is returned by ob_device_get_depth_work_mode_list.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_status The switch result. OB_STATUS_OK: success, other failed.
 */
OB_EXPORT ob_status ob_device_switch_depth_work_mode(ob_device *device, const ob_depth_work_mode *work_mode, ob_error **error);

/**
 * @brief Switch the depth work mode by work mode name.
 *
 * @param[in] device The device object.
 * @param[in] mode_name The depth work mode name which is equal to ob_depth_work_mode.name.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_status The switch result. OB_STATUS_OK: success, other failed.
 */
OB_EXPORT ob_status ob_device_switch_depth_work_mode_by_name(ob_device *device, const char *mode_name, ob_error **error);

/**
 * @brief Request the list of supported depth work modes.
 *
 * @param[in] device The device object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_depth_work_mode_list The list of ob_depth_work_mode.
 */
OB_EXPORT ob_depth_work_mode_list *ob_device_get_depth_work_mode_list(const ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Get the depth work mode count that ob_depth_work_mode_list hold
 * @param[in] work_mode_list data struct contain list of ob_depth_work_mode
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return The total number contained in ob_depth_work_mode_list
 *
 */
OB_EXPORT uint32_t ob_depth_work_mode_list_get_count(const ob_depth_work_mode_list *work_mode_list, ob_error **error);

/**
 * @brief Get the index target of ob_depth_work_mode from work_mode_list
 *
 * @param[in] work_mode_list Data structure containing a list of ob_depth_work_mode
 * @param[in] index Index of the target ob_depth_work_mode
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_depth_work_mode
 *
 */
OB_EXPORT ob_depth_work_mode ob_depth_work_mode_list_get_item(const ob_depth_work_mode_list *work_mode_list, uint32_t index, ob_error **error);

/**
 * @brief Free the resources of ob_depth_work_mode_list
 *
 * @param[in] work_mode_list Data structure containing a list of ob_depth_work_mode
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 */
OB_EXPORT void ob_delete_depth_work_mode_list(ob_depth_work_mode_list *work_mode_list, ob_error **error);

/**
 * @breif Get the current preset name.
 * @brief The preset mean a set of parameters or configurations that can be applied to the device to achieve a specific effect or function.
 *
 * @param device The device object.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return  The current preset name, it should be one of the preset names returned by @ref ob_device_get_available_preset_list.
 */
OB_EXPORT const char *ob_device_get_current_preset_name(const ob_device *device, ob_error **error);

/**
 * @brief Get the available preset list.
 * @attention After loading the preset, the settings in the preset will set to the device immediately. Therefore, it is recommended to re-read the device
 * settings to update the user program temporarily.
 *
 * @param device The device object.
 * @param preset_name  Pointer to an error object that will be set if an error occurs. The name should be one of the preset names returned by @ref
 * ob_device_get_available_preset_list.
 * @param error  Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_load_preset(ob_device *device, const char *preset_name, ob_error **error);

/**
 * @brief Load preset from json string.
 * @brief After loading the custom preset, the settings in the custom preset will set to the device immediately.
 * @brief After loading the custom preset, the available preset list will be appended with the custom preset and named as the file name.
 *
 * @param device   The device object.
 * @param json_file_path  The json file path.
 * @param error   Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_load_preset_from_json_file(ob_device *device, const char *json_file_path, ob_error **error);

/**
 * @brief Load custom preset from data.
 * @brief After loading the custom preset, the settings in the custom preset will set to the device immediately.
 * @brief After loading the custom preset, the available preset list will be appended with the custom preset and named as the @ref presetName.
 *
 * @attention The user should ensure that the custom preset data is adapted to the device and the settings in the data are valid.
 * @attention It is recommended to re-read the device settings to update the user program temporarily after successfully loading the custom preset.
 *
 * @param data The custom preset data.
 * @param size The size of the custom preset data.
 */
OB_EXPORT void ob_device_load_preset_from_json_data(ob_device *device, const char *presetName, const uint8_t *data, uint32_t size, ob_error **error);

/**
 * @brief Export current settings as a preset json file.
 * @brief After exporting the custom preset, the available preset list will be appended with the custom preset and named as the file name.
 *
 * @param device   The device object.
 * @param json_file_path  The json file path.
 * @param error   Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_export_current_settings_as_preset_json_file(ob_device *device, const char *json_file_path, ob_error **error);

/**
 * @brief Export current device settings as a preset json data.
 * @brief After exporting the preset, a new preset named as the @ref presetName will be added to the available preset list.
 *
 * @attention The memory of the data is allocated by the SDK, and will automatically be released by the SDK.
 * @attention The memory of the data will be reused by the SDK on the next call, so the user should copy the data to a new buffer if it needs to be
 * preserved.
 *
 * @param[out] data return the preset json data.
 * @param[out] dataSize return the size of the preset json data.
 */
OB_EXPORT void ob_device_export_current_settings_as_preset_json_data(ob_device *device, const char *presetName, const uint8_t **data, uint32_t *dataSize,
                                                                     ob_error **error);

/**
 * @brief Get the available preset list.
 *
 * @param device The device object.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return  The available preset list.
 */
OB_EXPORT ob_device_preset_list *ob_device_get_available_preset_list(const ob_device *device, ob_error **error);

/**
 * @brief Delete the available preset list.
 *
 * @param preset_list The available preset list.
 * @param error  Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_preset_list(ob_device_preset_list *preset_list, ob_error **error);

/**
 * @brief Get the number of preset in the preset list.
 *
 * @param preset_list The available preset list.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return  The number of preset in the preset list.
 */
OB_EXPORT uint32_t ob_device_preset_list_get_count(const ob_device_preset_list *preset_list, ob_error **error);

/**
 * @brief Get the name of the preset in the preset list.
 *
 * @param preset_list The available preset list.
 * @param index  The index of the preset in the preset list.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return  The name of the preset in the preset list.
 */
OB_EXPORT const char *ob_device_preset_list_get_name(const ob_device_preset_list *preset_list, uint32_t index, ob_error **error);

/**
 * @brief Check if the preset list has the preset.
 *
 * @param preset_list The available preset list.
 * @param preset_name  The name of the preset.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return  Whether the preset list has the preset. If true, the preset list has the preset. If false, the preset list does not have the preset.
 */
OB_EXPORT bool ob_device_preset_list_has_preset(const ob_device_preset_list *preset_list, const char *preset_name, ob_error **error);

/**
 * @brief Check if the device supports the frame interleave feature.
 *
 * @param device The device object.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return bool Returns true if the device supports the frame interleave feature.
 */
OB_EXPORT bool ob_device_is_frame_interleave_supported(const ob_device *device, ob_error **error);
/**
 *
 * @brief load the frame interleave mode according to frame interleavee name.
 *
 * @param device The device object.
 * @param frame_interleave_name  The name should be one of the frame interleave names returned by @ref ob_device_get_available_frame_interleave_list.
 * @param error  Log error messages.
 */
OB_EXPORT void ob_device_load_frame_interleave(ob_device *device, const char *frame_interleave_name, ob_error **error);

/**
 * @brief Get the available frame interleave list.
 *
 * @param device The device object.
 * @param error  Log error messages.
 * @return The available frame interleave list.
 */
OB_EXPORT ob_device_frame_interleave_list *ob_device_get_available_frame_interleave_list(ob_device *device, ob_error **error);

/**
 * @brief Delete the available frame interleave list.
 *
 * @param frame_interleave_list The available frame interleave list.
 * @param error  Log error messages.
 */
OB_EXPORT void ob_delete_frame_interleave_list(ob_device_frame_interleave_list *frame_interleave_list, ob_error **error);

/**
 * @brief Get the number of frame interleave in the frame interleave list.
 *
 * @param frame_interleave_list The available frame interleave list.
 * @param error  Log error messages.
 * @return  The number of frame interleave in the frame interleave list.
 */
OB_EXPORT uint32_t ob_device_frame_interleave_list_get_count(ob_device_frame_interleave_list *frame_interleave_list, ob_error **error);

/**
 * @brief Get the name of frame interleave in the frame interleave list.
 *
 * @param frame_interleave_list The available frame interleave list.
 * @param index  The index of frame interleave in the frame interleave list.
 * @param error  Log error messages.
 * @return  The name of frame interleave in the frame interleave list..
 */
OB_EXPORT const char *ob_device_frame_interleave_list_get_name(ob_device_frame_interleave_list *frame_interleave_list, uint32_t index, ob_error **error);

/**
 * @brief Check if the interleave ae list has the interleave ae.
 *
 * @param frame_interleave_list The available interleave ae list.
 * @param frame_interleave_name  The name of the interleave ae.
 * @param error  Log error messages.
 * @return  Whether the interleave ae list has the interleave ae. If true, the interleave ae list has the interleave ae. If false, the interleave ae list does
 * not have the interleave ae.
 */
OB_EXPORT bool ob_device_frame_interleave_list_has_frame_interleave(ob_device_frame_interleave_list *frame_interleave_list, const char *frame_interleave_name,
                                                                    ob_error **error);

/* @brief Get the available preset resolution config list.
 *
 * @param device The device object.
 * @param error  Log error messages.
 * @return The available frame interleave list.
 *
 */
OB_EXPORT ob_preset_resolution_config_list *ob_device_get_available_preset_resolution_config_list(ob_device *device, ob_error **error);

/* @brief Get the number of preset resolution in the preset resolution list.
 *
 * @param ob_preset_resolution_config_list The available preset resolution list.
 * @param error  Log error messages.
 * @return  The number of preset resolution in the preset resolution list.
 */
OB_EXPORT uint32_t ob_device_preset_resolution_config_get_count(ob_preset_resolution_config_list *ob_preset_resolution_config_list, ob_error **error);

/**
 * @brief Get the preset resolution in the preset resolution list.
 *
 * @param ob_preset_resolution_config_list The available preset resolution list.
 * @param index  The index of preset resolution in the preset resolution list.
 * @param error  Log error messages.
 * @return  The preset resolution in the preset resolution list.
 */
OB_EXPORT OBPresetResolutionConfig ob_device_preset_resolution_config_list_get_item(const ob_preset_resolution_config_list *ob_preset_resolution_config_lis,
                                                                                    uint32_t index, ob_error **error);

/**
 * @brief Delete the available preset resolution list.
 *
 * @param frame_interleave_list The available preset resolution list.
 * @param error  Log error messages.
 */
OB_EXPORT void ob_delete_preset_resolution_config_list(ob_preset_resolution_config_list *ob_preset_resolution_config_list, ob_error **error);

// The following interfaces are deprecated and are retained here for compatibility purposes.
#define ob_depth_work_mode_list_count ob_depth_work_mode_list_get_count
#define ob_device_preset_list_count ob_device_preset_list_get_count

#ifdef __cplusplus
}
#endif
