/**
 * @file Device.h
 * @brief Device-related functions, including operations such as obtaining and creating a device, setting and obtaining device property, and obtaining sensors
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"
#include "Property.h"
#include "MultipleDevices.h"

/**
 * @brief Get the number of devices
 *
 * @param[in] list Device list object
 * @param[out] error Log error messages
 * @return uint32_t return the number of devices
 */
uint32_t ob_device_list_device_count(ob_device_list *list, ob_error **error);

/**
 * @brief Get device name (DEPRECATED)
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return const char* return device name
 */
const char *ob_device_list_get_device_name(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the pid of the specified device
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return int return the device pid
 */
int ob_device_list_get_device_pid(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the vid of the specified device
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return int return device vid
 */
int ob_device_list_get_device_vid(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the uid of the specified device
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return const char* return the device uid
 */
const char *ob_device_list_get_device_uid(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the serial number of the specified device.
 *
 * @param[in] list Device list object.
 * @param[in] index Device index.
 * @param[out] error Log error messages.
 * @return const char* The device UID.
 */
const char *ob_device_list_get_device_serial_number(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get device connection type
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return const char* returns the device connection type，currently supports："USB", "USB1.0", "USB1.1", "USB2.0", "USB2.1", "USB3.0", "USB3.1", "USB3.2",
 * "Ethernet"
 */
const char *ob_device_list_get_device_connection_type(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get device ip address
 *
 * @attention Only valid for network devices, otherwise it will return "0.0.0.0".
 *
 * @param list Device list object
 * @param index Device index
 * @param error Log error messages
 * @return const char* returns the device ip address，such as "192.168.1.10"
 */
const char *ob_device_list_get_device_ip_address(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the device extension information.
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return const char* The device extension information
 */
const char *ob_device_list_get_extension_info(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Create a device.
 *
 * @attention If the device has already been acquired and created elsewhere, repeated acquisitions will return an error.
 *
 * @param[in] list Device list object.
 * @param[in] index The index of the device to create.
 * @param[out] error Log error messages.
 * @return ob_device* The created device.
 *
 */
ob_device *ob_device_list_get_device(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Create a device.
 *
 * @attention If the device has already been acquired and created elsewhere, repeated acquisitions will return an error.
 *
 * @param[in] list Device list object.
 * @param[in] serial_number The serial number of the device to create.
 * @param[out] error Log error messages.
 * @return ob_device* The created device.
 */
ob_device *ob_device_list_get_device_by_serial_number(ob_device_list *list, const char *serial_number, ob_error **error);

/**
 * @brief Create device by uid
 * @brief On Linux platform, the uid of the device is composed of bus-port-dev, for example 1-1.2-1. But the SDK will remove the dev number and only keep the
 * bus-port as the uid to create the device, for example 1-1.2, so that we can create a device connected to the specified USB port. Similarly, users can also
 * directly pass in bus-port as uid to create device.
 *
 * @attention If the device has already been acquired and created elsewhere, repeated acquisitions will return an error.
 *
 * @param[in] list Device list object.
 * @param[in] uid The UID of the device to create.
 * @param[out] error Log error messages.
 * @return ob_device* The created device.
 */
ob_device *ob_device_list_get_device_by_uid(ob_device_list *list, const char *uid, ob_error **error);

/**
 * @brief Delete a device.
 *
 * @param[in] device The device to be deleted.
 * @param[out] error Log error messages.
 */
void ob_delete_device(ob_device *device, ob_error **error);

/**
 * @brief Delete device information.
 *
 * @param[in] info The device information to be deleted.
 * @param[out] error Log error messages.
 */
void ob_delete_device_info(ob_device_info *info, ob_error **error);

/**
 * @brief Delete a device list.
 *
 * @param[in] list The device list object to be deleted.
 * @param[out] error Log error messages.
 */
void ob_delete_device_list(ob_device_list *list, ob_error **error);

/**
 * @brief Get device information.
 *
 * @param[in] device The device to obtain information from.
 * @param[out] error Log error messages.
 * @return ob_device_info* The device information.
 */
ob_device_info *ob_device_get_device_info(ob_device *device, ob_error **error);

/**
 * @brief List all sensors.
 *
 * @param[in] device The device object.
 * @param[out] error Log error messages.
 * @return ob_sensor_list* The list of all sensors.
 */
ob_sensor_list *ob_device_get_sensor_list(ob_device *device, ob_error **error);

/**
 * @brief Get a device's sensor.
 *
 * @param[in] device The device object.
 * @param[in] type The type of sensor to get.
 * @param[out] error Log error messages.
 * @return ob_sensor* The acquired sensor.
 */
ob_sensor *ob_device_get_sensor(ob_device *device, ob_sensor_type type, ob_error **error);

/**
 * @brief Set an integer type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property to be set.
 * @param[in] property The property value to be set.
 * @param[out] error Log error messages.
 */
void ob_device_set_int_property(ob_device *device, ob_property_id property_id, int32_t property, ob_error **error);

/**
 * @brief Get an integer type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property ID.
 * @param[out] error Log error messages.
 * @return int32_t The property value.
 */
int32_t ob_device_get_int_property(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Set a float type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property to be set.
 * @param[in] property The property value to be set.
 * @param[out] error Log error messages.
 */
void ob_device_set_float_property(ob_device *device, ob_property_id property_id, float property, ob_error **error);

/**
 * @brief Get a float type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property ID.
 * @param[out] error Log error messages.
 * @return float The property value.
 */
float ob_device_get_float_property(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Set a boolean type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property to be set.
 * @param[in] property The property value to be set.
 * @param[out] error Log error messages.
 */
void ob_device_set_bool_property(ob_device *device, ob_property_id property_id, bool property, ob_error **error);

/**
 * @brief Get a boolean type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property ID.
 * @param[out] error Log error messages.
 * @return bool The property value.
 */
bool ob_device_get_bool_property(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Set structured data.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property to be set.
 * @param[in] data The property data to be set.
 * @param[in] data_size The size of the property to be set.
 * @param[out] error Log error messages.
 */
void ob_device_set_structured_data(ob_device *device, ob_property_id property_id, const void *data, uint32_t data_size, ob_error **error);

/**
 * @brief Get structured data of a device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property.
 * @param[out] data The obtained property data.
 * @param[out] data_size The size of the obtained property data.
 * @param[out] error Log error messages.
 */
void ob_device_get_structured_data(ob_device *device, ob_property_id property_id, void *data, uint32_t *data_size, ob_error **error);

/**
 * @brief Set structured data of a device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property.
 * @param[in] data_bundle The target data to set.
 * @param[in] cb The data callback.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Log error messages.
 */
void ob_device_set_structured_data_ext(ob_device *device, ob_property_id property_id, ob_data_bundle *data_bundle, ob_set_data_callback cb, void *user_data,
                                       ob_error **error);

/**
 * @brief Get structured data of a device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property.
 * @param[out] error Log error messages.
 * @return ob_data_bundle. NOTE: ob_data_bundle must be freed by ob_delete_data_bundle() because it comes from OrbbecSDK's API.
 */
ob_data_bundle *ob_device_get_structured_data_ext(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Set raw data of a device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property to be set.
 * @param[in] data The property data to be set.
 * @param[in] data_size The size of the property data to be set.
 * @param[in] cb The set data callback.
 * @param[in] async Whether to execute asynchronously.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Log error messages.
 */
void ob_device_set_raw_data(ob_device *device, ob_property_id property_id, void *data, uint32_t data_size, ob_set_data_callback cb, bool async, void *user_data,
                            ob_error **error);

/**
 * @brief Get raw data of a device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property.
 * @param[in] cb The get data callback.
 * @param[in] async Whether to execute asynchronously.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Log error messages.
 */
void ob_device_get_raw_data(ob_device *device, ob_property_id property_id, ob_get_data_callback cb, bool async, void *user_data, ob_error **error);

/**
 * @brief Get the protocol version of the device.
 *
 * @param[in] device The device object.
 * @param[out] error Log error messages.
 * @return The protocol version of the device.
 */
ob_protocol_version ob_device_get_protocol_version(ob_device *device, ob_error **error);

/**
 * @brief Get the cmdVersion of a property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property id.
 * @param[out] error Log error messages.
 * @return The cmdVersion of the property.
 */
ob_cmd_version ob_device_get_cmd_version(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Get the number of properties supported by the device.
 *
 * @param[in] device The device object.
 * @param[out] error Log error messages.
 * @return The number of properties supported by the device.
 */
uint32_t ob_device_get_supported_property_count(ob_device *device, ob_error **error);

/**
 * @brief Get the type of property supported by the device.
 *
 * @param[in] device The device object.
 * @param[in] index The property index.
 * @param[out] error Log error messages.
 * @return The type of property supported by the device.
 */
ob_property_item ob_device_get_supported_property(ob_device *device, uint32_t index, ob_error **error);

/**
 * @brief Check if a device property permission is supported.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property id.
 * @param[in] permission The type of permission that needs to be interpreted.
 * @param[out] error Log error messages.
 * @return Whether the property permission is supported.
 */
bool ob_device_is_property_supported(ob_device *device, ob_property_id property_id, ob_permission_type permission, ob_error **error);

/**
 * @brief Get the integer type of device property range.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property id.
 * @param[out] error Log error messages.
 * @return The property range.
 */
ob_int_property_range ob_device_get_int_property_range(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Get the float type of device property range.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property id.
 * @param[out] error Log error messages.
 * @return The property range.
 */
ob_float_property_range ob_device_get_float_property_range(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Get the boolean type of device property range.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property id.
 * @param[out] error Log error messages.
 * @return The property range.
 */
ob_bool_property_range ob_device_get_bool_property_range(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Write to an AHB register.
 *
 * @param[in] device The device object.
 * @param reg The register to be written.
 * @param mask The mask.
 * @param value The value to be written.
 * @param[out] error Log error messages.
 */
void ob_device_write_ahb(ob_device *device, uint32_t reg, uint32_t mask, uint32_t value, ob_error **error);

/**
 * @brief Read an AHB register.
 *
 * @param[in] device The device object.
 * @param reg The register to be read.
 * @param mask The mask.
 * @param value The value to be read.
 * @param[out] error Log error messages.
 */
void ob_device_read_ahb(ob_device *device, uint32_t reg, uint32_t mask, uint32_t *value, ob_error **error);

/**
 * @brief Write to an I2C register.
 *
 * @param[in] device The device object.
 * @param module_id The I2C module id to be written.
 * @param reg The register to be written.
 * @param mask The mask.
 * @param value The value to be written.
 * @param[out] error Log error messages.
 */
void ob_device_write_i2c(ob_device *device, uint32_t module_id, uint32_t reg, uint32_t mask, uint32_t value, ob_error **error);

/**
 * @brief Read an I2C register.
 *
 * @param[in] device The device object.
 * @param module_id The id of the I2C module to be read.
 * @param reg The register to be read.
 * @param mask The mask.
 * @param value The value to be read.
 * @param[out] error Log error messages.
 */
void ob_device_read_i2c(ob_device *device, uint32_t module_id, uint32_t reg, uint32_t mask, uint32_t *value, ob_error **error);

/**
 * @brief Set the properties of writing to Flash [Asynchronous Callback].
 *
 * @param[in] device The device object.
 * @param offset The flash offset address.
 * @param data The property data to be written.
 * @param data_size The size of the property to be written.
 * @param cb The set data callback.
 * @param[in] async Whether to execute asynchronously.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Log error messages.
 */
void ob_device_write_flash(ob_device *device, uint32_t offset, const void *data, uint32_t data_size, ob_set_data_callback cb, bool async, void *user_data,
                           ob_error **error);

/**
 * @brief Read Flash properties [asynchronous callback].
 *
 * @param[in] device The device object.
 * @param offset The flash offset address.
 * @param data_size The size of the data to be read.
 * @param cb The read flash data and progress callback.
 * @param[in] async Whether to execute asynchronously.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Log error messages.
 */
void ob_device_read_flash(ob_device *device, uint32_t offset, uint32_t data_size, ob_get_data_callback cb, bool async, void *user_data, ob_error **error);

/**
 * @brief Set customer data.
 *
 * @param[in] device The device object.
 * @param[in] data The property data to be set.
 * @param[in] data_size The size of the property to be set,the maximum length cannot exceed 65532 bytes.
 * @param[out] error Log error messages.
 */
void ob_device_write_customer_data(ob_device *device, const void *data, uint32_t data_size, ob_error **error);

/**
 * @brief Get customer data of a device property.
 *
 * @param[in] device The device object.
 * @param[out] data The obtained property data.
 * @param[out] data_size The size of the obtained property data.
 * @param[out] error Log error messages.
 */
void ob_device_read_customer_data(ob_device *device, void *data, uint32_t *data_size, ob_error **error);

/**
 * @brief Upgrade the device firmware.
 *
 * @param[in] device The device object.
 * @param[in] path The firmware path.
 * @param[in] callback The firmware upgrade progress callback.
 * @param[in] async Whether to execute asynchronously.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Log error messages.
 */
void ob_device_upgrade(ob_device *device, const char *path, ob_device_upgrade_callback callback, bool async, void *user_data, ob_error **error);

/**
 * @brief Upgrade the device firmware.
 *
 * @param[in] device The device object.
 * @param[in] file_data The firmware file data.
 * @param[in] file_size The firmware file size.
 * @param[in] callback The firmware upgrade progress callback.
 * @param[in] async Whether to execute asynchronously.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Log error messages.
 */
void ob_device_upgrade_from_data(ob_device *device, const char *file_data, uint32_t file_size, ob_device_upgrade_callback callback, bool async, void *user_data,
                                 ob_error **error);

/**
 * @brief Get the current device status.
 *
 * @param[in] device The device object.
 * @param[out] error Log error messages.
 *
 * @return ob_device_state The device state information.
 */
ob_device_state ob_device_get_device_state(ob_device *device, ob_error **error);

/**
 * @brief Monitor device state changes.
 *
 * @param[in] device The device object.
 * @param[in] callback The callback function to be called when the device status changes.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Log error messages.
 */
void ob_device_state_changed(ob_device *device, ob_device_state_callback callback, void *user_data, ob_error **error);

/**
 * @brief Send files to the specified path on the device.
 *
 * @param[in] device The device object.
 * @param[in] file_path The source file path.
 * @param[in] dst_path The destination path on the device.
 * @param[in] callback The file sending progress callback.
 * @param[in] async Whether to execute asynchronously.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Log error messages.
 */
void ob_device_send_file_to_destination(ob_device *device, const char *file_path, const char *dst_path, ob_file_send_callback callback, bool async,
                                        void *user_data, ob_error **error);

/**
 * @brief Verify the device authorization code.
 *
 * @param[in] device The device object.
 * @param[in] auth_code The authorization code.
 * @param[out] error Log error messages.
 *
 * @return bool Whether the activation is successful.
 */
bool ob_device_activate_authorization(ob_device *device, const char *auth_code, ob_error **error);

/**
 * @brief Write the device authorization code.
 *
 * @param[in] device The device object.
 * @param[in] auth_code The authorization code.
 * @param[out] error Log error messages.
 */
void ob_device_write_authorization_code(ob_device *device, const char *auth_code, ob_error **error);

/**
 * @brief Get the original parameter list of camera calibration saved on the device.
 *
 * @attention The parameters in the list do not correspond to the current open-stream configuration.You need to select the parameters according to the actual
 * situation, and may need to do scaling, mirroring and other processing. Non-professional users are recommended to use the ob_pipeline_get_camera_param()
 * interface.
 *
 * @param[in] device The device object.
 * @param[out] error Log error messages.
 *
 * @return ob_camera_param_list The camera parameter list.
 */
ob_camera_param_list *ob_device_get_calibration_camera_param_list(ob_device *device, ob_error **error);

/**
 * @brief Get the current depth work mode.
 *
 * @param[in] device The device object.
 * @param[out] error Log error messages.
 *
 * @return ob_depth_work_mode The current depth work mode.
 */
ob_depth_work_mode ob_device_get_current_depth_work_mode(ob_device *device, ob_error **error);

/**
 * @brief Switch the depth work mode by ob_depth_work_mode.
 *        Prefer to use ob_device_switch_depth_work_mode_by_name to switch depth mode when the complete name of the depth work mode is known.
 *
 * @param[in] device The device object.
 * @param[in] work_mode The depth work mode from ob_depth_work_mode_list which is returned by ob_device_get_depth_work_mode_list.
 * @param[out] error Log error messages.
 *
 * @return ob_status The switch result. OB_STATUS_OK: success, other failed.
 */
ob_status ob_device_switch_depth_work_mode(ob_device *device, const ob_depth_work_mode *work_mode, ob_error **error);

/**
 * @brief Switch the depth work mode by work mode name.
 *
 * @param[in] device The device object.
 * @param[in] mode_name The depth work mode name which is equal to ob_depth_work_mode.name.
 * @param[out] error Log error messages.
 *
 * @return ob_status The switch result. OB_STATUS_OK: success, other failed.
 */
ob_status ob_device_switch_depth_work_mode_by_name(ob_device *device, const char *mode_name, ob_error **error);

/**
 * @brief Request the list of supported depth work modes.
 *
 * @param[in] device The device object.
 * @param[out] error Log error messages.
 *
 * @return ob_depth_work_mode_list The list of ob_depth_work_mode.
 */
ob_depth_work_mode_list *ob_device_get_depth_work_mode_list(ob_device *device, ob_error **error);

/**
 * @brief Device reboot
 * @attention The device will be disconnected and reconnected. After the device is disconnected, the interface access to the device handle may be abnormal.
 * Please use the ob_delete_device interface to delete the handle directly. After the device is reconnected, it can be obtained again.
 *
 * @param[in] device Device object
 * @param[out] error Log error messages
 */
void ob_device_reboot(ob_device *device, ob_error **error);

/**
 * @brief Get the current device synchronization configuration
 * @brief Device synchronization: including exposure synchronization function and multi-camera synchronization function of different sensors within a single
 * machine
 *
 * @param[in] device Device object
 * @param[out] error Log error messages
 * @return ob_device_sync_config Return the device synchronization configuration
 */
ob_device_sync_config ob_device_get_sync_config(ob_device *device, ob_error **error);

/**
 * @brief Set the device synchronization configuration
 * @brief Used to configure the exposure synchronization function and multi-camera synchronization function of different sensors in a single machine
 *
 * @attention Calling this function will directly write the configuration to the device Flash, and it will still take effect after the device restarts. To avoid
 * affecting the Flash lifespan, do not update the configuration frequently.
 *
 * @param[in] device Device object
 * @param[out] device_sync_config Device synchronization configuration
 * @param[out] error Log error messages
 */
void ob_device_set_sync_config(ob_device *device, ob_device_sync_config device_sync_config, ob_error **error);

/**
 * @brief Get device name
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* return the device name
 */
const char *ob_device_info_name(ob_device_info *info, ob_error **error);

/**
* @brief Get device pid

 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return int return the device pid
*/
int ob_device_info_pid(ob_device_info *info, ob_error **error);

/**
 * @brief Get device vid
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return int return device vid
 */
int ob_device_info_vid(ob_device_info *info, ob_error **error);

/**
 * @brief Get device uid
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* return device uid
 */
const char *ob_device_info_uid(ob_device_info *info, ob_error **error);

/**
 * @brief Get device serial number
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* return device serial number
 */
const char *ob_device_info_serial_number(ob_device_info *info, ob_error **error);

/**
 * @brief Get the firmware version number
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return int return the firmware version number
 */
const char *ob_device_info_firmware_version(ob_device_info *info, ob_error **error);

/**
 * @brief Get the USB connection type (DEPRECATED)
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* The USB connection type
 */
const char *ob_device_info_usb_type(ob_device_info *info, ob_error **error);

/**
 * @brief Get the device connection type
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* The connection type，currently supports："USB", "USB1.0", "USB1.1", "USB2.0", "USB2.1", "USB3.0", "USB3.1", "USB3.2", "Ethernet"
 */
const char *ob_device_info_connection_type(ob_device_info *info, ob_error **error);

/**
 * @brief Get the device IP address
 *
 * @attention Only valid for network devices, otherwise it will return "0.0.0.0"
 *
 * @param info Device Information
 * @param error Log error messages
 * @return const char* The IP address，such as "192.168.1.10"
 */
const char *ob_device_info_ip_address(ob_device_info *info, ob_error **error);

/**
 * @brief Get the hardware version number
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* The hardware version number
 */
const char *ob_device_info_hardware_version(ob_device_info *info, ob_error **error);

/**
 * @brief Get the device extension information.
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* The device extension information
 */
const char *ob_device_info_get_extension_info(ob_device_info *info, ob_error **error);

/**
 * @brief Get the minimum SDK version number supported by the device
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* The minimum SDK version number supported by the device
 */
const char *ob_device_info_supported_min_sdk_version(ob_device_info *info, ob_error **error);

/**
 * @brief Get the chip name
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* The ASIC name
 */
const char *ob_device_info_asicName(ob_device_info *info, ob_error **error);

/**
 * @brief Get the device type
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return ob_device_type The device type
 */
ob_device_type ob_device_info_device_type(ob_device_info *info, ob_error **error);

/**
 * @brief Get the number of camera parameter lists
 *
 * @param param_list Camera parameter list
 * @param error Log error messages
 * @return uint32_t The number of lists
 */
uint32_t ob_camera_param_list_count(ob_camera_param_list *param_list, ob_error **error);

/**
 * @brief Get camera parameters from the camera parameter list
 *
 * @param param_list Camera parameter list
 * @param index Parameter index
 * @param error Log error messages
 * @return ob_camera_param The camera parameters. Since it returns the structure object directly, there is no need to provide a delete interface.
 */
ob_camera_param ob_camera_param_list_get_param(ob_camera_param_list *param_list, uint32_t index, ob_error **error);

/**
 * @brief Delete the camera parameter list
 *
 * @param param_list Camera parameter list
 * @param error Log error messages
 */
void ob_delete_camera_param_list(ob_camera_param_list *param_list, ob_error **error);

/**
 * \if English
 * @brief Get the depth work mode count that ob_depth_work_mode_list hold
 * @param[in] work_mode_list data struct contain list of ob_depth_work_mode
 * @param[out] error Log error messages
 * @return The total number contained in ob_depth_work_mode_list
 *
 */
uint32_t ob_depth_work_mode_list_count(ob_depth_work_mode_list *work_mode_list, ob_error **error);

/**
 * @brief Get the index target of ob_depth_work_mode from work_mode_list
 *
 * @param[in] work_mode_list Data structure containing a list of ob_depth_work_mode
 * @param[in] index Index of the target ob_depth_work_mode
 * @param[out] error Log error messages
 * @return ob_depth_work_mode
 *
 */
ob_depth_work_mode ob_depth_work_mode_list_get_item(ob_depth_work_mode_list *work_mode_list, uint32_t index, ob_error **error);

/**
 * @brief Free the resources of ob_depth_work_mode_list
 *
 * @param[in] work_mode_list Data structure containing a list of ob_depth_work_mode
 * @param[out] error Log error messages
 *
 */
void ob_delete_depth_work_mode_list(ob_depth_work_mode_list *work_mode_list, ob_error **error);

/**
 * @brief Free the resources of data_bundle which come from OrbbecSDK's API
 *
 * @param data_bundle Data bundle
 * @param[out] error Log error messages
 */
void ob_delete_data_bundle(ob_data_bundle *data_bundle, ob_error **error);

/**
 * @brief Check if the device supports global timestamp.
 *
 * @param[in] device The device object.
 * @param[out] error Log error messages.
 * @return bool Whether the device supports global timestamp.
 */
bool ob_device_is_global_timestamp_supported(ob_device *device, ob_error **error);

/**
 * @brief Load depth filter config from file.
 * @param[in] device The device object.
 * @param[in] file_path Path of the config file.
 * @param[out] error Log error messages.
 */
void ob_device_load_depth_filter_config(ob_device *device, const char *file_path, ob_error **error);

/**
 * @brief Reset depth filter config to device default define.
 * @param[in] device The device object.
 * @param[out] error Log error messages.
 */
void ob_device_reset_default_depth_filter_config(ob_device *device, ob_error **error);

/**
 * @breif Get the current preset name.
 * @brief The preset mean a set of parameters or configurations that can be applied to the device to achieve a specific effect or function.
 *
 * @param device The device object.
 * @param error  Log error messages.
 * @return  The current preset name, it should be one of the preset names returned by @ref ob_device_get_available_preset_list.
 */
const char *ob_device_get_current_preset_name(ob_device *device, ob_error **error);

/**
 * @brief Get the available preset list.
 * @attention After loading the preset, the settings in the preset will set to the device immediately. Therefore, it is recommended to re-read the device
 * settings to update the user program temporarily.
 *
 * @param device The device object.
 * @param preset_name  Log error messages. The name should be one of the preset names returned by @ref ob_device_get_available_preset_list.
 * @param error  Log error messages.
 */
void ob_device_load_preset(ob_device *device, const char *preset_name, ob_error **error);

/**
 * @brief Load preset from json string.
 * @brief After loading the custom preset, the settings in the custom preset will set to the device immediately.
 * @brief After loading the custom preset, the available preset list will be appended with the custom preset and named as the file name.
 *
 * @param device   The device object.
 * @param json_file_path  The json file path.
 * @param error   Log error messages.
 */
void ob_device_load_preset_from_json_file(ob_device *device, const char *json_file_path, ob_error **error);

/**
 * @brief Export current settings as a preset json file.
 * @brief After exporting the custom preset, the available preset list will be appended with the custom preset and named as the file name.
 *
 * @param device   The device object.
 * @param json_file_path  The json file path.
 * @param error   Log error messages.
 */
void ob_device_export_current_settings_as_preset_json_file(ob_device *device, const char *json_file_path, ob_error **error);

/**
 * @brief Get the available preset list.
 *
 * @param device The device object.
 * @param error  Log error messages.
 * @return  The available preset list.
 */
ob_device_preset_list *ob_device_get_available_preset_list(ob_device *device, ob_error **error);

/**
 * @brief Delete the available preset list.
 *
 * @param preset_list The available preset list.
 * @param error  Log error messages.
 */
void ob_delete_preset_list(ob_device_preset_list *preset_list, ob_error **error);

/**
 * @brief Get the number of preset in the preset list.
 *
 * @param preset_list The available preset list.
 * @param error  Log error messages.
 * @return  The number of preset in the preset list.
 */
uint32_t ob_device_preset_list_count(ob_device_preset_list *preset_list, ob_error **error);

/**
 * @brief Get the name of the preset in the preset list.
 *
 * @param preset_list The available preset list.
 * @param index  The index of the preset in the preset list.
 * @param error  Log error messages.
 * @return  The name of the preset in the preset list.
 */
const char *ob_device_preset_list_get_name(ob_device_preset_list *preset_list, uint32_t index, ob_error **error);

/**
 * @brief Check if the preset list has the preset.
 *
 * @param preset_list The available preset list.
 * @param preset_name  The name of the preset.
 * @param error  Log error messages.
 * @return  Whether the preset list has the preset. If true, the preset list has the preset. If false, the preset list does not have the preset.
 */
bool ob_device_preset_list_has_preset(ob_device_preset_list *preset_list, const char *preset_name, ob_error **error);

#ifdef __cplusplus
}
#endif
