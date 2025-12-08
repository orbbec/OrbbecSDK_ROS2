// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

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
#include "Advanced.h"

/**
 * @brief Delete a device.
 *
 * @param[in] device The device to be deleted.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_device(ob_device *device, ob_error **error);

/**
 * @brief List all sensors.
 *
 * @param[in] device The device object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_sensor_list* The list of all sensors.
 */
OB_EXPORT ob_sensor_list *ob_device_get_sensor_list(const ob_device *device, ob_error **error);

/**
 * @brief Get a device's sensor.
 *
 * @param[in] device The device object.
 * @param[in] type The type of sensor to get.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_sensor* The acquired sensor.
 */
OB_EXPORT ob_sensor *ob_device_get_sensor(ob_device *device, ob_sensor_type type, ob_error **error);

/**
 * @brief Set an integer type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property to be set.
 * @param[in] value The property value to be set.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_set_int_property(ob_device *device, ob_property_id property_id, int32_t value, ob_error **error);

/**
 * @brief Get an integer type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property ID.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return int32_t The property value.
 */
OB_EXPORT int32_t ob_device_get_int_property(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Get the integer type of device property range.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property id.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return The property range.
 */
OB_EXPORT ob_int_property_range ob_device_get_int_property_range(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Set a float type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property to be set.
 * @param[in] value The property value to be set.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_set_float_property(ob_device *device, ob_property_id property_id, float value, ob_error **error);

/**
 * @brief Get a float type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property ID.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return float The property value.
 */
OB_EXPORT float ob_device_get_float_property(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Get the float type of device property range.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property id.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return The property range.
 */
OB_EXPORT ob_float_property_range ob_device_get_float_property_range(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Set a boolean type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property to be set.
 * @param[in] value The property value to be set.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_set_bool_property(ob_device *device, ob_property_id property_id, bool value, ob_error **error);

/**
 * @brief Get a boolean type of device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property ID.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool The property value.
 */
OB_EXPORT bool ob_device_get_bool_property(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Get the boolean type of device property range.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property id.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return The property range.
 */
OB_EXPORT ob_bool_property_range ob_device_get_bool_property_range(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * @brief Set structured data.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property to be set.
 * @param[in] data The property data to be set.
 * @param[in] data_size The size of the property to be set.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_set_structured_data(ob_device *device, ob_property_id property_id, const uint8_t *data, uint32_t data_size, ob_error **error);

/**
 * @brief Get structured data of a device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property.
 * @param[out] data The obtained property data.
 * @param[out] data_size The size of the obtained property data.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_get_structured_data(ob_device *device, ob_property_id property_id, uint8_t *data, uint32_t *data_size, ob_error **error);

/**
 * @brief Get raw data of a device property.
 *
 * @param[in] device The device object.
 * @param[in] property_id The ID of the property.
 * @param[out] cb The get data callback.
 * @param[out] user_data User-defined data that will be returned in the callback.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_get_raw_data(ob_device *device, ob_property_id property_id, ob_get_data_callback cb, void *user_data, ob_error **error);

/**
 * @brief Set customer data.
 *
 * @param[in] device The device object.
 * @param[in] data The property data to be set.
 * @param[in] data_size The size of the property to be set,the maximum length cannot exceed 65532 bytes.
 * @param[out] error Log error messages.
 */
OB_EXPORT void ob_device_write_customer_data(ob_device *device, const void *data, uint32_t data_size, ob_error **error);

/**
 * @brief Get customer data of a device property.
 *
 * @param[in] device The device object.
 * @param[out] data The obtained property data.
 * @param[out] data_size The size of the obtained property data.
 * @param[out] error Log error messages.
 */
OB_EXPORT void ob_device_read_customer_data(ob_device *device, void *data, uint32_t *data_size, ob_error **error);

/**
 * @brief Get the number of properties supported by the device.
 *
 * @param[in] device The device object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return The number of properties supported by the device.
 */
OB_EXPORT uint32_t ob_device_get_supported_property_count(const ob_device *device, ob_error **error);

/**
 * @brief Get the type of property supported by the device.
 *
 * @param[in] device The device object.
 * @param[in] index The property index.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return The type of property supported by the device.
 */
OB_EXPORT ob_property_item ob_device_get_supported_property_item(const ob_device *device, uint32_t index, ob_error **error);

/**
 * @brief Check if a device property permission is supported.
 *
 * @param[in] device The device object.
 * @param[in] property_id The property id.
 * @param[in] permission The type of permission that needs to be interpreted.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return Whether the property permission is supported.
 */
OB_EXPORT bool ob_device_is_property_supported(const ob_device *device, ob_property_id property_id, ob_permission_type permission, ob_error **error);

/**
 * @brief Check if the device supports global timestamp.
 *
 * @param[in] device The device object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Whether the device supports global timestamp.
 */
OB_EXPORT bool ob_device_is_global_timestamp_supported(const ob_device *device, ob_error **error);

/**
 * @brief Enable or disable global timestamp.
 *
 * @param[in] device The device object.
 * @param[in] enable Whether to enable global timestamp.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_enable_global_timestamp(ob_device *device, bool enable, ob_error **error);

/**
 * @brief Update the device firmware.
 *
 * @param[in] device The device object.
 * @param[in] path The firmware path.
 * @param[in] callback The firmware upgrade progress callback.
 * @param[in] async Whether to execute asynchronously.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_update_firmware(ob_device *device, const char *path, ob_device_fw_update_callback callback, bool async, void *user_data,
                                         ob_error **error);

/**
 * @brief Update the device firmware from data.
 *
 * @param[in] device The device object.
 * @param[in] data The firmware file data.
 * @param[in] data_size The firmware file size.
 * @param[in] callback The firmware upgrade progress callback.
 * @param[in] async Whether to execute asynchronously.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_update_firmware_from_data(ob_device *device, const uint8_t *data, uint32_t data_size, ob_device_fw_update_callback callback,
                                                   bool async, void *user_data, ob_error **error);

/**
 * @brief Update the device optional depth presets.
 *
 * @param[in] device The device object.
 * @param[in] file_path_list A list(2D array) of preset file paths, each up to OB_PATH_MAX characters.
 * @param[in] path_count The number of the preset file paths.
 * @param[in] callback The preset upgrade progress callback.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_update_optional_depth_presets(ob_device *device, const char file_path_list[][OB_PATH_MAX], uint8_t path_count,
                                                       ob_device_fw_update_callback callback, void *user_data, ob_error **error);

/**
 * @brief Device reboot
 * @attention The device will be disconnected and reconnected. After the device is disconnected, the interface access to the device handle may be abnormal.
 * Please use the ob_delete_device interface to delete the handle directly. After the device is reconnected, it can be obtained again.
 *
 * @param[in] device Device object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_reboot(ob_device *device, ob_error **error);

/**
 * @brief Get the current device status.
 *
 * @param[in] device The device object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_device_state The device state information.
 */
OB_EXPORT ob_device_state ob_device_get_device_state(const ob_device *device, ob_error **error);

/**
 * @brief Set the device state changed callback.
 *
 * @param[in] device The device object.
 * @param[in] callback The callback function to be called when the device status changes.
 * @param[in] user_data User-defined data that will be returned in the callback.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_set_state_changed_callback(ob_device *device, ob_device_state_callback callback, void *user_data, ob_error **error);

/**
 * @brief Enable or disable the device heartbeat.
 * @brief After enable the device heartbeat, the sdk will start a thread to send heartbeat signal to the device error every 3 seconds.

 * @attention If the device does not receive the heartbeat signal for a long time, it will be disconnected and rebooted.
 *
 * @param[in] device The device object.
 * @param[in] enable Whether to enable the device heartbeat.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_enable_heartbeat(ob_device *device, bool enable, ob_error **error);

/**
 * @brief Send data to the device and receive data from the device.
 * @brief This is a factory and debug function, which can be used to send and receive data from the device. The data format is secret and belongs to the device
 * vendor.
 *
 * @param[in] device The device object.
 * @param[in] send_data The data to be sent to the device.
 * @param[in] send_data_size The size of the data to be sent to the device.
 * @param[out] receive_data The data received from the device.
 * @param[in,out] receive_data_size Pass in the expected size of the receive data, and return the actual size of the received data.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_send_and_receive_data(ob_device *device, const uint8_t *send_data, uint32_t send_data_size, uint8_t *receive_data,
                                               uint32_t *receive_data_size, ob_error **error);

/**
 * @brief Get device information.
 *
 * @param[in] device The device to obtain information from.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_device_info* The device information.
 */
OB_EXPORT ob_device_info *ob_device_get_device_info(const ob_device *device, ob_error **error);

/**
 * @brief Delete device information.
 *
 * @param[in] info The device information to be deleted.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_device_info(ob_device_info *info, ob_error **error);

/**
 * @brief Get device name
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* return the device name
 */
OB_EXPORT const char *ob_device_info_get_name(const ob_device_info *info, ob_error **error);

/**
* @brief Get device pid

 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return int return the device pid
*/
OB_EXPORT int ob_device_info_get_pid(const ob_device_info *info, ob_error **error);

/**
 * @brief Get device vid
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return int return device vid
 */
OB_EXPORT int ob_device_info_get_vid(const ob_device_info *info, ob_error **error);

/**
 * @brief Get device uid
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* return device uid
 */
OB_EXPORT const char *ob_device_info_get_uid(const ob_device_info *info, ob_error **error);

/**
 * @brief Get device serial number
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * 
 * @return const char* return device serial number
 */
OB_EXPORT const char *ob_device_info_get_serial_number(const ob_device_info *info, ob_error **error);

/**
 * @brief Get the firmware version number
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * 
 * @return int return the firmware version number
 */
OB_EXPORT const char *ob_device_info_get_firmware_version(const ob_device_info *info, ob_error **error);

/**
 * @brief Get the device connection type
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* The connection type, currently supports: "USB", "USB1.0", "USB1.1", "USB2.0", "USB2.1", "USB3.0", "USB3.1", "USB3.2", "Ethernet", "GMSL2"
 */
OB_EXPORT const char *ob_device_info_get_connection_type(const ob_device_info *info, ob_error **error);

/**
 * @brief Get the device IP address
 *
 * @attention Only valid for network devices, otherwise it will return "0.0.0.0"
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * 
 * @return const char* The IP address, such as "192.168.1.10"
 */
OB_EXPORT const char *ob_device_info_get_ip_address(const ob_device_info *info, ob_error **error);

/**
 * @brief Get the network device subnet mask
 *
 * @param[in] info Device information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return const char* The subnet mask, such as "255.255.255.0"
 */
OB_EXPORT const char *ob_device_info_get_subnet_mask(const ob_device_info *info, ob_error **error);

/**
 * @brief Get the network device gateway address
 *
 * @param[in] info Device information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* The gateway address, such as "192.168.1.1"
 */
OB_EXPORT const char *ob_device_info_get_gateway(const ob_device_info *info, ob_error **error);

/**
 * @brief Get the hardware version number
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* The hardware version number
 */
OB_EXPORT const char *ob_device_info_get_hardware_version(const ob_device_info *info, ob_error **error);

/**
 * @brief Check if the device extension information exists.
 *
 * @param[in] device The device object.
 * @param[in] info_key The key of the device extension information.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Whether the device extension information exists.
 */
OB_EXPORT bool ob_device_is_extension_info_exist(const ob_device *device, const char *info_key, ob_error **error);

/**
 * @brief Get the device extension information.
 * @brief Extension information is a set of key-value pair of string, user cat get the information by the key.
 *
 * @param[in] device The device object.
 * @param[in] info_key The key of the device extension information.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* The device extension information
 */
OB_EXPORT const char *ob_device_get_extension_info(const ob_device *device, const char *info_key, ob_error **error);

/**
 * @brief Get the minimum SDK version number supported by the device
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * 
 * @return const char* The minimum SDK version number supported by the device
 */
OB_EXPORT const char *ob_device_info_get_supported_min_sdk_version(const ob_device_info *info, ob_error **error);

/**
 * @brief Get the chip name
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * 
 * @return const char* The ASIC name
 */
OB_EXPORT const char *ob_device_info_get_asicName(const ob_device_info *info, ob_error **error);

/**
 * @brief Get the device type
 *
 * @param[in] info Device Information
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * 
 * @return ob_device_type The device type
 */
OB_EXPORT ob_device_type ob_device_info_get_device_type(const ob_device_info *info, ob_error **error);

/**
 * @brief Delete a device list.
 *
 * @param[in] list The device list object to be deleted.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_device_list(ob_device_list *list, ob_error **error);

/**
 * @brief Get the number of devices
 *
 * @param[in] list Device list object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * 
 * @return uint32_t return the number of devices
 */
OB_EXPORT uint32_t ob_device_list_get_count(const ob_device_list *list, ob_error **error);

/**
 * @brief Get device name
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * 
 * @return const char* return device name
 */
OB_EXPORT const char *ob_device_list_get_device_name(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the pid of the specified device
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * 
 * @return int return the device pid
 */
OB_EXPORT int ob_device_list_get_device_pid(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the vid of the specified device
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return int return device vid
 */
OB_EXPORT int ob_device_list_get_device_vid(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the uid of the specified device
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* return the device uid
 */
OB_EXPORT const char *ob_device_list_get_device_uid(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the serial number of the specified device.
 *
 * @param[in] list Device list object.
 * @param[in] index Device index.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* The device UID.
 */
OB_EXPORT const char *ob_device_list_get_device_serial_number(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get device connection type
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* returns the device connection type, currently supports: "USB", "USB1.0", "USB1.1", "USB2.0", "USB2.1", "USB3.0", "USB3.1", "USB3.2",
 * "Ethernet", "GMSL2"
 */
OB_EXPORT const char *ob_device_list_get_device_connection_type(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get device ip address
 *
 * @attention Only valid for network devices, otherwise it will return "0.0.0.0".
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* returns the device ip address, such as "192.168.1.10"
 */
OB_EXPORT const char *ob_device_list_get_device_ip_address(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get device subnet mask
 *
 * @attention Only valid for network devices, otherwise it will return "0.0.0.0".
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* returns the device subnet mask, such as "255.255.255.0"
 */
OB_EXPORT const char *ob_device_list_get_device_subnet_mask(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get device gateway
 *
 * @attention Only valid for network devices, otherwise it will return "0.0.0.0".
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* returns the device gateway, such as "192.168.1.1"
 */
OB_EXPORT const char *ob_device_list_get_device_gateway(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the MAC address of the host network interface corresponding to the network device.
 *
 * @attention Only valid for network devices. Returns "0:0:0:0:0:0" for non-network devices.
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* The MAC address of the host network interface associated with the device.
 */
OB_EXPORT const char *ob_device_list_get_device_local_mac(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the IP address of the host network interface corresponding to the network device.
 *
 * @attention Only valid for network devices. Returns "0.0.0.0" for non-network devices.
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* The IP address of the host network interface associated with the device.
 */
OB_EXPORT const char *ob_device_list_get_device_local_ip(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the subnet length of the host network interface corresponding to the network device.
 *
 * @attention Only valid for network devices. Returns 0 for non-network devices.
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const uint8_t The subnet length (0~32) of the host network interface associated with the device.
 */
OB_EXPORT uint8_t ob_device_list_get_device_local_subnet_length(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Get the gateway of the host network interface corresponding to the network device.
 *
 * @attention Only valid for network devices. Returns "0.0.0.0" for non-network devices.
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const char* The gateway of the host network interface associated with the device.
 */
OB_EXPORT const char *ob_device_list_get_device_local_gateway(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Create a device.
 *
 * @attention If the device has already been acquired and created elsewhere, repeated acquisitions will return an error.
 *
 * @param[in] list Device list object.
 * @param[in] index The index of the device to create.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_device* The created device.
 *
 * @note Default access mode is OB_DEVICE_DEFAULT_ACCESS
 */
OB_EXPORT ob_device *ob_device_list_get_device(const ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief Create a device.
 *
 * @attention If the device has already been acquired and created elsewhere, repeated acquisitions will return an error.
 *
 * @param[in] list Device list object.
 * @param[in] index The index of the device to create.
 * @param[in] accessMode Device access mode. @ref ob_device_access_mode.
 *                       If the device does not support setting the Access Mode, the default OB_DEVICE_DEFAULT_ACCESS is used.
 *                       Applies only on first device creation or after release and re-creation; subsequent calls ignore it.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_device* The created device.
 */
OB_EXPORT ob_device *ob_device_list_get_device_ex(const ob_device_list *list, uint32_t index, ob_device_access_mode accessMode, ob_error **error);

/**
 * @brief Create a device.
 *
 * @attention If the device has already been acquired and created elsewhere, repeated acquisitions will return an error.
 *
 * @param[in] list Device list object.
 * @param[in] serial_number The serial number of the device to create.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_device* The created device.
 *
 * @note Default access mode is OB_DEVICE_DEFAULT_ACCESS
 */
OB_EXPORT ob_device *ob_device_list_get_device_by_serial_number(const ob_device_list *list, const char *serial_number, ob_error **error);

/**
 * @brief Create a device.
 *
 * @attention If the device has already been acquired and created elsewhere, repeated acquisitions will return an error.
 *
 * @param[in] list Device list object.
 * @param[in] serial_number The serial number of the device to create.
 * @param[in] accessMode Device access mode. @ref ob_device_access_mode.
 *                       If the device does not support setting the Access Mode, the default OB_DEVICE_DEFAULT_ACCESS is used.
 *                       Applies only on first device creation or after release and re-creation; subsequent calls ignore it.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_device* The created device.
 */
OB_EXPORT ob_device *ob_device_list_get_device_by_serial_number_ex(const ob_device_list *list, const char *serial_number, ob_device_access_mode accessMode,
                                                                   ob_error **error);

/**
 * @brief Create device by uid
 * @brief On Linux platform, for usb device, the uid of the device is composed of bus-port-dev, for example 1-1.2-1. But the SDK will remove the dev number and
 * only keep the bus-port as the uid to create the device, for example 1-1.2, so that we can create a device connected to the specified USB port. Similarly,
 * users can also directly pass in bus-port as uid to create device.
 * @brief For GMSL device, the uid is GMSL port with "gmsl2-" prefix, for example gmsl2-1.
 *
 * @attention If the device has already been acquired and created elsewhere, repeated acquisitions will return an error.
 *
 * @param[in] list Device list object.
 * @param[in] uid The UID of the device to create.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_device* The created device.
 *
 * @note Default access mode is OB_DEVICE_DEFAULT_ACCESS
 */
OB_EXPORT ob_device *ob_device_list_get_device_by_uid(const ob_device_list *list, const char *uid, ob_error **error);

/**
 * @brief Create device by uid
 * @brief On Linux platform, for usb device, the uid of the device is composed of bus-port-dev, for example 1-1.2-1. But the SDK will remove the dev number and
 * only keep the bus-port as the uid to create the device, for example 1-1.2, so that we can create a device connected to the specified USB port. Similarly,
 * users can also directly pass in bus-port as uid to create device.
 * @brief For GMSL device, the uid is GMSL port with "gmsl2-" prefix, for example gmsl2-1.
 *
 * @attention If the device has already been acquired and created elsewhere, repeated acquisitions will return an error.
 *
 * @param[in] list Device list object.
 * @param[in] uid The UID of the device to create.
 * @param[in] accessMode Device access mode. @ref ob_device_access_mode.
 *                       If the device does not support setting the Access Mode, the default OB_DEVICE_DEFAULT_ACCESS is used.
 *                       Applies only on first device creation or after release and re-creation; subsequent calls ignore it.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_device* The created device.
 */
OB_EXPORT ob_device *ob_device_list_get_device_by_uid_ex(const ob_device_list *list, const char *uid, ob_device_access_mode accessMode, ob_error **error);

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
OB_EXPORT ob_camera_param_list *ob_device_get_calibration_camera_param_list(ob_device *device, ob_error **error);

/**
 * @brief Get the number of camera parameter lists
 *
 * @param[in] param_list Camera parameter list
 * @param[out] error Log error messages
 *
 * @return uint32_t The number of lists
 */
OB_EXPORT uint32_t ob_camera_param_list_get_count(ob_camera_param_list *param_list, ob_error **error);

/**
 * @brief Get camera parameters from the camera parameter list
 *
 * @param[in] param_list Camera parameter list
 * @param[in] index Parameter index
 * @param[out] error Log error messages
 *
 * @return ob_camera_param The camera parameters. Since it returns the structure object directly, there is no need to provide a delete interface.
 */
OB_EXPORT ob_camera_param ob_camera_param_list_get_param(ob_camera_param_list *param_list, uint32_t index, ob_error **error);

/**
 * @brief Delete the camera parameter list
 *
 * @param[in] param_list Camera parameter list
 * @param[out] error Log error messages
 */
OB_EXPORT void ob_delete_camera_param_list(ob_camera_param_list *param_list, ob_error **error);

// The following interfaces are deprecated and are retained here for compatibility purposes.
#define ob_device_list_device_count ob_device_list_get_count
#define ob_device_list_get_extension_info ob_device_info_get_extension_info
#define ob_device_upgrade ob_device_update_firmware
#define ob_device_upgrade_from_data ob_device_update_firmware_from_data
#define ob_device_get_supported_property ob_device_get_supported_property_item
#define ob_device_state_changed ob_device_set_state_changed_callback
#define ob_device_info_name ob_device_info_get_name
#define ob_device_info_pid ob_device_info_get_pid
#define ob_device_info_vid ob_device_info_get_vid
#define ob_device_info_uid ob_device_info_get_uid
#define ob_device_info_serial_number ob_device_info_get_serial_number
#define ob_device_info_firmware_version ob_device_info_get_firmware_version
#define ob_device_info_connection_type ob_device_info_get_connection_type
#define ob_device_info_ip_address ob_device_info_get_ip_address
#define ob_device_info_hardware_version ob_device_info_get_hardware_version
#define ob_device_info_supported_min_sdk_version ob_device_info_get_supported_min_sdk_version
#define ob_device_info_asicName ob_device_info_get_asicName
#define ob_device_info_device_type ob_device_info_get_device_type
#define ob_device_list_get_device_count ob_device_list_get_count
#define ob_camera_param_list_count ob_camera_param_list_get_count

#ifdef __cplusplus
}
#endif
