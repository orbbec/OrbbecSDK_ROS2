/**
 * \if English
 * @file Device.h
 * @brief Device-related functions, including operations such as obtaining and creating a device, setting and obtaining device property, and obtaining sensors
 * \else
 * @file Device.h
 * @brief 设备相关函数，包括获取创建设备，设置及获取设备属性，获取传感器等操作
 * \endif
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"
#include "Property.h"

/**
 * \if English
 * @brief Get the number of devices
 *
 * @param[in] list Device list object
 * @param[out] error Log error messages
 * @return uint32_t returns the number of devices
 * \else
 * @brief 获取设备数量
 *
 * @param[in] list 设备列表对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回设备数量
 * \endif
 */
uint32_t ob_device_list_device_count(ob_device_list *list, ob_error **error);

/**
 *\if English
 * @brief Get device name (DEPRECATED)
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return const char* returns device name
 * \else
 * @brief 获取设备名称 (废弃接口)
 *
 * @param[in] list 设备列表对象
 * @param[in] index 设备索引
 * @param[out] error 记录错误信息
 * @return const char* 返回设备名称
 * \endif
 */
const char *ob_device_list_get_device_name(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * \if English
 * @brief Get the pid of the specified device
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return int returns the device pid
 * \else
 * @brief 获取指定设备pid
 *
 * @param[in] list 设备列表对象
 * @param[in] index 设备索引
 * @param[out] error 记录错误信息
 * @return int 返回设备pid
 * \endif
 */
int ob_device_list_get_device_pid(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * \if English
 * @brief Get the vid of the specified device
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return int return device vid
 * \else
 * @brief 获取指定设备vid
 *
 * @param[in] list 设备列表对象
 * @param[in] index 设备索引
 * @param[out] error 记录错误信息
 * @return int 返回设备vid
 * \endif
 */
int ob_device_list_get_device_vid(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * \if English
 * @brief Get the uid of the specified device
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return const char* returns the device uid
 * \else
 * @brief 获取指定设备uid
 *
 * @param[in] list 设备列表对象
 * @param[in] index 设备索引
 * @param[out] error 记录错误信息
 * @return const char* 返回设备uid
 * \endif
 */
const char *ob_device_list_get_device_uid(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * \if English
 * @brief Get the serial number of the specified device
 *
 * @param[in] list Device list object
 * @param[in] index Device index
 * @param[out] error Log error messages
 * @return const char* returns the device uid
 * \else
 * @brief 获取指定设备序列号
 *
 * @param[in] list 设备列表对象
 * @param[in] index 设备索引
 * @param[out] error 记录错误信息
 * @return const char* 返回设备序列号
 * \endif
 */
const char *ob_device_list_get_device_serial_number(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * \if English
 * @brief Create device
 * @attention If the device has been acquired and created elsewhere, repeated acquisitions will return an error
 *
 * @param[in] list Device list object
 * @param index Creat index of the device
 * @param[out] error Log error messages
 * @return ob_device* returns the created device
 * \else
 * @brief 创建设备
 * @attention 如果设备有在其他地方被获取创建，重复获取将会返回错误
 *
 * @param[in] list 设备列表对象
 * @param index 要创建设备的索引
 * @param[out] error 记录错误信息
 * @return ob_device* 返回创建的设备
 * \endif
 */
ob_device *ob_device_list_get_device(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * \if English
 * @brief Create device
 * @attention If the device has been acquired and created elsewhere, repeated acquisitions will return an error
 *
 * @param[in] list Device list object
 * @param index Creat the serial number of the device
 * @param[out] error Log error messages
 * @return ob_device* returns the created device
 * \else
 * @brief 创建设备
 * @attention 如果设备有在其他地方被获取创建，重复获取将会返回错误
 *
 * @param[in] list 设备列表对象
 * @param[in] serial_number 要创建设备的序列号
 * @param[out] error 记录错误信息
 * @return ob_device* 返回创建的设备
 * \endif
 */
ob_device *ob_device_list_get_device_by_serial_number(ob_device_list *list, const char *serial_number, ob_error **error);

/**
 * \if English
 * @brief Create device
 * @attention If the device has been acquired and created elsewhere, repeated acquisitions will return an error
 *
 * @param[in] list Device list object
 * @param index Creat the uid of the device
 * @param[out] error Log error messages
 * @return ob_device* returns the created device
 * \else
 * @brief 创建设备
 * @attention 如果设备有在其他地方被获取创建，重复获取将会返回错误
 *
 * @param[in] list 设备列表对象
 * @param[in] uid  要创建设备的uid
 * @param[out] error 记录错误信息
 * @return ob_device* 返回创建的设备
 *\endif
 */
ob_device *ob_device_list_get_device_by_uid(ob_device_list *list, const char *uid, ob_error **error);

/**
 * \if English
 * @brief Delete device
 *
 * @param[in] device Device to be deleted
 * @param[out] error Log error messages
 * \else
 * @brief 删除设备
 *
 * @param[in] device 要删除的设备
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_device(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Delete device information
 *
 * @param[in] info Device information to be deleted
 * @param[out] error Log error messages
 * \else
 * @brief 删除设备信息
 *
 * @param[in] info 要删除的设备信息
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_device_info(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Delete device list
 *
 * @param[in] list Device list object to be deleted
 * @param[out] error Log error messages
 * \else
 * @brief 删除设备列表
 *
 * @param[in] list 要删除的设备列表对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_device_list(ob_device_list *list, ob_error **error);

/**
 * \if English
 * @brief Get device information
 *
 * @param[in] device Device to obtain information
 * @param[out] error Log error messages
 * @return ob_device_info* returns device information
 * \else
 * @brief 获取设备信息
 *
 * @param[in] device 要获取信息的设备
 * @param[out] error 记录错误信息
 * @return ob_device_info* 返回设备信息
 * \endif
 */
ob_device_info *ob_device_get_device_info(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief List all sensors
 *
 * @param[in] device Device object
 * @param[out] error Log error messages
 * @return ob_sensor_list* returns the list of all sensors
 * \else
 * @brief 获取设备的所有传感器
 *
 * @param[in] device 设备对象
 * @param[out] error 记录错误信息
 * @return ob_sensor_list* 返回所有传感器列表
 * \endif
 */
ob_sensor_list *ob_device_get_sensor_list(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Get the device's sensors
 *
 * @param[in] device Device object
 * @param[in] type Type of sensor to get
 * @param[out] error Log error messages
 * @return ob_sensor* returns the acquired sensor
 * \else
 * @brief 获取设备的传感器
 *
 * @param[in] device 设备对象
 * @param[in] type 要获取的传感器类型
 * @param[out] error 记录错误信息
 * @return ob_sensor* 返回获取的传感器
 * \endif
 */
ob_sensor *ob_device_get_sensor(ob_device *device, ob_sensor_type type, ob_error **error);
/**
 * \if English
 * @brief Set int type of device property
 *
 * @param[in] device Device object
 * @param[in] property_id The id of the property to be set
 * @param[in] property Property value to be set
 * @param[out] error Log error messages
 * \else
 * @brief 设置int类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] property 要设置的属性值
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_set_int_property(ob_device *device, ob_property_id property_id, int32_t property, ob_error **error);

/**
 * \if English
 * @brief Get int type of device property
 *
 * @param[in] device Device object
 * @param[in] property_id Property id
 * @param[out] error Log error messages
 * @return int32_t returns the property value
 * \else
 * @brief 获取int类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return int32_t 返回属性值
 * \endif
 */
int32_t ob_device_get_int_property(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * \if English
 * @brief Set float type of device property
 *
 * @param[in] device Device object
 * @param[in] property_id The id of the property to be set
 * @param[in] property Property value to be set
 * @param[out] error Log error messages
 * \else
 * @brief 设置float类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] property 要设置的属性值
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_set_float_property(ob_device *device, ob_property_id property_id, float property, ob_error **error);

/**
 * \if English
 * @brief Get float type of device property
 *
 * @param[in] device Device object
 * @param[in] property_id Property id
 * @param[out] error Log error messages
 * @return int32_t returns the property value
 * \else
 * @brief 获取float类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return int32_t 返回属性值
 * \endif
 */
float ob_device_get_float_property(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * \if English
 * @brief Set bool type of device property
 *
 * @param[in] device Device object
 * @param[in] property_id The id of the property to be set
 * @param[in] property Property value to be set
 * @param[out] error Log error messages
 * \else
 * @brief 设置bool类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] property 要设置的属性值
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_set_bool_property(ob_device *device, ob_property_id property_id, bool property, ob_error **error);

/**
 * \if English
 * @brief Get bool type of device property
 *
 * @param[in] device Device object
 * @param[in] property_id Property id
 * @param[out] error Log error messages
 * @return bool returns the property value
 * \else
 * @brief 获取bool类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return bool 返回属性值
 * \endif
 */
bool ob_device_get_bool_property(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * \if English
 * @brief Set structured type
 *
 * @param[in] device Device object
 * @param[in] property_id The id of the property to be set
 * @param[in] data Property data to be set
 * @param[in] data_size The size of the property to be set
 * @param[out] error Log error messages
 * \else
 * @brief 设置结构体类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] data 要设置的属性数据
 * @param[in] data_size 要设置的属性大小
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_set_structured_data(ob_device *device, ob_property_id property_id, const void *data, uint32_t data_size, ob_error **error);

/**
 * \if English
 * @brief Get structured type
 *
 * @param[in] device Device object
 * @param[in] property_id Id of the property
 * @param[out] data Property data obtained
 * @param[out] data_size Get the size of the property
 * @param[out] error Log error messages
 * \else
 * @brief 获取结构体类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 要获取的属性id
 * @param[out] data 获取的属性数据
 * @param[out] data_size 获取的属性大小
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_get_structured_data(ob_device *device, ob_property_id property_id, void *data, uint32_t *data_size, ob_error **error);

/**
 * \if English
 * @brief Set property struct data.
 *
 * @param[in] device Device object
 * @param[in] property_id Id of the property
 * @param[in] data_bundle Target data to set
 * @param[in] Set data callback
 * @param[in] user_data User-defined data will be returned in the callback
 * @param[out] error Log error messages
 * \else
 * @brief 设置结构体类型的设备属性

 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] data_bundle
 要设置的数据内容，注意data_bundle指针为OrbbecSDK外部构建，所以自行释放指针资源，禁止调用ob_delete_data_bundle释放资源，会造成不可预知的错误；
 * @param[in] cb 设置进度回调
 * @param[in] user_data 用户自定义数据，会在回调中返回
 * @param[out] error 记录错误信息
 * @return void 无返回值
 * \endif
 */
void ob_device_set_structured_data_ext(ob_device *device, ob_property_id property_id, ob_data_bundle *data_bundle, ob_set_data_callback cb, void *user_data,
                                       ob_error **error);

/**
 * \if English
 * @brief Get property struct data.
 * @param[in] device Device object
 * @param[in] property_id Id of the property
 * @param[out] error Log error messages
 * @return ob_data_dundle, NOTE: ob_data_dundle must free by ob_delete_data_bundle() because it come from OrbbecSDK's API
 * \else
 * @brief 获取结构体类型的设备属性
 * @param[in] device 设备对象
 * @param[in] property_id 要获取的属性id
 * @param[out] error 记录错误信息
 * @return 返回ob_data_dundle 结构体对象指针，返回的必须调用ob_delete_data_bundle()函数释放资源，否则会造成内存泄漏
 * \endif
 */
ob_data_bundle *ob_device_get_structured_data_ext(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * \if English
 * @brief Set raw data type of device property
 *
 * @param[in] device Device object
 * @param[in] property_id The id of the property to be set
 * @param[in] data Property data to be set
 * @param[in] data_size The size of the property to be set
 * @param[in] cb Set data callback
 * @param[in] async Whether to execute asynchronously
 * @param[in] user_data User-defined data will be returned in the callback
 * @param[out] error Log error messages
 * \else
 * @brief 设置raw data类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] data 要设置的属性数据
 * @param[in] data_size 要设置的属性大小
 * @param[in] cb 设置进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_set_raw_data(ob_device *device, ob_property_id property_id, void *data, uint32_t data_size, ob_set_data_callback cb, bool async, void *user_data,
                            ob_error **error);

/**
 * \if English
 * @brief Get raw data type of device property
 *
 * @param[in] device Device object
 * @param[in] property_id Property id
 * @param[in] cb Get data callback
 * @param[in] async Whether to execute asynchronously
 * @param[in] user_data User-defined data will be returned in the callback
 * @param[out] error Log error messages
 * \else
 * @brief 获取raw data类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 要获取的属性id
 * @param[in] cb 获取数据给返回和进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_get_raw_data(ob_device *device, ob_property_id property_id, ob_get_data_callback cb, bool async, void *user_data, ob_error **error);

/**
 * \if English
 * @brief Get the property protocol version
 * 
 * @param[in] device Device object
 * @param[out] error Log error messages
 * @return ob_protocol_version
 * 
 * \else
 * 
 * @brief 获取设备的控制命令协议版本
 * 
 * @param[in] device 设备对象
 * @param[out] error 记录错误信息
 * @return ob_protocol_version
 * \endif
 */
ob_protocol_version ob_device_get_protocol_version(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Get cmdVersion of property
 *
 * @param[in] device Device object
 * @param[in] propertyId Property id
 * @param[out] error Log error messages
 * @return ob_cmd_version 
 * \else
 * @brief 获取控制命令的版本号
 *
 * @param[in] device 设备对象
 * @param[in] propertyId 属性id
 * @param[out] error 记录错误信息
 * @return ob_cmd_version 
 * \endif
 */
ob_cmd_version ob_device_get_cmd_version(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * \if English
 * @brief Get supported device properties count
 *
 * @param[in] device Device object
 * @param[out] error Log error messages
 * @return uint32_t returns the number of properties supported by the device
 * \else
 * @brief 获取设备支持的属性的数量
 *
 * @param[in] device 设备对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回设备支持的属性的数量
 * \endif
 */
uint32_t ob_device_get_supported_property_count(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Get supported device properties
 *
 * @param[in] device Device object
 * @param[in] index Property index
 * @param[out] error Log error messages
 * @return ob_global_unified_property_item returns the type of property supported by the device
 * \else
 * @brief 获取设备支持的属性
 *
 * @param[in] device 设备对象
 * @param[in] index 属性的index
 * @param[out] error 记录错误信息
 * @return ob_global_unified_property_item 返回设备支持的属性的类型
 * \endif
 */
ob_property_item ob_device_get_supported_property(ob_device *device, uint32_t index, ob_error **error);

/**
 * \if English
 * @brief Check if device property permission is supported
 *
 * @param[in] device Device object
 * @param[in] property_id Property id
 * @param[in] permission The type of permission that needs to be interpreted
 * @param[out] error Log error messages
 * @return bool returns whether it is supported
 * \else
 * @brief 判断属性权限支持情况
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[in] permission 需要判读的权限类型
 * @param[out] error 记录错误信息
 * @return bool 返回是否支持
 * \endif
 */
bool ob_device_is_property_supported(ob_device *device, ob_property_id property_id, ob_permission_type permission, ob_error **error);

/**
 * \if English
 * @brief Get int type of device property range
 *
 * @param[in] device Device object
 * @param[in] property_id Property id
 * @param[out] error Log error messages
 * @return ob_int_property_range returns the property range
 * \else
 * @brief 获取int类型的设备属性范围
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return ob_int_property_range 返回属性范围
 * \endif
 */
ob_int_property_range ob_device_get_int_property_range(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * \if English
 * @brief Get float type of device property range
 *
 * @param[in] device Device object
 * @param[in] property_id Property id
 * @param[out] error Log error messages
 * @return ob_float_property_range returns the property range
 * \else
 * @brief 获取float类型的设备属性范围
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return ob_float_property_range 返回属性范围
 */
ob_float_property_range ob_device_get_float_property_range(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * \if English
 * @brief Get the bool type of device property range
 *
 * @param[in] device Device object
 * @param[in] property_id Property id
 * @param[out] error Log error messages
 * @return ob_bool_property_range returns the property range
 * \else
 * @brief 获取bool类型的设备属性范围
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return ob_bool_property_range 返回属性范围
 * \endif
 */
ob_bool_property_range ob_device_get_bool_property_range(ob_device *device, ob_property_id property_id, ob_error **error);

/**
 * \if English
 * @brief ahb write register
 *
 * @param[in] device Device object
 * @param reg Register to be written
 * @param mask Mask
 * @param value The value to be written
 * @param[out] error Log error messages
 * \else
 * @brief ahb写寄存器
 *
 * @param[in] device 设备对象
 * @param reg 要写入的寄存器
 * @param mask 掩码
 * @param value 要写入的值
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_write_ahb(ob_device *device, uint32_t reg, uint32_t mask, uint32_t value, ob_error **error);

/**
 * \if English
 * @brief Read ahb register
 *
 * @param[in] device Device object
 * @param reg Register to be read
 * @param mask Mask
 * @param value The value to be read
 * @param[out] error Log error messages
 * \else
 * @brief ahb读寄存器
 *
 * @param[in] device 设备对象
 * @param reg 要读取的寄存器
 * @param mask 掩码
 * @param value 要读取的值
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_read_ahb(ob_device *device, uint32_t reg, uint32_t mask, uint32_t *value, ob_error **error);

/**
 * \if English
 * @brief Write i2c register
 *
 * @param[in] device Device object
 * @param module_id I2c module id to be written
 * @param reg Register to be written
 * @param mask Mask
 * @param value The value to be written
 * @param[out] error Log error messages
 * \else
 * @brief i2c写寄存器
 *
 * @param[in] device 设备对象
 * @param module_id 要写入的i2c模块id
 * @param reg 要写入的寄存器
 * @param mask 掩码
 * @param value 要写入的值
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_write_i2c(ob_device *device, uint32_t module_id, uint32_t reg, uint32_t mask, uint32_t value, ob_error **error);

/**
 * \if English
 * @brief Read i2c register
 *
 * @param[in] device Device object
 * @param module_id The id of the i2c module to be read
 * @param reg Register to be read
 * @param mask Mask
 * @param value The value to be read
 * @param[out] error Log error messages
 * \else
 * @brief i2c读寄存器
 *
 * @param[in] device 设备对象
 * @param module_id 要读取的的i2c模块id
 * @param reg 要读取的寄存器
 * @param mask 掩码
 * @param value 要读取的值
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_read_i2c(ob_device *device, uint32_t module_id, uint32_t reg, uint32_t mask, uint32_t *value, ob_error **error);

/**
 * \if English
 * @brief Set the properties of writing to Flash [Asynchronous Callback]
 *
 * @param[in] device Device object
 * @param offset flash offset address
 * @param data Property data to be written
 * @param data_size The size of the property to be written
 * @param cb Set data callback
 * @param[in] async Whether to execute asynchronously
 * @param[in] user_data User-defined data will be returned in the callback
 * @param[out] error Log error messages
 * \else
 * @brief 设置写入Flash的属性[异步回调]
 *
 * @param[in] device 设备对象
 * @param offset flash偏移地址
 * @param data 要写入的属性数据
 * @param data_size 要写入的属性大小
 * @param cb 写进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_write_flash(ob_device *device, uint32_t offset, const void *data, uint32_t data_size, ob_set_data_callback cb, bool async, void *user_data,
                           ob_error **error);

/**
 * \if English
 * @brief Read Flash properties [asynchronous callback]
 *
 * @param[in] device Device object
 * @param offset flash offset address
 * @param data_size The size of the data to be read
 * @param cb Read flash data and progress callback
 * @param[in] async Whether to execute asynchronously
 * @param[in] user_data User-defined data will be returned in the callback
 * @param[out] error Log error messages
 * \else
 * @brief 读取Flash的属性[异步回调]
 *
 * @param[in] device 设备对象
 * @param offset flash偏移地址
 * @param data_size 要读取的数据大小
 * @param cb 读flash数据及进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_read_flash(ob_device *device, uint32_t offset, uint32_t data_size, ob_get_data_callback cb, bool async, void *user_data, ob_error **error);

/**
 * \if English
 * @brief Synchronize the device time (time to the device, synchronize the local system time to the device)
 *
 * @param[in] device Device object
 * @param[out] uint64_t Command round trip time (rtt)
 * @param[out] error Log error messages
 * \else
 * @brief 同步设备时间（向设备授时，同步本地系统时间到设备）
 *
 * @param[in] device 设备对象
 * @param[out] uint64_t 命令往返时间延时（round trip time， rtt）
 * @param[out] error 记录错误信息
 * \endif
 */
uint64_t ob_device_sync_device_time(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Device firmware upgrade
 *
 * @param[in] device Device object
 * @param[in] pathFirmware path
 * @param[in] callback Firmware upgrade progress callback
 * @param[in] async Whether to execute asynchronously
 * @param[in] user_data User-defined data will be returned in the callback
 * @param[out] error Log error messages
 * \else
 * @brief 设备固件升级
 *
 * @param[in] device 设备对象
 * @param[in] path 固件路径
 * @param[in] callback 固件升级进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_upgrade(ob_device *device, const char *path, ob_device_upgrade_callback callback, bool async, void *user_data, ob_error **error);

/**
 * \if English
 * @brief Get current device status
 * @param[out] error Log error messages
 * @return ob_device_state device state information
 * \else
 * @brief 获取当前设备状态
 * @param[out] error 记录错误信息
 * @return ob_device_state 设备状态信息
 * \endif
 */
ob_device_state ob_device_get_device_state(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Monitor device state change
 *
 * @param[in] device Device object
 * @param[in] callback Callback when the device status changes
 * @param[in] user_data User-defined data will be returned in the callback
 * @param[out] error Log error messages
 * \else
 * @brief 设置设备状态监听
 *
 * @param[in] device 设备对象
 * @param[in] callback 设备状态发生改变时的回调
 * @param[in] user_data 用户自定义数据，会在回调中返回
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_state_changed(ob_device *device, ob_device_state_callback callback, void *user_data, ob_error **error);

/**
 * \if English
 * @brief Send files to the specified path of the device
 *
 * @param[in] device Device object
 * @param[in] file_path Source file path
 * @param[in] dst_path Specify the device-side file acceptance path
 * @param[in] callback File sending progress callback
 * @param[in] async Whether to execute asynchronously
 * @param[in] user_data User-defined data will be returned in the callback
 * @param[out] error Log error messages
 * \else
 * @brief 发送文件到设备指定路径
 *
 * @param[in] device 设备对象
 * @param[in] file_path 源文件路径
 * @param[in] dst_path 指定设备端文件接受路径
 * @param[in] callback 文件发送进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_send_file_to_destination(ob_device *device, const char *file_path, const char *dst_path, ob_file_send_callback callback, bool async,
                                        void *user_data, ob_error **error);

/**
 * \if English
 * @brief Verify device authorization code
 *
 * @param[in] device Device object
 * @param[in] auth_code Authorization code
 * @param[out] error Log error messages
 * @return bool whether the activation is successful
 * \else
 * @brief 验证设备授权码
 *
 * @param[in] device 设备对象
 * @param[in] auth_code 授权码
 * @param[out] error 记录错误信息
 * @return bool 激活是否成功
 * \endif
 */
bool ob_device_activate_authorization(ob_device *device, const char *auth_code, ob_error **error);

/**
 * \if English
 * @brief Write device authorization code
 *
 * @param[in] device Device object
 * @param[in] auth_code Authorization code
 * @param[out] error Log error messages
 * \else
 * @brief 写入设备授权码
 *
 * @param[in] device 设备对象
 * @param[in] auth_code 授权码
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_write_authorization_code(ob_device *device, const char *auth_code, ob_error **error);

/**
 * \if English
 * @brief Get the original parameter list of camera calibration saved in the device. The parameters in the list do not correspond to the current open-stream
 * configuration. You need to select the parameters according to the actual situation, and may need to do scaling, mirroring and other processing.
 * Non-professional users are recommended to use the ob_pipeline_get_camera_param() interface.
 * @param[in] device Device object
 * @param[out] error Log error messages
 *
 * @return ob_camera_param_list camera parameter list
 * \else
 * @brief 获取设备内保存的相机标定的原始参数列表，列表内参数不与当前开流配置相对应，
 * 需要自行根据实际情况选用参数并可能需要做缩放、镜像等处理。非专业用户建议使用ob_pipeline_get_camera_param()接口。
 * @param[in] device 设备对象
 * @param[out] error 记录错误信息
 *
 * @return ob_camera_param_list 相机参数列表
 * \endif
 */
ob_camera_param_list *ob_device_get_calibration_camera_param_list(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Get current depth work mode
 * @param[in] device Device object
 * @param[out] error Log error messages
 * @return ob_depth_work_mode Current depth work mode
 * \else
 * @brief 查询当前相机深度模式
 * @param[in] device 设备对象
 * @param[out] error 记录错误信息
 * @return ob_depth_work_mode 当前深度工作模式
 * \endif
 */
ob_depth_work_mode ob_device_get_current_depth_work_mode(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Switch depth work mode by ob_depth_work_mode. Prefer invoke ob_device_switch_depth_work_mode_by_name to switch depth mode
 *        when known the complete name of depth work mode.
 * @param[in] device Device object
 * @param[in] work_mode Depth work mode come from ob_depth_work_mode_list which return by ob_device_get_depth_work_mode_list
 * @param[out] error Log error messages
 * @return Return switch result. OB_STATUS_OK: success, other failed.
 * \else
 * @brief 切换相机深度模式（根据深度工作模式对象），如果知道设备支持的深度工作模式名称，那么推荐用ob_device_switch_depth_work_mode_by_name
 * @param[in] device 设备对象
 * @param[in] work_mode 深度工作模式，从深度工作模式列表ob_depth_work_mode_list获取的模式对象(ob_device_get_depth_work_mode_list返回的对象)
 * @param[out] error 记录错误信息
 * @return ob_status 设置设置结果，OB_STATUS_OK成功，其他：设置失败
 * \endif
 */
ob_status ob_device_switch_depth_work_mode(ob_device *device, const ob_depth_work_mode *work_mode, ob_error **error);

/**
 * \if English
 * @brief Switch depth work mode by work mode name.
 * @param[in] device Device object
 * @param[in] mode name Depth work mode name which equals to ob_depth_work_mode.name
 * @param[out] error Log error messages
 * 
 * @return Return switch result. OB_STATUS_OK: success, other failed.
 * \else
 * @brief 切换相机深度模式（根据深度工作模式名称）
 * @param[in] device 设备对象
 * @param[in] mode_name 深度工作模式名称，模式名称必须与ob_depth_work_mode.name一致
 * @param[out] error 记录错误信息
 * @return ob_status 设置设置结果，OB_STATUS_OK成功，其他：设置失败
 * \endif
 */
ob_status ob_device_switch_depth_work_mode_by_name(ob_device *device, const char *mode_name, ob_error **error);

/**
 * \if English
 * @brief Request support depth work mode list
 * @param[in] device Device object
 * @param[out] error Log error messages
 * @return ob_depth_work_mode_list list of ob_depth_work_mode
 * \else
 * @brief 查询当前相机深度模式的列表
 * @param[in] device 设备对象
 * @param[out] error 记录错误信息
 * @return ob_depth_work_mode_list 深度模式列表。注意：返回对象要用ob_delete_depth_work_mode_list()释放资源，否者会内存泄漏
 * \endif
 */
ob_depth_work_mode_list *ob_device_get_depth_work_mode_list(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Device reboot
 * @attention The device will be disconnected and reconnected. After the device is disconnected, the interface access to the device handle may be abnormal.
 * Please use the ob_delete_device interface to delete the handle directly. After the device is reconnected, it can be obtained again.
 *
 * @param[in] device Device object
 * @param[out] error Log error messages
 * \else
 * @brief 设备重启
 * @attention 设备会掉线重连，设备掉线后对device句柄的接口访问可能会发生异常，请直接使用ob_delete_device接口删除句柄，
 *   待设备重连后可重新获取。
 *
 * @param[in] device 设备对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_device_reboot(ob_device *device, ob_error **error);

/**
 * @brief 获取当前设备同步配置
 * @brief 设备同步：包括单机内的不同 Sensor 的曝光同步功能 和 多机同步功能
 *
 * @param[in] device 设备对象
 * @param[out] error 记录错误信息
 * @return ob_device_sync_config 返回设备同步配置
 */
ob_device_sync_config ob_device_get_sync_config(ob_device *device, ob_error **error);

/**
 * @brief 设置设备同步配置
 * @brief 用于配置 单机内的不同 Sensor 的曝光同步功能 和 多机同步功能
 *
 * @attention 调用本函数会直接将配置写入设备Flash，设备重启后依然会生效。为了避免影响Flash寿命，不要频繁更新配置。
 *
 * @param[in] device 设备对象
 * @param[out] device_sync_config 设备同步配置
 * @param[out] error 记录错误信息
 */
void ob_device_set_sync_config(ob_device *device, ob_device_sync_config device_sync_config, ob_error **error);

/**
 * \if English
 * @brief Get device name
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* returns the device name
 * \else
 * @brief 获取设备名
 *
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return const char* 返回设备名
 * \endif
 */
const char *ob_device_info_name(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get device pid

 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return int returns the device pid
 * \else
 * @brief 获取设备pid
 *
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return int 返回设备pid
 * \endif
 */
int ob_device_info_pid(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get device vid
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return int returns device vid
 * \else
 * @brief 获取设备vid
 *
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return int 返回设备vid
 * \endif
 */
int ob_device_info_vid(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get device uid
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* returns device uid
 * \else
 * @brief 获取设备uid
 *
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return const char* 返回设备uid
 * \endif
 */
const char *ob_device_info_uid(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get device serial number
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* returns device serial number
 * \else
 * @brief 获取设备序列号
 *
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return const char* 返回设备序列号
 * \endif
 */
const char *ob_device_info_serial_number(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get the firmware version number
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return int returns the firmware version number
 * \else
 * @brief 获取固件版本号
 *
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return int 返回固件版本号
 * \endif
 */
const char *ob_device_info_firmware_version(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get usb connection type (DEPRECATED)
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return char* returns usb connection type
 * \else
 * @brief 获取usb连接类型 (废弃接口)
 *
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return char* 返回usb连接类型
 * \endif
 */
const char *ob_device_info_usb_type(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get device connection type
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return char* returns connection type
 * \else
 * @brief 获取设备连接类型
 *
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return char* 返回连接类型
 * \endif
 */
const char *ob_device_info_connection_type(ob_device_info *info, ob_error **error);

/**
 *\if English
 * @brief Get hardware version number
 *
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* returns hardware version number
 * \else
 * @brief 获取硬件的版本号
 *
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return const char* 返回硬件的版本号
 * \endif
 */
const char *ob_device_info_hardware_version(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get the minimum SDK version number supported by the device
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* returns the minimum SDK version number supported by the device
 * \else
 * @brief 获取设备支持的SDK最小版本号
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return const char* 返回设备支持的SDK最小版本号
 * \endif
 */
const char *ob_device_info_supported_min_sdk_version(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get chip name
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return const char* returns asicName
 * \else
 * @brief 获取芯片类型名称
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return const char* 返回芯片类型名称
 * \endif
 */
const char *ob_device_info_asicName(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get device type
 * @param[in] info Device Information
 * @param[out] error Log error messages
 * @return ob_device_type returns device type
 * \else
 * @brief 获取设备类型
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return ob_device_type 返回设备类型
 * \endif
 */
ob_device_type ob_device_info_device_type(ob_device_info *info, ob_error **error);

/**
 * \if English
 * @brief Get the number of camera parameter lists
 *
 * @param param_list Camera parameter list
 * @param error Log error messages
 * @return uint32_t returns the number of lists
 * \else
 * @brief 获取相机参数列表内参数组数
 *
 * @param param_list 相机参数列表
 * @param error 记录错误信息
 * @return uint32_t 返回组数
 * \endif
 */
uint32_t ob_camera_param_list_count(ob_camera_param_list *param_list, ob_error **error);

/**
 * \if English
 * @brief Get camera parameters from camera parameter list
 *
 * @param param_list Camera parameter list
 * @param index Parameter index
 * @param error Log error messages
 * @return ob_camera_param returns the camera parameters. Since it returns the structure object directly, there is no need to provide a delete interface.
 * \else
 * @brief 从相机参数列表内获取相机参数
 *
 * @param param_list 相机参数列表
 * @param index 参数索引
 * @param error 记录错误信息
 * @return ob_camera_param 返回相机参数, 由于是直接返回结构体对象，所以不需要提供删除接口
 * \endif
 */
ob_camera_param ob_camera_param_list_get_param(ob_camera_param_list *param_list, uint32_t index, ob_error **error);

/**
 * \if English
 * @brief Delete camera parameter list
 *
 * @param param_list Camera parameter list
 * @param error Log error messages
 * \else
 * @brief 删除相机参数列表
 *
 * @param param_list 相机参数列表
 * @param error 记录错误信息
 * \endif
 */
void ob_delete_camera_param_list(ob_camera_param_list *param_list, ob_error **error);

/**
 * \if English
 * @brief Get the depth work mode count that ob_depth_work_mode_list hold
 * @param[in] work_mode_list data struct contain list of ob_depth_work_mode
 * @param[out] error Log error messages
 * @return The total number contain in ob_depth_work_mode_list
 * \else
 * @brief 获取深度工作模式列表的元素数量
 * @param[in] work_mode_list 工作模式列表对象
 * @param[in] index 下标，从0开始
 * @return work_mode_list包含的深度模式总数
 * \endif
 *
 */
uint32_t ob_depth_work_mode_list_count(ob_depth_work_mode_list *work_mode_list, ob_error **error);

/**
 * \if English
 * @brief Get the index target of ob_depth_work_mode from work_mode_list
 * @param[in] work_mode_list data struct contain list of ob_depth_work_mode
 * @param[in] index index of target ob_depth_work_mode
 * @param[out] error Log error messages
 * @return ob_depth_work_mode
 * \else
 * @brief 从深度工作模式列表获取第N个对象
 * @param[in] work_mode_list 工作模式列表对象
 * @param[in] index 下标，从0开始
 * @param[out] error 记录错误信息
 * @return 深度工作模式对象
 * \endif
 *
 */
ob_depth_work_mode ob_depth_work_mode_list_get_item(ob_depth_work_mode_list *work_mode_list, uint32_t index, ob_error **error);

/**
 * \if English
 * @brief Free resource of ob_depth_work_mode_list
 * @param[in] work_mode_list data struct contain list of ob_depth_work_mode
 * @param[out] error Log error messages
 * \else
 * @brief 删除深度工作模式列表对象，释放资源
 * @param[in] work_mode_list 工作模式列表对象
 * @param[out] error 记录错误信息
 * \endif
 *
 */
void ob_delete_depth_work_mode_list(ob_depth_work_mode_list *work_mode_list, ob_error **error);

/**
 * \if English
 * @brief Free resource of data_bundle which come from OrbbecSDK's API
 * @param data_bundle Data bundle
 * @param[out] error Log error messages
 * \else
 * @brief 删除ob_data_bundle对象，释放资源；
 * 注意：只能释放OrbbecSDK库返回的ob_data_bundle对象，非OrbbecSDK构建的对象不能调用该接口，否则会造成意想不到的错误
 * @param[in] data_bundle 数据集
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_data_bundle(ob_data_bundle *data_bundle, ob_error **error);

#ifdef __cplusplus
}
#endif
