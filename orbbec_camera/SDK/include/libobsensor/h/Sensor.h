/**
 * \if English
 * @file Sensor.h
 * @brief Sensor related types, used to obtain stream configuration, open and close streams, set and get sensor properties.
 * \else
 * @file Sensor.h
 * @brief 传感器相关函数，用于获取流配置，开关流，设置及获取传感器属性等操作
 * \endif
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * \if English
 * @brief Get sensor type
 *
 * @param[in] sensor Sensor object
 * @param[out] error Log error messages
 * @return ob_sensor_type returns the sensor type
 * \else
 * @brief 获取传感器类型
 *
 * @param[in] sensor 传感器对象
 * @param[out] error 记录错误信息
 * @return ob_sensor_type 返回传感器类型
 * \endif
 */
ob_sensor_type ob_sensor_get_type(ob_sensor *sensor, ob_error **error);

/**
 * \if English
 * @brief Get all supported stream profiles
 *
 * @param[in] sensor Sensor object
 * @param[out] error Log error messages
 * @return ob_stream_profile_list* returns a list of stream profiles
 * \else
 * @brief 获取传感器支持的所有流的配置列表
 *
 * @param[in] sensor 传感器对象
 * @param[out] error 记录错误信息
 * @return ob_stream_profile_list* 返回流配置的列表
 * \endif
 */
ob_stream_profile_list *ob_sensor_get_stream_profile_list(ob_sensor *sensor, ob_error **error);

/**
 * \if English
 * @brief Open the current sensor, and set the callback data frame
 *
 * @param[in] sensor Sensor object
 * @param[in] profile Stream configuration information
 * @param[in] callback Callback function triggered when frame data arrives
 * @param[in] user_data  pass in any user data and get it from the callback
 * @param[out] error Log error messages
 * \else
 * @brief 打开传感器的流，并设置帧数据回调
 *
 * @param[in] sensor 传感器对象
 * @param[in] profile 流的配置信息
 * @param[in] callback 帧数据到达时触发的回调函数
 * @param[in] user_data 可以传入任意用户数据，并从回调中获取
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_sensor_start(ob_sensor *sensor, ob_stream_profile *profile, ob_frame_callback callback, void *user_data, ob_error **error);

/**
 * \if English
 * @brief Stop sensor stream
 *
 * @param[in] sensor sensor object
 * @param[out] error Log error messages
 * \else
 * @brief 停止传感器的流
 *
 * @param[in] sensor 传感器对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_sensor_stop(ob_sensor *sensor, ob_error **error);

/**
 * \if English
 * @brief Dynamically switch resolutions
 *
 * @param[in] sensor  	Sensor object
 * @param[in] profile Stream configuration information
 * @param[out] error Log error messages
 * \else
 * @brief 动态切换分辨率
 *
 * @param[in] sensor  传感器对象
 * @param[in] profile 流的配置信息
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_sensor_switch_profile(ob_sensor *sensor, ob_stream_profile *profile, ob_error **error);

/**
 * \if English
 * @brief Delete sensor object list
 *
 * @param[in] sensor_list List of sensor objects to delete
 * @param[out] error Log error messages
 * \else
 * @brief 删除传感器对象列表
 *
 * @param[in] sensor_list 要删除的传感器对象列表
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_sensor_list(ob_sensor_list *sensor_list, ob_error **error);

/**
 * \if English
 * @brief Get the number of sensors in the sensor list
 *
 * @param[in] sensor_list  List of sensor objects to delete
 * @param[in] count Number of sensors
 * @param[out] error Log error messages
 * \else
 * @brief 获取传感器列表内传感器个数
 *
 * @param[in] sensor_list 要删除的传感器对象列表
 * @param[in] count 传感器数量
 * @param[out] error 记录错误信息
 * \endif
 */
uint32_t ob_sensor_list_get_sensor_count(ob_sensor_list *sensor_list, ob_error **error);

/**
 * \if English
 * @brief Get the sensor types
 *
 * @param index Sensor index
 * @return OBSensorType returns the Sensor type
 * \else
 * @brief 获取指定Sensor的类型
 *
 * @param index Sensor索引
 * @return OBSensorType 返回Sensor类型
 * \endif
 */
ob_sensor_type ob_sensor_list_get_sensor_type(ob_sensor_list *sensor_list, uint32_t index, ob_error **error);

/**
/**
 * \if English
 * @brief Get sensor by sensor type
 *
 * @param sensor_list Get the returned sensor list through device
 * @param sensorType  Sensor type to be obtained
 * @return  Return the Sensor pointer, if the specified type of Sensor does not exist, it will return null
 * \else
 * @brief 通过Sensor类型获取Sensor
 *
 * @param sensor_list 通过device获取返回的sensor列表
 * @param sensorType 要获取的Sensor类型
 * @return  返回Sensor指针，如果指定类型Sensor不存在，将返回空
 * \endif
 */
ob_sensor *ob_sensor_list_get_sensor_by_type(ob_sensor_list *sensor_list, ob_sensor_type sensorType, ob_error **error);

/**
 * \if English
 * @brief Get Sensor by index number
 *
 * @param sensor_list Get the returned sensor list through device
 * @param index To create a device cable, the range is [0, count-1], if the index exceeds the range, an exception will be thrown
 * @return std::shared_ptr<Sensor> returns the Sensor object
 * \else
 * @brief 通过索引号获取Sensor
 *
 * @param sensor_list 通过device获取返回的sensor列表
 * @param index 要创建设备的索，范围 [0, count-1]，如果index超出范围将抛异常
 * @return std::shared_ptr<Sensor> 返回Sensor对象
 * \endif
 */
ob_sensor *ob_sensor_list_get_sensor(ob_sensor_list *sensor_list, uint32_t index, ob_error **error);

/**
 * \if English
 * @brief Delete sensor object
 *
 * @param[in] sensor The sensor object to delete
 * @param[out] error Log error messages
 * \else
 * @brief 删除传感器对象
 *
 * @param[in] sensor 要删除的传感器对象
 * @param[out] error 记录错误信息
 * \endif
 */
void ob_delete_sensor(ob_sensor *sensor, ob_error **error);

#ifdef __cplusplus
}
#endif