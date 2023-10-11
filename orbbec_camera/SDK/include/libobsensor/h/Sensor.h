/**
 * @file Sensor.h
 * @brief Defines types related to sensors, used for obtaining stream configurations, opening and closing streams, and setting and getting sensor properties.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Get the type of the sensor.
 *
 * @param[in] sensor The sensor object.
 * @param[out] error Logs error messages.
 * @return The sensor type.
 */
ob_sensor_type ob_sensor_get_type(ob_sensor *sensor, ob_error **error);

/**
 * @brief Get a list of all supported stream profiles.
 *
 * @param[in] sensor The sensor object.
 * @param[out] error Logs error messages.
 * @return A list of stream profiles.
 */
ob_stream_profile_list *ob_sensor_get_stream_profile_list(ob_sensor *sensor, ob_error **error);

/**
 * @brief Open the current sensor and set the callback data frame.
 *
 * @param[in] sensor The sensor object.
 * @param[in] profile The stream configuration information.
 * @param[in] callback The callback function triggered when frame data arrives.
 * @param[in] user_data Any user data to pass in and get from the callback.
 * @param[out] error Logs error messages.
 */
void ob_sensor_start(ob_sensor *sensor, ob_stream_profile *profile, ob_frame_callback callback, void *user_data, ob_error **error);

/**
 * @brief Stop the sensor stream.
 *
 * @param[in] sensor The sensor object.
 * @param[out] error Logs error messages.
 */
void ob_sensor_stop(ob_sensor *sensor, ob_error **error);

/**
 * @brief Dynamically switch resolutions.
 *
 * @param[in] sensor The sensor object.
 * @param[in] profile The stream configuration information.
 * @param[out] error Logs error messages.
 */
void ob_sensor_switch_profile(ob_sensor *sensor, ob_stream_profile *profile, ob_error **error);

/**
 * @brief Delete a list of sensor objects.
 *
 * @param[in] sensor_list The list of sensor objects to delete.
 * @param[out] error Logs error messages.
 */
void ob_delete_sensor_list(ob_sensor_list *sensor_list, ob_error **error);

/**
 * @brief Get the number of sensors in the sensor list.
 *
 * @param[in] sensor_list The list of sensor objects.
 * @param[out] error Logs error messages.
 * @return The number of sensors in the list.
 */
uint32_t ob_sensor_list_get_sensor_count(ob_sensor_list *sensor_list, ob_error **error);

/**
 * @brief Get the sensor type.
 *
 * @param[in] sensor_list The list of sensor objects.
 * @param[in] index The index of the sensor on the list.
 * @param[out] error Logs error messages.
 * @return The sensor type.
 */
ob_sensor_type ob_sensor_list_get_sensor_type(ob_sensor_list *sensor_list, uint32_t index, ob_error **error);

/**
 * @brief Get a sensor by sensor type.
 *
 * @param[in] sensor_list The list of sensor objects.
 * @param[in] sensorType The sensor type to be obtained.
 * @param[out] error Logs error messages.
 * @return The sensor pointer. If the specified type of sensor does not exist, it will return null.
 */
ob_sensor *ob_sensor_list_get_sensor_by_type(ob_sensor_list *sensor_list, ob_sensor_type sensorType, ob_error **error);

/**
 * @brief Get a sensor by index number.
 *
 * @param[in] sensor_list The list of sensor objects.
 * @param[in] index The index of the sensor on the list.
 * @param[out] error Logs error messages.
 * @return The sensor object.
 */
ob_sensor *ob_sensor_list_get_sensor(ob_sensor_list *sensor_list, uint32_t index, ob_error **error);

/**
 * @brief Delete a sensor object.
 *
 * @param[in] sensor The sensor object to delete.
 * @param[out] error Logs error messages.
 */
void ob_delete_sensor(ob_sensor *sensor, ob_error **error);

#ifdef __cplusplus
}
#endif