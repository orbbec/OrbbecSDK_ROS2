#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Transform a 3d point of a source coordinate system into a 3d point of the target coordinate system.
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] source_point3f Source 3d point value
 * @param[in] source_sensor_type Source sensor type
 * @param[in] target_sensor_type Target sensor type
 * @param[out] target_point3f Target 3d point value
 * @param[out] error Log error messages
 *
 * @return bool Transform result
 */
bool ob_calibration_3d_to_3d(const ob_calibration_param calibration_param, const ob_point3f source_point3f, const ob_sensor_type source_sensor_type,
                             const ob_sensor_type target_sensor_type, ob_point3f *target_point3f, ob_error **error);

/**
 * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 3d point of the target coordinate system.
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] source_point2f Source 2d point value
 * @param[in] source_depth_pixel_value The depth of sourcePoint2f in millimeters
 * @param[in] source_sensor_type Source sensor type
 * @param[in] target_sensor_type Target sensor type
 * @param[out] target_point3f Target 3d point value
 * @param[out] error Log error messages
 *
 * @return bool Transform result
 */
bool ob_calibration_2d_to_3d(const ob_calibration_param calibration_param, const ob_point2f source_point2f, const float source_depth_pixel_value,
                             const ob_sensor_type source_sensor_type, const ob_sensor_type target_sensor_type, ob_point3f *target_point3f, ob_error **error);

/**
 * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 3d point of the target coordinate system.
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] source_point2f Source 2d point value
 * @param[in] source_depth_pixel_value The depth of sourcePoint2f in millimeters
 * @param[in] source_sensor_type Source sensor type
 * @param[in] target_sensor_type Target sensor type
 * @param[out] target_point3f Target 3d point value
 * @param[out] error Log error messages
 *
 * @return bool Transform result
 */
bool ob_calibration_2d_to_3d_undistortion(const ob_calibration_param calibration_param, const ob_point2f source_point2f, const float source_depth_pixel_value,
                                          const ob_sensor_type source_sensor_type, const ob_sensor_type target_sensor_type, ob_point3f *target_point3f,
                                          ob_error **error);

/**
 * @brief Transform a 3d point of a source coordinate system into a 2d pixel coordinate of the target camera.
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] source_point3f Source 3d point value
 * @param[in] source_sensor_type Source sensor type
 * @param[in] target_sensor_type Target sensor type
 * @param[out] target_point2f Target 2d point value
 * @param[out] error Log error messages
 *
 * @return bool Transform result
 */
bool ob_calibration_3d_to_2d(const ob_calibration_param calibration_param, const ob_point3f source_point3f, const ob_sensor_type source_sensor_type,
                             const ob_sensor_type target_sensor_type, ob_point2f *target_point2f, ob_error **error);

/**
 * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 2d pixel coordinate of the target camera
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] source_point2f Source 2d point value
 * @param[in] source_depth_pixel_value The depth of sourcePoint2f in millimeters
 * @param[in] source_sensor_type Source sensor type
 * @param[in] target_sensor_type Target sensor type
 * @param[out] target_point2f Target 2d point value
 * @param[out] error Log error messages
 *
 * @return bool Transform result
 */
bool ob_calibration_2d_to_2d(const ob_calibration_param calibration_param, const ob_point2f source_point2f, const float source_depth_pixel_value,
                             const ob_sensor_type source_sensor_type, const ob_sensor_type target_sensor_type, ob_point2f *target_point2f, ob_error **error);

/**
 * @brief Transforms the depth frame into the geometry of the color camera.
 *
 * @param[in] device Device handle
 * @param[in] depth_frame Input depth frame
 * @param[in] target_color_camera_width Target color camera width
 * @param[in] target_color_camera_height Target color camera height
 * @param[out] error Log error messages
 *
 * @return ob_frame* Transformed depth frame
 */
ob_frame *transformation_depth_frame_to_color_camera(ob_device *device, ob_frame *depth_frame, uint32_t target_color_camera_width,
                                                     uint32_t target_color_camera_height, ob_error **error);

/**
 * @brief Init transformation tables
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] sensor_type sensor type
 * @param[in] data input data,needs to be allocated externally.During initialization, the external allocation size is 'data_size', for example, data_size = 1920
 * * 1080 * 2*sizeof(float) (1920 * 1080 represents the image resolution, and 2 represents two LUTs, one for x-coordinate and one for y-coordinate).
 * @param[in] data_size input data size
 * @param[out] xy_tables output xy tables
 * @param[out] error Log error messages
 *
 * @return bool Transform result
 */
bool transformation_init_xy_tables(const ob_calibration_param calibration_param, const ob_sensor_type sensor_type, float *data, uint32_t *data_size,
                                   ob_xy_tables *xy_tables, ob_error **error);

/**
 * @brief Transform depth image to point cloud data
 *
 * @param[in] xy_tables input xy tables,see transformation_init_xy_tables
 * @param[in] depth_image_data input depth image data
 * @param[out] pointcloud_data output point cloud data
 * @param[out] error Log error messages
 */
void transformation_depth_to_pointcloud(ob_xy_tables *xy_tables, const void *depth_image_data, void *pointcloud_data, ob_error **error);

/**
 * @brief Transform depth image to point cloud data
 *
 * @param[in] xy_tables input xy tables,see transformation_init_xy_tables
 * @param[in] depth_image_data input depth image data
 * @param[in] color_image_data input color image data (only RGB888 support)
 * @param[out] pointcloud_data output point cloud data
 * @param[out] error Log error messages
 */
void transformation_depth_to_rgbd_pointcloud(ob_xy_tables *xy_tables, const void *depth_image_data, const void *color_image_data, void *pointcloud_data,
                                             ob_error **error);

#ifdef __cplusplus
}
#endif