// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Transform a 3d point of a source coordinate system into a 3d point of the target coordinate system.
 *
 * @param[in] source_point3f Source 3d point value
 * @param[in] extrinsic Transformation matrix from source to target
 * @param[out] target_point3f Target 3d point value
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Transform result
 */
OB_EXPORT bool ob_transformation_3d_to_3d(const OBPoint3f source_point3f, OBExtrinsic extrinsic, OBPoint3f *target_point3f, ob_error **error);

/**
 * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 3d point of the target coordinate system.
 *
 * @param[in] source_point2f Source 2d point value
 * @param[in] source_depth_pixel_value The depth of sourcePoint2f in millimeters
 * @param[in] source_intrinsic Source intrinsic parameters
 * @param[in] extrinsic Transformation matrix from source to target
 * @param[out] target_point3f Target 3d point value
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Transform result
 */
OB_EXPORT bool ob_transformation_2d_to_3d(const OBPoint2f source_point2f, const float source_depth_pixel_value, const OBCameraIntrinsic source_intrinsic,
                                          OBExtrinsic extrinsic, OBPoint3f *target_point3f, ob_error **error);

/**
 * @brief Transform a 3d point of a source coordinate system into a 2d pixel coordinate of the target camera.
 *
 * @param[in] source_point3f Source 3d point value
 * @param[in] target_intrinsic Target intrinsic parameters
 * @param[in] target_distortion Target distortion parameters
 * @param[in] extrinsic Transformation matrix from source to target
 * @param[out] target_point2f Target 2d point value
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Transform result
 */
OB_EXPORT bool ob_transformation_3d_to_2d(const OBPoint3f source_point3f, const OBCameraIntrinsic target_intrinsic, const OBCameraDistortion target_distortion,
                                          OBExtrinsic extrinsic, OBPoint2f *target_point2f, ob_error **error);

/**
 * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 2d pixel coordinate of the target camera
 *
 * @param[in] source_intrinsic Source intrinsic parameters
 * @param[in] source_distortion Source distortion parameters
 * @param[in] source_point2f Source 2d point value
 * @param[in] source_depth_pixel_value The depth of sourcePoint2f in millimeters
 * @param[in] target_intrinsic Target intrinsic parameters
 * @param[in] target_distortion Target distortion parameters
 * @param[in] extrinsic Transformation matrix from source to target
 * @param[out] target_point2f Target 2d point value
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Transform result
 */
OB_EXPORT bool ob_transformation_2d_to_2d(const OBPoint2f source_point2f, const float source_depth_pixel_value, const OBCameraIntrinsic source_intrinsic,
                                          const OBCameraDistortion source_distortion, const OBCameraIntrinsic target_intrinsic,
                                          const OBCameraDistortion target_distortion, OBExtrinsic extrinsic, OBPoint2f *target_point2f, ob_error **error);

// \deprecated This function is deprecated and will be removed in a future version.
OB_EXPORT ob_frame *transformation_depth_frame_to_color_camera(ob_device *device, ob_frame *depth_frame, uint32_t target_color_camera_width,
                                                               uint32_t target_color_camera_height, ob_error **error);

// \deprecated This function is deprecated and will be removed in a future version.
OB_EXPORT bool transformation_init_xy_tables(const ob_calibration_param calibration_param, const ob_sensor_type sensor_type, float *data, uint32_t *data_size,
                                             ob_xy_tables *xy_tables, ob_error **error);

// \deprecated This function is deprecated and will be removed in a future version.
OB_EXPORT void transformation_depth_to_pointcloud(ob_xy_tables *xy_tables, const void *depth_image_data, void *pointcloud_data, ob_error **error);

// \deprecated This function is deprecated and will be removed in a future version.
OB_EXPORT void transformation_depth_to_rgbd_pointcloud(ob_xy_tables *xy_tables, const void *depth_image_data, const void *color_image_data,
                                                       void *pointcloud_data, ob_error **error);

// \deprecated This function is deprecated and will be removed in a future version.
//             Use the ob_transformation_3d_to_3d instead.
OB_EXPORT bool ob_calibration_3d_to_3d(const ob_calibration_param calibration_param, const ob_point3f source_point3f, const ob_sensor_type source_sensor_type,
                                       const ob_sensor_type target_sensor_type, ob_point3f *target_point3f, ob_error **error);

// \deprecated This function is deprecated and will be removed in a future version.
//             Use the ob_transformation_2d_to_3d instead.
OB_EXPORT bool ob_calibration_2d_to_3d(const ob_calibration_param calibration_param, const ob_point2f source_point2f, const float source_depth_pixel_value,
                                       const ob_sensor_type source_sensor_type, const ob_sensor_type target_sensor_type, ob_point3f *target_point3f,
                                       ob_error **error);

// \deprecated This function is deprecated and will be removed in a future version.
//             Use the ob_transformation_3d_to_2d instead.
OB_EXPORT bool ob_calibration_3d_to_2d(const ob_calibration_param calibration_param, const ob_point3f source_point3f, const ob_sensor_type source_sensor_type,
                                       const ob_sensor_type target_sensor_type, ob_point2f *target_point2f, ob_error **error);

// \deprecated This function is deprecated and will be removed in a future version.
//             Use the ob_transformation_2d_to_2d instead.
OB_EXPORT bool ob_calibration_2d_to_2d(const ob_calibration_param calibration_param, const ob_point2f source_point2f, const float source_depth_pixel_value,
                                       const ob_sensor_type source_sensor_type, const ob_sensor_type target_sensor_type, ob_point2f *target_point2f,
                                       ob_error **error);
/**
 * @brief save point cloud to ply file.
 *
 * @param[in] file_name Point cloud save path
 * @param[in] frame Point cloud frame
 * @param[in] save_binary Binary or textual,true: binary, false: textual
 * @param[in] use_mesh Save mesh or not, true: save as mesh, false: not save as mesh
 * @param[in] mesh_threshold Distance threshold for creating faces in point cloud,default value :50
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool save point cloud result
 */
OB_EXPORT bool ob_save_pointcloud_to_ply(const char *file_name, ob_frame *frame, bool save_binary, bool use_mesh, float mesh_threshold, ob_error **error);

/**
 * @brief save LiDAR point cloud to ply file.
 *
 * @param[in] file_name Point cloud save path
 * @param[in] frame LiDAR point cloud frame
 * @param[in] save_binary Binary or textual,true: binary, false: textual
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool save LiDAR point cloud result
 */
OB_EXPORT bool ob_save_lidar_pointcloud_to_ply(const char *file_name, ob_frame *frame, bool save_binary, ob_error **error);
#ifdef __cplusplus
}
#endif
