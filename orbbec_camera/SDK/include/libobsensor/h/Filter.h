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
 * @brief Create a PointCloud Filter.
 *
 * @param[out] error Log error messages.
 *
 * @return A pointcloud_filter object.
 */
ob_filter *ob_create_pointcloud_filter(ob_error **error);

/**
 * @brief Set the camera parameters for the PointCloud Filter.
 *
 * @param[in] filter A pointcloud_filter object.
 * @param[in] param Camera parameters.
 * @param[out] error Log error messages.
 */
void ob_pointcloud_filter_set_camera_param(ob_filter *filter, ob_camera_param param, ob_error **error);

/**
 * @brief Set the point cloud type parameters for the PointCloud Filter.
 *
 * @param[in] filter A pointcloud_filter object.
 * @param[in] type Point cloud type: depth point cloud or RGBD point cloud.
 * @param[out] error Log error messages.
 */
void ob_pointcloud_filter_set_point_format(ob_filter *filter, ob_format type, ob_error **error);

/**
 * @brief Set the alignment state of the frames that will be input to produce the point cloud.
 *
 * @param[in] filter A pointcloud_filter object.
 * @param[in] state Alignment status, True: aligned; False: unaligned.
 * @param[out] error Log error messages.
 */
void ob_pointcloud_filter_set_frame_align_state(ob_filter *filter, bool state, ob_error **error);

/**
 * @brief Set the point cloud data scaling factor.
 *
 * @attention Calling this function to set the scale will change the point coordinate scaling factor of the output point cloud frame: posScale = posScale /
 * scale. The point coordinate scaling factor for the output point cloud frame can be obtained via the @ref ob_points_frame_get_position_value_scale function.
 *
 * @param[in] filter A pointcloud_filter object.
 * @param[in] scale Set the point cloud coordinate data zoom factor.
 * @param[out] error Log error messages.
 */
void ob_pointcloud_filter_set_position_data_scale(ob_filter *filter, float scale, ob_error **error);

/**
 * @brief Set the point cloud color data normalization.
 *
 * @param[in] filter A pointcloud_filter object.
 * @param[in] state Sets whether the point cloud color data is normalized.
 * @param[out] error Log error messages.
 */
void ob_pointcloud_filter_set_color_data_normalization(ob_filter *filter, bool state, ob_error **error);

/**
 * @brief Set the point cloud coordinate system.
 *
 * @param[in] filter A pointcloud_filter object.
 * @param[in] type Coordinate system type.
 * @param[out] error Log error messages.
 */
void ob_pointcloud_filter_set_coordinate_system(ob_filter *filter, ob_coordinate_system_type type, ob_error **error);

/**
 * @brief Create a format convert Filter.
 *
 * @param[out] error Log error messages.
 *
 * @return A format_convert object.
 */
ob_filter *ob_create_format_convert_filter(ob_error **error);

/**
 * @brief Set the type of format conversion for the format convert Filter.
 *
 * @param[in] filter A format convert filter object.
 * @param[in] type Format conversion type.
 * @param[out] error Log error messages.
 */
void ob_format_convert_filter_set_format(ob_filter *filter, ob_convert_format type, ob_error **error);

/**
 * @brief Create a compression Filter.
 *
 * @param[out] error Log error messages.
 *
 * @return A depth_filter object.
 */
ob_filter *ob_create_compression_filter(ob_error **error);

/**
 * @brief Set the compression parameters for the compression Filter.
 *
 * @param[in] filter A compression_filter object.
 * @param[in] mode Compression mode OB_COMPRESSION_LOSSLESS or OB_COMPRESSION_LOSSY.
 * @param[in] params Compression params, struct ob_compression_params, when mode is OB_COMPRESSION_LOSSLESS, params is NULL.
 * @param[out] error Log error messages.
 */
void ob_compression_filter_set_compression_params(ob_filter *filter, ob_compression_mode mode, void *params, ob_error **error);

/**
 * @brief Create a decompression Filter.
 *
 * @param[out] error Log error messages.
 *
 * @return A decompression Filter object.
 */
ob_filter *ob_create_decompression_filter(ob_error **error);

/**
 * @brief Create a HoleFilling Filter.
 *
 * @param[out] error Log error messages.
 *
 * @return A depth_filter object.
 */
ob_filter *ob_create_holefilling_filter(ob_error **error);

/**
 * @brief Set the HoleFillingFilter mode.
 *
 * @param[in] filter A holefilling_filter object.
 * @param[in] mode holefilling mode OB_HOLE_FILL_TOP,OB_HOLE_FILL_NEAREST or OB_HOLE_FILL_FAREST.
 * @param[out] error Log error messages.
 */
void ob_holefilling_filter_set_mode(ob_filter *filter, ob_hole_filling_mode mode, ob_error **error);

/**
 * @brief Get the HoleFillingFilter mode.
 *
 * @param[in] filter A holefilling_filter object.
 * @param[out] error Log error messages.
 * @return ob_hole_filling_mode
 */
ob_hole_filling_mode ob_holefilling_filter_get_mode(ob_filter *filter, ob_error **error);

/**
 * @brief Create a Temporal Filter.
 *
 * @param[out] error Log error messages.
 *
 * @return A depth_filter object.
 */
ob_filter *ob_create_temporal_filter(ob_error **error);

/**
 * @brief Get the TemporalFilter diffscale range.
 *
 * @param[in] filter A temporal_filter object.
 * @param[out] error Log error messages.
 * @return ob_float_property_range the value of property range.
 */
ob_float_property_range ob_temporal_filter_get_diffscale_range(ob_filter *filter, ob_error **error);

/**
 * @brief Set the TemporalFilter diffscale value.
 *
 * @param[in] filter A temporal_filter object.
 * @param[in] value diffscale value.
 * @param[out] error Log error messages.
 */
void ob_temporal_filter_set_diffscale_value(ob_filter *filter, float value, ob_error **error);

/**
 * @brief Get the TemporalFilter weight range.
 *
 * @param[in] filter A temporal filter object.
 * @param[out] error Log error messages.
 */
ob_float_property_range ob_temporal_filter_get_weight_range(ob_filter *filter, ob_error **error);

/**
 * @brief Set the TemporalFilter weight range.
 *
 * @param[in] filter A temporal_filter object.
 * @param[in] value weight value.
 * @param[out] error Log error messages.
 */
void ob_temporal_filter_set_weight_value(ob_filter *filter, float value, ob_error **error);

/**
 * @brief Create a spatial advanced filter.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_spatial_advanced_filter(ob_error **error);

/**
 * @brief Get the spatial advanced filter alpha range.
 *
 * @param[in] filter A spatial advanced filter object.
 * @param[out] error Log error messages.
 * @return ob_float_property_range the alpha value of property range.
 */
ob_float_property_range ob_spatial_advanced_filter_get_alpha_range(ob_filter *filter, ob_error **error);

/**
 * @brief Get the spatial advanced filter disp diff range.
 *
 * @param[in] filter A spatial advanced filter object.
 * @param[out] error Log error messages.
 * @return ob_uint16_property_range the dispdiff value of property range.
 */
ob_uint16_property_range ob_spatial_advanced_filter_get_disp_diff_range(ob_filter *filter, ob_error **error);

/**
 * @brief Get the spatial advanced filter radius range.
 *
 * @param[in] filter A spatial advanced filter object.
 * @param[out] error Log error messages.
 * @return ob_uint16_property_range the radius value of property range.
 */
ob_uint16_property_range ob_spatial_advanced_filter_get_radius_range(ob_filter *filter, ob_error **error);

/**
 * @brief Get the spatial advanced filter magnitude range.
 *
 * @param[in] filter A spatial advanced filter object.
 * @param[out] error Log error messages.
 * @return ob_int_property_range the magnitude value of property range.
 */
ob_int_property_range ob_spatial_advanced_filter_get_magnitude_range(ob_filter *filter, ob_error **error);

/**
 * @brief Get the spatial advanced filter params.
 *
 * @param[in] filter A spatial advanced filter object.
 * @param[out] error Log error messages.
 * @return ob_spatial_advanced_filter_params.
 */
ob_spatial_advanced_filter_params ob_spatial_advanced_filter_get_filter_params(ob_filter *filter, ob_error **error);

/**
 * @brief Set the spatial advanced filter params.
 *
 * @param[in] filter A spatial advanced filter object.
 * @param[in] params ob_spatial_advanced_filter_params.
 * @param[out] error Log error messages.
 */
void ob_spatial_advanced_filter_set_filter_params(ob_filter *filter, ob_spatial_advanced_filter_params params, ob_error **error);

/**
 * @brief Create a spatial fast filter.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_spatial_fast_filter(ob_error **error);

/**
 * @brief Get the spatial fast filter window size range.
 *
 * @param[in] filter A spatial fast filter object.
 * @param[out] error Log error messages.
 * @return ob_uint8_property_range the filter window size value of property range.
 */
ob_uint8_property_range ob_spatial_fast_filter_get_size_range(ob_filter *filter, ob_error **error);

/**
 * @brief Set the spatial fast filter params.
 *
 * @param[in] filter A spatial fast filter object.
 * @param[in] params ob_spatial_fast_filter_params.
 * @param[out] error Log error messages.
 */
void ob_spatial_fast_filter_set_filter_params(ob_filter *filter, ob_spatial_fast_filter_params params, ob_error **error);

/**
 * @brief Get the spatial fast filter params.
 *
 * @param[in] filter A spatial fast filter object.
 * @param[out] error Log error messages.
 * @return ob_spatial_fast_filter_params.
 */
ob_spatial_fast_filter_params ob_spatial_fast_filter_get_filter_params(ob_filter *filter, ob_error **error);

/**
 * @brief Create a spatial moderate filter.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_spatial_moderate_filter(ob_error **error);
/**
 * @brief Get the spatial moderate filter disp diff range.
 *
 * @param[in] filter A spatial moderate filter object.
 * @param[out] error Log error messages.
 * @return ob_uint16_property_range the dispdiff value of property range.
 */
ob_uint16_property_range ob_spatial_moderate_filter_get_disp_diff_range(ob_filter *filter, ob_error **error);
/**
 * @brief Get the spatial moderate filter magnitude range.
 *
 * @param[in] filter A spatial moderate filter object.
 * @param[out] error Log error messages.
 * @return ob_uint8_property_range the magnitude value of property range.
 */
ob_uint8_property_range ob_spatial_moderate_filter_get_magnitude_range(ob_filter *filter, ob_error **error);
/**
 * @brief Get the spatial moderate filter window size range.
 *
 * @param[in] filter A spatial moderate filter object.
 * @param[out] error Log error messages.
 * @return ob_uint8_property_range the filter window size value of property range.
 */
ob_uint8_property_range ob_spatial_moderate_filter_get_size_range(ob_filter *filter, ob_error **error);
/**
 * @brief Get the spatial moderate filter params.
 *
 * @param[in] filter A spatial moderate filter object.
 * @param[out] error Log error messages.
 * @return ob_spatial_moderate_filter_params.
 */
ob_spatial_moderate_filter_params ob_spatial_moderate_filter_get_filter_params(ob_filter *filter, ob_error **error);
/**
 * @brief Set the spatial moderate filter params.
 *
 * @param[in] filter A spatial moderate filter object.
 * @param[in] params ob_spatial_moderate_filter_params.
 * @param[out] error Log error messages.
 */
void ob_spatial_moderate_filter_set_filter_params(ob_filter *filter, ob_spatial_moderate_filter_params params, ob_error **error);

/**
 * @brief Create a noise removal filter.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_noise_removal_filter(ob_error **error);

/**
 * @brief Get the noise removal filter disp diff range.
 *
 * @param[in] filter A noise removal filter object.
 * @param[out] error Log error messages.
 * @return ob_uint16_property_range the disp_diff value of property range.
 */
ob_uint16_property_range ob_noise_removal_filter_get_disp_diff_range(ob_filter *filter, ob_error **error);

/**
 * @brief Get the noise removal filter max size range.
 *
 * @param[in] filter noise removal filter object.
 * @param[out] error Log error messages.
 * @return ob_int_property_range the _max_size value of property range.
 */
ob_int_property_range ob_noise_removal_filter_get_max_size_range(ob_filter *filter, ob_error **error);

/**
 * @brief Set the noise removal filter params.
 *
 * @param[in] filter noise removal filter object.
 * @param[in] params ob_noise_removal_filter_params.
 * @param[out] error Log error messages.
 */
void ob_noise_removal_filter_set_filter_params(ob_filter *filter, ob_noise_removal_filter_params params, ob_error **error);

/**
 * @brief Get the noise removal filter params.
 *
 * @param[in] filter noise removal filter object.
 * @param[out] error Log error messages.
 * @return ob_noise_removal_filter_params.
 */
ob_noise_removal_filter_params ob_noise_removal_filter_get_filter_params(ob_filter *filter, ob_error **error);

/**
 * @brief Create a edge noise removal filter.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_edge_noise_removal_filter(ob_error **error);

/**
 * @brief Set the edge noise removal filter params.
 *
 * @param[in] filter edge noise removal filter object.
 * @param[in] params ob_edge_noise_removal_filter_params.
 * @param[out] error Log error messages.
 */
void ob_edge_noise_removal_filter_set_filter_params(ob_filter *filter, ob_edge_noise_removal_filter_params params, ob_error **error);

/**
 * @brief Get the edge noise removal filter params.
 *
 * @param[in] filter edge noise removal filter object.
 * @param[out] error Log error messages.
 * @return ob_edge_noise_removal_filter_params.
 */
ob_edge_noise_removal_filter_params ob_edge_noise_removal_filter_get_filter_params(ob_filter *filter, ob_error **error);


/**
 * @brief Get the noise removal filter margin left th range.
 *
 * @param[in] filter A edge noise removal filter object.
 * @param[out] error Log error messages.
 * @return ob_uint16_property_range the margin_left_th value of property range.
 */
ob_uint16_property_range ob_edge_noise_removal_filter_get_margin_left_th_range(ob_filter *filter, ob_error **error);

/**
 * @brief Get the noise removal filter margin right th range.
 *
 * @param[in] filter A edge noise removal filter object.
 * @param[out] error Log error messages.
 * @return ob_uint16_property_range the margin_right_th value of property range.
 */
ob_uint16_property_range ob_edge_noise_removal_filter_get_margin_right_th_range(ob_filter *filter, ob_error **error);

/**
 * @brief Get the noise removal filter margin top th range.
 *
 * @param[in] filter A edge noise removal filter object.
 * @param[out] error Log error messages.
 * @return ob_uint16_property_range the margin_top_th value of property range.
 */
ob_uint16_property_range ob_edge_noise_removal_filter_get_margin_top_th_range(ob_filter *filter, ob_error **error);

/**
 * @brief Get the noise removal filter margin bottom th range.
 *
 * @param[in] filter A edge noise removal filter object.
 * @param[out] error Log error messages.
 * @return ob_uint16_property_range the margin_bottom_th value of property range.
 */
ob_uint16_property_range ob_edge_noise_removal_filter_get_margin_bottom_th_range(ob_filter *filter, ob_error **error);


/**
 * @brief Create a decimation filter.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_decimation_filter(ob_error **error);

/**
 * @brief Get the decimation filter scale range.
 *
 * @param[in] filter A decimation filter object.
 * @param[out] error Log error messages.
 */
ob_uint8_property_range ob_decimation_filter_get_scale_range(ob_filter *filter, ob_error **error);

/**
 * @brief Set the decimation filter scale value.
 *
 * @param[in] filter A decimation object.
 * @param[in] value decimation filter scale value.
 * @param[out] error Log error messages.
 */
void ob_decimation_filter_set_scale_value(ob_filter *filter, uint8_t value, ob_error **error);

/**
 * @brief Get the decimation filter scale value.
 *
 * @param[in] filter A decimation object.
 * @param[out] error Log error messages.
 * @return decimation filter scale value.
 */
uint8_t ob_decimation_filter_get_scale_value(ob_filter *filter, ob_error **error);

/**
 * @brief Create a threshold filter.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_threshold_filter(ob_error **error);

/**
 * @brief Get the threshold filter min range.
 *
 * @param[in] filter A threshold filter object.
 * @param[out] error Log error messages.
 */
ob_int_property_range ob_threshold_filter_get_min_range(ob_filter *filter, ob_error **error);

/**
 * @brief Get the threshold filter max range.
 *
 * @param[in] filter A threshold filter object.
 * @param[out] error Log error messages.
 */
ob_int_property_range ob_threshold_filter_get_max_range(ob_filter *filter, ob_error **error);

/**
 * @brief Set the threshold filter scale range.
 *
 * @param[in] filter A threshold object.
 * @param[in] min threshold filter scale min value.
 * @param[in] max threshold filter scale max value.
 * @param[out] error Log error messages.
 */
bool ob_threshold_filter_set_scale_value(ob_filter *filter, uint16_t min, uint16_t max, ob_error **error);

/**
 * @brief Create a SequenceId filter.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_sequenceId_filter(ob_error **error);

/**
 * @brief Set the sequence id filter select sequence id.
 *
 * @param[in] filter A sequence id object.
 * @param[in] sequence_id sequence id to pass the filter.
 * @param[out] error Log error messages.
 */
void ob_sequence_id_filter_select_sequence_id(ob_filter *filter, int sequence_id, ob_error **error);

/**
 * @brief Get the current sequence id.
 *
 * @param[in] filter A sequence id object.
 * @param[out] error Log error messages.
 * @return sequence id to pass the filter.
 */
int ob_sequence_id_filter_get_sequence_id(ob_filter *filter, ob_error **error);


/**
 * @brief Get the current sequence id list.
 *
 * @param[in] filter A sequence id object.
 * @param[out] error Log error messages.
 */
ob_sequence_id_item *ob_sequence_id_filter_get_sequence_id_list(ob_filter *filter, ob_error **error);

/**
 * @brief Get the current sequence id list size.
 *
 * @param[in] filter A sequence id object.
 * @param[out] error Log error messages.
 */
int ob_sequence_id_filter_get_sequence_id_list_size(ob_filter *filter, ob_error **error);

/**
 * @brief Create a hdr merge.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_hdr_merge(ob_error **error);

/**
 * @brief Create a align.
 * @param[in] align_to_stream ob_stream_type.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_align(ob_error **error, ob_stream_type align_to_stream);

/**
 * @brief Get the algin stream type.
 * 
 * @param[in] filter A align object.
 * @param[out] error Log error messages.
 * @return A ob_stream_type.
 */
ob_stream_type ob_align_get_to_stream_type(ob_filter *filter, ob_error **error);

/**
 * @brief Create a disparity transform.
 * @param[in] depth_to_disparity disparity to depth, depth to disparity Conversion.
 * @param[out] error Log error messages.
 * @return A depth_filter object.
 */
ob_filter *ob_create_disparity_transform(ob_error **error, bool depth_to_disparity);

/**
 * @brief Reset the filter, clears the cache, and resets the state. If the asynchronous interface is used, the processing thread will also be stopped and the
 * pending cache frames will be cleared.
 *
 * @param[in] filter A filter object.
 * @param[out] error Log error messages.
 */
void ob_filter_reset(ob_filter *filter, ob_error **error);

/**
 * @brief Process the frame (synchronous interface).
 *
 * @param[in] filter A filter object.
 * @param[in] frame Pointer to the frame object to be processed.
 * @param[out] error Log error messages.
 *
 * @return The frame object processed by the filter.
 */
ob_frame *ob_filter_process(ob_filter *filter, ob_frame *frame, ob_error **error);

/**
 * @brief Enable the frame post processing
 *
 * @param[in] filter A filter object.
 * @param[in] enable enable status
 * @param[out] error Log error messages.
 */
void ob_filter_enable(ob_filter *filter, bool enable, ob_error **error);

/**
 * @brief Get the enable status of the frame post processing
 *
 * @param[in] filter A filter object.
 * @param[out] error Log error messages.
 *
 * @return The post processing filter status.
 */
bool ob_filter_is_enable(ob_filter *filter, ob_error **error);

/**
 * @brief Set the processing result callback function for the filter (asynchronous callback interface).
 *
 * @param[in] filter A filter object.
 * @param[in] callback Callback function.
 * @param[in] user_data Arbitrary user data pointer can be passed in and returned from the callback.
 * @param[out] error Log error messages.
 */
void ob_filter_set_callback(ob_filter *filter, ob_filter_callback callback, void *user_data, ob_error **error);

/**
 * @brief Push the frame into the pending cache for the filter (asynchronous callback interface).
 *
 * @param[in] filter A filter object.
 * @param[in] frame Pointer to the frame object to be processed.
 * @param[out] error Log error messages.
 */
void ob_filter_push_frame(ob_filter *filter, ob_frame *frame, ob_error **error);

/**
 * @brief Delete the filter.
 *
 * @param[in] filter A filter object.
 * @param[out] error Log error messages.
 */
void ob_delete_filter(ob_filter *filter, ob_error **error);

#ifdef __cplusplus
}
#endif
