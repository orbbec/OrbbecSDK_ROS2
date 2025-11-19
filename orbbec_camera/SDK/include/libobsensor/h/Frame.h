// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Frame.h
 * @brief Frame related function is mainly used to obtain frame data and frame information
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Crate a frame object based on the specified parameters.
 *
 * @attention The frame object is created with a reference count of 1, and the reference count should be decreased by calling @ref ob_delete_frame() when it is
 * no longer needed.
 *
 * @param[in] frame_type The frame object type.
 * @param[in] format The frame object format.
 * @param[in] data_size The size of the frame object data.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the frame object.
 */
OB_EXPORT ob_frame *ob_create_frame(ob_frame_type frame_type, ob_format format, uint32_t data_size, ob_error **error);

/**
 * @brief Create (clone) a frame object based on the specified other frame object.
 * @brief The new frame object will have the same properties as the other frame object, but the data buffer is newly allocated.
 *
 * @attention The frame object is created with a reference count of 1, and the reference count should be decreased by calling @ref ob_delete_frame() when it is
 * no longer needed.
 *
 * @param[in] other_frame The frame object to create the new frame object according to.
 * @param[in] should_copy_data If true, the data of the source frame object will be copied to the new frame object. If false, the new frame object will
 * have a data buffer with random data.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the new frame object.
 */
OB_EXPORT ob_frame *ob_create_frame_from_other_frame(const ob_frame *other_frame, bool should_copy_data, ob_error **error);

/**
 * @brief Create a frame object according to the specified stream profile.
 *
 * @attention The frame object is created with a reference count of 1, and the reference count should be decreased by calling @ref ob_delete_frame() when it is
 * no logger needed.
 *
 * @param[in] stream_profile The stream profile to create the new frame object according to.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the new frame object.
 */
OB_EXPORT ob_frame *ob_create_frame_from_stream_profile(const ob_stream_profile *stream_profile, ob_error **error);

/**
 * @brief Create an video frame object based on the specified parameters.
 *
 * @attention The frame object is created with a reference count of 1, and the reference count should be decreased by calling @ref ob_delete_frame() when it is
 * no longer needed.
 *
 * @param[in] frame_type Frame object type.
 * @param[in] format Frame object format.
 * @param[in] width Frame object width.
 * @param[in] height Frame object height.
 * @param[in] stride_bytes Row span in bytes. If 0, the stride is calculated based on the width and format.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return an empty frame object.
 */
OB_EXPORT ob_frame *ob_create_video_frame(ob_frame_type frame_type, ob_format format, uint32_t width, uint32_t height, uint32_t stride_bytes, ob_error **error);

/**
 * @brief Create a frame object based on an externally created buffer.
 *
 * @attention The buffer is owned by the user and will not be destroyed by the frame object. The user should ensure that the buffer is valid and not modified.
 * @attention The frame object is created with a reference count of 1, and the reference count should be decreased by calling @ref ob_delete_frame() when it is
 * no longer needed.
 *
 * @param[in] frame_type Frame object type.
 * @param[in] format Frame object format.
 * @param[in] buffer Frame object buffer.
 * @param[in] buffer_size Frame object buffer size.
 * @param[in] buffer_destroy_cb Destroy callback, will be called when the frame object is destroyed.
 * @param[in] buffer_destroy_context Destroy context, user-defined context to be passed to the destroy callback.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the frame object.
 */
OB_EXPORT ob_frame *ob_create_frame_from_buffer(ob_frame_type frame_type, ob_format format, uint8_t *buffer, uint32_t buffer_size,
                                                ob_frame_destroy_callback *buffer_destroy_cb, void *buffer_destroy_context, ob_error **error);

/**
 * @brief Create a video frame object based on an externally created buffer.
 *
 * @attention The buffer is owned by the user and will not be destroyed by the frame object. The user should ensure that the buffer is valid and not modified.
 * @attention The frame object is created with a reference count of 1, and the reference count should be decreased by calling @ref ob_delete_frame() when it is
 * no longer needed.
 *
 * @param[in] frame_type Frame object type.
 * @param[in] format Frame object format.
 * @param[in] width Frame object width.
 * @param[in] height Frame object height.
 * @param[in] stride_bytes Row span in bytes. If 0, the stride is calculated based on the width and format.
 * @param[in] buffer Frame object buffer.
 * @param[in] buffer_size Frame object buffer size.
 * @param[in] buffer_destroy_cb Destroy callback, user-defined function to destroy the buffer.
 * @param[in] buffer_destroy_context Destroy context, user-defined context to be passed to the destroy callback.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the frame object.
 */
OB_EXPORT ob_frame *ob_create_video_frame_from_buffer(ob_frame_type frame_type, ob_format format, uint32_t width, uint32_t height, uint32_t stride_bytes,
                                                      uint8_t *buffer, uint32_t buffer_size, ob_frame_destroy_callback *buffer_destroy_cb,
                                                      void *buffer_destroy_context, ob_error **error);

/**
 * @brief Create an empty frameset object.
 * @brief A frameset object is a special type of frame object that can be used to store multiple frames.
 *
 * @attention The frameset object is created with a reference count of 1, and the reference count should be decreased by calling @ref ob_delete_frame() when it
 * is no longer needed.
 *
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_frame* Return the frameset object.
 */
OB_EXPORT ob_frame *ob_create_frameset(ob_error **error);

/**
 * @brief Increase the reference count of a frame object.
 * @brief The reference count is used to manage the lifetime of the frame object.
 *
 * @attention When calling this function, the reference count of the frame object is
 * increased and requires to be decreased by calling @ref ob_delete_frame().
 *
 * @param[in] frame Frame object to increase the reference count.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_frame_add_ref(const ob_frame *frame, ob_error **error);

/**
 * @brief Delete a frame object
 * @brief This function will decrease the reference count of the frame object and release the memory if the reference count becomes 0.
 *
 * @param[in] frame The frame object to delete.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_frame(const ob_frame *frame, ob_error **error);

/**
 * @brief Copy the information of the source frame object to the destination frame object.
 * @brief Including the index, timestamp, system timestamp, global timestamp and metadata will be copied.
 *
 * @param[in] src_frame Source frame object to copy the information from.
 * @param[in] dst_frame Destination frame object to copy the information to.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_frame_copy_info(const ob_frame *src_frame, ob_frame *dst_frame, ob_error **error);

/**
 * @brief Get the frame index
 *
 * @param[in] frame Frame object
 * @param[out] error Log wrong message
 *
 * @return uint64_t return the frame index
 */
OB_EXPORT uint64_t ob_frame_get_index(const ob_frame *frame, ob_error **error);

/**
 * @brief Get the frame format
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_format return the frame format
 */
OB_EXPORT ob_format ob_frame_get_format(const ob_frame *frame, ob_error **error);

/**
 * @brief Get the frame type
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame_type return the frame type
 */
OB_EXPORT ob_frame_type ob_frame_get_type(const ob_frame *frame, ob_error **error);

/**
 * @brief Get the frame timestamp (also known as device timestamp, hardware timestamp) of the frame in microseconds.
 * @brief The hardware timestamp is the time point when the frame was captured by the device (Typically in the mid-exposure, unless otherwise stated), on device
 * clock domain.
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint64_t return the frame hardware timestamp in microseconds
 */
OB_EXPORT uint64_t ob_frame_get_timestamp_us(const ob_frame *frame, ob_error **error);

/**
 * @brief Set the frame timestamp (also known as the device timestamp, hardware timestamp) of a frame object.
 *
 * @param[in] frame Frame object to set the timestamp.
 * @param[in] timestamp_us frame timestamp to set in microseconds.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_frame_set_timestamp_us(ob_frame *frame, uint64_t timestamp_us, ob_error **error);

/**
 * @brief Get the system timestamp of the frame in microseconds.
 * @brief The system timestamp is the time point when the frame was received by the host, on host clock domain.
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint64_t return the frame system timestamp in microseconds
 */
OB_EXPORT uint64_t ob_frame_get_system_timestamp_us(const ob_frame *frame, ob_error **error);

/**
 * @brief Set the system timestamp of the frame in microseconds.
 *
 * @param[in] frame Frame object
 * @param[in] system_timestamp_us frame system timestamp to set in microseconds.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_frame_set_system_timestamp_us(ob_frame *frame, uint64_t system_timestamp_us, ob_error **error);

/**
 * @brief Get the global timestamp of the frame in microseconds.
 * @brief The global timestamp is the time point when the frame was captured by the device, and has been converted to the host clock domain. The
 * conversion process base on the frame timestamp and can eliminate the timer drift of the device
 *
 * @attention The global timestamp is disabled by default. If global timestamp is not enabled, the function will return 0. To enable it, call @ref
 * ob_device_enable_global_timestamp() function.
 * @attention Only some models of device support getting the global timestamp.  Check the device support status by @ref
 * ob_device_is_global_timestamp_supported() function.
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint64_t The global timestamp of the frame in microseconds.
 */
OB_EXPORT uint64_t ob_frame_get_global_timestamp_us(const ob_frame *frame, ob_error **error);

/**
 * @brief Get the data buffer of a frame.
 *
 * @attention The returned data buffer is mutable, but it is not recommended to modify it directly. Modifying the data directly may cause issues if the frame is
 * being used in other threads  or future use. If you need to modify the data, it is recommended to create a new frame object.
 *
 * @param[in] frame The frame object from which to retrieve the data.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint8_t* Pointer to the frame data buffer.
 */
OB_EXPORT uint8_t *ob_frame_get_data(const ob_frame *frame, ob_error **error);

/**
 * @brief Update the data of a frame.
 * @brief The data will be memcpy to the frame data buffer.
 * @brief The frame data size will be also updated as the input data size.
 *
 * @attention It is not recommended to update the frame data if the frame was not created by the user. If you must update it, ensure that the frame is not being
 * used in other threads.
 * @attention The size of the new data should be equal to or less than the current data size of the frame. Exceeding the original size may cause memory
 * exceptions.
 *
 * @param[in] frame The frame object to update.
 * @param[in] data The new data to update the frame with.
 * @param[in] data_size The size of the new data.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_frame_update_data(ob_frame *frame, const uint8_t *data, uint32_t data_size, ob_error **error);

/**
 * @brief Get the frame data size
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint32_t return the frame data size
 * If it is point cloud data, it return the number of bytes occupied by all point sets. If you need to find the number of points, you need to divide dataSize
 * by the structure size of the corresponding point type.
 */
OB_EXPORT uint32_t ob_frame_get_data_size(const ob_frame *frame, ob_error **error);

/**
 * @brief Get the metadata of the frame
 *
 * @attention The returned metadata is mutable, but it is not recommended to modify it directly. Modifying the metadata directly may cause issues if the frame
 * is being used in other threads or future use. If you need to modify the metadata, it is recommended to create a new frame object.
 *
 * @param[in] frame frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return const uint8_t * return the metadata pointer of the frame
 */
OB_EXPORT uint8_t *ob_frame_get_metadata(const ob_frame *frame, ob_error **error);
#define ob_video_frame_metadata ob_frame_get_metadata  // for compatibility

/**
 * @brief Get the metadata size of the frame
 *
 * @param[in] frame frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint32_t return the metadata size of the frame
 */
OB_EXPORT uint32_t ob_frame_get_metadata_size(const ob_frame *frame, ob_error **error);
#define ob_video_frame_metadata_size ob_frame_get_metadata_size  // for compatibility

/**
 * @brief Update the metadata of the frame
 * @brief The metadata will be memcpy to the frame metadata buffer.
 * @brief The frame metadata size will be also updated as the input metadata size.
 *
 * @attention It is not recommended to update the frame metadata if the frame was not created by the user. If you must update it, ensure that the frame is not
 * being used in other threads or future use.
 * @attention The metadata size should be equal to or less than 256 bytes, otherwise it will cause memory exception.
 *
 * @param[in] frame frame object
 * @param[in] metadata The new metadata to update.
 * @param[in] metadata_size The size of the new metadata.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_frame_update_metadata(ob_frame *frame, const uint8_t *metadata, uint32_t metadata_size, ob_error **error);

/**
 * @brief check if the frame contains the specified metadata
 *
 * @param[in] frame frame object
 * @param[in] type metadata type, refer to @ref ob_frame_metadata_type
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT bool ob_frame_has_metadata(const ob_frame *frame, ob_frame_metadata_type type, ob_error **error);

/**
 * @brief Get the metadata value of the frame
 *
 * @param[in] frame frame object
 * @param[in] type metadata type, refer to @ref ob_frame_metadata_type
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return int64_t return the metadata value of the frame
 */
OB_EXPORT int64_t ob_frame_get_metadata_value(const ob_frame *frame, ob_frame_metadata_type type, ob_error **error);

/**
 * @brief Get the stream profile of the frame
 *
 * @attention Require @ref ob_delete_stream_profile() to release the return stream profile.
 *
 * @param[in] frame frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_stream_profile* Return the stream profile of the frame, if the frame is not captured by a sensor stream, it will return NULL
 */
OB_EXPORT ob_stream_profile *ob_frame_get_stream_profile(const ob_frame *frame, ob_error **error);

/**
 * @brief Set (override) the stream profile of the frame
 *
 * @param[in] frame frame object
 * @param[in] stream_profile The stream profile to set for the frame.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_frame_set_stream_profile(ob_frame *frame, const ob_stream_profile *stream_profile, ob_error **error);

/**
 * @brief Get the sensor of the frame
 *
 * @attention Require @ref ob_delete_sensor() to release the return sensor.
 *
 * @param[in] frame frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_sensor* return the sensor of the frame, if the frame is not captured by a sensor or the sensor stream has been destroyed, it will return NULL
 */
OB_EXPORT ob_sensor *ob_frame_get_sensor(const ob_frame *frame, ob_error **error);

/**
 * @brief Get the device of the frame
 *
 * @attention Require @ref ob_delete_device() to release the return device.
 *
 * @param[in] frame frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_device* return the device of the frame, if the frame is not captured by a sensor stream or the device has been destroyed, it will return NULL
 */
OB_EXPORT ob_device *ob_frame_get_device(const ob_frame *frame, ob_error **error);

/**
 * @brief Get video frame width
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint32_t return the frame width
 */
OB_EXPORT uint32_t ob_video_frame_get_width(const ob_frame *frame, ob_error **error);

/**
 * @brief Get video frame height
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint32_t return the frame height
 */
OB_EXPORT uint32_t ob_video_frame_get_height(const ob_frame *frame, ob_error **error);

/**
 * @brief Get video frame pixel format
 * @brief Usually used to determine the pixel type of depth frame (depth, disparity, raw phase, etc.)
 *
 * @attention Always return OB_PIXEL_UNKNOWN for non-depth frame currently if user has not set the pixel type by @ref ob_video_frame_set_pixel_type()
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_pixel_type return the pixel format of the frame.
 */
OB_EXPORT ob_pixel_type ob_video_frame_get_pixel_type(const ob_frame *frame, ob_error **error);

/**
 * @brief Set video frame pixel format
 *
 * @param[in] frame Frame object
 * @param[in] pixel_type the pixel format of the frame
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_video_frame_set_pixel_type(ob_frame *frame, ob_pixel_type pixel_type, ob_error **error);

/**
 * @brief Get the effective number of pixels (such as Y16 format frame, but only the lower 10 bits are effective bits, and the upper 6 bits are filled with 0)
 * @attention Only valid for Y8/Y10/Y11/Y12/Y14/Y16 format
 *
 * @param[in] frame video frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint8_t return the effective number of pixels in the pixel, or 0 if it is an unsupported format
 */
OB_EXPORT uint8_t ob_video_frame_get_pixel_available_bit_size(const ob_frame *frame, ob_error **error);

/**
 * @brief Set the effective number of pixels (such as Y16 format frame, but only the lower 10 bits are effective bits, and the upper 6 bits are filled with 0)
 * @attention Only valid for Y8/Y10/Y11/Y12/Y14/Y16 format
 *
 * @param[in] frame video frame object
 * @param[in] bit_size the effective number of pixels in the pixel, or 0 if it is an unsupported format
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_video_frame_set_pixel_available_bit_size(ob_frame *frame, uint8_t bit_size, ob_error **error);

/**
 * @brief Get the value scale of the depth frame. The pixel value of the depth frame is multiplied by the scale to give a depth value in millimeters.
 * For example, if valueScale=0.1 and a certain coordinate pixel value is pixelValue=10000, then the depth value = pixelValue*valueScale = 10000*0.1=1000mm.
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return float The value scale of the depth frame
 */
OB_EXPORT float ob_depth_frame_get_value_scale(const ob_frame *frame, ob_error **error);

/**
 * @brief Set the value scale of the depth frame. The pixel value of the depth frame is multiplied by the scale to give a depth value in millimeters.
 * For example, if valueScale=0.1 and a certain coordinate pixel value is pixelValue=10000, then the depth value = pixelValue*valueScale = 10000*0.1=1000mm.
 *
 * @param[in] frame Frame object
 * @param[in] value_scale The value scale of the depth frame
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_depth_frame_set_value_scale(ob_frame *frame, float value_scale, ob_error **error);

/**
 * @brief Get the point coordinate value scale of the points frame. The point position value of the points frame is multiplied by the scale to give a position
 * value in millimeters. For example, if scale=0.1, the x-coordinate value of a point is x = 10000, which means that the actual x-coordinate value = x*scale =
 * 10000*0.1 = 1000mm.
 *
 * @param[in] frame Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return float The coordinate value scale of the points frame
 */
OB_EXPORT float ob_points_frame_get_coordinate_value_scale(const ob_frame *frame, ob_error **error);

/**
 * @brief Get accelerometer frame data.
 *
 * @param[in] frame Accelerometer frame.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_accel_value Return the accelerometer data.
 */
OB_EXPORT ob_accel_value ob_accel_frame_get_value(const ob_frame *frame, ob_error **error);

/**
 * @brief Get the temperature when acquiring the accelerometer frame.
 *
 * @param[in] frame Accelerometer frame.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return float Return the temperature value.
 */
OB_EXPORT float ob_accel_frame_get_temperature(const ob_frame *frame, ob_error **error);

/**
 * @brief Get gyroscope frame data.
 *
 * @param[in] frame Gyroscope frame.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_gyro_value Return the gyroscope data.
 */
OB_EXPORT ob_gyro_value ob_gyro_frame_get_value(const ob_frame *frame, ob_error **error);

/**
 * @brief Get the temperature when acquiring the gyroscope frame.
 *
 * @param[in] frame Gyroscope frame.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return float Return the temperature value.
 */
OB_EXPORT float ob_gyro_frame_get_temperature(const ob_frame *frame, ob_error **error);

/**
 * @brief Get the number of frames contained in the frameset
 *
 * @attention The frame returned by this function should call @ref ob_delete_frame() to decrease the reference count when it is no longer needed.
 *
 * @param[in] frameset frameset object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint32_t return the number of frames
 */
OB_EXPORT uint32_t ob_frameset_get_count(const ob_frame *frameset, ob_error **error);

/**
 * @brief Get the depth frame from the frameset.
 *
 * @attention The frame returned by this function should call @ref ob_delete_frame() to decrease the reference count when it is no longer needed.
 *
 * @param[in] frameset Frameset object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the depth frame.
 */
OB_EXPORT ob_frame *ob_frameset_get_depth_frame(const ob_frame *frameset, ob_error **error);

/**
 * @brief Get the color frame from the frameset.
 *
 * @attention The frame returned by this function should call @ref ob_delete_frame() to decrease the reference count when it is no longer needed.
 *
 * @param[in] frameset Frameset object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the color frame.
 */
OB_EXPORT ob_frame *ob_frameset_get_color_frame(const ob_frame *frameset, ob_error **error);

/**
 * @brief Get the infrared frame from the frameset.
 *
 * @attention The frame returned by this function should call @ref ob_delete_frame() to decrease the reference count when it is no longer needed.
 *
 * @param[in] frameset Frameset object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the infrared frame.
 */
OB_EXPORT ob_frame *ob_frameset_get_ir_frame(const ob_frame *frameset, ob_error **error);

/**
 * @brief Get point cloud frame from the frameset.
 *
 * @attention The frame returned by this function should call @ref ob_delete_frame() to decrease the reference count when it is no longer needed.
 *
 * @param[in] frameset Frameset object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the point cloud frame.
 */
OB_EXPORT ob_frame *ob_frameset_get_points_frame(const ob_frame *frameset, ob_error **error);

/**
 * @brief Get a frame of a specific type from the frameset.
 *
 * @attention The frame returned by this function should call @ref ob_delete_frame() to decrease the reference count when it is no longer needed.
 *
 * @param[in] frameset Frameset object.
 * @param[in] frame_type Frame type.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the frame of the specified type, or nullptr if it does not exist.
 */
OB_EXPORT ob_frame *ob_frameset_get_frame(const ob_frame *frameset, ob_frame_type frame_type, ob_error **error);

/**
 * @brief Get a frame at a specific index from the FrameSet
 *
 * @param[in] frameset Frameset object.
 * @param[in] index The index of the frame.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_frame* Return the frame at the specified index, or nullptr if it does not exist.
 */
OB_EXPORT ob_frame *ob_frameset_get_frame_by_index(const ob_frame *frameset, uint32_t index, ob_error **error);

/**
 * @brief Push a frame to the frameset
 *
 * @attention If a frame with same type already exists in the frameset, it will be replaced by the new frame.
 * @attention The frame push to the frameset will be add reference count, so you still need to call @ref ob_delete_frame() to decrease the reference count when
 * it is no longer needed.
 *
 * @param[in] frameset Frameset object.
 * @param[in] frame Frame object to push.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_frameset_push_frame(ob_frame *frameset, const ob_frame *frame, ob_error **error);

/**
 * @brief Get point cloud frame width
 *
 * @param[in] frame point cloud Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint32_t return the point cloud frame width
 */
OB_EXPORT uint32_t ob_point_cloud_frame_get_width(const ob_frame *frame, ob_error **error);

/**
 * @brief Get point cloud frame height
 *
 * @param[in] frame point cloud Frame object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return uint32_t return the point cloud frame height
 */
OB_EXPORT uint32_t ob_point_cloud_frame_get_height(const ob_frame *frame, ob_error **error);

// The following interfaces are deprecated and are retained here for compatibility purposes.
#define ob_frame_index ob_frame_get_index
#define ob_frame_format ob_frame_get_format
#define ob_frame_time_stamp_us ob_frame_get_timestamp_us
#define ob_frame_set_device_time_stamp_us ob_frame_set_timestamp_us
#define ob_frame_system_time_stamp_us ob_frame_get_system_timestamp_us
#define ob_frame_global_time_stamp_us ob_frame_get_global_timestamp_us
#define ob_frame_data ob_frame_get_data
#define ob_frame_data_size ob_frame_get_data_size
#define ob_frame_metadata ob_frame_get_metadata
#define ob_frame_metadata_size ob_frame_get_metadata_size
#define ob_video_frame_width ob_video_frame_get_width
#define ob_video_frame_height ob_video_frame_get_height
#define ob_video_frame_pixel_available_bit_size ob_video_frame_get_pixel_available_bit_size
#define ob_points_frame_get_position_value_scale ob_points_frame_get_coordinate_value_scale
#define ob_frameset_frame_count ob_frameset_get_count
#define ob_frameset_depth_frame ob_frameset_get_depth_frame
#define ob_frameset_color_frame ob_frameset_get_color_frame
#define ob_frameset_ir_frame ob_frameset_get_ir_frame
#define ob_frameset_points_frame ob_frameset_get_points_frame
#define ob_accel_frame_value ob_accel_frame_get_value
#define ob_accel_frame_temperature ob_accel_frame_get_temperature
#define ob_gyro_frame_value ob_gyro_frame_get_value
#define ob_gyro_frame_temperature ob_gyro_frame_get_temperature
#define ob_frameset_get_frame_count ob_frameset_get_count

#define ob_frame_time_stamp(frame, err) (ob_frame_get_timestamp_us(frame, err) / 1000)
#define ob_frame_system_time_stamp(frame, err) (ob_frame_get_system_timestamp_us(frame, err))
#define ob_frame_set_system_time_stamp(frame, system_timestamp, err) (ob_frame_set_system_timestamp_us(frame, system_timestamp * 1000, err))
#define ob_frame_set_device_time_stamp(frame, device_timestamp, err) (ob_frame_set_timestamp_us(frame, device_timestamp * 1000, err))

#ifdef __cplusplus
}
#endif
