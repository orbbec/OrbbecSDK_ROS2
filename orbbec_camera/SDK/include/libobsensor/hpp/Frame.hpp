// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Frame.hpp
 * @brief Frame related type, which is mainly used to obtain frame data and frame information.
 *
 */
#pragma once

#include "Types.hpp"
#include "libobsensor/h/Frame.h"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"
#include <memory>
#include <iostream>
#include <typeinfo>
#include <functional>

/**
 *  Frame classis inheritance hierarchy:
 *         Frame
 *          |
 *      +-----------+----------+----------+-----------+
 *      |           |          |          |           |
 *   VideoFrame PointsFrame AccelFrame GyroFrame   FrameSet
 *        |
 *   +----+----------+-------------------+
 *   |               |                   |
 * ColorFrame    DepthFrame           IRFrame
 *                                       |
 *                                 +-----+-----+
 *                                 |           |
 *                            IRLeftFrame  IRRightFrame
 */

namespace ob {
class Device;
class Sensor;

/**
 * @brief Define the frame class, which is the base class of all frame types.
 *
 */
class Frame : public std::enable_shared_from_this<Frame> {
protected:
    /**
     * @brief The pointer to the internal (c api level) frame object.
     */
    const ob_frame *impl_ = nullptr;

public:
    /**
     * @brief Construct a new Frame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit Frame(const ob_frame *impl) : impl_(impl) {}

    /**
     * @brief Get the internal (impl) frame object
     *
     * @return const ob_frame* the pointer to the internal frame object.
     */
    const ob_frame *getImpl() const {
        return impl_;
    }

    /**
     * @brief Destroy the Frame object
     */
    virtual ~Frame() noexcept {
        if(impl_) {
            ob_error *error = nullptr;
            ob_delete_frame(impl_, &error);
            Error::handle(&error, false);
            impl_ = nullptr;
        }
    }

    /**
     * @brief Get the type of frame.
     *
     * @return OBFrameType The type of frame.
     */
    virtual OBFrameType getType() const {
        ob_error *error = nullptr;
        auto      type  = ob_frame_get_type(impl_, &error);
        Error::handle(&error);

        return type;
    }

    /**
     * @brief Get the format of the frame.
     *
     * @return OBFormat The format of the frame.
     */
    virtual OBFormat getFormat() const {
        ob_error *error  = nullptr;
        auto      format = ob_frame_get_format(impl_, &error);
        Error::handle(&error);

        return format;
    }

    /**
     * @brief Get the sequence number of the frame.
     *
     * @note The sequence number for each frame is managed by the SDK. It increments by 1 for each frame on each stream.
     *
     * @return uint64_t The sequence number of the frame.
     */
    virtual uint64_t getIndex() const {
        ob_error *error = nullptr;
        auto      index = ob_frame_get_index(impl_, &error);
        Error::handle(&error);

        return index;
    }

    /**
     * @brief Get frame data
     *
     * @return const uint8_t * The frame data pointer.
     */
    virtual uint8_t *getData() const {
        ob_error *error = nullptr;
        auto      data  = ob_frame_get_data(impl_, &error);
        Error::handle(&error);

        return data;
    }

    /**
     * @brief Get the size of the frame data.
     *
     * @return uint32_t The size of the frame data.
     * For point cloud data, this returns the number of bytes occupied by all point sets. To find the number of points, divide the dataSize by the structure
     * size of the corresponding point type.
     */
    virtual uint32_t getDataSize() const {
        ob_error *error    = nullptr;
        auto      dataSize = ob_frame_get_data_size(impl_, &error);
        Error::handle(&error);

        return dataSize;
    }

    /**
     * @brief Get the hardware timestamp of the frame in microseconds.
     * @brief The hardware timestamp is the time point when the frame was captured by the device, on device clock domain.
     *
     * @return uint64_t The hardware timestamp of the frame in microseconds.
     */
    uint64_t getTimeStampUs() const {
        ob_error *error       = nullptr;
        auto      timeStampUs = ob_frame_get_timestamp_us(impl_, &error);
        Error::handle(&error);

        return timeStampUs;
    }

    /**
     * @brief Get the system timestamp of the frame in microseconds.
     * @brief The system timestamp is the time point when the frame was received by the host, on host clock domain.
     *
     * @return uint64_t The system timestamp of the frame in microseconds.
     */
    uint64_t getSystemTimeStampUs() const {
        ob_error *error             = nullptr;
        auto      systemTimeStampUs = ob_frame_get_system_timestamp_us(impl_, &error);
        Error::handle(&error);

        return systemTimeStampUs;
    }

    /**
     * @brief Get the global timestamp of the frame in microseconds.
     * @brief The global timestamp is the time point when the frame was captured by the device, and has been converted to the host clock domain. The
     * conversion process base on the device timestamp and can eliminate the timer drift of the device
     *
     * @attention The global timestamp disable by default. If global timestamp is not enabled, the function will return 0. To enable the global timestamp,
     * please call @ref Device::enableGlobalTimestamp() function.
     * @attention Only some devices support getting the global timestamp. Check the device support status by @ref Device::isGlobalTimestampSupported() function.
     *
     * @return uint64_t The global timestamp of the frame in microseconds.
     */
    uint64_t getGlobalTimeStampUs() const {
        ob_error *error             = nullptr;
        auto      globalTimeStampUs = ob_frame_get_global_timestamp_us(impl_, &error);
        Error::handle(&error);

        return globalTimeStampUs;
    }

    /**
     * @brief Get the metadata pointer of the frame.
     *
     * @return const uint8_t * The metadata pointer of the frame.
     */
    uint8_t *getMetadata() const {
        ob_error *error    = nullptr;
        auto      metadata = ob_frame_get_metadata(impl_, &error);
        Error::handle(&error);

        return metadata;
    }

    /**
     * @brief Get the size of the metadata of the frame.
     *
     * @return uint32_t The size of the metadata of the frame.
     */
    uint32_t getMetadataSize() const {
        ob_error *error        = nullptr;
        auto      metadataSize = ob_frame_get_metadata_size(impl_, &error);
        Error::handle(&error);

        return metadataSize;
    }

    /**
     * @brief Check if the frame object has metadata of a given type.
     *
     * @param type The metadata type. refer to @ref OBFrameMetadataType
     * @return bool The result.
     */
    bool hasMetadata(OBFrameMetadataType type) const {
        ob_error *error  = nullptr;
        auto      result = ob_frame_has_metadata(impl_, type, &error);
        Error::handle(&error);

        return result;
    }

    /**
     * @brief Get the metadata value
     *
     * @param type The metadata type. refer to @ref OBFrameMetadataType
     * @return int64_t The metadata value.
     */
    int64_t getMetadataValue(OBFrameMetadataType type) const {
        ob_error *error = nullptr;
        auto      value = ob_frame_get_metadata_value(impl_, type, &error);
        Error::handle(&error);

        return value;
    }

    /**
     * @brief get StreamProfile of the frame
     *
     * @return std::shared_ptr<StreamProfile> The StreamProfile of the frame, may return nullptr if the frame is not captured from a stream.
     */
    std::shared_ptr<StreamProfile> getStreamProfile() const {
        ob_error *error   = nullptr;
        auto      profile = ob_frame_get_stream_profile(impl_, &error);
        Error::handle(&error);
        return StreamProfileFactory::create(profile);
    }

    /**
     * @brief get owner sensor of the frame
     *
     * @return std::shared_ptr<Sensor> The owner sensor of the frame, return nullptr if the frame is not owned by any sensor or the sensor is destroyed
     */
    std::shared_ptr<Sensor> getSensor() const {
        ob_error *error  = nullptr;
        auto      sensor = ob_frame_get_sensor(impl_, &error);
        Error::handle(&error);

        return std::make_shared<Sensor>(sensor);
    }

    /**
     * @brief get owner device of the frame
     *
     * @return std::shared_ptr<Device> The owner device of the frame, return nullptr if the frame is not owned by any device or the device is destroyed
     */
    std::shared_ptr<Device> getDevice() const {
        ob_error *error  = nullptr;
        auto      device = ob_frame_get_device(impl_, &error);
        Error::handle(&error);

        return std::make_shared<Device>(device);
    }

    /**
     * @brief Check if the runtime type of the frame object is compatible with a given type.
     *
     * @tparam T The given type.
     * @return bool The result.
     */
    template <typename T> bool is() const;

    /**
     * @brief Convert the frame object to a target type.
     *
     * @tparam T The target type.
     * @return std::shared_ptr<T> The result. If it cannot be converted, an exception will be thrown.
     */
    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>()) {
            throw std::runtime_error("unsupported operation, object's type is not require type");
        }

        ob_error *error = nullptr;
        ob_frame_add_ref(impl_, &error);
        Error::handle(&error);

        return std::make_shared<T>(impl_);
    }

    /**
     * @brief Convert the frame object to a target type.
     *
     * @tparam T The target type.
     * @return std::shared_ptr<T> The result. If it cannot be converted, an exception will be thrown.
     */
    template <typename T> std::shared_ptr<const T> as() const {
        if(!is<const T>()) {
            throw std::runtime_error("unsupported operation, object's type is not require type");
        }

        ob_error *error = nullptr;
        ob_frame_add_ref(impl_, &error);
        Error::handle(&error);

        return std::make_shared<const T>(impl_);
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    OBFrameType type() const {
        return getType();
    }

    virtual OBFormat format() const {
        return getFormat();
    }

    virtual uint64_t index() const {
        return getIndex();
    }

    virtual void *data() const {
        auto data = getData();
        return reinterpret_cast<void *>(data);
    }

    virtual uint32_t dataSize() const {
        return getDataSize();
    }

    uint64_t timeStamp() const {
        return getTimeStampUs() / 1000;
    }

    uint64_t timeStampUs() const {
        return getTimeStampUs();
    }

    uint64_t systemTimeStamp() const {
        return getSystemTimeStampUs() / 1000;
    }

    uint64_t systemTimeStampUs() const {
        return getSystemTimeStampUs();
    }

    uint64_t globalTimeStampUs() const {
        return getGlobalTimeStampUs();
    }

    uint8_t *metadata() const {
        return getMetadata();
    }

    uint32_t metadataSize() const {
        return getMetadataSize();
    }
};

/**
 * @brief Define the VideoFrame class, which inherits from the Frame class
 */
class VideoFrame : public Frame {
public:
    /**
     * @brief Construct a new VideoFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit VideoFrame(const ob_frame *impl) : Frame(impl) {};

    ~VideoFrame() noexcept override = default;

    /**
     * @brief Get the width of the frame.
     *
     * @return uint32_t The width of the frame.
     */
    uint32_t getWidth() const {
        ob_error *error = nullptr;
        auto      width = ob_video_frame_get_width(impl_, &error);
        Error::handle(&error);

        return width;
    }

    /**
     * @brief Get the height of the frame.
     *
     * @return uint32_t The height of the frame.
     */
    uint32_t getHeight() const {
        ob_error *error  = nullptr;
        auto      height = ob_video_frame_get_height(impl_, &error);
        Error::handle(&error);

        return height;
    }

    /**
     * @brief Get the Pixel Type object
     * @brief Usually used to determine the pixel type of depth frame (depth, disparity, raw phase, etc.)
     *
     * @attention Always return OB_PIXEL_UNKNOWN for non-depth frame currently
     *
     * @return OBPixelType
     */
    OBPixelType getPixelType() const {
        ob_error *error     = nullptr;
        auto      pixelType = ob_video_frame_get_pixel_type(impl_, &error);
        Error::handle(&error);

        return pixelType;
    }

    /**
     * @brief Get the effective number of pixels in the frame.
     * @attention Only valid for Y8/Y10/Y11/Y12/Y14/Y16 format.
     *
     * @return uint8_t The effective number of pixels in the frame, or 0 if it is an unsupported format.
     */
    uint8_t getPixelAvailableBitSize() const {
        ob_error *error   = nullptr;
        auto      bitSize = ob_video_frame_get_pixel_available_bit_size(impl_, &error);
        Error::handle(&error);

        return bitSize;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    uint32_t width() const {
        return getWidth();
    }

    uint32_t height() const {
        return getHeight();
    }

    uint8_t pixelAvailableBitSize() const {
        return getPixelAvailableBitSize();
    }
};

/**
 * @brief Define the ColorFrame class, which inherits from the VideoFrame classd
 */
class ColorFrame : public VideoFrame {
public:
    /**
     * @brief Construct a new ColorFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit ColorFrame(const ob_frame *impl) : VideoFrame(impl) {};

    ~ColorFrame() noexcept override = default;
};

/**
 * @brief Define the DepthFrame class, which inherits from the VideoFrame class
 */
class DepthFrame : public VideoFrame {

public:
    /**
     * @brief Construct a new DepthFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit DepthFrame(const ob_frame *impl) : VideoFrame(impl) {};

    ~DepthFrame() noexcept override = default;

    /**
     * @brief Get the value scale of the depth frame. The pixel value of depth frame is multiplied by the scale to give a depth value in millimeters.
     *   For example, if valueScale=0.1 and a certain coordinate pixel value is pixelValue=10000, then the depth value = pixelValue*valueScale =
     *   10000*0.1=1000mm.
     *
     * @return float The scale.
     */
    float getValueScale() const {
        ob_error *error = nullptr;
        auto      scale = ob_depth_frame_get_value_scale(impl_, &error);
        Error::handle(&error);

        return scale;
    }
};

/**
 * @brief Define the IRFrame class, which inherits from the VideoFrame class
 *
 */
class IRFrame : public VideoFrame {

public:
    /**
     * @brief Construct a new IRFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit IRFrame(const ob_frame *impl) : VideoFrame(impl) {};

    ~IRFrame() noexcept override = default;
};

/**
 * @brief Define the ConfidenceFrame class, which inherits from the VideoFrame class
 *
 */
class ConfidenceFrame : public VideoFrame {

public:
    /**
     * @brief Construct a new ConfidenceFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit ConfidenceFrame(const ob_frame *impl) : VideoFrame(impl) {};

    ~ConfidenceFrame() noexcept override = default;
};

/**
 * @brief Define the PointsFrame class, which inherits from the Frame class
 * @brief The PointsFrame class is used to obtain pointcloud data and point cloud information.
 *
 * @note The pointcloud data format can be obtained from the @ref Frame::getFormat() function. Witch can be one of the following formats:
 * - @ref OB_FORMAT_POINT: 32-bit float format with 3D point coordinates (x, y, z), @ref OBPoint
 * - @ref OB_FORMAT_RGB_POINT: 32-bit float format with 3D point coordinates (x, y, z) and point colors (r, g, b) @ref, OBColorPoint
 */
class PointsFrame : public Frame {

public:
    /**
     * @brief Construct a new PointsFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit PointsFrame(const ob_frame *impl) : Frame(impl) {};

    ~PointsFrame() noexcept override = default;

    /**
     * @brief Get the point coordinate value scale of the points frame. The point position value of the points frame is multiplied by the scale to give a
     * position value in millimeters. For example, if scale=0.1, the x-coordinate value of a point is x = 10000, which means that the actual x-coordinate value
     * = x*scale = 10000*0.1 = 1000mm.
     *
     * @return float The coordinate value scale.
     */
    float getCoordinateValueScale() const {
        ob_error *error = nullptr;
        auto      scale = ob_points_frame_get_coordinate_value_scale(impl_, &error);
        Error::handle(&error);

        return scale;
    }

    /**
     * @brief Get the width of the frame.
     *
     * @return uint32_t The width of the frame.
     */
    uint32_t getWidth() const {
        ob_error *error = nullptr;
        // TODO
        auto width = ob_point_cloud_frame_get_width(impl_, &error);
        Error::handle(&error);

        return width;
    }

    /**
     * @brief Get the height of the frame.
     *
     * @return uint32_t The height of the frame.
     */
    uint32_t getHeight() const {
        ob_error *error  = nullptr;
        auto      height = ob_point_cloud_frame_get_height(impl_, &error);
        Error::handle(&error);

        return height;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
#define getPositionValueScale getCoordinateValueScale
};

/**
 * @brief Define the AccelFrame class, which inherits from the Frame class
 *
 */
class AccelFrame : public Frame {

public:
    explicit AccelFrame(const ob_frame *impl) : Frame(impl) {};

    ~AccelFrame() noexcept override = default;

    /**
     * @brief Get the accelerometer frame data
     *
     * @return OBAccelValue The accelerometer frame data
     */
    OBAccelValue getValue() const {
        ob_error *error = nullptr;
        auto      value = ob_accel_frame_get_value(impl_, &error);
        Error::handle(&error);

        return value;
    }

    /**
     * @brief Get the temperature when the frame was sampled
     *
     * @return float The temperature value in celsius
     */
    float getTemperature() const {
        ob_error *error = nullptr;
        auto      temp  = ob_accel_frame_get_temperature(impl_, &error);
        Error::handle(&error);

        return temp;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    OBAccelValue value() {
        return getValue();
    }

    float temperature() {
        return getTemperature();
    }
};

/**
 * @brief Define the GyroFrame class, which inherits from the Frame class
 */
class GyroFrame : public Frame {

public:
    explicit GyroFrame(const ob_frame *impl) : Frame(impl) {};

    ~GyroFrame() noexcept override = default;

    /**
     * @brief Get the gyro frame data
     *
     * @return OBAccelValue The gyro frame data
     */
    OBGyroValue getValue() const {
        ob_error *error = nullptr;
        auto      value = ob_gyro_frame_get_value(impl_, &error);
        Error::handle(&error);

        return value;
    }

    /**
     * @brief Get the temperature when the frame was sampled
     *
     * @return float The temperature value in celsius
     */
    float getTemperature() const {
        ob_error *error       = nullptr;
        auto      temperature = ob_gyro_frame_get_temperature(impl_, &error);
        Error::handle(&error);

        return temperature;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    OBGyroValue value() {
        return getValue();
    }

    float temperature() {
        return getTemperature();
    }
};

/**
 * @brief Define the FrameSet class, which inherits from the Frame class
 * @brief A FrameSet is a container for multiple frames of different types.
 */
class FrameSet : public Frame {

public:
    explicit FrameSet(const ob_frame *impl) : Frame(impl) {};

    ~FrameSet() noexcept override = default;

    /**
     * @brief Get the number of frames in the FrameSet
     *
     * @return uint32_t The number of frames
     */
    uint32_t getCount() const {
        ob_error *error = nullptr;
        auto      count = ob_frameset_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get a frame of a specific type from the FrameSet
     *
     * @param frameType The type of sensor
     * @return std::shared_ptr<Frame> The corresponding type of frame
     */
    std::shared_ptr<Frame> getFrame(OBFrameType frameType) const {
        ob_error *error = nullptr;
        auto      frame = ob_frameset_get_frame(impl_, frameType, &error);
        if(!frame) {
            return nullptr;
        }
        Error::handle(&error);
        return std::make_shared<Frame>(frame);
    }

    /**
     * @brief Get a frame at a specific index from the FrameSet
     *
     * @param index The index of the frame
     * @return std::shared_ptr<Frame> The frame at the specified index
     */
    std::shared_ptr<Frame> getFrameByIndex(uint32_t index) const {
        ob_error *error = nullptr;
        auto      frame = ob_frameset_get_frame_by_index(impl_, index, &error);
        if(!frame) {
            return nullptr;
        }
        Error::handle(&error);
        return std::make_shared<Frame>(frame);
    }

    /**
     * @brief Push a frame to the FrameSet
     *
     * @attention If the FrameSet contains the same type of frame, the new frame will replace the old one.
     *
     * @param frame The frame to be pushed
     */
    void pushFrame(std::shared_ptr<const Frame> frame) const {
        ob_error *error = nullptr;

        // unsafe operation, need to cast const to non-const
        auto unConstImpl = const_cast<ob_frame *>(impl_);

        auto otherImpl = frame->getImpl();
        ob_frameset_push_frame(unConstImpl, otherImpl, &error);

        Error::handle(&error);
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    uint32_t frameCount() const {
        return getCount();
    }

    std::shared_ptr<DepthFrame> depthFrame() const {
        auto frame = getFrame(OB_FRAME_DEPTH);
        if(frame == nullptr) {
            return nullptr;
        }
        auto depthFrame = frame->as<ob::DepthFrame>();
        return depthFrame;
    }

    std::shared_ptr<ColorFrame> colorFrame() const {
        auto frame = getFrame(OB_FRAME_COLOR);
        if(frame == nullptr) {
            return nullptr;
        }
        auto colorFrame = frame->as<ob::ColorFrame>();
        return colorFrame;
    }

    std::shared_ptr<IRFrame> irFrame() const {
        auto frame = getFrame(OB_FRAME_IR);
        if(frame == nullptr) {
            return nullptr;
        }
        auto irFrame = frame->as<ob::IRFrame>();
        return irFrame;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    std::shared_ptr<PointsFrame> pointsFrame() const {
        auto frame = getFrame(OB_FRAME_POINTS);
        if(frame == nullptr) {
            return nullptr;
        }
        auto pointsFrame = frame->as<ob::PointsFrame>();
        return pointsFrame;
    }

    std::shared_ptr<Frame> getFrame(int index) const {
        return getFrameByIndex(index);
    }
};

/**
 * @brief FrameFactory class, which provides some static functions to create frame objects
 */
class FrameFactory {
public:
    /**
     * @brief Create a Frame object of a specific type with a given format and data size.
     *
     * @param frameType The type of the frame.
     * @param format The format of the frame.
     * @param dataSize The size of the data in bytes.
     * @return std::shared_ptr<Frame> The created frame object.
     */
    static std::shared_ptr<Frame> createFrame(OBFrameType frameType, OBFormat format, uint32_t dataSize) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_frame(frameType, format, dataSize, &error);
        Error::handle(&error);

        return std::make_shared<Frame>(impl);
    }

    /**
     * @brief Create a VideoFrame object of a specific type with a given format, width, height, and stride.
     * @note If stride is not specified, it will be calculated based on the width and format.
     *
     * @param frameType The type of the frame.
     * @param format The format of the frame.
     * @param width The width of the frame.
     * @param height The height of the frame.
     * @param stride The stride of the frame.
     *
     * @return std::shared_ptr<VideoFrame> The created video frame object.
     */
    static std::shared_ptr<VideoFrame> createVideoFrame(OBFrameType frameType, OBFormat format, uint32_t width, uint32_t height, uint32_t stride = 0) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_video_frame(frameType, format, width, height, stride, &error);
        Error::handle(&error);

        auto frame = std::make_shared<Frame>(impl);
        return frame->as<VideoFrame>();
    }

    /**
     * @brief Create (clone) a frame object based on the specified other frame object.
     * @brief The new frame object will have the same properties as the other frame object, but the data buffer is newly allocated.
     *
     * @param shouldCopyData If true, the data of the source frame object will be copied to the new frame object. If false, the new frame object will
     * have a data buffer with random data. The default value is true.
     *
     * @return std::shared_ptr<Frame> The new frame object.
     */
    static std::shared_ptr<Frame> createFrameFromOtherFrame(std::shared_ptr<const Frame> otherFrame, bool shouldCopyData = true) {
        ob_error *error     = nullptr;
        auto      otherImpl = otherFrame->getImpl();
        auto      impl      = ob_create_frame_from_other_frame(otherImpl, shouldCopyData, &error);
        Error::handle(&error);

        return std::make_shared<Frame>(impl);
    }

    /**
     * @brief Create a Frame From (according to)Stream Profile object
     *
     * @param profile The stream profile object to create the frame from.
     *
     * @return std::shared_ptr<Frame>  The created frame object.
     */
    static std::shared_ptr<Frame> createFrameFromStreamProfile(std::shared_ptr<const StreamProfile> profile) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_frame_from_stream_profile(profile->getImpl(), &error);
        Error::handle(&error);

        return std::make_shared<Frame>(impl);
    }

    /**
     * @brief The callback function to destroy the buffer when the frame is destroyed.
     */
    typedef std::function<void(uint8_t *)> BufferDestroyCallback;

    /**
     * @brief Create a frame object based on an externally created buffer.
     *
     * @attention The buffer is owned by the caller, and will not be destroyed by the frame object. The user should ensure that the buffer is valid and not
     * modified.
     *
     * @param[in] frameType Frame object type.
     * @param[in] format Frame object format.
     * @param[in] buffer Frame object buffer.
     * @param[in] destroyCallback Destroy callback, will be called when the frame object is destroyed.
     * @param[in] bufferSize Frame object buffer size.
     *
     * @return std::shared_ptr<Frame> The created frame object.
     */
    static std::shared_ptr<Frame> createFrameFromBuffer(OBFrameType frameType, OBFormat format, uint8_t *buffer, BufferDestroyCallback destroyCallback,
                                                        uint32_t bufferSize) {
        ob_error *error = nullptr;
        auto      ctx   = new BufferDestroyContext{ destroyCallback };
        auto      impl  = ob_create_frame_from_buffer(frameType, format, buffer, bufferSize, &FrameFactory::BufferDestroy, ctx, &error);
        Error::handle(&error);

        return std::make_shared<Frame>(impl);
    }

    /**
     * @brief Create a video frame object based on an externally created buffer.
     *
     * @attention The buffer is owned by the user and will not be destroyed by the frame object. The user should ensure that the buffer is valid and not
     * modified.
     * @attention The frame object is created with a reference count of 1, and the reference count should be decreased by calling @ref ob_delete_frame() when it
     * is no longer needed.
     *
     * @param[in] frameType Frame object type.
     * @param[in] format Frame object format.
     * @param[in] width Frame object width.
     * @param[in] height Frame object height.
     * @param[in] buffer Frame object buffer.
     * @param[in] bufferSize Frame object buffer size.
     * @param[in] destroyCallback Destroy callback, will be called when the frame object is destroyed.
     * @param[in] stride Row span in bytes. If 0, the stride is calculated based on the width and format.
     *
     * @return std::shared_ptr<VideoFrame> The created video frame object.
     */
    static std::shared_ptr<VideoFrame> createVideoFrameFromBuffer(OBFrameType frameType, OBFormat format, uint32_t width, uint32_t height, uint8_t *buffer,
                                                                  BufferDestroyCallback destroyCallback, uint32_t bufferSize, uint32_t stride = 0) {
        ob_error *error = nullptr;
        auto      ctx   = new BufferDestroyContext{ destroyCallback };
        auto impl = ob_create_video_frame_from_buffer(frameType, format, width, height, stride, buffer, bufferSize, &FrameFactory::BufferDestroy, ctx, &error);
        Error::handle(&error);

        auto frame = std::make_shared<Frame>(impl);
        return frame->as<VideoFrame>();
    }

    /**
     * @brief Create a new FrameSet object.
     *
     * This function creates a new FrameSet instance by internally calling the native C API.
     * The returned FrameSet is managed by a std::shared_ptr, and its lifetime will be
     * automatically managed. When no references remain, the underlying native resources will be released.
     *
     * @return std::shared_ptr<FrameSet> The created FrameSet object.
     */
    static std::shared_ptr<FrameSet> createFrameSet() {
        ob_error *error = nullptr;
        auto      impl  = ob_create_frameset(&error);
        Error::handle(&error);
        return std::make_shared<FrameSet>(impl);
    }

private:
    struct BufferDestroyContext {
        BufferDestroyCallback callback;
    };

    static void BufferDestroy(uint8_t *buffer, void *context) {
        auto *ctx = static_cast<BufferDestroyContext *>(context);
        if(ctx->callback) {
            ctx->callback(buffer);
        }
        delete ctx;
    }
};

/**
 * @brief FrameHepler class, which provides some static functions to set timestamp for frame objects
 * FrameHepler inherited from the FrameFactory and the timestamp interface implement here both for compatibility purposes.
 */
class FrameHelper : public FrameFactory {
public:
    /**
     * @brief Set the device timestamp of the frame.
     *
     * @param frame The frame object.
     * @param deviceTimestampUs The device timestamp to set in microseconds.
     */
    static void setFrameDeviceTimestampUs(std::shared_ptr<Frame> frame, uint64_t deviceTimestampUs) {
        ob_error *error = nullptr;
        auto      impl  = const_cast<ob_frame *>(frame->getImpl());
        ob_frame_set_timestamp_us(impl, deviceTimestampUs, &error);
        Error::handle(&error);
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    static void setFrameSystemTimestamp(std::shared_ptr<Frame> frame, uint64_t systemTimestamp) {
        // In order to compile, some high-version compilers will warn that the function parameters are not used.
        (void)frame;
        (void)systemTimestamp;
    }

    static void setFrameDeviceTimestamp(std::shared_ptr<Frame> frame, uint64_t deviceTimestamp) {
        // In order to compile, some high-version compilers will warn that the function parameters are not used.
        (void)frame;
        (void)deviceTimestamp;
    }
};

// Define the is() template function for the Frame class
template <typename T> bool Frame::is() const {
    switch(this->getType()) {
    case OB_FRAME_IR_LEFT:   // Follow
    case OB_FRAME_IR_RIGHT:  // Follow
    case OB_FRAME_IR:
        return (typeid(T) == typeid(IRFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_DEPTH:
        return (typeid(T) == typeid(DepthFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_COLOR:
        return (typeid(T) == typeid(ColorFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_CONFIDENCE:
        return (typeid(T) == typeid(ConfidenceFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_GYRO:
        return (typeid(T) == typeid(GyroFrame));
    case OB_FRAME_ACCEL:
        return (typeid(T) == typeid(AccelFrame));
    case OB_FRAME_POINTS:
        return (typeid(T) == typeid(PointsFrame));
    case OB_FRAME_SET:
        return (typeid(T) == typeid(FrameSet));
    default:
        std::cout << "ob::Frame::is() did not catch frame type: " << (int)this->getType() << std::endl;
        break;
    }
    return false;
}

}  // namespace ob
