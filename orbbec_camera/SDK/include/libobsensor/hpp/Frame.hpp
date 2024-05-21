/**
 * @file Frame.hpp
 * @brief Frame related type, which is mainly used to obtain frame data and frame information.
 *
 */
#pragma once

#include "Types.hpp"

#include <memory>
#include <iostream>
#include <typeinfo>

/**
 *    Frame class：
 *                 Frame
 *                   |
 *            +------+----------+----------+-----------+
 *            |      |          |         |            |
 *      VideoFrame PointsFrame AccelFrame GyroFrame FrameSet
 *          |
 *       +--+------+---------+
 *      |          |         |
 *  ColorFrame DepthFrame IRFrame
 *                           |
 *                     +-----+-----+
 *                     |           |
 *                IRLeftFrame IRRightFrame
 */

struct FrameImpl;

namespace ob {
class Device;
class Sensor;
class StreamProfile;
class Filter;
class FrameHelper;

typedef std::function<void(void *buffer, void *context)> BufferDestroyCallback;

class OB_EXTENSION_API Frame : public std::enable_shared_from_this<Frame> {
protected:
    std::unique_ptr<FrameImpl> impl_;

public:
    explicit Frame(std::unique_ptr<FrameImpl> impl);
    Frame(Frame &frame);

    virtual ~Frame() noexcept;

    /**
     * @brief Get the type of frame.
     *
     * @return OBFrameType The type of frame.
     */
    virtual OBFrameType type();

    /**
     * @brief Get the format of the frame.
     *
     * @return OBFormat The format of the frame.
     */
    virtual OBFormat format();

    /**
     * @brief Get the sequence number of the frame.
     *
     * @return uint64_t The sequence number of the frame.
     */
    virtual uint64_t index();

    /**
     * @brief Get the frame data.
     *
     * @return void* The frame data.
     */
    virtual void *data();

    /**
     * @brief Get the size of the frame data.
     *
     * @return uint32_t The size of the frame data.
     * For point cloud data, this returns the number of bytes occupied by all point sets. To find the number of points, divide the dataSize by the structure
     * size of the corresponding point type.
     */
    virtual uint32_t dataSize();

    /**
     * @brief Get the hardware timestamp of the frame in milliseconds.
     * @brief The hardware timestamp is the time point when the frame was captured by the device, on device clock domain.
     *
     * @return uint64_t The hardware timestamp of the frame in milliseconds.
     */
    uint64_t timeStamp();

    /**
     * @brief Get the hardware timestamp of the frame in microseconds.
     * @brief The hardware timestamp is the time point when the frame was captured by the device, on device clock domain.
     *
     * @return uint64_t The hardware timestamp of the frame in microseconds.
     */
    uint64_t timeStampUs();

    /**
     * @brief Get the system timestamp of the frame in milliseconds.
     * @brief The system timestamp is the time point when the frame was received by the host, on host clock domain.
     *
     * @return uint64_t The system timestamp of the frame in milliseconds.
     */
    uint64_t systemTimeStamp();

    /**
     * @brief Get the system timestamp of the frame in microseconds.
     * @brief The system timestamp is the time point when the frame was received by the host, on host clock domain.
     *
     * @return uint64_t The system timestamp of the frame in microseconds.
     */
    uint64_t systemTimeStampUs();

    /**
     * @brief Get the global timestamp of the frame in microseconds.
     * @brief The global timestamp is the time point when the frame was was captured by the device, and has been converted to the host clock domain. The
     * conversion process base on the device timestamp and can eliminate the timer drift of the device
     *
     * @attention Only some devices support getting the global timestamp. If the device does not support it, this function will return 0. Check the device
     * support status by @ref Device::isGlobalTimestampSupported() function.
     *
     * @return uint64_t The global timestamp of the frame in microseconds.
     */
    uint64_t globalTimeStampUs();

    /**
     * @brief Get the metadata of the frame.
     *
     * @return void* The metadata of the frame.
     */
    void *metadata();

    /**
     * @brief Get the size of the metadata of the frame.
     *
     * @return uint32_t The size of the metadata of the frame.
     */
    uint32_t metadataSize();

    /**
     * @brief Check if the frame object has metadata of a given type.
     *
     * @param type The metadata type. refer to @ref OBFrameMetadataType
     * @return bool The result.
     */
    bool hasMetadata(OBFrameMetadataType type);

    /**
     * @brief Get the metadata value
     *
     * @param type The metadata type. refer to @ref OBFrameMetadataType
     * @return int64_t The metadata value.
     */
    int64_t getMetadataValue(OBFrameMetadataType type);

    /**
     * @brief get StreamProfile of the frame
     *
     * @return std::shared_ptr<StreamProfile> The StreamProfile of the frame, may return nullptr if the frame is not captured from a stream.
     */
    std::shared_ptr<StreamProfile> getStreamProfile();

    /**
     * @brief get owner sensor of the frame
     *
     * @return std::shared_ptr<Sensor> The owner sensor of the frame, return nullptr if the frame is not owned by any sensor or the sensor is destroyed
     */
    std::shared_ptr<Sensor> getSensor();

    /**
     * @brief get owner device of the frame
     *
     * @return std::shared_ptr<Device> The owner device of the frame, return nullptr if the frame is not owned by any device or the device is destroyed
     */
    std::shared_ptr<Device> getDevice();

    /**
     * @brief Check if the runtime type of the frame object is compatible with a given type.
     *
     * @tparam T The given type.
     * @return bool The result.
     */
    template <typename T> bool is();

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

        return std::dynamic_pointer_cast<T>(shared_from_this());
    }

private:
    friend class Filter;
    friend class Recorder;
    friend class FrameHelper;
    friend class CoordinateTransformHelper;
};

class OB_EXTENSION_API VideoFrame : public Frame {
public:
    explicit VideoFrame(Frame &frame);

    explicit VideoFrame(std::unique_ptr<FrameImpl> impl);

    ~VideoFrame() noexcept override = default;

    /**
     * @brief Get the width of the frame.
     *
     * @return uint32_t The width of the frame.
     */
    uint32_t width();

    /**
     * @brief Get the height of the frame.
     *
     * @return uint32_t The height of the frame.
     */
    uint32_t height();

    /**
     * @brief Get the effective number of pixels in the frame.
     * @attention Only valid for Y8/Y10/Y11/Y12/Y14/Y16 format.
     *
     * @return uint8_t The effective number of pixels in the frame, or 0 if it is an unsupported format.
     */
    uint8_t pixelAvailableBitSize();
};

class OB_EXTENSION_API ColorFrame : public VideoFrame {
public:
    explicit ColorFrame(Frame &frame);

    explicit ColorFrame(std::unique_ptr<FrameImpl> impl);

    ~ColorFrame() noexcept override = default;
};

class OB_EXTENSION_API DepthFrame : public VideoFrame {
public:
    explicit DepthFrame(Frame &frame);

    explicit DepthFrame(std::unique_ptr<FrameImpl> impl);

    ~DepthFrame() noexcept override = default;

    /**
     * @brief Get the value scale of the depth frame. The pixel value of depth frame is multiplied by the scale to give a depth value in millimeters.
     *      For example, if valueScale=0.1 and a certain coordinate pixel value is pixelValue=10000, then the depth value = pixelValue*valueScale =
     *      10000*0.1=1000mm.
     *
     * @return float The scale.
     */
    float getValueScale();
};

class OB_EXTENSION_API IRFrame : public VideoFrame {
public:
    explicit IRFrame(Frame &frame);

    explicit IRFrame(std::unique_ptr<FrameImpl> impl);

    ~IRFrame() noexcept override = default;

public:
    OBSensorType getDataSource();
};

class OB_EXTENSION_API PointsFrame : public Frame {
public:
    explicit PointsFrame(Frame &frame);

    explicit PointsFrame(std::unique_ptr<FrameImpl> impl);

    ~PointsFrame() noexcept override = default;

    /**
     * @brief Get the point position value scale of the points frame. The point position value of the points frame is multiplied by the scale to give a position
     * value in millimeters. For example, if scale=0.1, the x-coordinate value of a point is x = 10000, which means that the actual x-coordinate value = x*scale
     * = 10000*0.1 = 1000mm.
     *
     * @return float The position value scale.
     */
    float getPositionValueScale();
};

/**
 * @brief Define the FrameSet class, which inherits from the Frame class
 *
 */
class OB_EXTENSION_API FrameSet : public Frame {

public:
    explicit FrameSet(std::unique_ptr<FrameImpl> impl);

    explicit FrameSet(Frame &frame);

    ~FrameSet() noexcept override;

    /**
     * @brief Get the number of frames in the FrameSet
     *
     * @return uint32_t The number of frames
     */
    uint32_t frameCount();

    /**
     * @brief Get the depth frame in the FrameSet
     *
     * @return std::shared_ptr<DepthFrame> The depth frame
     */
    std::shared_ptr<DepthFrame> depthFrame();

    /**
     * @brief Get the color frame in the FrameSet
     *
     * @return std::shared_ptr<ColorFrame> The color frame
     */
    std::shared_ptr<ColorFrame> colorFrame();

    /**
     * @brief Get the infrared frame in the FrameSet
     *
     * @return std::shared_ptr<IRFrame> The infrared frame
     */
    std::shared_ptr<IRFrame> irFrame();

    /**
     * @brief Get the point cloud frame in the FrameSet
     *
     * @return  std::shared_ptr<PointsFrame> The point cloud data frame
     */
    std::shared_ptr<PointsFrame> pointsFrame();

    /**
     * @brief Get a frame of a specific type from the FrameSet
     *
     * @param frameType The type of sensor
     * @return std::shared_ptr<Frame> The corresponding type of frame
     */
    std::shared_ptr<Frame> getFrame(OBFrameType frameType);

    /**
     * @brief Get a frame at a specific index from the FrameSet
     *
     * @param index The index of the frame
     * @return std::shared_ptr<Frame> The frame at the specified index
     */
    std::shared_ptr<Frame> getFrame(int index);

    // Declare Pipeline and Filter classes as friends
    friend class Pipeline;
    friend class Filter;
};

/**
 * @brief Define the AccelFrame class, which inherits from the Frame class
 *
 */
class OB_EXTENSION_API AccelFrame : public Frame {
public:
    explicit AccelFrame(Frame &frame);

    explicit AccelFrame(std::unique_ptr<FrameImpl> impl);

    ~AccelFrame() noexcept override = default;

    /**
     * @brief Get the accelerometer frame data
     *
     * @return OBAccelValue The accelerometer frame data
     */
    OBAccelValue value();

    /**
     * @brief Get the temperature when the frame was sampled
     *
     * @return float The temperature value
     */
    float temperature();
};

/**
 * @brief Define the GyroFrame class, which inherits from the Frame class
 */
class OB_EXTENSION_API GyroFrame : public Frame {
public:
    explicit GyroFrame(Frame &frame);

    explicit GyroFrame(std::unique_ptr<FrameImpl> impl);

    ~GyroFrame() noexcept override = default;

    /**
     * @brief Get the gyro frame data
     *
     * @return OBAccelValue The gyro frame data
     */
    OBGyroValue value();

    /**
     * @brief Get the temperature when the frame was sampled
     *
     * @return float The temperature value
     */
    float temperature();
};

/**
 * @brief Define the RawPhaseFrame class, which inherits from the VideoFrame class
 */
class OB_EXTENSION_API RawPhaseFrame : public VideoFrame {
public:
    explicit RawPhaseFrame(Frame &frame);

    explicit RawPhaseFrame(std::unique_ptr<FrameImpl> impl);

    ~RawPhaseFrame() noexcept override = default;
};

/**
 * @brief Define the FrameHelper class
 */
class OB_EXTENSION_API FrameHelper {
public:
    /**
     * @brief Create a Frame object.
     *
     * @param[in] type The type of frame. See @ref OBFrameType.
     * @param[in] format The format of the frame. See @ref OBFormat.
     * @param[in] width The width of the frame.
     * @param[in] height The height of the frame.
     * @param[in] strideBytes The stride of the frame in bytes. If strideBytes > 0, the frame data size = height * strideBytes. If strideBytes = 0, the frame
     * datasize = height * width * pixelSize (pixelSize according to the format).
     *
     * @return std::shared_ptr<Frame> The created frame object.
     */
    static std::shared_ptr<Frame> createFrame(OBFrameType type, OBFormat format, uint32_t width, uint32_t height, uint32_t strideBytes);

    /**
     * @brief Create a frame object based on an externally created buffer
     *
     * @param[in] format The format of the frame. See @ref OBFormat.
     * @param[in] width The width of the frame.
     * @param[in] height The height of the frame.
     * @param[in] buffer The frame object buffer
     * @param[in] bufferSize The frame object buffer size
     * @param[in] destroyCallback The frame object buffer destroy callback
     * @param[in] destroyCallbackContext The frame object buffer destroy callback context
     *
     * @return std::shared_ptr<Frame> The created frame object
     */
    static std::shared_ptr<Frame> createFrameFromBuffer(OBFormat format, uint32_t width, uint32_t height, uint8_t *buffer, uint32_t bufferSize,
                                                        BufferDestroyCallback destroyCallback, void *destroyCallbackContext);

    /**
     * @brief Create an empty FrameSet object
     *
     * @return std::shared_ptr<Frame> The FrameSet object
     */
    static std::shared_ptr<FrameSet> createFrameSet();

    /**
     * @brief Add a frame of a specific type to the FrameSet
     *
     * @param frameSet The FrameSet object
     * @param frameType The type of frame to add
     * @param frame The frame object to add
     */
    static void pushFrame(std::shared_ptr<Frame> frameSet, OBFrameType frameType, std::shared_ptr<Frame> frame);

    /**
     * @brief Set the system timestamp of the frame.
     *
     * @param frame The frame object.
     * @param systemTimestamp The system timestamp to set in milliseconds.
     */
    static void setFrameSystemTimestamp(std::shared_ptr<Frame> frame, uint64_t systemTimestamp);

    /**
     * @brief Set the device timestamp of the frame.
     *
     * @param frame The frame object.
     * @param deviceTimestamp The device timestamp to set in milliseconds.
     */
    static void setFrameDeviceTimestamp(std::shared_ptr<Frame> frame, uint64_t deviceTimestamp);

    /**
     * @brief Set the device timestamp of the frame.
     *
     * @param frame The frame object.
     * @param deviceTimestampUs The device timestamp to set in microseconds.
     */
    static void setFrameDeviceTimestampUs(std::shared_ptr<Frame> frame, uint64_t deviceTimestampUs);
};

// Define the is() template function for the Frame class
template <typename T> bool Frame::is() {
    switch(this->type()) {
    case OB_FRAME_IR_LEFT:   // Follow
    case OB_FRAME_IR_RIGHT:  // Follow
    case OB_FRAME_IR:
        return (typeid(T) == typeid(IRFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_DEPTH:
        return (typeid(T) == typeid(DepthFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_COLOR:
        return (typeid(T) == typeid(ColorFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_GYRO:
        return (typeid(T) == typeid(GyroFrame));
    case OB_FRAME_ACCEL:
        return (typeid(T) == typeid(AccelFrame));
    case OB_FRAME_SET:
        return (typeid(T) == typeid(FrameSet));
    case OB_FRAME_POINTS:
        return (typeid(T) == typeid(PointsFrame));
    case OB_FRAME_RAW_PHASE:
        return (typeid(T) == typeid(RawPhaseFrame) || typeid(T) == typeid(VideoFrame));
    default:
        std::cout << "ob::Frame::is() did not catch frame type: " << (int)this->type() << std::endl;
        break;
    }
    return false;
}
}  // namespace ob
