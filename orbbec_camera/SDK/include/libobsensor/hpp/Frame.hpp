/**
 * \if English
 * @file Frame.hpp
 * @brief Frame related type, which is mainly used to obtain frame data and frame information.
 * \else
 * @file Frame.hpp
 * @brief 帧相关类型，主要用于获取帧数据及帧的信息
 * \endif
 *
 */
#pragma once

#include "Types.hpp"

#include <memory>
#include <typeinfo>

/**
 *    Frame类继承关系：
 *                 Frame
 *                   |
 *            +------+----------+----------+-----------+
 *            |      |          |         |            |
 *      VideoFrame PointFrame AccelFrame GyroFrame FrameSet
 *          |
 *       +--+------+---------+
 *      |          |         |
 *  ColorFrame DepthFrame IRFrame
 *
 */

struct FrameImpl;

namespace ob {
class StreamProfile;
class Filter;
class FrameHelper;

typedef std::function<void(void *buffer, void *context)> BufferDestroyCallback;

class OB_EXTENSION_API Frame : public std::enable_shared_from_this<Frame> {
protected:
    std::unique_ptr<FrameImpl> impl_;

public:
    Frame(std::unique_ptr<FrameImpl> impl);
    Frame(Frame &frame);

    virtual ~Frame() noexcept;
    /**
     * \if English
     * @brief Get the type of frame
     *
     * @return OBFrameType returns the type of frame
     * \else
     * @brief 获取帧的类型
     *
     * @return OBFrameType 返回帧的类型
     * \endif
     */
    virtual OBFrameType type();

    /**
     * \if English
     * @brief Get the format of the frame
     *
     * @return OBFormat returns the format of the frame
     * \else
     * @brief 获取帧的格式
     *
     * @return OBFormat 返回帧的格式
     * \endif
     */
    virtual OBFormat format();

    /**
     * \if English
     * @brief Get the sequence number of the frame
     *
     * @return uint64_t returns the sequence number of the frame
     * \else
     * @brief 获取帧的序号
     *
     * @return uint64_t 返回帧的序号
     * \endif
     */
    virtual uint64_t index();

    /**
     * \if English
     * @brief Get frame data
     *
     * @return void* returns frame data
     * \else
     * @brief 获取帧数据
     *
     * @return void* 返回帧数据
     * \endif
     */
    virtual void *data();

    /**
     * \if English
     * @brief Get the frame data size
     *
     * @return uint32_t returns the frame data size
     * If it is point cloud data, it returns the number of bytes occupied by all point sets. If you need to find the number of points, you need to divide the
     * dataSize by the structure size of the corresponding point type. \else
     * @brief 获取帧数据大小
     *
     * @return uint32_t 返回帧数据的大小
     * 如果是点云数据返回的是所有点集合占的字节数，若需要求出点的个数需要将dataSize除以对应的点类型的结构体大小
     * \endif
     */
    virtual uint32_t dataSize();

    /**
     * \if English
     * @brief Get the hardware timestamp of the frame
     *
     * @return uint64_t returns the time stamp of the frame hardware
     * \else
     * @brief 获取帧的硬件时间戳
     *
     * @return uint64_t 返回帧硬件的时间戳
     * \endif
     */
    uint64_t timeStamp();

    /**
     * \if English
     * @brief Get the hardware timestamp of the frame us
     *
     * @return uint64_t returns the time stamp of the frame hardware, unit us
     * \else
     * @brief 获取帧的硬件时间戳
     *
     * @return uint64_t 返回帧硬件的时间戳
     * \endif
     */
    uint64_t timeStampUs();

    /**
     * \if English
     * @brief Get frame system timestamp
     *
     * @return uint64_t returns the time stamp of the frame hardware
     * \else
     * @brief 获取帧的系统时间戳
     *
     * @return uint64_t 返回帧的系统时间戳
     * \endif
     */
    uint64_t systemTimeStamp();

    /**
     * \if English
     * @brief Check if the runtime type of the frame object is compatible with a given type
     *
     * @tparam T Given type
     * @return bool returns result
     * \else
     * @brief 检查帧对象的运行时类型是否与给定类型兼容
     *
     * @tparam T 给定的类型
     * @return bool 返回结果
     * \endif
     */
    template <typename T> bool is();

    /**
     * \if English
     * @brief Frame object type conversion
     *
     * @tparam T Target type
     * @return std::shared_ptr<T> returns the result, if it cannot be converted, an exception will be thrown
     * \else
     * @brief 帧对象类型转换
     *
     * @tparam T 目标类型
     * @return std::shared_ptr<T> 返回结果, 如果不能够转换，将抛异常
     * \endif
     */
    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>())
            throw "unsupported operation, object's type is not require type";

        return std::static_pointer_cast<T>(std::const_pointer_cast<Frame>(shared_from_this()));
    }

private:
    friend class Filter;
    friend class Recorder;
    friend class FrameHelper;
};

class OB_EXTENSION_API VideoFrame : public Frame {
public:
    VideoFrame(Frame &frame);
    virtual ~VideoFrame() noexcept {};

    /**
     * \if English
     * @brief Get frame width
     *
     * @return uint32_t returns the width of the frame
     * \else
     * @brief 获取帧的宽
     *
     * @return uint32_t 返回帧的宽
     * \endif
     */
    uint32_t width();

    /**
     * \if English
     * @brief Get frame height
     *
     * @return uint32_t returns the height of the frame
     * \else
     * @brief 获取帧的高
     *
     * @return uint32_t 返回帧的高
     * \endif
     */
    uint32_t height();

    /**
     * \if English
     * @brief Get the metadata of the frame
     *
     * @return void* returns the metadata of the frame
     * \else
     * @brief 获取帧的元数据
     *
     * @return void* 返回帧的元数据
     * \endif
     */
    void *metadata();

    /**
     * \if English
     * @brief Get the metadata size of the frame
     *
     * @return uint32_t returns the metadata size of the frame
     * \else
     * @brief 获取帧的元数据大小
     *
     * @return uint32_t 返回帧的元数据大小
     * \endif
     */
    uint32_t metadataSize();

    /**
     * \if English
     * @brief Get the effective number of pixels (such as Y16 format frame, but only the lower 10 bits are valid bits, and the upper 6 bits are filled with 0)
     * @attention Only valid for Y8/Y10/Y11/Y12/Y14/Y16 format
     *
     * @return uint8_t returns the effective number of pixels in the pixel, or 0 if it is an unsupported format
     * \else
     * @brief 获取像素有效位数（如Y16格式帧，每个像素占16bit，但实际只有低10位是有效位，高6位填充0）
     * @attention 仅对Y8/Y10/Y11/Y12/Y14/Y16格式有效
     *
     * @return uint8_t 返回像素有效位数，如果是不支持的格式，返回0
     * \endif
     */
    uint8_t pixelAvailableBitSize();
};

class OB_EXTENSION_API ColorFrame : public VideoFrame {
public:
    ColorFrame(Frame &frame);
    ~ColorFrame() noexcept {};
};

class OB_EXTENSION_API DepthFrame : public VideoFrame {
public:
    DepthFrame(Frame &frame);
    ~DepthFrame() noexcept {};

    /**
     * \if English
     * @brief Get the value scale of the depth frame, the unit is mm/step,
     *        such as valueScale=0.1, and a certain coordinate pixel value is pixelValue=10000,
     *        then the depth value value = pixelValue*valueScale = 10000*0.1=1000mm.
     *
     * @return float
     * \else
     * @brief 获取深度帧的值刻度，单位为 mm/step，
     *      如valueScale=0.1, 某坐标像素值为pixelValue=10000，
     *     则表示深度值value = pixelValue*valueScale = 10000*0.1=1000mm。
     *
     * @return float
     * \endif
     */
    float getValueScale();
};

class OB_EXTENSION_API IRFrame : public VideoFrame {
public:
    IRFrame(Frame &frame);
    virtual ~IRFrame() noexcept {};
};

class OB_EXTENSION_API PointsFrame : public Frame {
public:
    PointsFrame(Frame &frame);
    ~PointsFrame() noexcept {};
};

class OB_EXTENSION_API FrameSet : public Frame {

public:
    // FrameSet();
    FrameSet(Frame &frame);
    ~FrameSet() noexcept;

    /**
     * \if English
     * @brief Get frame count
     *
     * @return uint32_t returns the number of frames
     * \else
     * @brief 帧集合中包含的帧数量
     *
     * @return uint32_t 返回帧的数量
     * \endif
     */
    uint32_t frameCount();

    /**
     * \if English
     * @brief Get depth frame
     *
     * @return std::shared_ptr<DepthFrame> returns the depth frame
     * \else
     * @brief 获取深度帧
     *
     * @return std::shared_ptr<DepthFrame> 返回深度帧
     * \endif
     */
    std::shared_ptr<DepthFrame> depthFrame();

    /**
     * \if English
     * @brief Get color frame
     *
     * @return std::shared_ptr<ColorFrame> returns the color frame
     * \else
     * @brief 获取彩色帧
     *
     * @return std::shared_ptr<ColorFrame> 返回彩色帧
     * \endif
     */
    std::shared_ptr<ColorFrame> colorFrame();

    /**
     * \if English
     * @brief Get infrared frame
     *
     * @return std::shared_ptr<IRFrame> returns infrared frame
     * \else
     * @brief 获取红外帧
     *
     * @return std::shared_ptr<IRFrame> 返回红外帧
     * \endif
     */
    std::shared_ptr<IRFrame> irFrame();

    /**
     * \if English
     * @brief Get point cloud frame
     *
     * @return  std::shared_ptr<PointsFrame> returns the point cloud data frame
     * \else
     * @brief 获取点云帧
     *
     * @return  std::shared_ptr<PointsFrame> 返回点云帧
     * \endif
     */
    std::shared_ptr<PointsFrame> pointsFrame();

    /**
     * \if English
     * @brief Get frame by sensor type
     *
     * @param frameType  Type of sensor
     * @return std::shared_ptr<Frame> returns the corresponding type of frame
     * \else
     * @brief 通过传感器类型获取帧
     *
     * @param frameType 传感器的类型
     * @return std::shared_ptr<Frame> 返回相应类型的帧
     * \endif
     */
    std::shared_ptr<Frame> getFrame(OBSensorType sensorType);

    friend class Pipeline;
    friend class Filter;
};

class OB_EXTENSION_API AccelFrame : public Frame {
public:
    AccelFrame(Frame &frame);
    ~AccelFrame() noexcept = default;

    /**
     * \if English
     * @brief Accelerometer frame data
     *
     * @return OBAccelValue
     * \else
     * @brief 获取加速度帧X数据
     *
     * @return OBAccelValue
     * \endif
     */
    OBAccelValue value();

    /**
     * \if English
     * @brief Get the temperature when the frame is sampled
     *
     * @return float temperature value
     * \else
     * @brief 获取帧采样时的温度
     *
     * @return float 温度值
     * \endif
     */
    float temperature();
};

class OB_EXTENSION_API GyroFrame : public Frame {
public:
    GyroFrame(Frame &frame);
    ~GyroFrame() noexcept = default;

    /**
     * \if English
     * @brief Get gyro frame data
     *
     * @return OBAccelValue
     * \else
     * @brief 获取陀螺仪帧数据
     *
     * @return OBAccelValue
     * \endif
     */
    OBGyroValue value();

    /**
     * \if English
     * @brief Get temperature when the frame is sampled.
     *
     * @return float temperature value
     * \else
     * @brief 获取帧采样时的温度
     *
     * @return float 温度值
     * \endif
     */
    float temperature();
};

class OB_EXTENSION_API FrameHelper {
public:
    FrameHelper();
    ~FrameHelper();
    /**
     * @brief 根据外部创建的Buffer创建帧对象
     *
     * @param format 帧对象格式
     * @param frameWidth 帧对象宽
     * @param frameHeight 帧对象高
     * @param buffer 帧对象数据
     * @param bufferSize 帧对象数据大小
     * @return std::shared_ptr<Frame> 返回帧对象
     */
    static std::shared_ptr<Frame> createFrameFromBuffer(OBFormat format, uint32_t frameWidth, uint32_t frameHeight, uint8_t *buffer, uint32_t bufferSize,
                                                        BufferDestroyCallback destroyCallback, void *destroyCallbackContext);

    /**
     * @brief 创建空的帧集合对象
     *
     * @return std::shared_ptr<Frame> 返回帧集合对象
     */
    static std::shared_ptr<Frame> createFrameSet();

    /**
     * @brief 往帧集合中填入对应类型的帧
     *
     * @param frameSet 帧集合对象
     * @param frameType 填入帧的类型
     * @param frame 填入帧的对象
     */
    static void pushFrame(std::shared_ptr<Frame> frameSet, OBFrameType frameType, std::shared_ptr<Frame> frame);

    /**
     * @brief 设置帧的系统时间戳
     *
     * @param frame 设置的帧对象
     * @param systemTimestamp 设置的系统时间戳
     */
    static void setFrameSystemTimestamp(std::shared_ptr<Frame> frame, uint64_t systemTimestamp);

    /**
     * @brief 设置帧的设备时间戳
     *
     * @param frame 设置的帧对象
     * @param deviceTimestamp 设置的设备时间戳
     */
    static void setFrameDeviceTimestamp(std::shared_ptr<Frame> frame, uint64_t deviceTimestamp);

    /**
     * @brief 设置帧的设备时间戳
     *
     * @param frame 设置的帧对象
     * @param deviceTimestampUs 设置的设备时间戳（Us）
     */
    static void setFrameDeviceTimestampUs(std::shared_ptr<Frame> frame, uint64_t deviceTimestampUs);
};

template <typename T> bool Frame::is() {
    switch(this->type()) {
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
    default:
        break;
    }
    return false;
}
}  // namespace ob
