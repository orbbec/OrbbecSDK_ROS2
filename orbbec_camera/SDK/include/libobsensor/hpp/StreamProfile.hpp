/**
 * \if English
 * @file StreamProfile.hpp
 * @brief The stream configuration related type is used to get information such as the width, height, frame rate, and format of the stream.
 * \else
 * @file StreamProfile.hpp
 * @brief 流配置相关类型，用于获取流的宽、高、帧率及格式等信息
 * \endif
 */
#pragma once

#include "Types.hpp"

#include <memory>

struct StreamProfileImpl;
struct StreamProfileListImpl;

namespace ob {

class VideoStreamProfile;
class GyroStreamProfile;
class AccelStreamProfile;
class Config;

class OB_EXTENSION_API StreamProfile : public std::enable_shared_from_this<StreamProfile> {
protected:
    std::unique_ptr<StreamProfileImpl> impl_;

public:
    StreamProfile(std::unique_ptr<StreamProfileImpl> impl);
    StreamProfile(StreamProfile &streamProfile);
    ~StreamProfile() noexcept;

    /**
     * \if English
     * @brief Get the format of the stream
     *
     * @return OBFormat returns the format of the stream
     * \else
     * @brief 获取流的格式
     *
     * @return OBFormat 返回流的格式
     * \endif
     */
    OBFormat format();
    /**
     * \if English
     * @brief Get the type of stream
     *
     * @return OBStreamType returns the type of the stream
     * \else
     * @brief 获取流的类型
     *
     * @return OBStreamType 返回流的类型
     * \endif
     */
    OBStreamType type();

    /**
     * \if English
     * @brief Check if frame object is compatible with the given type
     *
     * @tparam T  Given type
     * @return bool return result
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
     * @brief Object type conversion
     *
     * @tparam T  Target type
     * @return std::shared_ptr<T>  returns the result, if it cannot be converted, an exception will be thrown
     * \else
     * @brief 对象类型转换
     *
     * @tparam T 目标类型
     * @return std::shared_ptr<T> 返回结果, 如果不能够转换，将抛异常
     * \endif
     */
    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>())
            throw "unsupported operation, object's type is not require type";

        return std::static_pointer_cast<T>(std::const_pointer_cast<StreamProfile>(shared_from_this()));
    }

    friend class Sensor;
    friend class Config;
    friend class Pipeline;
};

class OB_EXTENSION_API VideoStreamProfile : public StreamProfile {
public:
    VideoStreamProfile(StreamProfile &profile);
    ~VideoStreamProfile() noexcept;

    /**
     * \if English
     * @brief Get stream frame rate
     *
     * @return uint32_t returns the frame rate of the stream
     * \else
     * @brief 获取流的帧率
     *
     * @return uint32_t 返回流的帧率
     * \endif
     */
    uint32_t fps();
    /**
     * \if English
     * @brief Get stream width
     *
     * @return uint32_t returns the width of the stream
     * \else
     * @brief 获取流的宽
     *
     * @return uint32_t 返回流的宽
     * \endif
     */
    uint32_t width();
    /**
     * \if English
     * @brief Get stream height
     *
     * @return uint32_t returns the high of the stream
     * \else
     * @brief 获取流的高
     *
     * @return uint32_t 返回流的高
     * \endif
     */
    uint32_t height();
};

class OB_EXTENSION_API AccelStreamProfile : public StreamProfile {
public:
    AccelStreamProfile(StreamProfile &profile);
    ~AccelStreamProfile() noexcept;

    /**
     * \if English
     * @brief Get full scale range
     *
     * @return OBAccelFullScaleRange  returns the scale range value
     * \else
     * @brief 获取满量程范围
     *
     * @return OBAccelFullScaleRange  返回量程范围值
     * \endif
     */
    OBAccelFullScaleRange fullScaleRange();

    /**
     * \if English
     * @brief Get sampling frequency
     *
     * @return OBAccelFullScaleRange  returns the sampling frequency
     * \else
     * @brief 获取采样频率
     *
     * @return OBAccelFullScaleRange  返回采样频率
     * \endif
     */
    OBAccelSampleRate sampleRate();
};

class OB_EXTENSION_API GyroStreamProfile : public StreamProfile {
public:
    GyroStreamProfile(StreamProfile &profile);
    ~GyroStreamProfile() noexcept;

    /**
     * \if English
     * @brief Get full scale range
     *
     * @return OBAccelFullScaleRange  returns the scale range value
     * \else
     * @brief 获取满量程范围
     *
     * @return OBAccelFullScaleRange  返回量程范围值
     * \endif
     */
    OBGyroFullScaleRange fullScaleRange();

    /**
     * \if English
     * @brief Get sampling frequency
     *
     * @return OBAccelFullScaleRange  returns the sampling frequency
     * \else
     * @brief 获取采样频率
     *
     * @return OBAccelFullScaleRange  返回采样频率
     * \endif
     */
    OBGyroSampleRate sampleRate();
};

template <typename T> bool StreamProfile::is() {
    switch(this->type()) {
    case OB_STREAM_VIDEO:
    case OB_STREAM_IR:
    case OB_STREAM_COLOR:
    case OB_STREAM_DEPTH:
        return typeid(T) == typeid(VideoStreamProfile);
    case OB_STREAM_ACCEL:
        return typeid(T) == typeid(AccelStreamProfile);
    case OB_STREAM_GYRO:
        return typeid(T) == typeid(GyroStreamProfile);
    default:
        break;
    }
    return false;
}

class OB_EXTENSION_API StreamProfileList {
protected:
    std::unique_ptr<StreamProfileListImpl> impl_;

public:
    StreamProfileList(std::unique_ptr<StreamProfileListImpl> impl);
    ~StreamProfileList() noexcept;

    /**
     * \if English
     * @brief Get stream profile count
     *
     * @return uint32_t returns the number of StreamProfile
     * \else
     * @brief 获取StreamProfile数量
     *
     * @return uint32_t 返回StreamProfile的数量
     * \endif
     */
    uint32_t count();

    /**
     * \if English
     * @brief Get StreamProfile by index number
     *
     * @param index Device index to be created，the range is [0, count-1],if the index exceeds the range, an exception will be thrown
     * @return std::shared_ptr<StreamProfile> returns StreamProfile object
     * \else
     * @brief 通过索引号获取StreamProfile
     *
     * @param index 要创建设备的索，范围 [0, count-1]，如果index超出范围将抛异常
     * @return std::shared_ptr<StreamProfile> 返回StreamProfile对象
     * \endif
     */
    const std::shared_ptr<StreamProfile> getProfile(uint32_t index);

    /**
     * \if English
     * @brief Match the corresponding streamprofile through the passed in parameters. If there are multiple matches, the first one in the list will be returned
     * by default.
     *
     * @param width Width. If no matching condition is required, it can be passed to 0
     * @param height Height. If no matching condition is required, it can be passed to 0
     * @param format Type. If no matching condition is required, it can be passed to OB_FORMAT_UNKNOWN
     * @param fps Frame rate. If no matching condition is required, it can be passed to 0
     * @return std::shared_ptr<StreamProfile> Returns the matching resolution
     * \else
     * @brief 通过传入的参数进行匹配对应的StreamProfile，若有多个匹配项默认返回列表中的第一个
     *
     * @param width 宽度，如不要求加入匹配条件，可传0
     * @param height 高度，如不要求加入匹配条件，可传0
     * @param format 类型，如不要求加入匹配条件，可传OB_FORMAT_UNKNOWN
     * @param fps 帧率，如不要求加入匹配条件，可传0
     * @return std::shared_ptr<StreamProfile> 返回匹配的分辨率
     * \endif
     */
    const std::shared_ptr<VideoStreamProfile> getVideoStreamProfile(int width = 0, int height = 0, OBFormat format = OB_FORMAT_UNKNOWN, int fps = 0);
};

}  // namespace ob
