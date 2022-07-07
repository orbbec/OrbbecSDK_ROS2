/**
 * \if English
 * @file Sensor.hpp
 * @brief Sensor related types, used to obtain stream configuration, open and close streams, set and get sensor properties.
 * \else
 * @file Sensor.hpp
 * @brief 传感器相关类型，用于获取流配置，开关流，设置及获取传感器属性等操作
 * \endif
 */
#pragma once

#include "Types.hpp"

#include <functional>
#include <memory>

struct SensorImpl;
struct SensorListImpl;

namespace ob {
class StreamProfile;
class StreamProfileList;
class Device;
class Frame;
class ImuFrame;

/**
 * \if English
 * @brief frame data callback
 *
 * @param frame  frame data
 * \else
 * @brief 帧数据回调
 *
 * @param frame  帧数据
 * \endif
 */
using FrameCallback = std::function<void(std::shared_ptr<Frame> frame)>;

class OB_EXTENSION_API Sensor {
protected:
    std::unique_ptr<SensorImpl> impl_;

public:
    Sensor(std::unique_ptr<SensorImpl> impl);
    virtual ~Sensor() noexcept;

    /**
     * \if English
     * @brief sensor type
     *
     * @return OBSensorType returns the sensor type
     * \else
     * @brief 传感器类型
     *
     * @return OBSensorType 返回传感器类型
     * \endif
     */
    OBSensorType type();

    /**
     * \if English
     * @brief Get the list of stream profiles
     *
     * @return std::shared_ptr<StreamProfileList> returns the stream configuration list
     * \else
     * @brief 获取传感器的流配置列表
     *
     * @return std::shared_ptr<StreamProfileList> 返回流配置列表
     * \endif
     */
    const std::shared_ptr<StreamProfileList> getStreamProfileList();

    /**
     * \if English
     * @brief Open frame data stream and set up a callback
     *
     * @param streamProfile Stream configuration
     * @param callback Set the callback when frame data arrives
     * \else
     * @brief 开启流并设置帧数据回调
     *
     * @param streamProfile 流的配置
     * @param callback 设置帧数据到达时的回调
     * \endif
     */
    void start(std::shared_ptr<StreamProfile> streamProfile, FrameCallback callback);
    /**
     * \if English
     * @brief Stop stream
     * \else
     * @brief 停止流
     * \endif
     */
    void stop();

    /**
	 * \if English
	 * @brief Dynamically switch resolutions
     *
     * @param streamProfile Resolution to switch
	 * \else
     * @brief 动态切换分辨率
     *
     * @param streamProfile 需要切换的分辨率
	 * \endif
     */
    void switchProfile(std::shared_ptr<StreamProfile> streamProfile);
};

class OB_EXTENSION_API SensorList {
private:
    std::unique_ptr<SensorListImpl> impl_;

public:
    SensorList(std::unique_ptr<SensorListImpl> impl);

    virtual ~SensorList() noexcept;

    /**
     * \if English
     * @brief Get sensor count
     *
     * @return uint32_t returns the number of Sensors
     * \else
     * @brief 获取Sensor数量
     *
     * @return uint32_t 返回Sensor的数量
     * \endif
     */
    uint32_t count();

    /**
     * \if English
     * @brief Get the type of the specified Sensor
     *
     * @param index  Sensor index
     * @return OBSensorType returns the Sensor type
     * \else
     * @brief 获取指定Sensor的类型
     *
     * @param index Sensor索引
     * @return OBSensorType 返回Sensor类型
     * \endif
     */
    OBSensorType type(uint32_t index);

    /**
     * \if English
     * @brief Get Sensor by index number
     *
     * @param index  To create a device cable, the range is [0, count-1], if the index exceeds the range, an exception will be thrown

     * @return std::shared_ptr<Sensor> returns the Sensor object
     * \else
     * @brief 通过索引号获取Sensor
     *
     * @param index 要创建设备的索，范围 [0, count-1]，如果index超出范围将抛异常
     * @return std::shared_ptr<Sensor> 返回Sensor对象
     * \endif
     */
    std::shared_ptr<Sensor> getSensor(uint32_t index);

    /**
     * \if English
     * @brief Obtain the Sensor through the Sensor type
     *
     * @param sensorType Sensor type to be obtained
     * @return std::shared_ptr<Sensor>  returns a Sensor object, if the specified type of Sensor does not exist, it will return empty
     * \else
     * @brief 通过Sensor类型获取Sensor
     *
     * @param sensorType 要获取的Sensor类型
     * @return std::shared_ptr<Sensor> 返回Sensor对象，如果指定类型Sensor不存在，将返回空
     * \endif
     */
    std::shared_ptr<Sensor> getSensor(OBSensorType sensorType);
};
}  // namespace ob
