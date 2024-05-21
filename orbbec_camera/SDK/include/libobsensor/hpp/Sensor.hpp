/**
 * @file Sensor.hpp
 * @brief Defines types related to sensors, which are used to obtain stream configurations, open and close streams, and set and get sensor properties.
 */
#pragma once

#include "Types.hpp"
#include "libobsensor/hpp/Filter.hpp"

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
class OBFilterList;

/**
 * @brief Callback function for frame data.
 *
 * @param frame The frame data.
 */
using FrameCallback = std::function<void(std::shared_ptr<Frame> frame)>;

class OB_EXTENSION_API Sensor {
protected:
    std::unique_ptr<SensorImpl> impl_;

public:
    Sensor(std::unique_ptr<SensorImpl> impl);
    virtual ~Sensor() noexcept;

    /**
     * @brief Get the sensor type.
     *
     * @return OBSensorType The sensor type.
     */
    OBSensorType type();

    /**
     * @brief Get the list of stream profiles.
     *
     * @return std::shared_ptr<StreamProfileList> The stream profile list.
     */
    const std::shared_ptr<StreamProfileList> getStreamProfileList();

    /**
     * @brief Request recommended filters
     * @return OBFilterList list of frame processing block
     */
    const std::shared_ptr<OBFilterList> getRecommendedFilters();

    /**
     * @brief Open a frame data stream and set up a callback.
     *
     * @param streamProfile The stream configuration.
     * @param callback The callback to set when frame data arrives.
     */
    void start(std::shared_ptr<StreamProfile> streamProfile, FrameCallback callback);

    /**
     * @brief Stop the stream.
     */
    void stop();

    /**
     * @brief Dynamically switch resolutions.
     *
     * @param streamProfile The resolution to switch to.
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
     * @brief Get the number of sensors.
     *
     * @return uint32_t The number of sensors.
     */
    uint32_t count();

    /**
     * @brief Get the type of the specified sensor.
     *
     * @param index The sensor index.
     * @return OBSensorType The sensor type.
     */
    OBSensorType type(uint32_t index);

    /**
     * @brief Get a sensor by index number.
     *
     * @param index The sensor index. The range is [0, count-1]. If the index exceeds the range, an exception will be thrown.
     * @return std::shared_ptr<Sensor> The sensor object.
     */
    std::shared_ptr<Sensor> getSensor(uint32_t index);

    /**
     * @brief Get a sensor by sensor type.
     *
     * @param sensorType The sensor type to obtain.
     * @return std::shared_ptr<Sensor> A sensor object. If the specified sensor type does not exist, it will return empty.
     */
    std::shared_ptr<Sensor> getSensor(OBSensorType sensorType);
};


/**
 * @brief Class representing a list of FrameProcessingBlock
 */
class OB_EXTENSION_API OBFilterList {
private:
    std::unique_ptr<OBFilterListImpl> impl_;

public:
    OBFilterList(std::unique_ptr<OBFilterListImpl> impl_);
    ~OBFilterList() noexcept;

    /**
     * @brief Get the number of OBDepthWorkMode FrameProcessingBlock in the list
     *
     * @return uint32_t the number of FrameProcessingBlock objects in the list
     */
    uint32_t count();

    /**
     * @brief Get the Filter object at the specified index
     *
     * @param index the index of the target Filter object
     * @return the Filter object at the specified index
     */
    std::shared_ptr<Filter> getFilter(uint32_t index);
};

}  // namespace ob
