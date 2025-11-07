// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Sensor.hpp
 * @brief Defines types related to sensors, which are used to obtain stream configurations, open and close streams, and set and get sensor properties.
 */
#pragma once

#include "Types.hpp"
#include "libobsensor/hpp/Filter.hpp"
#include "libobsensor/h/Sensor.h"
#include "libobsensor/h/Filter.h"
#include "Error.hpp"
#include "StreamProfile.hpp"
#include "Device.hpp"
#include "Frame.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace ob {

class Sensor {
public:
    /**
     * @brief Callback function for frame data.
     *
     * @param[in] frame The frame data.
     */
    typedef std::function<void(std::shared_ptr<Frame> frame)> FrameCallback;

protected:
    ob_sensor_t  *impl_;
    FrameCallback callback_;

public:
    explicit Sensor(ob_sensor_t *impl) : impl_(impl) {}

    Sensor(Sensor &&sensor) noexcept : impl_(sensor.impl_) {
        sensor.impl_ = nullptr;
    }

    Sensor &operator=(Sensor &&sensor) noexcept {
        if(this != &sensor) {
            ob_error *error = nullptr;
            ob_delete_sensor(impl_, &error);
            Error::handle(&error);
            impl_        = sensor.impl_;
            sensor.impl_ = nullptr;
        }
        return *this;
    }

    Sensor(const Sensor &sensor)            = delete;
    Sensor &operator=(const Sensor &sensor) = delete;

    virtual ~Sensor() noexcept {
        ob_error *error = nullptr;
        ob_delete_sensor(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the sensor type.
     *
     * @return OBSensorType The sensor type.
     */
    OBSensorType getType() const {
        ob_error *error = nullptr;
        auto      type  = ob_sensor_get_type(impl_, &error);
        Error::handle(&error);
        return type;
    }

    /**
     * @brief Get the list of stream profiles.
     *
     * @return std::shared_ptr<StreamProfileList> The stream profile list.
     */
    std::shared_ptr<StreamProfileList> getStreamProfileList() const {
        ob_error *error = nullptr;
        auto      list  = ob_sensor_get_stream_profile_list(impl_, &error);
        Error::handle(&error);
        return std::make_shared<StreamProfileList>(list);
    }

    /**
     * @brief Create a list of recommended filters for the sensor.
     *
     * @return OBFilterList list of frame processing block
     */
    std::vector<std::shared_ptr<Filter>> createRecommendedFilters() const {
        ob_error *error = nullptr;
        auto      list  = ob_sensor_create_recommended_filter_list(impl_, &error);
        Error::handle(&error);
        auto filter_count = ob_filter_list_get_count(list, &error);

        std::vector<std::shared_ptr<Filter>> filters;
        for(uint32_t i = 0; i < filter_count; i++) {
            auto filterImpl = ob_filter_list_get_filter(list, i, &error);
            Error::handle(&error);
            filters.push_back(std::make_shared<Filter>(filterImpl));
        }
        ob_delete_filter_list(list, &error);
        Error::handle(&error, false);
        return filters;
    }

    /**
     * @brief Open a frame data stream and set up a callback.
     *
     * @param[in] streamProfile The stream configuration.
     * @param[in] callback The callback to set when frame data arrives.
     */
    void start(std::shared_ptr<StreamProfile> streamProfile, FrameCallback callback) {
        ob_error *error = nullptr;
        callback_       = std::move(callback);
        ob_sensor_start(impl_, const_cast<ob_stream_profile_t *>(streamProfile->getImpl()), &Sensor::frameCallback, this, &error);
        Error::handle(&error);
    }

    /**
     * @brief Stop the stream.
     */
    void stop() const {
        ob_error *error = nullptr;
        ob_sensor_stop(impl_, &error);
        Error::handle(&error);
    }

    /**
     * @brief Dynamically switch resolutions.
     *
     * @param[in] streamProfile The resolution to switch to.
     */
    void switchProfile(std::shared_ptr<StreamProfile> streamProfile) {
        ob_error *error = nullptr;
        ob_sensor_switch_profile(impl_, const_cast<ob_stream_profile_t *>(streamProfile->getImpl()), &error);
        Error::handle(&error);
    }

private:
    static void frameCallback(ob_frame *frame, void *userData) {
        auto sensor = static_cast<Sensor *>(userData);
        sensor->callback_(std::make_shared<Frame>(frame));
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    OBSensorType type() const {
        return getType();
    }

    std::vector<std::shared_ptr<Filter>> getRecommendedFilters() const {
        return createRecommendedFilters();
    }
};

class SensorList {
private:
    ob_sensor_list_t *impl_ = nullptr;

public:
    explicit SensorList(ob_sensor_list_t *impl) : impl_(impl) {}

    ~SensorList() noexcept {
        ob_error *error = nullptr;
        ob_delete_sensor_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of sensors.
     *
     * @return uint32_t The number of sensors.
     */
    uint32_t getCount() const {
        ob_error *error = nullptr;
        auto      count = ob_sensor_list_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the type of the specified sensor.
     *
     * @param[in] index The sensor index.
     *
     * @return OBSensorType The sensor type.
     */
    OBSensorType getSensorType(uint32_t index) const {
        ob_error *error = nullptr;
        auto      type  = ob_sensor_list_get_sensor_type(impl_, index, &error);
        Error::handle(&error);
        return type;
    }

    /**
     * @brief Get a sensor by index number.
     *
     * @param[in] index The sensor index. The range is [0, count-1]. If the index exceeds the range, an exception will be thrown.
     *
     * @return std::shared_ptr<Sensor> The sensor object.
     */
    std::shared_ptr<Sensor> getSensor(uint32_t index) const {
        ob_error *error  = nullptr;
        auto      sensor = ob_sensor_list_get_sensor(impl_, index, &error);
        Error::handle(&error);
        return std::make_shared<Sensor>(sensor);
    }

    /**
     * @brief Get a sensor by sensor type.
     *
     * @param[in] sensorType The sensor type to obtain.
     *
     * @return std::shared_ptr<Sensor> A sensor object. If the specified sensor type does not exist, it will return empty.
     */
    std::shared_ptr<Sensor> getSensor(OBSensorType sensorType) const {
        ob_error *error  = nullptr;
        auto      sensor = ob_sensor_list_get_sensor_by_type(impl_, sensorType, &error);
        Error::handle(&error);
        return std::make_shared<Sensor>(sensor);
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    uint32_t count() const {
        return getCount();
    }

    OBSensorType type(uint32_t index) const {
        return getSensorType(index);
    }
};

}  // namespace ob

