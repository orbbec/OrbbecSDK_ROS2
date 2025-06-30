// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file RecordPlayback.hpp
 * @brief Record and playback device-related types, including interfaces to create recording and playback devices,
 record and playback streaming data, etc.
 */

#pragma once

#include "Types.hpp"
#include "Error.hpp"
#include "libobsensor/h/RecordPlayback.h"
#include "libobsensor/hpp/Device.hpp"

namespace ob {

typedef std::function<void(OBPlaybackStatus status)> PlaybackStatusChangeCallback;

class RecordDevice {
private:
    ob_record_device_t *impl_ = nullptr;

public:
    explicit RecordDevice(std::shared_ptr<Device> device, const std::string &file, bool compressionEnabled = true) {
        ob_error *error = nullptr;
        impl_           = ob_create_record_device(device->getImpl(), file.c_str(), compressionEnabled, &error);
        Error::handle(&error);
    }

    virtual ~RecordDevice() noexcept {
        ob_error *error = nullptr;
        ob_delete_record_device(impl_, &error);
        Error::handle(&error, false);
    }

    RecordDevice(RecordDevice &&other) noexcept {
        if(this != &other) {
            impl_       = other.impl_;
            other.impl_ = nullptr;
        }
    }

    RecordDevice &operator=(RecordDevice &&other) noexcept {
        if(this != &other) {
            impl_       = other.impl_;
            other.impl_ = nullptr;
        }

        return *this;
    }

    RecordDevice(const RecordDevice &)            = delete;
    RecordDevice &operator=(const RecordDevice &) = delete;

public:
    void pause() {
        ob_error *error = nullptr;
        ob_record_device_pause(impl_, &error);
        Error::handle(&error);
    }

    void resume() {
        ob_error *error = nullptr;
        ob_record_device_resume(impl_, &error);
        Error::handle(&error);
    }
};

class PlaybackDevice : public Device {
public:
    explicit PlaybackDevice(const std::string &file) : Device(nullptr) {
        ob_error *error = nullptr;
        impl_           = ob_create_playback_device(file.c_str(), &error);
        Error::handle(&error);
    }

    virtual ~PlaybackDevice() noexcept override = default;

    PlaybackDevice(PlaybackDevice &&other) noexcept : Device(std::move(other)) {}

    PlaybackDevice &operator=(PlaybackDevice &&other) noexcept {
        Device::operator=(std::move(other));
        return *this;
    }

    PlaybackDevice(const PlaybackDevice &)            = delete;
    PlaybackDevice &operator=(const PlaybackDevice &) = delete;

public:
    /**
     * @brief Pause the streaming data from the playback device.
     */
    void pause() {
        ob_error *error = nullptr;
        ob_playback_device_pause(impl_, &error);
        Error::handle(&error);
    }

    /**
     * @brief Resume the streaming data from the playback device.
     */
    void resume() {
        ob_error *error = nullptr;
        ob_playback_device_resume(impl_, &error);
        Error::handle(&error);
    }

    /**
     * @brief Seek to a specific timestamp when playing back a recording.
     * @param[in] timestamp The timestamp to seek to, in milliseconds.
     */
    void seek(const int64_t timestamp) {
        ob_error *error = nullptr;
        ob_playback_device_seek(impl_, timestamp, &error);
        Error::handle(&error);
    }

    /**
     * @brief Set the playback rate of the playback device.
     * @param[in] rate The playback rate to set.
     */
    void setPlaybackRate(const float rate) {
        ob_error *error = nullptr;
        ob_playback_device_set_playback_rate(impl_, rate, &error);
        Error::handle(&error);
    }

    /**
     * @brief Set a callback function to be called when the playback status changes.
     * @param[in] callback The callback function to set.
     */
    void setPlaybackStatusChangeCallback(PlaybackStatusChangeCallback callback) {
        callback_       = callback;
        ob_error *error = nullptr;
        ob_playback_device_set_playback_status_changed_callback(impl_, &PlaybackDevice::playbackStatusCallback, this, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get the current playback status of the playback device.
     * @return The current playback status.
     */
    OBPlaybackStatus getPlaybackStatus() const {
        ob_error        *error  = nullptr;
        OBPlaybackStatus status = ob_playback_device_get_current_playback_status(impl_, &error);
        Error::handle(&error);

        return status;
    }

    /**
     * @brief Get the current position of the playback device.
     * @return The current position of the playback device, in milliseconds.
     */
    uint64_t getPosition() const {
        ob_error *error    = nullptr;
        uint64_t  position = ob_playback_device_get_position(impl_, &error);
        Error::handle(&error);

        return position;
    }

    /**
     * @brief Get the duration of the playback device.
     * @return The duration of the playback device, in milliseconds.
     */
    uint64_t getDuration() const {
        ob_error *error    = nullptr;
        uint64_t  duration = ob_playback_device_get_duration(impl_, &error);
        Error::handle(&error);

        return duration;
    }

private:
    static void playbackStatusCallback(OBPlaybackStatus status, void *userData) {
        auto *playbackDevice = static_cast<PlaybackDevice *>(userData);
        if(playbackDevice && playbackDevice->callback_) {
            playbackDevice->callback_(status);
        }
    }

private:
    PlaybackStatusChangeCallback callback_;
};
}  // namespace ob