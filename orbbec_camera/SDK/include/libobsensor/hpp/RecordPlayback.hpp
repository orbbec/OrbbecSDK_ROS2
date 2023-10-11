/**
 * @file RecordPlayback.hpp
 * @brief Header file for recording and playback functions.
 */

#pragma once

#include "Types.hpp"
#include <memory>

struct RecorderImpl;
struct PlaybackImpl;

namespace ob {

class Device;
class Frame;
class DeviceInfo;

using PlaybackCallback   = std::function<void(std::shared_ptr<Frame> frame)>;
using MediaStateCallback = std::function<void(OBMediaState state)>;

class OB_EXTENSION_API Recorder {
private:
    std::unique_ptr<RecorderImpl> impl_;

public:
    /**
     * @brief Create a recorder for data recording.
     */
    Recorder();
    Recorder(std::unique_ptr<RecorderImpl> impl);

    /**
     * @brief Create a recorder for data recording.
     * @param device The device for which to record device information.
     */
    Recorder(std::shared_ptr<Device> device);

    virtual ~Recorder() noexcept;

    /**
     * @brief Enable the recorder. Throws an exception on failure.
     *
     * @param filename The name of the recorded file.
     * @param async    Whether to execute asynchronously.
     */
    void start(const char *filename, bool async = false);

    /**
     * @brief Stop the recorder. Throws an exception on failure.
     */
    void stop();

    /**
     * @brief Write frame data to the recorder.
     *
     * @param frame The frame data to write.
     */
    void write(std::shared_ptr<Frame> frame);
};

class OB_EXTENSION_API Playback {
private:
    std::unique_ptr<PlaybackImpl> impl_;

public:
    /**
     * @brief Create a playback object.
     * @param filename The name of the playback file.
     */
    Playback(const char *filename);
    Playback(std::unique_ptr<PlaybackImpl> impl);
    virtual ~Playback() noexcept;

    /**
     * @brief Start playback. The playback data is returned from the callback. Throws an exception on failure.
     * @param callback The callback for playback data.
     * @param type     	The type of playback data.
     */
    void start(PlaybackCallback callback, OBMediaType type = OB_MEDIA_ALL);

    /**
     * @brief Stop playback. Throws an exception on failure.
     */
    void stop();

    /**
     * @brief Set the playback state.
     * @param state The playback status callback.
     */
    void setPlaybackStateCallback(MediaStateCallback state);

    /**
     * @brief Get the device information in the recording file.
     *
     * @return DeviceInfo The device information.
     */
    std::shared_ptr<DeviceInfo> getDeviceInfo();

    /**
     * @brief Get the intrinsic and extrinsic parameter in the recording file.
     *
     * @return OBCameraParam Camera intrinsic and extrinsic parameter
     */
    OBCameraParam getCameraParam();
};

}  // namespace ob
