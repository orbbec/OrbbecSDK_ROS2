/**
 * \if English
 *
 * @file RecordPlayback.hpp
 * @brief For recording and playback functions
 * \else
 * @file RecordPlayback.hpp
 * @brief 用于录制和回放功能
 * \endif
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

using PlaybackCallback = std::function<void(std::shared_ptr<Frame> frame)>;
using MediaStateCallback = std::function<void(OBMediaState state)>;

class OB_EXTENSION_API Recorder {
private:
    std::unique_ptr<RecorderImpl> impl_;

public:
    /**
     * \if English
     * @brief Create a recorder for data recording
     * \else
     * @brief 创建录制器用于数据的录制
     * \endif
     */
    Recorder();
    Recorder(std::unique_ptr<RecorderImpl> impl);

    /**
     * \if English
     * @brief Create a recorder for data recording
     * @param device Devices that need to record device information
     * \else
     * @brief 创建录制器用于数据的录制
     * @param device 需要录制设备信息的设备
     * \endif
     */
    Recorder(std::shared_ptr<Device> device);

    virtual ~Recorder() noexcept;

    /**
     * \if English
     * @brief Enable the recorder, throw an exception after failure
     *
     * @param filename Recorded file name
     * @param async    Whether to execute asynchronously
     * \else
     * @brief 开启录制器，失败后抛出异常
     *
     * @param filename 录制的文件名称
     * @param async    是否异步执行
     * \endif
     */
    void start(const char *filename, bool async = false);

    /**
     * \if English
     * @brief Stop the recorder, throw an exception on failure
     * \else
     * @brief 停止录制器，失败后抛出异常
     * \endif
     */
    void stop();

    /**
     * \if English
     * @brief Write frame data to the recorder
     *
     * @param frame Write frame data
     * \else
     * @brief 向录制器内写入帧数据
     *
     * @param frame 写入的帧数据
     * \endif
     */
    void write(std::shared_ptr<Frame> frame);
};

class OB_EXTENSION_API Playback {
private:
    std::unique_ptr<PlaybackImpl> impl_;

public:
    /**
     * \if English
     * @brief Create playback object
     * @param filename Playback filename
     * \else
     * @brief 创建回放对象
     * @param filename 回放的文件名
     * \endif
     */
    Playback(const char *filename);
    Playback(std::unique_ptr<PlaybackImpl> impl);
    virtual ~Playback() noexcept;

    /**
     * \if English
     * @brief Start playback, the playback data is returned from the callback, and an exception is thrown after failure
     * @param filename Playback filename
     * @param callback Callback for playback data
     * @param type     	Type of playback data
     * \else
     * @brief 开启回放，回放数据从回调中返回，失败后抛出异常
     * @param filename 回放的文件名
     * @param callback 回放数据的回调
     * @param type     回放数据的类型
     * \endif
     */
    void start(PlaybackCallback callback, OBMediaType type = OB_MEDIA_ALL);

    /**
     * \if English
     * @brief Stop playback and throw an exception if it fails
     * \else
     * @brief 停止回放，失败后抛出异常
     * \endif
     */
    void stop();

    /**
     * \if English
     * @brief Set playback state
     * @param state playback status callback
     * \else
     * @brief 设置回放状态
     * @param state 回放状态回调
     * \endif
     */
    void setPlaybackStateCallback(MediaStateCallback state);

    /**
     * \if English
     * @brief Get the device information in the recording file
     *
     * @return DeviceInfo returns device information
     * \else
     * @brief 获取录制文件内的设备信息
     *
     * @return DeviceInfo 返回的设备信息
     * \endif
     */
    std::shared_ptr<DeviceInfo> getDeviceInfo();

    /**
     * \if English
     * @brief Get the intrinsic and extrinsic parameter information in the recording file
     *
     * @return OBCameraParam returns internal and external parameter information
     * \else
     * @brief 获取录制文件内的内外参信息
     *
     * @return OBCameraParam 返回的内外参信息
     * \endif
     */
    OBCameraParam getCameraParam();
};

}  // namespace ob
