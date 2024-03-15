/**
 * @file StreamProfile.hpp
 * @brief The stream profile related type is used to get information such as the width, height, frame rate, and format of the stream.
 */
#pragma once

#include "Types.hpp"

#include <iostream>
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
    virtual ~StreamProfile() noexcept;

    /**
     * @brief Get the format of the stream
     *
     * @return OBFormat return the format of the stream
     */
    OBFormat format() const;

    /**
     * @brief Get the type of stream
     *
     * @return OBStreamType return the type of the stream
     */
    OBStreamType type() const;

    /**
     * @brief Get the extrinsic parameters from current stream profile to the given target stream profile
     *
     * @return OBExtrinsic Return the extrinsic parameters.
     */
    OBExtrinsic getExtrinsicTo(std::shared_ptr<StreamProfile> target);

    /**
     * @brief Check if frame object is compatible with the given type
     *
     * @tparam T  Given type
     * @return bool return result
     */
    template <typename T> bool is();

    /**
     * @brief Converts object type to target type
     *
     * @tparam T Target type
     * @return std::shared_ptr<T> Return the result. Throws an exception if conversion is not possible.
     */
    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>()) {
            throw std::runtime_error("Unsupported operation. Object's type is not the required type.");
        }

        return std::static_pointer_cast<T>(std::const_pointer_cast<StreamProfile>(shared_from_this()));
    }

    friend class Sensor;
    friend class Config;
    friend class Pipeline;
};

/**
 * @brief Class representing a video stream profile.
 */
class OB_EXTENSION_API VideoStreamProfile : public StreamProfile {
public:
    explicit VideoStreamProfile(StreamProfile &profile);

    explicit VideoStreamProfile(std::unique_ptr<StreamProfileImpl> impl);

    ~VideoStreamProfile() noexcept override;

    /**
     * @brief Return the frame rate of the stream.
     *
     * @return uint32_t Return the frame rate of the stream.
     */
    uint32_t fps() const;

    /**
     * @brief Return the width of the stream.
     *
     * @return uint32_t Return the width of the stream.
     */
    uint32_t width() const;

    /**
     * @brief Return the height of the stream.
     *
     * @return uint32_t Return the height of the stream.
     */
    uint32_t height() const;

    /**
     * @brief Get the intrinsic parameters of the stream.
     *
     * @return OBCameraIntrinsic Return the intrinsic parameters.
     */
    OBCameraIntrinsic getIntrinsic();

    /**
     * @brief Get the distortion parameters of the stream.
     * @brief Brown distortion model
     *
     * @return OBCameraDistortion Return the distortion parameters.
     */
    OBCameraDistortion getDistortion();
};

/**
 * @brief Class representing an accelerometer stream profile.
 */
class OB_EXTENSION_API AccelStreamProfile : public StreamProfile {
public:
    explicit AccelStreamProfile(StreamProfile &profile);

    explicit AccelStreamProfile(std::unique_ptr<StreamProfileImpl> impl);

    ~AccelStreamProfile() noexcept override;

    /**
     * @brief Return the full scale range.
     *
     * @return OBAccelFullScaleRange Return the scale range value.
     */
    OBAccelFullScaleRange fullScaleRange() const;

    /**
     * @brief Return the sampling frequency.
     *
     * @return OBAccelFullScaleRange Return the sampling frequency.
     */
    OBAccelSampleRate sampleRate() const;

    /**
     * @brief get the intrinsic parameters of the stream.
     *
     * @return OBAccelIntrinsic Return the intrinsic parameters.
     */
    OBAccelIntrinsic getIntrinsic();
};

/**
 * @brief Class representing a gyroscope stream profile.
 */
class OB_EXTENSION_API GyroStreamProfile : public StreamProfile {
public:
    explicit GyroStreamProfile(StreamProfile &profile);

    explicit GyroStreamProfile(std::unique_ptr<StreamProfileImpl> impl);
    ~GyroStreamProfile() noexcept override;

    /**
     * @brief Return the full scale range.
     *
     * @return OBAccelFullScaleRange Return the scale range value.
     */
    OBGyroFullScaleRange fullScaleRange() const;

    /**
     * @brief Return the sampling frequency.
     *
     * @return OBAccelFullScaleRange Return the sampling frequency.
     */
    OBGyroSampleRate sampleRate() const;

    /**
     * @brief get the intrinsic parameters of the stream.
     *
     * @return OBGyroIntrinsic Return the intrinsic parameters.
     */
    OBGyroIntrinsic getIntrinsic();
};

template <typename T> bool StreamProfile::is() {
    switch(this->type()) {
    case OB_STREAM_VIDEO:
    case OB_STREAM_IR:
    case OB_STREAM_IR_LEFT:
    case OB_STREAM_IR_RIGHT:
    case OB_STREAM_COLOR:
    case OB_STREAM_DEPTH:
    case OB_STREAM_RAW_PHASE:
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
    explicit StreamProfileList(std::unique_ptr<StreamProfileListImpl> impl);
    ~StreamProfileList() noexcept;

    /**
     * @brief Return the number of StreamProfile objects.
     *
     * @return uint32_t Return the number of StreamProfile objects.
     */
    uint32_t count() const;

    /**
     * @brief Return the StreamProfile object at the specified index.
     *
     * @param index The index of the StreamProfile object to be retrieved. Must be in the range [0, count-1]. Throws an exception if the index is out of range.
     * @return std::shared_ptr<StreamProfile> Return the StreamProfile object.
     */
    const std::shared_ptr<StreamProfile> getProfile(uint32_t index);

    /**
     * @brief Match the corresponding video stream profile based on the passed-in parameters. If multiple Match are found, the first one in the list is
     * returned by default. Throws an exception if no matching profile is found.
     *
     * @param width The width of the stream. Pass OB_WIDTH_ANY if no matching condition is required.
     * @param height The height of the stream. Pass OB_HEIGHT_ANY if no matching condition is required.
     * @param format The type of the stream. Pass OB_FORMAT_ANY if no matching condition is required.
     * @param fps The frame rate of the stream. Pass OB_FPS_ANY if no matching condition is required.
     * @return std::shared_ptr<VideoStreamProfile> Return the matching resolution.
     */
    const std::shared_ptr<VideoStreamProfile> getVideoStreamProfile(int width = OB_WIDTH_ANY, int height = OB_HEIGHT_ANY, OBFormat format = OB_FORMAT_ANY,
                                                                    int fps = OB_FPS_ANY);

    /**
     * @brief Match the corresponding accelerometer stream profile based on the passed-in parameters. If multiple Match are found, the first one in the list
     * is returned by default. Throws an exception if no matching profile is found.
     *
     * @param fullScaleRange The full scale range. Pass 0 if no matching condition is required.
     * @param sampleRate The sampling frequency. Pass 0 if no matching condition is required.
     */
    const std::shared_ptr<AccelStreamProfile> getAccelStreamProfile(OBAccelFullScaleRange fullScaleRange, OBAccelSampleRate sampleRate);

    /**
     * @brief Match the corresponding gyroscope stream profile based on the passed-in parameters. If multiple Match are found, the first one in the list is
     * returned by default. Throws an exception if no matching profile is found.
     *
     * @param fullScaleRange The full scale range. Pass 0 if no matching condition is required.
     * @param sampleRate The sampling frequency. Pass 0 if no matching condition is required.
     */
    const std::shared_ptr<GyroStreamProfile> getGyroStreamProfile(OBGyroFullScaleRange fullScaleRange, OBGyroSampleRate sampleRate);
};

}  // namespace ob
