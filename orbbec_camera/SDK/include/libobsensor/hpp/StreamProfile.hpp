// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file StreamProfile.hpp
 * @brief The stream profile related type is used to get information such as the width, height, frame rate, and format of the stream.
 */
#pragma once

#include "Types.hpp"
#include "libobsensor/h/StreamProfile.h"
#include "libobsensor/h/Error.h"
#include <iostream>
#include <memory>

namespace ob {

class StreamProfile : public std::enable_shared_from_this<StreamProfile> {
protected:
    const ob_stream_profile_t *impl_ = nullptr;

public:
    StreamProfile(StreamProfile &streamProfile)            = delete;
    StreamProfile &operator=(StreamProfile &streamProfile) = delete;

    StreamProfile(StreamProfile &&streamProfile) noexcept : impl_(streamProfile.impl_) {
        streamProfile.impl_ = nullptr;
    }

    StreamProfile &operator=(StreamProfile &&streamProfile) noexcept {
        if(this != &streamProfile) {
            ob_error *error = nullptr;
            ob_delete_stream_profile(impl_, &error);
            Error::handle(&error);
            impl_               = streamProfile.impl_;
            streamProfile.impl_ = nullptr;
        }
        return *this;
    }

    virtual ~StreamProfile() noexcept {
        if(impl_) {
            ob_error *error = nullptr;
            ob_delete_stream_profile(impl_, &error);
            Error::handle(&error);
        }
    }

    const ob_stream_profile *getImpl() const {
        return impl_;
    }

    /**
     * @brief Get the format of the stream
     *
     * @return OBFormat return the format of the stream
     */
    OBFormat getFormat() const {
        ob_error *error  = nullptr;
        auto      format = ob_stream_profile_get_format(impl_, &error);
        Error::handle(&error);
        return format;
    }

    /**
     * @brief Get the type of stream
     *
     * @return OBStreamType return the type of the stream
     */
    OBStreamType getType() const {
        ob_error *error = nullptr;
        auto      type  = ob_stream_profile_get_type(impl_, &error);
        Error::handle(&error);
        return type;
    }

    /**
     * @brief Get the extrinsic parameters from current stream profile to the given target stream profile
     *
     * @return OBExtrinsic Return the extrinsic parameters.
     */
    OBExtrinsic getExtrinsicTo(std::shared_ptr<StreamProfile> target) const {
        ob_error *error     = nullptr;
        auto      extrinsic = ob_stream_profile_get_extrinsic_to(impl_, const_cast<ob_stream_profile_t *>(target->getImpl()), &error);
        Error::handle(&error);
        return extrinsic;
    }

    /**
     * @brief Check if frame object is compatible with the given type
     *
     * @tparam T  Given type
     * @return bool return result
     */
    template <typename T> bool is() const;

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

        return std::dynamic_pointer_cast<T>(shared_from_this());
    }

    /**
     * @brief Converts object type to target type (const version)
     *
     * @tparam T Target type
     * @return std::shared_ptr<T> Return the result. Throws an exception if conversion is not possible.
     */
    template <typename T> std::shared_ptr<const T> as() const {
        if(!is<T>()) {
            throw std::runtime_error("Unsupported operation. Object's type is not the required type.");
        }

        return std::static_pointer_cast<const T>(shared_from_this());
    }

    // The following interfaces are deprecated and are retained here for compatibility purposes.
    OBFormat format() const {
        return getFormat();
    }

    OBStreamType type() const {
        return getType();
    }

protected:
    explicit StreamProfile(const ob_stream_profile_t *impl) : impl_(impl) {}
};

/**
 * @brief Class representing a video stream profile.
 */
class VideoStreamProfile : public StreamProfile {
public:
    explicit VideoStreamProfile(const ob_stream_profile_t *impl) : StreamProfile(impl) {}

    ~VideoStreamProfile() noexcept override = default;

    /**
     * @brief Return the frame rate of the stream.
     *
     * @return uint32_t Return the frame rate of the stream.
     */
    uint32_t getFps() const {
        ob_error *error = nullptr;
        auto      fps   = ob_video_stream_profile_get_fps(impl_, &error);
        Error::handle(&error);
        return fps;
    }

    /**
     * @brief Return the width of the stream.
     *
     * @return uint32_t Return the width of the stream.
     */
    uint32_t getWidth() const {
        ob_error *error = nullptr;
        auto      width = ob_video_stream_profile_get_width(impl_, &error);
        Error::handle(&error);
        return width;
    }

    /**
     * @brief Return the height of the stream.
     *
     * @return uint32_t Return the height of the stream.
     */
    uint32_t getHeight() const {
        ob_error *error  = nullptr;
        auto      height = ob_video_stream_profile_get_height(impl_, &error);
        Error::handle(&error);
        return height;
    }

    /**
     * @brief Get the intrinsic parameters of the stream.
     *
     * @return OBCameraIntrinsic Return the intrinsic parameters.
     */
    OBCameraIntrinsic getIntrinsic() const {
        ob_error *error     = nullptr;
        auto      intrinsic = ob_video_stream_profile_get_intrinsic(impl_, &error);
        Error::handle(&error);
        return intrinsic;
    }

    /**
     * @brief Get the distortion parameters of the stream.
     * @brief Brown distortion model
     *
     * @return OBCameraDistortion Return the distortion parameters.
     */
    OBCameraDistortion getDistortion() const {
        ob_error *error      = nullptr;
        auto      distortion = ob_video_stream_profile_get_distortion(impl_, &error);
        Error::handle(&error);
        return distortion;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    uint32_t fps() const {
        return getFps();
    }

    uint32_t width() const {
        return getWidth();
    }

    uint32_t height() const {
        return getHeight();
    }
};

/**
 * @brief Class representing an accelerometer stream profile.
 */
class AccelStreamProfile : public StreamProfile {
public:
    explicit AccelStreamProfile(const ob_stream_profile_t *impl) : StreamProfile(impl) {}

    ~AccelStreamProfile() noexcept override = default;

    /**
     * @brief Return the full scale range.
     *
     * @return OBAccelFullScaleRange Return the scale range value.
     */
    OBAccelFullScaleRange getFullScaleRange() const {
        ob_error *error          = nullptr;
        auto      fullScaleRange = ob_accel_stream_profile_get_full_scale_range(impl_, &error);
        Error::handle(&error);
        return fullScaleRange;
    }

    /**
     * @brief Return the sampling frequency.
     *
     * @return OBAccelFullScaleRange Return the sampling frequency.
     */
    OBAccelSampleRate getSampleRate() const {
        ob_error *error      = nullptr;
        auto      sampleRate = ob_accel_stream_profile_get_sample_rate(impl_, &error);
        Error::handle(&error);
        return sampleRate;
    }

    /**
     * @brief get the intrinsic parameters of the stream.
     *
     * @return OBAccelIntrinsic Return the intrinsic parameters.
     */
    OBAccelIntrinsic getIntrinsic() const {
        ob_error *error     = nullptr;
        auto      intrinsic = ob_accel_stream_profile_get_intrinsic(impl_, &error);
        Error::handle(&error);
        return intrinsic;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    OBAccelFullScaleRange fullScaleRange() const {
        return getFullScaleRange();
    }

    OBAccelSampleRate sampleRate() const {
        return getSampleRate();
    }
};

/**
 * @brief Class representing a gyroscope stream profile.
 */
class GyroStreamProfile : public StreamProfile {
public:
    explicit GyroStreamProfile(const ob_stream_profile_t *impl) : StreamProfile(impl) {}

    ~GyroStreamProfile() noexcept override = default;

    /**
     * @brief Return the full scale range.
     *
     * @return OBAccelFullScaleRange Return the scale range value.
     */
    OBGyroFullScaleRange getFullScaleRange() const {
        ob_error *error          = nullptr;
        auto      fullScaleRange = ob_gyro_stream_profile_get_full_scale_range(impl_, &error);
        Error::handle(&error);
        return fullScaleRange;
    }

    /**
     * @brief Return the sampling frequency.
     *
     * @return OBAccelFullScaleRange Return the sampling frequency.
     */
    OBGyroSampleRate getSampleRate() const {
        ob_error *error      = nullptr;
        auto      sampleRate = ob_gyro_stream_profile_get_sample_rate(impl_, &error);
        Error::handle(&error);
        return sampleRate;
    }

    /**
     * @brief get the intrinsic parameters of the stream.
     *
     * @return OBGyroIntrinsic Return the intrinsic parameters.
     */
    OBGyroIntrinsic getIntrinsic() const {
        ob_error *error     = nullptr;
        auto      intrinsic = ob_gyro_stream_get_intrinsic(impl_, &error);
        Error::handle(&error);
        return intrinsic;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    OBGyroFullScaleRange fullScaleRange() const {
        return getFullScaleRange();
    }

    OBGyroSampleRate sampleRate() const {
        return getSampleRate();
    }
};

template <typename T> bool StreamProfile::is() const {
    switch(this->getType()) {
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

class StreamProfileFactory {
public:
    static std::shared_ptr<StreamProfile> create(const ob_stream_profile_t *impl) {
        ob_error  *error = nullptr;
        const auto type  = ob_stream_profile_get_type(impl, &error);
        Error::handle(&error);
        switch(type) {
        case OB_STREAM_IR:
        case OB_STREAM_IR_LEFT:
        case OB_STREAM_IR_RIGHT:
        case OB_STREAM_DEPTH:
        case OB_STREAM_COLOR:
        case OB_STREAM_VIDEO:
            return std::make_shared<VideoStreamProfile>(impl);
        case OB_STREAM_ACCEL:
            return std::make_shared<AccelStreamProfile>(impl);
        case OB_STREAM_GYRO:
            return std::make_shared<GyroStreamProfile>(impl);
        default: {
            ob_error *err = ob_create_error(OB_STATUS_ERROR, "Unsupported stream type.", "StreamProfileFactory::create", "", OB_EXCEPTION_TYPE_INVALID_VALUE);
            Error::handle(&err);
            return nullptr;
        }
        }
    }
};

class StreamProfileList {
protected:
    const ob_stream_profile_list_t *impl_;

public:
    explicit StreamProfileList(ob_stream_profile_list_t *impl) : impl_(impl) {}
    ~StreamProfileList() noexcept {
        ob_error *error = nullptr;
        ob_delete_stream_profile_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Return the number of StreamProfile objects.
     *
     * @return uint32_t Return the number of StreamProfile objects.
     */
    uint32_t getCount() const {
        ob_error *error = nullptr;
        auto      count = ob_stream_profile_list_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Return the StreamProfile object at the specified index.
     *
     * @param index The index of the StreamProfile object to be retrieved. Must be in the range [0, count-1]. Throws an exception if the index is out of range.
     * @return std::shared_ptr<StreamProfile> Return the StreamProfile object.
     */
    std::shared_ptr<StreamProfile> getProfile(uint32_t index) const {
        ob_error *error   = nullptr;
        auto      profile = ob_stream_profile_list_get_profile(impl_, index, &error);
        Error::handle(&error);
        return StreamProfileFactory::create(profile);
    }

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
    std::shared_ptr<VideoStreamProfile> getVideoStreamProfile(int width = OB_WIDTH_ANY, int height = OB_HEIGHT_ANY, OBFormat format = OB_FORMAT_ANY,
                                                              int fps = OB_FPS_ANY) const {
        ob_error *error   = nullptr;
        auto      profile = ob_stream_profile_list_get_video_stream_profile(impl_, width, height, format, fps, &error);
        Error::handle(&error);
        auto vsp = StreamProfileFactory::create(profile);
        return vsp->as<VideoStreamProfile>();
    }

    /**
     * @brief Match the corresponding accelerometer stream profile based on the passed-in parameters. If multiple Match are found, the first one in the list
     * is returned by default. Throws an exception if no matching profile is found.
     *
     * @param fullScaleRange The full scale range. Pass 0 if no matching condition is required.
     * @param sampleRate The sampling frequency. Pass 0 if no matching condition is required.
     */
    std::shared_ptr<AccelStreamProfile> getAccelStreamProfile(OBAccelFullScaleRange fullScaleRange, OBAccelSampleRate sampleRate) const {
        ob_error *error   = nullptr;
        auto      profile = ob_stream_profile_list_get_accel_stream_profile(impl_, fullScaleRange, sampleRate, &error);
        Error::handle(&error);
        auto asp = StreamProfileFactory::create(profile);
        return asp->as<AccelStreamProfile>();
    }

    /**
     * @brief Match the corresponding gyroscope stream profile based on the passed-in parameters. If multiple Match are found, the first one in the list is
     * returned by default. Throws an exception if no matching profile is found.
     *
     * @param fullScaleRange The full scale range. Pass 0 if no matching condition is required.
     * @param sampleRate The sampling frequency. Pass 0 if no matching condition is required.
     */
    std::shared_ptr<GyroStreamProfile> getGyroStreamProfile(OBGyroFullScaleRange fullScaleRange, OBGyroSampleRate sampleRate) const {
        ob_error *error   = nullptr;
        auto      profile = ob_stream_profile_list_get_gyro_stream_profile(impl_, fullScaleRange, sampleRate, &error);
        Error::handle(&error);
        auto gsp = StreamProfileFactory::create(profile);
        return gsp->as<GyroStreamProfile>();
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    uint32_t count() const {
        return getCount();
    }
};

}  // namespace ob
