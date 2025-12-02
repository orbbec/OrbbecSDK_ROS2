// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file  Filter.hpp
 * @brief This file contains the Filter class, which is the processing unit of the SDK that can perform point cloud generation, format conversion, and other
 * functions.
 */
#pragma once

#include "Types.hpp"
#include "Error.hpp"
#include "Frame.hpp"
#include "libobsensor/h/Filter.h"
#include "libobsensor/h/Frame.h"
#include <functional>
#include <memory>
#include <map>
#include <string>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <typeindex>
#include <unordered_map>
#include <cstring>

namespace ob {

/**
 * @brief A callback function that takes a shared pointer to a Frame object as its argument.
 */
typedef std::function<void(std::shared_ptr<Frame>)> FilterCallback;

/**
 * @brief Get the type of a PropertyRange member
 */
template <typename T> struct RangeTraits {
    using valueType = void;
};

template <> struct RangeTraits<OBUint8PropertyRange> {
    using valueType = uint8_t;
};

template <> struct RangeTraits<OBUint16PropertyRange> {
    using valueType = uint16_t;
};

template <> struct RangeTraits<OBIntPropertyRange> {
    using valueType = uint32_t;
};

template <> struct RangeTraits<OBFloatPropertyRange> {
    using valueType = float;
};

/**
 * @brief Get T Property Range
 */
template <typename T> T getPropertyRange(const OBFilterConfigSchemaItem &item, const double cur) {
    // If T type is illegal, T will be void
    using valueType = typename RangeTraits<T>::valueType;
    T range{};
    // Compilate error will be reported here if T is void
    range.cur  = static_cast<valueType>(cur);
    range.def  = static_cast<valueType>(item.def);
    range.max  = static_cast<valueType>(item.max);
    range.min  = static_cast<valueType>(item.min);
    range.step = static_cast<valueType>(item.step);
    return range;
}

/**
 * @brief The Filter class is the base class for all filters in the SDK.
 */
class Filter : public std::enable_shared_from_this<Filter> {
protected:
    ob_filter                            *impl_ = nullptr;
    std::string                           name_;
    FilterCallback                        callback_;
    std::vector<OBFilterConfigSchemaItem> configSchemaVec_;

protected:
    /**
     * @brief Default constructor with nullptr impl, used for derived classes only.
     */
    Filter() = default;

    virtual void init(ob_filter *impl) {
        impl_           = impl;
        ob_error *error = nullptr;
        name_           = ob_filter_get_name(impl_, &error);
        Error::handle(&error);

        auto configSchemaList = ob_filter_get_config_schema_list(impl_, &error);
        Error::handle(&error);

        auto count = ob_filter_config_schema_list_get_count(configSchemaList, &error);
        Error::handle(&error);

        for(uint32_t i = 0; i < count; i++) {
            auto item = ob_filter_config_schema_list_get_item(configSchemaList, i, &error);
            Error::handle(&error);
            configSchemaVec_.emplace_back(item);
        }

        ob_delete_filter_config_schema_list(configSchemaList, &error);
        Error::handle(&error);
    }

public:
    explicit Filter(ob_filter *impl) {
        init(impl);
    }

    virtual ~Filter() noexcept {
        if(impl_ != nullptr) {
            ob_error *error = nullptr;
            ob_delete_filter(impl_, &error);
            Error::handle(&error, false);
        }
    }

    /**
     * @brief Get the Impl object of the filter.
     *
     * @return ob_filter* The Impl object of the filter.
     */
    ob_filter *getImpl() const {
        return impl_;
    }

    /**
     * @brief Get the type of filter.
     *
     * @return string The type of filte.
     */
    virtual const std::string &getName() const {
        return name_;
    }

    /**
     * @brief Reset the filter, freeing the internal cache, stopping the processing thread, and clearing the pending buffer frame when asynchronous processing
     * is used.
     */
    virtual void reset() const {
        ob_error *error = nullptr;
        ob_filter_reset(impl_, &error);
        Error::handle(&error);
    }

    /**
     * @brief enable the filter
     */
    virtual void enable(bool enable) const {
        ob_error *error = nullptr;
        ob_filter_enable(impl_, enable, &error);
        Error::handle(&error);
    }

    /**
     * @brief Return Enable State
     */
    virtual bool isEnabled() const {
        ob_error *error  = nullptr;
        bool      enable = ob_filter_is_enabled(impl_, &error);
        Error::handle(&error);
        return enable;
    }

    /**
     * @brief Processes a frame synchronously.
     *
     * @param[in] frame The frame to be processed.
     *
     * @return std::shared_ptr< Frame > The processed frame.
     */
    virtual std::shared_ptr<Frame> process(std::shared_ptr<const Frame> frame) const {
        ob_error *error  = nullptr;
        auto      result = ob_filter_process(impl_, frame->getImpl(), &error);
        Error::handle(&error);
        if(!result) {
            return nullptr;
        }
        return std::make_shared<Frame>(result);
    }

    /**
     * @brief Pushes the pending frame into the cache for asynchronous processing.
     *
     * @param[in] frame The pending frame. The processing result is returned by the callback function.
     */
    virtual void pushFrame(std::shared_ptr<Frame> frame) const {
        ob_error *error = nullptr;
        ob_filter_push_frame(impl_, frame->getImpl(), &error);
        Error::handle(&error);
    }

    /**
     * @brief Set the callback function for asynchronous processing.
     *
     * @param[in] callback The processing result callback.
     */
    virtual void setCallBack(FilterCallback callback) {
        callback_       = callback;
        ob_error *error = nullptr;
        ob_filter_set_callback(impl_, &Filter::filterCallback, this, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get config schema of the filter
     * @brief The returned string is a csv format string representing the configuration schema of the filter. The format of the string is:
     * `<parameter_name>`, `<parameter_type: "int", "float", "bool">`, `<minimum_value>`, `<maximum_value>`, `<value_step>`, `<default_value>`,
     * `<parameter_description>`
     *
     * @return std::string The config schema of the filter.
     */
    virtual std::string getConfigSchema() const {
        ob_error *error  = nullptr;
        auto      schema = ob_filter_get_config_schema(impl_, &error);
        Error::handle(&error);
        return schema;
    }

    /**
     * @brief Get the Config Schema Vec object
     * @brief The returned vector contains the config schema items. Each item in the vector is an @ref OBFilterConfigSchemaItem object.
     *
     * @return std::vector<OBFilterConfigSchemaItem> The vector of the filter config schema.
     */
    virtual std::vector<OBFilterConfigSchemaItem> getConfigSchemaVec() const {
        return configSchemaVec_;
    }

    /**
     * @brief Set the filter config value by name.
     *
     * @attention The pass into value type is double, witch will be cast to the actual type inside the filter. The actual type can be queried by the filter
     * config schema returned by @ref getConfigSchemaVec.
     *
     * @param[in] configName The name of the config.
     * @param[in] value The value of the config.
     */
    virtual void setConfigValue(const std::string &configName, double value) const {
        ob_error *error = nullptr;
        ob_filter_set_config_value(impl_, configName.c_str(), value, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get the Config Value object by name.
     *
     * @attention The returned value type has been casted to double inside the filter. The actual type can be queried by the filter config schema returned by
     * @ref getConfigSchemaVec.
     *
     * @param[in] configName The name of the config.
     *
     * @return double The value of the config.
     */
    virtual double getConfigValue(const std::string &configName) const {
        ob_error *error = nullptr;
        double    value = ob_filter_get_config_value(impl_, configName.c_str(), &error);
        Error::handle(&error);
        return value;
    }

private:
    static void filterCallback(ob_frame *frame, void *userData) {
        auto filter = static_cast<Filter *>(userData);
        filter->callback_(std::make_shared<Frame>(frame));
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    virtual const char *type() {
        return getName().c_str();
    }

    /**
     * @brief Check if the runtime type of the filter object is compatible with a given type.
     *
     * @tparam T The given type.
     *
     * @return bool The result.
     */
    template <typename T> bool is();

    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>()) {
            throw std::runtime_error("unsupported operation, object's type is not require type");
        }

        return std::static_pointer_cast<T>(shared_from_this());
    }
};

/**
 *  @brief A factory class for creating filters.
 */
class FilterFactory {
public:
    /**
     * @brief Create a filter by name.
     */
    static std::shared_ptr<Filter> createFilter(const std::string &name) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_filter(name.c_str(), &error);
        Error::handle(&error);
        return std::make_shared<Filter>(impl);
    }

    /**
     * @brief Create a private filter by name and activation key.
     * @brief Some private filters require an activation key to be activated, its depends on the vendor of the filter.
     *
     * @param[in] name The name of the filter.
     * @param[in] activationKey The activation key of the filter.
     */
    static std::shared_ptr<Filter> createPrivateFilter(const std::string &name, const std::string &activationKey) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_private_filter(name.c_str(), activationKey.c_str(), &error);
        Error::handle(&error);
        return std::make_shared<Filter>(impl);
    }

    /**
     * @brief Get the vendor specific code of a filter by filter name.
     * @brief A private filter can define its own vendor specific code for specific purposes.
     *
     * @param[in] name The name of the filter.
     *
     * @return std::string The vendor specific code of the filter.
     */
    static std::string getFilterVendorSpecificCode(const std::string &name) {
        ob_error *error = nullptr;
        auto      code  = ob_filter_get_vendor_specific_code(name.c_str(), &error);
        Error::handle(&error);
        return code;
    }
};

/**
 * @brief The PointCloudFilter class is a subclass of Filter that generates point clouds.
 */
class PointCloudFilter : public Filter {
public:
    PointCloudFilter() {
        ob_error *error = nullptr;
        auto      impl  = ob_create_filter("PointCloudFilter", &error);
        Error::handle(&error);
        init(impl);
    }

    virtual ~PointCloudFilter() noexcept override = default;

    /**
     * @brief Set the output pointcloud frame format.
     *
     * @param[in] format The point cloud frame format: OB_FORMAT_POINT or OB_FORMAT_RGB_POINT
     */
    void setCreatePointFormat(OBFormat format) {
        setConfigValue("pointFormat", static_cast<double>(format));
    }

    /**
     * @brief Set the point cloud coordinate data zoom factor.
     *
     * @brief Calling this function to set the scale will change the point coordinate scaling factor of the output point cloud frame, The point coordinate
     * scaling factor for the output point cloud frame can be obtained via @ref PointsFrame::getCoordinateValueScale function.
     *
     * @param[in] factor The scale factor.
     */
    void setCoordinateDataScaled(float factor) {
        setConfigValue("coordinateDataScale", factor);
    }

    /**
     * @brief Set point cloud color data normalization.
     * @brief If normalization is required, the output point cloud frame's color data will be normalized to the range [0, 1].
     *
     * @attention This function only works for when create point format is set to OB_FORMAT_RGB_POINT.
     *
     * @param[in] state Whether normalization is required.
     */
    void setColorDataNormalization(bool state) {
        setConfigValue("colorDataNormalization", state);
    }

    /**
     * @brief Set the point cloud coordinate system.
     *
     * @param[in] type The coordinate system type.
     */
    void setCoordinateSystem(OBCoordinateSystemType type) {
        setConfigValue("coordinateSystemType", static_cast<double>(type));
    }

    /**
     * @brief Set the point cloud decimation factor.
     *        Calling this function to decimation factor will output thedownsampled data of the the cloud frame
     *
     * @param value The decimation factor.
     */
    void setDecimationFactor(int value) {
        setConfigValue("decimate", value);
    }

    /**
     * @brief Get the property range of the decimation factor range.
     */
    OBIntPropertyRange getDecimationFactorRange() {
        OBIntPropertyRange scaleRange{};
        if(configSchemaVec_.size() != 0) {
            const auto &item = configSchemaVec_[5];
            scaleRange       = getPropertyRange<OBIntPropertyRange>(item, getConfigValue("decimate"));
        }
        return scaleRange;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    void setPositionDataScaled(float scale) {
        setCoordinateDataScaled(scale);
    }

    // The following interfaces are deprecated and are retained here for compatibility purposes.
    void setFrameAlignState(bool state) {
        (void)state;  // to complie
    }
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    void setCameraParam(OBCameraParam param) {
        (void)param;
    }
};

/**
 * @brief Align for depth to other or other to depth.
 */
class Align : public Filter {
public:
    Align(OBStreamType alignToStreamType) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_filter("Align", &error);
        Error::handle(&error);
        init(impl);

        setConfigValue("AlignType", static_cast<double>(alignToStreamType));
    }

    virtual ~Align() noexcept override = default;

    OBStreamType getAlignToStreamType() {
        return static_cast<OBStreamType>(static_cast<int>(getConfigValue("AlignType")));
    }

    /**
     * @brief Sets whether the output frame resolution should match the target resolution.
     *        When enabled, the output frame resolution will be adjusted to match (same as) the target resolution.
     *        When disabled, the output frame resolution will match the original resolution while maintaining
     *        the aspect ratio of the target resolution.
     *
     *
     * @param[in] state If true, output frame resolution will match the target resolution; otherwise, it will
     *              maintain the original resolution with the target's aspect ratio.
     */
    void setMatchTargetResolution(bool state) {
        setConfigValue("MatchTargetRes", state);
    }

    /**
     * @brief Set the Align To Stream Profile
     * @brief  It is useful when the align target stream dose not started (without any frame to get intrinsics and extrinsics).
     *
     * @param[in] profile The Align To Stream Profile.
     */
    void setAlignToStreamProfile(std::shared_ptr<const StreamProfile> profile) {
        ob_error *error = nullptr;
        ob_align_filter_set_align_to_stream_profile(impl_, profile->getImpl(), &error);
        Error::handle(&error);
    }
};

/**
 * @brief The FormatConvertFilter class is a subclass of Filter that performs format conversion.
 */
class FormatConvertFilter : public Filter {
public:
    FormatConvertFilter() {
        ob_error *error = nullptr;
        auto      impl  = ob_create_filter("FormatConverter", &error);
        Error::handle(&error);
        init(impl);
    }

    virtual ~FormatConvertFilter() noexcept override = default;

    /**
     * @brief Set the format conversion type.
     *
     * @param[in] type The format conversion type.
     */
    void setFormatConvertType(OBConvertFormat type) {
        setConfigValue("convertType", static_cast<double>(type));
    }
};

/**
 * @brief HdrMerge processing block,
 * the processing merges between depth frames with
 * different sub-preset sequence ids.
 */
class HdrMerge : public Filter {
public:
    HdrMerge() {
        ob_error *error = nullptr;
        auto      impl  = ob_create_filter("HDRMerge", &error);
        Error::handle(&error);
        init(impl);
    }

    virtual ~HdrMerge() noexcept override = default;
};

/**
 * @brief Create SequenceIdFilter processing block.
 */
class SequenceIdFilter : public Filter {
private:
    std::map<float, std::string> sequenceIdList_{ { 0.f, "all" }, { 1.f, "1" } };
    OBSequenceIdItem            *outputSequenceIdList_ = nullptr;

    void initSequenceIdList() {
        outputSequenceIdList_ = new OBSequenceIdItem[sequenceIdList_.size()];

        int i = 0;
        for(const auto &pair: sequenceIdList_) {
            outputSequenceIdList_[i].sequenceSelectId = static_cast<int>(pair.first);
            strncpy(outputSequenceIdList_[i].name, pair.second.c_str(), sizeof(outputSequenceIdList_[i].name) - 1);
            outputSequenceIdList_[i].name[sizeof(outputSequenceIdList_[i].name) - 1] = '\0';
            ++i;
        }
    }

public:
    SequenceIdFilter() {
        ob_error *error = nullptr;
        auto      impl  = ob_create_filter("SequenceIdFilter", &error);
        Error::handle(&error);
        init(impl);
        initSequenceIdList();
    }

    virtual ~SequenceIdFilter() noexcept override {
        if(outputSequenceIdList_) {
            delete[] outputSequenceIdList_;
            outputSequenceIdList_ = nullptr;
        }
    }

    /**
     * @brief Set the sequenceId filter params.
     *
     * @param[in] sequence_id id to pass the filter.
     */
    void selectSequenceId(int sequence_id) {
        setConfigValue("sequenceid", static_cast<double>(sequence_id));
    }

    /**
     * @brief Get the current sequence id.
     *
     * @return sequence id to pass the filter.
     */
    int getSelectSequenceId() {
        return static_cast<int>(getConfigValue("sequenceid"));
    }

    OBSequenceIdItem *getSequenceIdList() {
        return outputSequenceIdList_;
    }

    /**
     * @brief Get the sequenceId list size.
     *
     * @return the size of sequenceId list.
     */
    int getSequenceIdListSize() {
        return static_cast<int>(sequenceIdList_.size());
    }
};

/**
 * @brief Decimation filter, reducing complexity by subsampling depth maps and losing depth details.
 */
class DecimationFilter : public Filter {
public:
    DecimationFilter() {
        ob_error *error = nullptr;
        auto      impl  = ob_create_filter("DecimationFilter", &error);
        Error::handle(&error);
        init(impl);
    }

    virtual ~DecimationFilter() noexcept override = default;

    /**
     * @brief Set the decimation filter scale value.
     *
     * @param[in] value The decimation filter scale value.
     */
    void setScaleValue(uint8_t value) {
        setConfigValue("decimate", static_cast<double>(value));
    }

    /**
     * @brief Get the decimation filter scale value.
     */
    uint8_t getScaleValue() {
        return static_cast<uint8_t>(getConfigValue("decimate"));
    }

    /**
     * @brief Get the property range of the decimation filter scale value.
     */
    OBUint8PropertyRange getScaleRange() {
        OBUint8PropertyRange scaleRange{};
        if(configSchemaVec_.size() != 0) {
            const auto &item = configSchemaVec_[0];
            scaleRange       = getPropertyRange<OBUint8PropertyRange>(item, getConfigValue("decimate"));
        }
        return scaleRange;
    }
};

/**
 * @brief Creates depth Thresholding filter
 * By controlling min and max options on the block
 */
class ThresholdFilter : public Filter {
public:
    ThresholdFilter() {
        ob_error *error = nullptr;
        auto      impl  = ob_create_filter("ThresholdFilter", &error);
        Error::handle(&error);
        init(impl);
    }

    virtual ~ThresholdFilter() noexcept override = default;

    /**
     * @brief Get the threshold filter min range.
     *
     * @return OBIntPropertyRange The range of the threshold filter min.
     */
    OBIntPropertyRange getMinRange() {
        OBIntPropertyRange range{};
        const auto        &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "min") == 0) {
                range = getPropertyRange<OBIntPropertyRange>(item, getConfigValue("min"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Get the threshold filter max range.
     *
     * @return OBIntPropertyRange The range of the threshold filter max.
     */
    OBIntPropertyRange getMaxRange() {
        OBIntPropertyRange range{};
        const auto        &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "max") == 0) {
                range = getPropertyRange<OBIntPropertyRange>(item, getConfigValue("max"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Set the threshold filter max and min range.
     */
    bool setValueRange(uint16_t min, uint16_t max) {
        if(min >= max) {
            return false;
        }
        setConfigValue("min", min);
        setConfigValue("max", max);
        return true;
    }
};

/**
 * @brief Spatial advanced filte smooths the image by calculating frame with alpha and delta settings
 * alpha defines the weight of the current pixel for smoothing,
 * delta defines the depth gradient below which the smoothing will occur as number of depth levels.
 */
class SpatialAdvancedFilter : public Filter {
public:
    SpatialAdvancedFilter(const std::string &activationKey = "") {
        ob_error *error = nullptr;
        auto      impl  = ob_create_private_filter("SpatialAdvancedFilter", activationKey.c_str(), &error);
        Error::handle(&error);
        init(impl);
    }

    virtual ~SpatialAdvancedFilter() noexcept override = default;

    /**
     * @brief Get the spatial advanced filter alpha range.
     *
     * @return OBFloatPropertyRange the alpha value of property range.
     */
    OBFloatPropertyRange getAlphaRange() {
        OBFloatPropertyRange range{};
        const auto          &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "alpha") == 0) {
                range = getPropertyRange<OBFloatPropertyRange>(item, getConfigValue("alpha"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Get the spatial advanced filter dispdiff range.
     *
     * @return OBFloatPropertyRange the dispdiff value of property range.
     */
    OBUint16PropertyRange getDispDiffRange() {
        OBUint16PropertyRange range{};
        const auto           &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "disp_diff") == 0) {
                range = getPropertyRange<OBUint16PropertyRange>(item, getConfigValue("disp_diff"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Get the spatial advanced filter radius range.
     *
     * @return OBFloatPropertyRange the radius value of property range.
     */
    OBUint16PropertyRange getRadiusRange() {
        OBUint16PropertyRange range{};
        const auto           &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "radius") == 0) {
                range = getPropertyRange<OBUint16PropertyRange>(item, getConfigValue("radius"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Get the spatial advanced filter magnitude range.
     *
     * @return OBFloatPropertyRange the magnitude value of property range.
     */
    OBIntPropertyRange getMagnitudeRange() {
        OBIntPropertyRange range{};
        const auto        &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "magnitude") == 0) {
                range = getPropertyRange<OBIntPropertyRange>(item, getConfigValue("magnitude"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Get the spatial advanced filter params.
     *
     * @return OBSpatialAdvancedFilterParams
     */
    OBSpatialAdvancedFilterParams getFilterParams() {
        OBSpatialAdvancedFilterParams params{};
        params.alpha     = static_cast<float>(getConfigValue("alpha"));
        params.disp_diff = static_cast<uint16_t>(getConfigValue("disp_diff"));
        params.magnitude = static_cast<uint8_t>(getConfigValue("magnitude"));
        params.radius    = static_cast<uint16_t>(getConfigValue("radius"));
        return params;
    }

    /**
     * @brief Set the spatial advanced filter params.
     *
     * @param[in] params OBSpatialAdvancedFilterParams.
     */
    void setFilterParams(OBSpatialAdvancedFilterParams params) {
        setConfigValue("alpha", params.alpha);
        setConfigValue("disp_diff", params.disp_diff);
        setConfigValue("magnitude", params.magnitude);
        setConfigValue("radius", params.radius);
    }
};

/**
 * @brief The Spatial Fast Filter utilizes an enhanced median smoothing algorithm,
 * designed to significantly reduce CPU usage and optimize processing efficiency.
 */
class SpatialFastFilter : public Filter {
public:
    SpatialFastFilter(const std::string &activationKey = "") {
        ob_error *error = nullptr;
        auto      impl  = ob_create_private_filter("SpatialFastFilter", activationKey.c_str(), &error);
        Error::handle(&error);
        init(impl);
    }

    virtual ~SpatialFastFilter() noexcept override = default;

    /**
     * @brief Get the spatial fast filter radius range.
     *
     * @return OBIntPropertyRange the radius value of property range.
     */
    OBIntPropertyRange getRadiusRange() {
        OBIntPropertyRange range{};
        const auto        &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "radius") == 0) {
                range = getPropertyRange<OBIntPropertyRange>(item, getConfigValue("radius"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Get the spatial fast filter params.
     *
     * @return OBSpatialFastFilterParams
     */
    OBSpatialFastFilterParams getFilterParams() {
        OBSpatialFastFilterParams params{};
        params.radius = static_cast<uint8_t>(getConfigValue("radius"));
        return params;
    }

    /**
     * @brief Set the spatial fast filter params.
     *
     * @param[in] params OBSpatialFastFilterParams.
     */
    void setFilterParams(OBSpatialFastFilterParams params) {
        setConfigValue("radius", params.radius);
    }
};

/**
 * @brief The Spatial Moderate Filter utilizes an optimized average smoothing algorithm,
 * to achieve a balance between processing speed and the quality of smoothing achieved.
 */
class SpatialModerateFilter : public Filter {
public:
    SpatialModerateFilter(const std::string &activationKey = "") {
        ob_error *error = nullptr;
        auto      impl  = ob_create_private_filter("SpatialModerateFilter", activationKey.c_str(), &error);
        Error::handle(&error);
        init(impl);
    }

    virtual ~SpatialModerateFilter() noexcept override = default;

    /**
     * @brief Get the spatial moderate filter magnitude range.
     *
     * @return OBIntPropertyRange the magnitude value of property range.
     */
    OBIntPropertyRange getMagnitudeRange() {
        OBIntPropertyRange range{};
        const auto        &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "magnitude") == 0) {
                range = getPropertyRange<OBIntPropertyRange>(item, getConfigValue("magnitude"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Get the spatial moderate filter radius range.
     *
     * @return OBIntPropertyRange the radius value of property range.
     */
    OBIntPropertyRange getRadiusRange() {
        OBIntPropertyRange range{};
        const auto        &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "radius") == 0) {
                range = getPropertyRange<OBIntPropertyRange>(item, getConfigValue("radius"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Get the spatial moderate filter disp diff range.
     *
     * @return OBIntPropertyRange the disp diff value of property range.
     */
    OBIntPropertyRange getDispDiffRange() {
        OBIntPropertyRange range{};
        const auto        &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "disp_diff") == 0) {
                range = getPropertyRange<OBIntPropertyRange>(item, getConfigValue("disp_diff"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Get the spatial moderate filter params.
     *
     * @return OBSpatialModerateFilterParams
     */
    OBSpatialModerateFilterParams getFilterParams() {
        OBSpatialModerateFilterParams params{};
        params.magnitude = static_cast<uint8_t>(getConfigValue("magnitude"));
        params.radius    = static_cast<uint8_t>(getConfigValue("radius"));
        params.disp_diff = static_cast<uint16_t>(getConfigValue("disp_diff"));
        return params;
    }

    /**
     * @brief Set the spatial moderate filter params.
     *
     * @param[in] params OBSpatialModerateFilterParams.
     */
    void setFilterParams(OBSpatialModerateFilterParams params) {
        setConfigValue("magnitude", params.magnitude);
        setConfigValue("radius", params.radius);
        setConfigValue("disp_diff", params.disp_diff);
    }
};

/**
 * @brief Hole filling filter,the processing performed depends on the selected hole filling mode.
 */
class HoleFillingFilter : public Filter {
public:
    HoleFillingFilter(const std::string &activationKey = "") {
        ob_error *error = nullptr;
        auto      impl  = ob_create_private_filter("HoleFillingFilter", activationKey.c_str(), &error);
        Error::handle(&error);
        init(impl);
    }

    ~HoleFillingFilter() noexcept override = default;

    /**
     * @brief Set the HoleFillingFilter mode.
     *
     * @param[in] mode OBHoleFillingMode, OB_HOLE_FILL_TOP,OB_HOLE_FILL_NEAREST or OB_HOLE_FILL_FAREST.
     */
    void setFilterMode(OBHoleFillingMode mode) {
        setConfigValue("hole_filling_mode", static_cast<double>(mode));
    }

    /**
     * @brief Get the HoleFillingFilter mode.
     *
     * @return OBHoleFillingMode
     */
    OBHoleFillingMode getFilterMode() {
        return static_cast<OBHoleFillingMode>(static_cast<int>(getConfigValue("hole_filling_mode")));
    }
};

/**
 * @brief The noise removal filter,removing scattering depth pixels.
 */
class NoiseRemovalFilter : public Filter {
public:
    NoiseRemovalFilter(const std::string &activationKey = "") {
        ob_error *error = nullptr;
        auto      impl  = ob_create_private_filter("NoiseRemovalFilter", activationKey.c_str(), &error);
        Error::handle(&error);
        init(impl);
    }

    ~NoiseRemovalFilter() noexcept override = default;

    /**
     * @brief Set the noise removal filter params.
     *
     * @param[in] filterParams ob_noise_removal_filter_params.
     */
    void setFilterParams(OBNoiseRemovalFilterParams filterParams) {
        setConfigValue("max_size", static_cast<double>(filterParams.max_size));
        setConfigValue("min_diff", static_cast<double>(filterParams.disp_diff));
        // todo:set noise remove type
    }

    /**
     * @brief Get the noise removal filter params.
     *
     * @return OBNoiseRemovalFilterParams.
     */
    OBNoiseRemovalFilterParams getFilterParams() {
        OBNoiseRemovalFilterParams param{};
        param.max_size  = static_cast<uint16_t>(getConfigValue("max_size"));
        param.disp_diff = static_cast<uint16_t>(getConfigValue("min_diff"));
        // todo: type is not set
        return param;
    }

    /**
     * @brief Get the noise removal filter disp diff range.
     * @return OBUint16PropertyRange The disp diff of property range.
     */
    OBUint16PropertyRange getDispDiffRange() {
        OBUint16PropertyRange range{};
        const auto           &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "min_diff") == 0) {
                range = getPropertyRange<OBUint16PropertyRange>(item, getConfigValue("min_diff"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Get the noise removal filter max size range.
     * @return OBUint16PropertyRange The max size of property range.
     */
    OBUint16PropertyRange getMaxSizeRange() {
        OBUint16PropertyRange range{};
        const auto           &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "max_size") == 0) {
                range = getPropertyRange<OBUint16PropertyRange>(item, getConfigValue("max_size"));
                break;
            }
        }
        return range;
    }
};

/**
 * @brief Temporal filter
 */
class TemporalFilter : public Filter {
public:
    TemporalFilter(const std::string &activationKey = "") {
        ob_error *error = nullptr;
        auto      impl  = ob_create_private_filter("TemporalFilter", activationKey.c_str(), &error);
        Error::handle(&error);
        init(impl);
    }

    ~TemporalFilter() noexcept override = default;

    /**
     * @brief Get the TemporalFilter diffscale range.
     *
     * @return OBFloatPropertyRange the diffscale value of property range.
     */
    OBFloatPropertyRange getDiffScaleRange() {
        OBFloatPropertyRange range{};
        const auto          &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "diff_scale") == 0) {
                range = getPropertyRange<OBFloatPropertyRange>(item, getConfigValue("diff_scale"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Set the TemporalFilter diffscale value.
     *
     * @param[in] value diffscale value.
     */
    void setDiffScale(float value) {
        setConfigValue("diff_scale", static_cast<double>(value));
    }

    /**
     * @brief Get the TemporalFilter weight range.
     *
     * @return OBFloatPropertyRange the weight value of property range.
     */
    OBFloatPropertyRange getWeightRange() {
        OBFloatPropertyRange range{};
        const auto          &schemaVec = getConfigSchemaVec();
        for(const auto &item: schemaVec) {
            if(strcmp(item.name, "weight") == 0) {
                range = getPropertyRange<OBFloatPropertyRange>(item, getConfigValue("weight"));
                break;
            }
        }
        return range;
    }

    /**
     * @brief Set the TemporalFilter weight value.
     *
     * @param[in] value weight value.
     */
    void setWeight(float value) {
        setConfigValue("weight", static_cast<double>(value));
    }
};

/**
 * @brief Depth to disparity or disparity to depth
 */
class DisparityTransform : public Filter {
public:
    DisparityTransform(const std::string &activationKey = "") {
        ob_error *error = nullptr;
        auto      impl  = ob_create_private_filter("DisparityTransform", activationKey.c_str(), &error);
        Error::handle(&error);
        init(impl);
    }

    ~DisparityTransform() noexcept override = default;
};

class OBFilterList {
private:
    ob_filter_list_t *impl_;

public:
    explicit OBFilterList(ob_filter_list_t *impl) : impl_(impl) {}

    ~OBFilterList() noexcept {
        ob_error *error = nullptr;
        ob_delete_filter_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of filters
     *
     * @return uint32_t The number of filters
     */
    uint32_t getCount() const {
        ob_error *error = nullptr;
        auto      count = ob_filter_list_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the Filter object at the specified index
     *
     * @param[in] index The filter index. The range is [0, count-1]. If the index exceeds the range, an exception will be thrown.
     *
     * @return std::shared_ptr<Filter> The filter object.
     */
    std::shared_ptr<Filter> getFilter(uint32_t index) {
        ob_error *error  = nullptr;
        auto      filter = ob_filter_list_get_filter(impl_, index, &error);
        Error::handle(&error);
        return std::make_shared<Filter>(filter);
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    uint32_t count() const {
        return getCount();
    }
};

/**
 * @brief Returns the mapping of filter type names to their corresponding type_index.
 */
inline const std::unordered_map<std::string, std::type_index> &getFilterTypeMap() {
    static const std::unordered_map<std::string, std::type_index> filterTypeMap = {
        { "PointCloudFilter", typeid(PointCloudFilter) },   { "Align", typeid(Align) },
        { "FormatConverter", typeid(FormatConvertFilter) }, { "HDRMerge", typeid(HdrMerge) },
        { "SequenceIdFilter", typeid(SequenceIdFilter) },   { "DecimationFilter", typeid(DecimationFilter) },
        { "ThresholdFilter", typeid(ThresholdFilter) },     { "SpatialAdvancedFilter", typeid(SpatialAdvancedFilter) },
        { "HoleFillingFilter", typeid(HoleFillingFilter) }, { "NoiseRemovalFilter", typeid(NoiseRemovalFilter) },
        { "TemporalFilter", typeid(TemporalFilter) },       { "DisparityTransform", typeid(DisparityTransform) },
        { "SpatialFastFilter", typeid(SpatialFastFilter) }, { "SpatialModerateFilter", typeid(SpatialModerateFilter) },
    };
    return filterTypeMap;
}

/**
 * @brief Define the is() template function for the Filter class
 *
 * @note When adding a new filter class, ensure the filter type map
 *       (see getFilterTypeMap()) is updated accordingly to maintain correct type matching.
 */
template <typename T> bool Filter::is() {
    std::string name = type();

    const auto &filterTypeMap = getFilterTypeMap();
    auto        it            = filterTypeMap.find(name);
    if(it != filterTypeMap.end()) {
        return std::type_index(typeid(T)) == it->second;
    }
    return false;
}

}  // namespace ob
