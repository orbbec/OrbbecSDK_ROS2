/**
 * @file  Filter.hpp
 * @brief This file contains the Filter class, which is the processing unit of the SDK that can perform point cloud generation, format conversion, and other
 * functions.
 */
#pragma once

#include "Types.hpp"
#include <functional>
#include <memory>
#include <map>
#include <string>
#include <iostream>

namespace ob {
class Frame;
class OBFilterList;

/**
 * @brief A callback function that takes a shared pointer to a Frame object as its argument.
 */
typedef std::function<void(std::shared_ptr<Frame>)> FilterCallback;

/**
 * @brief The Filter class is the base class for all filters in the SDK.
 */
class OB_EXTENSION_API Filter : public std::enable_shared_from_this<Filter> {
public:
    Filter();

    Filter(std::shared_ptr<FilterImpl> impl);

    virtual ~Filter() = default;

    /**
     * @brief ReSet the filter, freeing the internal cache, stopping the processing thread, and clearing the pending buffer frame when asynchronous processing
     * is used.
     */
    virtual void reset();

    /**
     * @brief enable the filter
     */
    void enable(bool enable);

    /**
     * @brief Return Enable State
     */
    bool isEnabled();

    /**
     * @brief Processes a frame synchronously.
     *
     * @param frame The frame to be processed.
     * @return std::shared_ptr< Frame > The processed frame.
     */
    virtual std::shared_ptr<Frame> process(std::shared_ptr<Frame> frame);

    /**
     * @brief Pushes the pending frame into the cache for asynchronous processing.
     *
     * @param frame The pending frame. The processing result is returned by the callback function.
     */
    virtual void pushFrame(std::shared_ptr<Frame> frame);

    /**
     * @brief Set the callback function for asynchronous processing.
     *
     * @param callback The processing result callback.
     */
    virtual void setCallBack(FilterCallback callback);

    /**
     * @brief Get the type of filter.
     *
     * @return string The type of filte.
     */
    virtual const char *type();

    /**
     * @brief Check if the runtime type of the filter object is compatible with a given type.
     *
     * @tparam T The given type.
     * @return bool The result.
     */
    template <typename T> bool is();

    /**
     * @brief Convert the filter object to a target type.
     *
     * @tparam T The target type.
     * @return std::shared_ptr<T> The result. If it cannot be converted, an exception will be thrown.
     */
    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>()) {
            throw std::runtime_error("unsupported operation, object's type is not require type");
        }

        return std::static_pointer_cast<T>(shared_from_this());
    }

protected:
    std::shared_ptr<FilterImpl> impl_;
    std::string                 type_;

    friend class OBFilterList;
};

/**
 * @brief The PointCloudFilter class is a subclass of Filter that generates point clouds.
 */
class OB_EXTENSION_API PointCloudFilter : public Filter {
public:
    PointCloudFilter();

    /**
     * @brief Set the point cloud type parameters.
     *
     * @param type The point cloud type: depth point cloud or RGBD point cloud.
     */
    void setCreatePointFormat(OBFormat type);

    /**
     * @brief Set the camera parameters.
     *
     * @param param The camera internal and external parameters.
     */
    void setCameraParam(OBCameraParam param);

    /**
     * @brief Set the frame alignment state that will be input to generate point cloud.
     *
     * @param state The alignment status. True: enable alignment; False: disable alignment.
     */
    void setFrameAlignState(bool state);

    /**
     * @brief Set the point cloud coordinate data zoom factor.
     *
     * @attention Calling this function to set the scale will change the point coordinate scaling factor of the output point cloud frame: posScale = posScale /
     * scale. The point coordinate scaling factor for the output point cloud frame can be obtained via @ref PointsFrame::getPositionValueScale function.
     *
     * @param scale The zoom factor.
     */
    void setPositionDataScaled(float scale);

    /**
     * @brief Set point cloud color data normalization.
     *
     * @param state Whether normalization is required.
     */
    void setColorDataNormalization(bool state);

    /**
     * @brief Set the point cloud coordinate system.
     *
     * @param type The coordinate system type.
     */
    void setCoordinateSystem(OBCoordinateSystemType type);
};

/**
 * @brief The FormatConvertFilter class is a subclass of Filter that performs format conversion.
 */
class OB_EXTENSION_API FormatConvertFilter : public Filter {
public:
    FormatConvertFilter();

    /**
     * @brief Set the format conversion type.
     *
     * @param type The format conversion type.
     */
    void setFormatConvertType(OBConvertFormat type);
};

/**
 * @brief The CompressionFilter class is a subclass of Filter that performs compression.
 */
class OB_EXTENSION_API CompressionFilter : public Filter {
public:
    CompressionFilter();

    /**
     * @brief Set the compression parameters.
     *
     * @param mode The compression mode: OB_COMPRESSION_LOSSLESS or OB_COMPRESSION_LOSSY.
     * @param params The compression parameters. When mode is OB_COMPRESSION_LOSSLESS, params is NULL.
     */
    void setCompressionParams(OBCompressionMode mode, void *params);
};

/**
 * @brief The DecompressionFilter class is a subclass of Filter that performs decompression.
 */
class OB_EXTENSION_API DecompressionFilter : public Filter {
public:
    DecompressionFilter();
};

/**
 * @brief Hole filling filter,the processing performed depends on the selected hole filling mode.
 */
class OB_EXTENSION_API HoleFillingFilter : public Filter {
public:
    HoleFillingFilter();

    /**
     * @brief Set the HoleFillingFilter mode.
     *
     * @param[in] filter A holefilling_filter object.
     * @param mode OBHoleFillingMode, OB_HOLE_FILL_TOP,OB_HOLE_FILL_NEAREST or OB_HOLE_FILL_FAREST.
     */
    void setFilterMode(OBHoleFillingMode mode);

    /**
     * @brief Get the HoleFillingFilter mode.
     *
     * @return OBHoleFillingMode
     */
    OBHoleFillingMode getFilterMode();
};

/**
 * @brief Temporal filter
 */
class OB_EXTENSION_API TemporalFilter : public Filter {
public:
    TemporalFilter();

    /**
     * @brief Get the TemporalFilter diffscale range.
     *
     * @return OBFloatPropertyRange the diffscale value of property range.
     */
    OBFloatPropertyRange getDiffScaleRange();

    /**
     * @brief Set the TemporalFilter diffscale value.
     *
     * @param value diffscale value.
     */
    void setDiffScale(float value);

    /**
     * @brief Get the TemporalFilter weight range.
     *
     * @return OBFloatPropertyRange the weight value of property range.
     */
    OBFloatPropertyRange getWeightRange();

    /**
     * @brief Set the TemporalFilter weight value.
     *
     * @param value weight value.
     */
    void setWeight(float value);
};

/**
 * @brief Spatial advanced filter smooths the image by calculating frame with alpha and delta settings
 * alpha defines the weight of the current pixel for smoothing,
 * delta defines the depth gradient below which the smoothing will occur as number of depth levels.
 */
class OB_EXTENSION_API SpatialAdvancedFilter : public Filter {
public:
    SpatialAdvancedFilter();

    /**
     * @brief Get the spatial advanced filter alpha range.
     *
     * @return OBFloatPropertyRange the alpha value of property range.
     */
    OBFloatPropertyRange getAlphaRange();

    /**
     * @brief Get the spatial advanced filter dispdiff range.
     *
     * @return OBUint16PropertyRange the dispdiff value of property range.
     */
    OBUint16PropertyRange getDispDiffRange();

    /**
     * @brief Get the spatial advanced filter radius range.
     *
     * @return OBUint16PropertyRange the radius value of property range.
     */
    OBUint16PropertyRange getRadiusRange();

    /**
     * @brief Get the spatial advanced filter magnitude range.
     *
     * @return OBIntPropertyRange the magnitude value of property range.
     */
    OBIntPropertyRange getMagnitudeRange();

    /**
     * @brief Get the spatial advanced filter params.
     *
     * @return OBSpatialAdvancedFilterParams
     */
    OBSpatialAdvancedFilterParams getFilterParams();

    /**
     * @brief Set the spatial advanced filter params.
     *
     * @param params OBSpatialAdvancedFilterParams.
     */
    void setFilterParams(OBSpatialAdvancedFilterParams params);
};

/**
 * @brief Spatial fast filter smooths the image by calculating frame with filter window size settings
 */
class OB_EXTENSION_API SpatialFastFilter : public Filter {
public:
    SpatialFastFilter();
    /**
     * @brief Get the spatial fast filter window size range
     *
     * @return OBUint8PropertyRange the windows size value of property range.
     */
    OBUint8PropertyRange getSizeRange();

     /**
     * @brief Get the spatial fast filter params.
     *
     * @return OBSpatialFastFilterParams
     */
    OBSpatialFastFilterParams getFilterParams();
    /**
     * @brief Set the spatial fast filter params.
     *
     * @param params OBSpatialFastFilterParams.
     */
    void setFilterParams(OBSpatialFastFilterParams params);
};

/**
 * @brief Spatial moderate filter smooths the image by calculating frame with filter window size,magnitude and disp diff settings
 */
class OB_EXTENSION_API SpatialModerateFilter : public Filter {
public:
    SpatialModerateFilter();
    /**
     * @brief Get the spatial moderate filter window size range
     *
     * @return OBUint8PropertyRange the windows size value of property range.
     */
    OBUint8PropertyRange          getSizeRange();

    /**
     * @brief Get the spatial moderate filter magnitude range.
     *
     * @return OBUint8PropertyRange the magnitude value of property range.
     */
    OBUint8PropertyRange getMagnitudeRange();
    /**
     * @brief Get the spatial moderate filter dispdiff range.
     *
     * @return OBUint16PropertyRange the dispdiff value of property range.
     */
    OBUint16PropertyRange getDispDiffRange();

     /**
     * @brief Get the spatial moderate filter params.
     *
     * @return OBSpatialModerateFilterParams
     */
    OBSpatialModerateFilterParams getFilterParams();
    /**
     * @brief Set the spatial moderate filter params.
     *
     * @param params OBSpatialModerateFilterParams.
     */
    void setFilterParams(OBSpatialModerateFilterParams params);
};

    /**
 * @brief Depth to disparity or disparity to depth
 */
class OB_EXTENSION_API DisparityTransform : public Filter {
public:

    /**
     * @brief Create a disparity transform.
     * @param depth_to_disparity disparity to depth or depth to disparity Conversion.
     */
    DisparityTransform(bool depth_to_disparity);
};

/**
 * @brief HdrMerge processing block,
 * the processing merges between depth frames with
 * different sub-preset sequence ids.
 */
class OB_EXTENSION_API HdrMerge : public Filter {
public:
    HdrMerge();
};

/**
 * @brief Align for depth to other or other to depth.
 */
class OB_EXTENSION_API Align : public Filter {
public:
    /**
     * @brief Creaet Align filter.
     * @param OBStreamType alignment is performed between a depth image and another image.
     */
    Align(OBStreamType align_to_stream);

    /**
     * @brief Get the stream type to be aligned with.
     *
     * @return OBStreamType The stream type of align to.
     */
    OBStreamType getAlignToStreamType();
};

/**
 * @brief Creates depth Thresholding filter
 * By controlling min and max options on the block
 */
class OB_EXTENSION_API ThresholdFilter : public Filter {
public:
    ThresholdFilter();

    /**
     * @brief Get the threshold filter min range.
     *
     * @return OBIntPropertyRange The range of the threshold filter min.
     */
    OBIntPropertyRange getMinRange();

    /**
     * @brief Get the threshold filter max range.
     *
     * @return OBIntPropertyRange The range of the threshold filter max.
     */
    OBIntPropertyRange getMaxRange();

    /**
     * @brief Get the threshold filter max and min range.
     */
    bool setValueRange(uint16_t min, uint16_t max);
};

/**
 * @brief Create SequenceIdFilter processing block.
 */
class OB_EXTENSION_API SequenceIdFilter : public Filter {
public:
    SequenceIdFilter();

    /**
     * @brief Set the sequenceId filter params.
     *
     * @param sequence id to pass the filter.
     */
    void selectSequenceId(int sequence_id);

    /**
     * @brief Get the current sequence id.
     *
     * @return sequence id to pass the filter.
     */
    int getSelectSequenceId();

    /**
     * @brief Get the current sequence id list.
     *
     * @return OBSequenceIdItem.
     */
    OBSequenceIdItem *getSequenceIdList();

    /**
     * @brief Get the sequenceId list size.
     *
     * @return the size of sequenceId list.
     */
    int getSequenceIdListSize();
};

/**
 * @brief The noise removal filter,removing scattering depth pixels.
 */
class OB_EXTENSION_API NoiseRemovalFilter : public Filter {
public:
    NoiseRemovalFilter();

    /**
     * @brief Set the noise removal filter params.
     *
     * @param[in] params ob_noise_removal_filter_params.
     */
    void setFilterParams(OBNoiseRemovalFilterParams filterParams);

    /**
     * @brief Get the noise removal filter params.
     *
     * @return OBNoiseRemovalFilterParams.
     */
    OBNoiseRemovalFilterParams getFilterParams();

    /**
     * @brief Get the noise removal filter disp diff range.
     * @return OBUint16PropertyRange The disp diff of property range.
     */
    OBUint16PropertyRange getDispDiffRange();

    /**
     * @brief Get the noise removal filter max size range.
     * @return OBUint16PropertyRange The max size of property range.
     */
    OBUint16PropertyRange getMaxSizeRange();
};

/**
 * @brief Decimation filter,reducing complexity by subsampling depth maps and losing depth details.
 */
class OB_EXTENSION_API DecimationFilter : public Filter {
public:
    DecimationFilter();

    /**
     * @brief Set the decimation filter scale value.
     *
     * @param type The decimation filter scale value.
     */
    void setScaleValue(uint8_t value);

    /**
     * @brief Get the decimation filter scale value.
     */
    uint8_t getScaleValue();

    /**
     * @brief Get the property range of the decimation filter scale value.
     */
    OBUint8PropertyRange getScaleRange();
};

/**
 * @brief The edge noise removal filter,removing scattering depth pixels.
 */
class OB_EXTENSION_API EdgeNoiseRemovalFilter : public Filter {
public:
    EdgeNoiseRemovalFilter();

    /**
     * @brief Set the edge noise removal filter params.
     *
     * @param[in] params ob_edge_noise_removal_filter_params.
     */
    void setFilterParams(OBEdgeNoiseRemovalFilterParams filterParams);

    /**
     * @brief Get the edge noise removal filter params.
     *
     * @return OBEdgeNoiseRemovalFilterParams.
     */
    OBEdgeNoiseRemovalFilterParams getFilterParams();

    /**
     * @brief Get the edge noise removal filter margin left th range.
     * @return OBUint16PropertyRange The disp diff of property range.
     */
    OBUint16PropertyRange getMarginLeftThRange();

    /**
     * @brief Get the edge noise removal filter margin right th range.
     * @return OBUint16PropertyRange The max size of property range.
     */
    OBUint16PropertyRange getMarginRightThRange();

    /**
     * @brief Get the edge noise removal filter margin top th range.
     * @return OBUint16PropertyRange The disp diff of property range.
     */
    OBUint16PropertyRange getMarginTopThRange();

    /**
     * @brief Get the edge noise removal filter margin bottom th range.
     * @return OBUint16PropertyRange The max size of property range.
     */
    OBUint16PropertyRange getMarginBottomThRange();
};


// Define the is() template function for the Filter class
template <typename T> bool Filter::is() {
    std::string name = type();
    if(name == "HDRMerge") {
        return typeid(T) == typeid(HdrMerge);
    }
    if(name == "SequenceIdFilter") {
        return typeid(T) == typeid(SequenceIdFilter);
    }
    if(name == "ThresholdFilter") {
        return typeid(T) == typeid(ThresholdFilter);
    }
    if(name == "DisparityTransform") {
        return typeid(T) == typeid(DisparityTransform);
    }
    if(name == "NoiseRemovalFilter") {
        return typeid(T) == typeid(NoiseRemovalFilter);
    }
    if(name == "SpatialAdvancedFilter") {
        return typeid(T) == typeid(SpatialAdvancedFilter);
    }

    if(name == "SpatialFastFilter") {
        return typeid(T) == typeid(SpatialFastFilter);
    }

    if(name == "SpatialModerateFilter") {
        return typeid(T) == typeid(SpatialModerateFilter);
    }

    if(name == "TemporalFilter") {
        return typeid(T) == typeid(TemporalFilter);
    }
    if(name == "HoleFillingFilter") {
        return typeid(T) == typeid(HoleFillingFilter);
    }
    if(name == "DecimationFilter") {
        return typeid(T) == typeid(DecimationFilter);
    }
    if(name == "PointCloudFilter") {
        return typeid(T) == typeid(PointCloudFilter);
    }
    if(name == "CompressionFilter") {
        return typeid(T) == typeid(CompressionFilter);
    }
    if(name == "DecompressionFilter") {
        return typeid(T) == typeid(DecompressionFilter);
    }
    if(name == "FormatConverter") {
        return typeid(T) == typeid(FormatConvertFilter);
    }
    if(name == "Align") {
        return typeid(T) == typeid(Align);
    }
    if(name == "EdgeNoiseRemovalFilter") {
        return typeid(T) == typeid(EdgeNoiseRemovalFilter);
    }
    return false;
}

}  // namespace ob
