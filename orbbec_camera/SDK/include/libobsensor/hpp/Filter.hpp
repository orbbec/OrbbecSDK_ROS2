/**
 * @file  Filter.hpp
 * @brief This file contains the Filter class, which is the processing unit of the SDK that can perform point cloud generation, format conversion, and other
 * functions.
 */
#pragma once

#include "Types.hpp"
#include <functional>
#include <memory>

namespace ob {
class Frame;

/**
 * @brief A callback function that takes a shared pointer to a Frame object as its argument.
 */
typedef std::function<void(std::shared_ptr<Frame>)> FilterCallback;

/**
 * @brief The Filter class is the base class for all filters in the SDK.
 */
class OB_EXTENSION_API Filter {
public:
    Filter();

    virtual ~Filter() = default;

    /**
     * @brief ReSet the filter, freeing the internal cache, stopping the processing thread, and clearing the pending buffer frame when asynchronous processing
     * is used.
     */
    virtual void reset();

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

protected:
    std::shared_ptr<FilterImpl> impl_;
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

}  // namespace ob
