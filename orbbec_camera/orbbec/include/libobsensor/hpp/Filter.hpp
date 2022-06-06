/**
 * \if English
 * @file  Filter.hpp
 * @brief The processing unit of the SDK can perform point cloud generation, format conversion and other functions.
 * \else
 * @file  Filter.hpp
 * @brief SDK的处理单元，可以进行点云的生成，格式转换等功能
 * \endif
 */
#pragma once

#include "Types.hpp"
#include <functional>
#include <memory>

namespace ob {
class Frame;

typedef std::function<void(std::shared_ptr<Frame>)> FilterCallback;

class OB_EXTENSION_API Filter {
public:
    Filter();

    virtual ~Filter() = default;

    /**
	 * \if English
	 * @brief filter reset, free the internal cache, stop the processing thread and clear the pending buffer frame when asynchronous processing
	 * \else
     * @brief filter重置，释放内部缓存，异步处理时停止处理线程并清空待处理的缓存帧
     * \endif
     */
    virtual void reset();

    /**
	 * \if English
     * @brief Processing frames (synchronous interface)
     *
     * @param frame frame to be processed
     * @return std::shared_ptr< Frame > processed frame
	 * \else
     * @brief 处理帧（同步接口）
     *
     * @param frame 需要处理的frame
     * @return std::shared_ptr< Frame > 处理后的frame
	 * \endif
     */
    virtual std::shared_ptr<Frame> process(std::shared_ptr<Frame> frame);

    /**
	 * \if English
	 * @brief Push the pending frame into the cache (asynchronous callback interface)
     *
     * @param frame The pending frame processing result is returned by the callback function
	 * \else
     * @brief 压入待处理frame到缓存（异步回调接口）
     *
     * @param frame 待处理的frame处理结果通过回调函数返回
	 * \endif
     */

    virtual void pushFrame(std::shared_ptr<Frame> frame);

    /**
	 * \if English
	 * @brief Set the callback function (asynchronous callback interface)
     *
     * @param callback Processing result callback
	 * \else
     * @brief 设置回调函数（异步回调接口）
     *
     * @param callback 处理结果回调
	 * \endif
     */
    virtual void setCallBack(FilterCallback callback);

protected:
    std::shared_ptr<FilterImpl> impl_;
};

class OB_EXTENSION_API PointCloudFilter : public Filter {
public:
    PointCloudFilter();
    /**
	 * \if English
     * @brief Set point cloud type parameters
     *
     * @param type Point cloud type    depth point cloud or RGBD point cloud
	 * \else
     * @brief 设置点云类型参数
     *
     * @param type 点云类型 深度点云或RGBD点云
	 * \endif
     */
    void setCreatePointFormat(OBFormat type);

    /**
	 * \if English
	 * @brief Set camera parameters
     *
     * @param param Camera internal and external parameters
	 * \else
     * @brief 设置相机参数
     *
     * @param param 相机内外参数
	 * \endif
     */
    void setCameraParam(OBCameraParam param);

    /**
	 * \if English
	 * @brief  Set the frame alignment state that will be input to generate point cloud (it needs to be enabled in D2C mode, as the basis for the algorithm to select the set of camera internal parameters)
     *
     * @param state Alignment status, True: enable alignment; False: disable alignment
	 * \else
     * @brief  设置将要输入的用于生成点云的帧对齐状态（D2C模式下需要开启，作为算法选用那组相机内参的依据）
     *
     * @param state 对齐状态，True：开启对齐； False：关闭对齐
	 * \endif
     */
    void setFrameAlignState(bool state);
};

class OB_EXTENSION_API FormatConvertFilter : public Filter {
public:
    FormatConvertFilter();
    /**
	 * \if English
	 * @brief Set format conversion type
     *
     * @param type Format conversion type
	 * \else
     * @brief 设置格式转化类型
     *
     * @param type 格式转化类型
	 * \endif
     */
    void setFormatConvertType(OBConvertFormat type);
};
}  // namespace ob