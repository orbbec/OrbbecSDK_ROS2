/**
 * \if English
 * @file Pipeline.hpp
 * @brief The SDK's advanced API type can quickly implement switching streaming, frame synchronization, software filtering, and generating point cloud
 * operations. \else
 * @file Pipeline.hpp
 * @brief SDK的高级API类型，可以快速实现开关流，帧同步，软件滤波，生成点云等操作
 * \endif
 */
#pragma once

#include "Types.hpp"

#include <functional>
#include <memory>

struct PipelineImpl;
struct ConfigImpl;

namespace ob {
class FrameSet;
class Frame;
class Device;
class Playback;
class DeviceInfo;
class Config;
class StreamProfile;
class StreamProfileList;

typedef std::function<void(std::shared_ptr<FrameSet> frame)> FrameSetCallback;

class OB_EXTENSION_API Pipeline {
private:
    std::unique_ptr<PipelineImpl> impl_;

public:
    /**
     * \if English
     * @brief Pipeline is a high-level interface for applications, algorithms related RGBD data streams. Pipeline can provide alignment inside and synchronized
     * FrameSet. Pipeline() no parameter version, which opens the first device in the list of devices connected to the OS by default. If the application has
     * obtained the device through the DeviceList, opening the Pipeline() at this time will throw an exception that the device has been created. \else
     * @brief Pipeline 是SDK的高级接口，适用于应用，算法等重点关注RGBD数据流常见，Pipeline在SDK内部可以提供对齐，同步后的FrameSet桢集合
     * 直接方便客户使用。
     * Pipeline()无参数版本，默认打开连接到OS的设备列表中的第一个设备。若应用已经通过DeviceList获取设备，此时打开Pipeline()会抛出设备已经创建异常。
     * 需要开发者捕获异常处理。
     * \endif
     */
    Pipeline();
    /**
     * \if English
     * @brief
     * Pipeline(std::shared_ptr< Device > device ) Function for multi-device operations. Multiple devices need to be obtained through DeviceList, and the device
     * and pipeline are bound through this interface. \else
     * @brief
     * Pipeline(std::shared_ptr< Device > device )函数，适用于多设备操作常见，此时需要通过DeviceList获取多个设备，通过该接口实现device和pipeline绑定。
     * \endif
     */
    Pipeline(std::shared_ptr<Device> device);

    /**
     * \if English
     * @brief Create a pipeline for playback of recorded stream files
     *
     * @param filename Playback file path
     * \else
     * @brief 创建pipeline用于回放录制好的流文件
     *
     * @param filename 回放文件路径
     * \endif
     */
    Pipeline(const char *filename);
    ~Pipeline() noexcept;

    /**
     * \if English
     * @brief Start the pipeline with configuration parameters
     *
     * @param config Parameter configuration of pipeline
     * \else
     * @brief 启动pipeline并配置参数
     *
     * @param config pipeline的参数配置
     * \endif
     */
    void start(std::shared_ptr<Config> config);

    /**
     * \if English
     * @brief Start the pipeline and set the frameset data callback
     *
     * @param config parameter configuration of pipeline
     * @param callback  Set the callback to be triggered when all frame data in the frame set arrives
     * \else
     * @brief 启动pipeline并设置帧集合数据回调
     *
     * @param config pipeline的参数配置
     * @param callback 设置帧集合中的所有帧数据都到达时触发回调
     * \endif
     */
    void start(std::shared_ptr<Config> config, FrameSetCallback callback);

    /**
     * \if English
     * @brief Stop pipeline
     * \else
     * @brief 停止pipeline
     * \endif
     */
    void stop();

    /**
     * \if English
     * @brief Get pipeline configuration parameters
     *
     * @return std::shared_ptr<Config> returns the configured parameters
     * \else
     * @brief 获取pipeline的配置参数
     *
     * @return std::shared_ptr<Config> 返回配置的参数
     * \endif
     */
    std::shared_ptr<Config> getConfig();

    /**
     * \if English
     * @brief Waiting for frame set data
     *
     * @param timeout_ms  Waiting timeout (ms)
     * @return std::shared_ptr<FrameSet> returns the waiting frame set data
     * \else
     * @brief 等待帧集合数据
     *
     * @param timeout_ms 等待超时时间(毫秒)
     * @return std::shared_ptr<FrameSet> 返回等待的帧集合数据
     * \endif
     */
    std::shared_ptr<FrameSet> waitForFrames(uint32_t timeout_ms);

    /**
     * \if English
     * @brief Get device object
     *
     * @return std::shared_ptr<Device> returns the device object
     * \else
     * @brief 获取设备对象
     *
     * @return std::shared_ptr<Device> 返回设备对象
     * \endif
     */
    std::shared_ptr<Device> getDevice();

    /**
     * \if English
     * @brief Get playback object
     *
     * @return std::shared_ptr<Playback> returns the playback object
     * \else
     * @brief 获取回放对象
     *
     * @return std::shared_ptr<Playback> 返回回放对象
     * \endif
     */
    std::shared_ptr<Playback> getPlayback();

    /**
     * \if English
     * @brief Get the stream profile of specified sensor
     *
     * @param sensorType Type of sensor

     * @return std::shared_ptr<StreamProfileList> returns the stream configuration list
     * \else
     * @brief 获取指定传感器的流配置
     *
     * @param sensorType 传感器的类型
     * @return std::shared_ptr<StreamProfileList> 返回流配置列表
     * \endif
     */
    std::shared_ptr<StreamProfileList> getStreamProfileList(OBSensorType sensorType);

    /**
     * \if English
     * @brief Turn on frame synchronization
     * \else
     * @brief 打开帧同步功能
     * \endif
     *
     */
    void enableFrameSync();

    /**
     * \if English
     * @brief Turn off frame synchronization
     * \else
     * @brief 关闭帧同步功能
     * \endif
     */
    void disableFrameSync();

    /**
     * \if English
     * @brief Get camera parameters
     * @attention If D2C is enabled, it will return the camera parameters after D2C, if not, it will return to the default parameters
     *
     * @return  OBCameraParam returns camera parameters
     * \else
     * @brief 获取相机参数
     * @attention 如果开启了D2C将返回D2C后的相机参数，如果没有将返回默认参数
     *
     * @return  OBCameraParam返回相机参数
     * \endif
     */
    OBCameraParam getCameraParam();

    /**
     * \if English
     * @brief Return a list of D2C-enabled depth sensor resolutions corresponding to the input color sensor resolution
     * @param colorProfile Input color sensor resolution
     * @param alignMode Input align mode
     *
     * @return std::shared_ptr<StreamProfileList> returns a list of depth sensor resolutions
     * \else
     * @brief 返回与输入的彩色传感器分辨率对应的支持D2C的深度传感器分辨率列表
     * @param colorProfile 输入的彩色传感器分辨率
     * @param alignMode 输入的对齐模式
     *
     * @return std::shared_ptr<StreamProfileList> 返回深度传感器分辨率列表
     * \endif
     */
    std::shared_ptr<StreamProfileList> getD2CDepthProfileList(std::shared_ptr<StreamProfile> colorProfile, OBAlignMode alignMode);

    /**
	 * \if English
	 * @brief Get valid area between minimum distance and maximum distance after D2C
     *
     * @param minimumDistance minimum working distance
     * @param maximumDistance maximum working distance
     * @return OBRect returns the area information valid after D2C at the working distance
	 * \else
     * @brief 获取D2C后给定工作范围的有效区域
     * 如果需要获取指定距离D2C后的ROI区域，将minimum_distance与maximum_distance设置成一样或者将maximum_distance设置成0
     *
     * @param minimumDistance 最小工作距离
     * @param maximumDistance 最大工作距离
     * @return OBRect 返回在工作距离下D2C后有效的区域信息
     * \endif
     */
    OBRect getD2CValidArea(uint32_t minimumDistance,uint32_t maximumDistance = 0);

    /**
     * \if English
     * @brief Dynamically switch the corresponding config configuration
     *
     * @param config Updated config configuration
     * \else
     * @brief 动态切换对应的config配置
     *
     * @param config 更新后的config配置
     * \endif
     */
    void switchConfig(std::shared_ptr<Config> config);

    /**
     * \if English
     * @brief start recording
     *
     * @param filename Record file name
     * \else
     * @brief 开始录制
     *
     * @param filename 录制文件名
     * \endif
     */
    void startRecord(const char *filename);

    /**
     * \if English
     * @brief Stop recording
     * \else
     * @brief 停止录制
     * \endif
     */
    void stopRecord();
};

class OB_EXTENSION_API Config {
private:
    std::unique_ptr<ConfigImpl> impl_;

public:
    Config();
    ~Config() noexcept;

    /**
     * \if English
     * @brief Configure the stream to be enabled
     *
     * @param streamProfile Stream  configuration
     * \else
     * @brief 设置要打开的流配置
     *
     * @param streamProfile 流的配置
     * \endif
     */
    void enableStream(std::shared_ptr<StreamProfile> streamProfile);
    /**
     * \if English
     * @brief Configure all streams to be enabled
     * \else
     * @brief 设置打开所有的流
     * \endif
     */
    void enableAllStream();
    /**
     * \if English
     * @brief Configure the stream to be disabled
     *
     * @param streamType Stream configuration
     * \else
     * @brief 设置要关闭的流配置
     *
     * @param streamType 流的配置
     * \endif
     */
    void disableStream(OBStreamType streamType);
    /**
     * \if English
     * @brief Configure all streams to be disabled
     * \else
     * @brief 设置关闭所有的流
     * \endif
     */
    void disableAllStream();

    /**
     * \if English
     * @brief Set the alignment mode
     *
     * @param mode Align State Mode
     * \else
     * @brief 设置对齐模式
     *
     * @param mode 对齐状态模式
     * \endif
     */
    void setAlignMode(OBAlignMode mode);

    /**
	 * \if English
	 * @brief Whether scale depth frame after D2C
     *
     * @param enable Scale or not
	 * \else
     * @brief 设置D2C后是否需要缩放深度
     *
     * @param enable 是否需要缩放
	 * \endif
     */
    void setDepthScaleRequire(bool enable);

    /**
	 * \if English
	 * @brief
     *
     * @param d2cTargetWidth  D2C target width
     * @param d2cTargetHeight D2C target height
	 * \else
     * @brief 设置D2C目标分辨率，适用于未使用OrbbecSDK开启Color流，且需要对深度进行D2C的情况
     * 注意:当使用OrbbecSDK开启Color流时，同时使用了此接口设置了D2C目标分辨率时。优先使用开启的Color流的配置进行D2C。
     *
     * @param d2cTargetWidth  D2C目标分辨率宽
     * @param d2cTargetHeight D2C目标分辨率高
	 * \endif
     */
    void setD2CTargetResolution(uint32_t d2cTargetWidth,uint32_t d2cTargetHeight);

    friend class Pipeline;
};

}  // namespace ob
