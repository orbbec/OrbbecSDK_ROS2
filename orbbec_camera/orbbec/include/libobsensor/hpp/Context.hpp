/**
 * \if English
 * @file Context.hpp
 * @brief The SDK context class, the entry to the underlying SDK, is used to query device lists，handle device callbacks，set the level of the log
 * \else
 * @file Context.hpp
 * @brief SDK上下文环境类，底层SDK的入口，用于获取设备列表，处理设备的回调，日志等级的设置操作
 * \endif
 *
 */
#pragma once

#include "Types.hpp"

#include <functional>
#include <memory>

struct ContextImpl;

namespace ob {
class Device;
class DeviceInfo;
class DeviceList;

class OB_EXTENSION_API Context {
private:
    std::unique_ptr<ContextImpl> impl_;

public:
    /**
     * \if English
     * @brief Context is a management class that describes the runtime of the SDK. It is responsible for the applying and releasing of resources for the SDK.
     * The context has the ability to manage multiple devices, is responsible for enumerating devices, monitoring device callbacks, and enabling functions such as multi-device synchronization. 
     * \else
     * @brief context是描述SDK的runtime一个管理类，负责SDK的资源申请与释放
     * context具备多设备的管理能力，负责枚举设备，监听设备回调，启用多设备同步等功能
     * \endif
     *
     */
    Context(const char *configPath = "");
    virtual ~Context() noexcept;

    /**
     *\if English
     * @brief Query enumerated device list
     *
     * @return std::shared_ptr<DeviceList> returns a pointer to the device list class
     * \else
     * @brief 获取枚举到设备列表
     *
     * @return std::shared_ptr<DeviceList>返回设备列表类的指针
     * \endif
     */
    std::shared_ptr<DeviceList> queryDeviceList();

    using DeviceChangedCallback = std::function<void(std::shared_ptr<DeviceList> removedList, std::shared_ptr<DeviceList> addedList)>;

    /**
     * \if English
     * @brief Set device plug-in callback function
     *
     * @param callback function triggered when the device is plugged and unplugged
     * \else
     * @brief 设置设备插拔回调函数
     *
     * @param callback 设备插拔时触发的回调函数
     * \endif
     */
    void setDeviceChangedCallback(DeviceChangedCallback callback);

    /**
     * \if English
     * @brief Activate the multi-device synchronization function to synchronize the clock of the created device(the device needs support this function)
     *
     * @param repeatInterval  synchronization time interval (unit: ms; if repeatInterval=0, it means that it will only be synchronized once and will not be executed regularly)
     * \else
     * @brief 启动多设备同步功能，同步已创建设备的时钟(需要使用的设备支持该功能)
     *
     * @param repeatInterval 定时同步时间间隔（单位ms；如果repeatInterval=0，表示只同步一次，不再定时执行）
     * \endif
     */
    void enableMultiDeviceSync(uint64_t repeatInterval);

    /**
	 * \if English
	 * @brief Set the level of the global log will affect both the log level output to the terminal and output to the file
     *
     * @param severity log output level
	 * \else
     * @brief 设置全局日志的等级，会同时作用于输出到终端和输出到文件的日志等级
     *
     * @param severity 日志输出等级
	 * \endif
     */
    static void setLoggerSeverity(OBLogSeverity severity);

    /**
	 * \if English
     * @brief Set log output to file
     *
     * @param severity log level output to file
     * @param directory The log file output path. If the path is empty, the existing settings will continue to be used (if the existing configuration is also empty, the log will not be output to the file)
	 * \else
     * @brief 设置日志输出到文件
     *
     * @param severity 输出到文件的日志等级
     * @param directory 日志文件输出路径，如果路径为空，则继续使用已有设置(已有配置也为空则不输出日志到文件)
	 * \endif
     */
    static void setLoggerToFile(OBLogSeverity severity, const char *directory);

    /**
	 * \if English
     * @brief Set log output to terminal
     *
     * @param severity 	log level output to the terminal
	 * \else
     * @brief 设置日志输出到终端
     *
     * @param severity 输出到终端的日志等级
	 * \endif
     */
    static void setLoggerToConsole(OBLogSeverity severity);
};
}  // namespace ob
