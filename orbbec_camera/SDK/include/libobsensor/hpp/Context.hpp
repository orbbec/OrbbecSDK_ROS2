// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Context.hpp
 * @brief The SDK context class, which serves as the entry point to the underlying SDK. It is used to query device lists, handle device callbacks, and set the
 * log level.
 *
 */
#pragma once

#include "libobsensor/h/Context.h"
#include "Types.hpp"
#include "Error.hpp"

#include <functional>
#include <memory>

namespace ob {

// forward declarations
class Device;
class DeviceInfo;
class DeviceList;

class Context {
public:
    /**
     * @brief Type definition for the device changed callback function.
     *
     * @param removedList The list of removed devices.
     * @param addedList The list of added devices.
     */
    typedef std::function<void(std::shared_ptr<DeviceList> removedList, std::shared_ptr<DeviceList> addedList)> DeviceChangedCallback;

    /**
     * @brief Type definition for the log output callback function.
     *
     * @param severity The current callback log level.
     * @param logMsg The log message.
     */
    typedef std::function<void(OBLogSeverity severity, const char *logMsg)> LogCallback;

private:
    ob_context           *impl_ = nullptr;
    DeviceChangedCallback deviceChangedCallback_;
    // static LogCallback    logCallback_;

public:
    /**
     * @brief Context constructor.
     * @brief The Context class is a management class that describes the runtime of the SDK. It is responsible for applying and releasing resources for the SDK.
     * The context has the ability to manage multiple devices, enumerate devices, monitor device callbacks, and enable functions such as multi-device
     * synchronization.
     *
     * @param[in] configPath The path to the configuration file. If the path is empty, the default configuration will be used.
     */
    explicit Context(const char *configPath = "") {
        ob_error *error = nullptr;
        impl_           = ob_create_context_with_config(configPath, &error);
        Error::handle(&error);
    }

    /**
     * @brief Context destructor.
     */
    ~Context() noexcept {
        ob_error *error = nullptr;
        ob_delete_context(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Queries the enumerated device list.
     *
     * @return std::shared_ptr<DeviceList> A pointer to the device list class.
     */
    std::shared_ptr<DeviceList> queryDeviceList() const {
        ob_error *error = nullptr;
        auto      list  = ob_query_device_list(impl_, &error);
        Error::handle(&error);
        return std::make_shared<DeviceList>(list);
    }

    /**
     * @brief enable or disable net device enumeration.
     * @brief after enable, the net device will be discovered automatically and can be retrieved by @ref queryDeviceList. The default state can be set in the
     * configuration file.
     *
     * @attention Net device enumeration by gvcp protocol, if the device is not in the same subnet as the host, it will be discovered but cannot be connected.
     *
     * @param[in] enable true to enable, false to disable
     */
    void enableNetDeviceEnumeration(bool enable) const {
        ob_error *error = nullptr;
        ob_enable_net_device_enumeration(impl_, enable, &error);
        Error::handle(&error);
    }

    /**
     * @brief Creates a network device with the specified IP address and port.
     *
     * @param[in] address The IP address, ipv4 only. such as "192.168.1.10"
     * @param[in] port The port number, currently only support 8090
     * @return std::shared_ptr<Device> The created device object.
     */
    std::shared_ptr<Device> createNetDevice(const char *address, uint16_t port) const {
        ob_error *error  = nullptr;
        auto      device = ob_create_net_device(impl_, address, port, &error);
        Error::handle(&error);
        return std::make_shared<Device>(device);
    }

    /**
     * @brief Set the device plug-in callback function.
     * @attention This function supports multiple callbacks. Each call to this function adds a new callback to an internal list.
     *
     * @param callback The function triggered when the device is plugged and unplugged.
     */
    void setDeviceChangedCallback(DeviceChangedCallback callback) {
        deviceChangedCallback_ = callback;
        ob_error *error        = nullptr;
        ob_set_device_changed_callback(impl_, &Context::deviceChangedCallback, this, &error);
        Error::handle(&error);
    }

    /**
     * @brief Activates device clock synchronization to synchronize the clock of the host and all created devices (if supported).
     *
     * @param repeatIntervalMsec The interval for auto-repeated synchronization, in milliseconds. If the value is 0, synchronization is performed only once.
     */
    void enableDeviceClockSync(uint64_t repeatIntervalMsec) const {
        ob_error *error = nullptr;
        ob_enable_device_clock_sync(impl_, repeatIntervalMsec, &error);
        Error::handle(&error);
    }

    /**
     * @brief Frees idle memory from the internal frame memory pool.
     * @brief The SDK includes an internal frame memory pool that caches memory allocated for frames received from devices.
     */
    void freeIdleMemory() const {
        ob_error *error = nullptr;
        ob_free_idle_memory(impl_, &error);
        Error::handle(&error);
    }

    /**
     * @brief For linux, there are two ways to enable the UVC backend: libuvc and v4l2. This function is used to set the backend type.
     * @brief It is effective when the new device is created.
     *
     * @attention This interface is only available for Linux.
     *
     * @param[in] type The backend type to be used.
     */
    void setUvcBackendType(OBUvcBackendType type) const {
        ob_error *error = nullptr;
        ob_set_uvc_backend_type(impl_, type, &error);
        Error::handle(&error);
    }

    /**
     * @brief Set the level of the global log, which affects both the log level output to the console, output to the file and output the user defined
     * callback.
     *
     * @param severity The log output level.
     */
    static void setLoggerSeverity(OBLogSeverity severity) {
        ob_error *error = nullptr;
        ob_set_logger_severity(severity, &error);
        Error::handle(&error);
    }

    /**
     * @brief Set log output to a file.
     *
     * @param severity The log level output to the file.
     * @param directory The log file output path. If the path is empty, the existing settings will continue to be used (if the existing configuration is also
     * empty, the log will not be output to the file).
     */
    static void setLoggerToFile(OBLogSeverity severity, const char *directory) {
        ob_error *error = nullptr;
        ob_set_logger_to_file(severity, directory, &error);
        Error::handle(&error);
    }

    /**
     * @brief Set log output to the console.
     *
     * @param severity The log level output to the console.
     */
    static void setLoggerToConsole(OBLogSeverity severity) {
        ob_error *error = nullptr;
        ob_set_logger_to_console(severity, &error);
        Error::handle(&error);
    }

    /**
     * @brief Set the logger to callback.
     *
     * @param severity The callback log level.
     * @param callback The callback function.
     */
    static void setLoggerToCallback(OBLogSeverity severity, LogCallback callback) {
        ob_error *error           = nullptr;
        Context::getLogCallback() = callback;
        ob_set_logger_to_callback(severity, &Context::logCallback, &Context::getLogCallback(), &error);
        Error::handle(&error);
    }

    /**
     * @brief Set the extensions directory
     * @brief The extensions directory is used to search for dynamic libraries that provide additional functionality to the SDK, such as the Frame filters.
     *
     * @attention Should be called before creating the context and pipeline, otherwise the default extensions directory (./extensions) will be used.
     *
     * @param directory Path to the extensions directory. If the path is empty, the existing settings will continue to be used (if the existing
     */
    static void setExtensionsDirectory(const char *directory) {
        ob_error *error = nullptr;
        ob_set_extensions_directory(directory, &error);
        Error::handle(&error);
    }

private:
    static void deviceChangedCallback(ob_device_list *removedList, ob_device_list *addedList, void *userData) {
        auto ctx = static_cast<Context *>(userData);
        if(ctx && ctx->deviceChangedCallback_) {
            auto removed = std::make_shared<DeviceList>(removedList);
            auto added   = std::make_shared<DeviceList>(addedList);
            ctx->deviceChangedCallback_(removed, added);
        }
    }

    static void logCallback(OBLogSeverity severity, const char *logMsg, void *userData) {
        auto cb = static_cast<LogCallback *>(userData);
        if(cb) {
            (*cb)(severity, logMsg);
        }
    }

    // Lazy initialization of the logcallback_. The purpose is to initialize logcallback_ in .hpp
    static LogCallback &getLogCallback() {
        static LogCallback logCallback_ = nullptr;
        return logCallback_;
    }

// for backward compatibility
#define enableMultiDeviceSync enableDeviceClockSync
};
}  // namespace ob
