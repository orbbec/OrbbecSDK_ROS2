/**
 * @file Context.hpp
 * @brief The SDK context class, which serves as the entry point to the underlying SDK. It is used to query device lists, handle device callbacks, and set the
 * log level.
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
     * @brief The Context class is a management class that describes the runtime of the SDK. It is responsible for applying and releasing resources for the SDK.
     * The context has the ability to manage multiple devices, enumerate devices, monitor device callbacks, and enable functions such as multi-device
     * synchronization.
     */
    Context(const char *configPath = "");
    virtual ~Context() noexcept;

    /**
     * @brief Queries the enumerated device list.
     *
     * @return std::shared_ptr<DeviceList> A pointer to the device list class.
     */
    std::shared_ptr<DeviceList> queryDeviceList();

    /**
     * @brief enable or disable net device enumeration.
     * @brief after enable, the net device will be discovered automatically and can be retrieved by @ref queryDeviceList. The default state can be set in the
     * configuration file.
     *
     * @attention Net device enumeration by gvcp protocol, if the device is not in the same subnet as the host, it will be discovered but cannot be connected.
     *
     * @param[out] enable true to enable, false to disable
     */
    void enableNetDeviceEnumeration(bool enable);

    /**
     * @brief Creates a network device object.
     *
     * @param address The IP address.
     * @param port The port.
     * @return std::shared_ptr<Device> The created device object.
     */
    std::shared_ptr<Device> createNetDevice(const char *address, uint16_t port);

    /**
     * @brief Changes the IP configuration of a network device.
     *
     * @param deviceUid The device unique ID, which is the network device MAC address. It can be obtained through the @ref DeviceList::uid() function.
     * @param config The new IP configuration.
     */
    void changeNetDeviceIpConfig(const char *deviceUid, const OBNetIpConfig &config);

    using DeviceChangedCallback = std::function<void(std::shared_ptr<DeviceList> removedList, std::shared_ptr<DeviceList> addedList)>;

    /**
     * @brief Set the device plug-in callback function.
     *
     * @param callback The function triggered when the device is plugged and unplugged.
     */
    void setDeviceChangedCallback(DeviceChangedCallback callback);

    /**
     * @brief Activates device clock synchronization to synchronize the clock of the host and all created devices (if supported).
     *
     * @param repeatInterval The interval for auto-repeated synchronization, in milliseconds. If the value is 0, synchronization is performed only once.
     */
    void enableDeviceClockSync(uint64_t repeatInterval);
#define enableMultiDeviceSync enableDeviceClockSync

    /**
     * @brief Frees idle memory from the internal frame memory pool.
     */
    void freeIdleMemory();

    /**
     * @brief Set the level of the global log, which affects both the log level output to the terminal and output to the file.
     *
     * @param severity The log output level.
     */
    static void setLoggerSeverity(OBLogSeverity severity);

    /**
     * @brief Set log output to a file.
     *
     * @param severity The log level output to the file.
     * @param directory The log file output path. If the path is empty, the existing settings will continue to be used (if the existing configuration is also
     * empty, the log will not be output to the file).
     */
    static void setLoggerToFile(OBLogSeverity severity, const char *directory);

    /**
     * @brief Set log output to the terminal.
     *
     * @param severity The log level output to the terminal.
     */
    static void setLoggerToConsole(OBLogSeverity severity);

    /**
     * @brief Log output callback function.
     *
     * @param severity The current callback log level.
     * @param logMsg The log message.
     */
    using LogCallback = std::function<void(OBLogSeverity severity, const char *logMsg)>;

    /**
     * @brief Set the logger to callback.
     *
     * @param severity The callback log level.
     * @param callback The callback function.
     */
    static void setLoggerToCallback(OBLogSeverity severity, LogCallback callback);

    /**
     * @brief Loads a license file.
     *
     * @param filePath The license file path.
     * @param key The decryption key.
     */
    static void loadLicense(const char *filePath, const char *key = OB_DEFAULT_DECRYPT_KEY);

    /**
     * @brief Loads a license from data.
     *
     * @param data The license data.
     * @param dataLen The license data length.
     * @param key The decryption key.
     */
    static void loadLicenseFromData(const char *data, uint32_t dataLen, const char *key = OB_DEFAULT_DECRYPT_KEY);

    /**
     * @brief Set the UVC backend for the specified context
     * This function configures the Universal Video Class (UVC) backend for the given context, allowing the selection of a specific backend for
     * video capture operations.
     *
     * @attention This function is only supported on Linux (ARM) platforms.
     * Some devices, like the Dabai series, do not support V4L2. Therefore, the default backend is LIBUVC. Ensure that the device
     * supports V4L2 before setting it as the backend.
     *
     * @param[in] uvcBackend Specifies the UVC backend to use:
     *                       - `UVC_BACKEND_AUTO`: Automatically selects between
     *                         V4L2 or libuvc based on metadata support.
     *                       - `UVC_BACKEND_LIBUVC`: Forces the use of libuvc.
     *                       - `UVC_BACKEND_V4L2`: Forces the use of V4L2.
     */
    void setUVCBackend(OBUvcBackend uvcBackend);
};
}  // namespace ob
