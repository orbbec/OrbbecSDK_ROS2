/**
 * @file Device.hpp
 * @brief Device related types, including operations such as getting and creating a device, setting and obtaining device attributes, and obtaining sensors
 *
 */
#pragma once
#include "Types.hpp"

#include "libobsensor/h/Property.h"

#include <memory>
#include <string>
#include <vector>

struct DeviceImpl;
struct DeviceInfoImpl;
struct DeviceListImpl;
struct CameraParamListImpl;

namespace ob {
class SensorList;
class Context;
class DeviceInfo;
class Sensor;
class CameraParamList;
class OBDepthWorkModeList;

class OB_EXTENSION_API Device {
private:
    std::unique_ptr<DeviceImpl> impl_;

public:
    /**
     * @brief Describe the entity of the RGBD camera, representing a specific model of RGBD camera
     */
    Device(std::unique_ptr<DeviceImpl> impl);
    virtual ~Device() noexcept;

    /**
     * @brief Get device information
     *
     * @return std::shared_ptr<DeviceInfo> return device information
     */
    std::shared_ptr<DeviceInfo> getDeviceInfo();

    /**
     * @brief Get device sensor list
     *
     * @return std::shared_ptr<SensorList> return the sensor list
     */
    std::shared_ptr<SensorList> getSensorList();

    /**
     * @brief Get specific type of sensor
     * if device not open, SDK will automatically open the connected device and return to the instance
     *
     * @return std::shared_ptr<Sensor> return the sensor example, if the device does not have the device,return nullptr
     */
    std::shared_ptr<Sensor> getSensor(OBSensorType type);

    /**
     * @brief Set int type of device property
     *
     * @param propertyId Property id
     * @param property Property to be set
     */
    void setIntProperty(OBPropertyID propertyId, int32_t property);

    /**
     * @brief Set float type of device property
     *
     * @param propertyId Property id
     * @param property Property to be set
     */
    void setFloatProperty(OBPropertyID propertyId, float property);

    /**
     * @brief Set bool type of device property
     *
     * @param propertyId Property id
     * @param property Property to be set
     */
    void setBoolProperty(OBPropertyID propertyId, bool property);

    /**
     * @brief Get int type of device property
     *
     * @param propertyId Property id
     * @return int32_t Property to get
     */
    int32_t getIntProperty(OBPropertyID propertyId);

    /**
     * @brief Get float type of device property
     *
     * @param propertyId Property id
     * @return float Property to get
     */
    float getFloatProperty(OBPropertyID propertyId);

    /**
     * @brief Get bool type of device property
     *
     * @param propertyId Property id
     * @return bool Property to get
     */
    bool getBoolProperty(OBPropertyID propertyId);

    /**
     * @brief Get int type device property range (including current value and default value)
     *
     * @param propertyId Property id
     * @return OBIntPropertyRange Property range
     */
    OBIntPropertyRange getIntPropertyRange(OBPropertyID propertyId);

    /**
     * @brief Get float type device property range((including current value and default value)
     *
     * @param propertyId Property id
     * @return OBFloatPropertyRange Property range
     */
    OBFloatPropertyRange getFloatPropertyRange(OBPropertyID propertyId);

    /**
     * @brief Get bool type device property range (including current value and default value)
     *
     * @param propertyId The ID of the property
     * @return OBBoolPropertyRange The range of the property
     */
    OBBoolPropertyRange getBoolPropertyRange(OBPropertyID propertyId);

    /**
     * @brief Write to an AHB register
     *
     * @param reg The register to write to
     * @param mask The mask to apply
     * @param value The value to write
     */
    void writeAHB(uint32_t reg, uint32_t mask, uint32_t value);

    /**
     * @brief Read from an AHB register
     *
     * @param reg The register to read from
     * @param mask The mask to apply
     * @param value The value to return
     */
    void readAHB(uint32_t reg, uint32_t mask, uint32_t *value);

    /**
     * @brief Write to an I2C register
     *
     * @param moduleId The ID of the I2C module to write to
     * @param reg The register to write to
     * @param mask The mask to apply
     * @param value The value to write
     */
    void writeI2C(uint32_t moduleId, uint32_t reg, uint32_t mask, uint32_t value);

    /**
     * @brief Read from an I2C register
     *
     * @param moduleId The ID of the I2C module to read from
     * @param reg The register to read from
     * @param mask The mask to apply
     * @param value The value to return
     */
    void readI2C(uint32_t moduleId, uint32_t reg, uint32_t mask, uint32_t *value);

    /**
     * @brief Set the properties for writing to Flash
     *
     * @param offset The offset address in Flash
     * @param data The data to write
     * @param dataSize The size of the data to write
     * @param callback The callback for write progress
     * @param async Whether to execute asynchronously
     */
    void writeFlash(uint32_t offset, const void *data, uint32_t dataSize, SetDataCallback callback, bool async = false);

    /**
     * @brief Read a property from Flash
     *
     * @param offset The offset address in Flash
     * @param dataSize The size of the property to read
     * @param callback The callback for read progress and data
     * @param async Whether to execute asynchronously
     */
    void readFlash(uint32_t offset, uint32_t dataSize, GetDataCallback callback, bool async = false);

    /**
     * @brief Set the customer data type of a device property
     *
     * @param data The data to set
     * @param dataSize The size of the data to set,the maximum length cannot exceed 65532 bytes.
     */
    void writeCustomerData(const void *data, uint32_t dataSize);

    /**
     * @brief Get the customer data type of a device property
     *
     * @param data The property data obtained
     * @param dataSize The size of the data obtained
     */
    void readCustomerData(void *data, uint32_t *dataSize);

    /**
     * @brief Set the raw data type of a device property (with asynchronous callback)
     *
     * @param propertyId The ID of the property
     * @param data The data to set
     * @param dataSize The size of the data to set
     * @param callback The callback for set progress
     * @param async Whether to execute asynchronously
     */
    void setRawData(OBPropertyID propertyId, const void *data, uint32_t dataSize, SetDataCallback callback, bool async = false);

    /**
     * @brief Get the raw data type of a device property (with asynchronous callback)
     *
     * @param propertyId The ID of the property
     * @param callback The callback for getting the data and progress
     * @param async Whether to execute asynchronously
     */
    void getRawData(OBPropertyID propertyId, GetDataCallback callback, bool async = false);

    /**
     * @brief Set the structured data type of a device property
     *
     * @param propertyId The ID of the property
     * @param data The data to set
     * @param dataSize The size of the data to set
     */
    void setStructuredData(OBPropertyID propertyId, const void *data, uint32_t dataSize);

    /**
     * @brief Get the structured data type of a device property
     *
     * @param propertyId The ID of the property
     * @param data The property data obtained
     * @param dataSize The size of the data obtained
     */
    void getStructuredData(OBPropertyID propertyId, void *data, uint32_t *dataSize);

    /**
     * @brief Set the structured data type of a device property (with extended data bundle)
     *
     * @param propertyId The ID of the property
     * @param dataBundle The target data bundle
     * @param callback The callback for setting
     */
    void setStructuredDataExt(OBPropertyID propertyId, std::shared_ptr<OBDataBundle> dataBundle, SetDataCallback callback);

    /**
     * @brief Get the structured data type of a device property (with extended data bundle)
     *
     * @param propertyId The ID of the property
     * @return The data bundle
     */
    std::shared_ptr<OBDataBundle> getStructuredDataExt(OBPropertyID propertyId);

    /**
     * @brief Get the property protocol version
     *
     * @return The protocol version
     */
    OBProtocolVersion getProtocolVersion();

    /**
     * @brief Get the cmdVersion of a property
     *
     * @param propertyId The ID of the property
     * @return The cmdVersion
     */
    OBCmdVersion getCmdVersion(OBPropertyID propertyId);

    /**
     * @brief Get the number of properties supported by the device
     *
     * @return The number of supported properties
     */
    uint32_t getSupportedPropertyCount();

    /**
     * @brief Get the supported properties of the device
     *
     * @param index The index of the property
     * @return The type of supported property
     */
    OBPropertyItem getSupportedProperty(uint32_t index);

    /**
     * @brief Check if a property permission is supported
     *
     * @param propertyId The ID of the property
     * @param permission The read and write permissions to check
     * @return Whether the property permission is supported
     */
    bool isPropertySupported(OBPropertyID propertyId, OBPermissionType permission);

    /**
     * @brief Synchronize the device time (synchronize local system time to device)
     *
     * @return The command (round trip time, rtt)
     */
    uint64_t syncDeviceTime();

    /**
     * @brief Upgrade the device firmware
     *
     * @param filePath Firmware path
     * @param callback  Firmware upgrade progress and status callback
     * @param async    Whether to execute asynchronously
     */
    void deviceUpgrade(const char *filePath, DeviceUpgradeCallback callback, bool async = true);

    /**
     * @brief Send files to the specified path on the device side [Asynchronouscallback]
     *
     * @param filePath Original file path
     * @param dstPath  Accept the save path on the device side
     * @param callback File transfer callback
     * @param async    Whether to execute asynchronously
     */
    void sendFile(const char *filePath, const char *dstPath, SendFileCallback callback, bool async = true);

    /**
     * @brief Get the current state
     * @return OBDeviceState device state information
     */
    OBDeviceState getDeviceState();

    /**
     * @brief Set the device state changed callbacks
     *
     * @param callback The callback function that is triggered when the device status changes (for example, the frame rate is automatically reduced or the
     * stream is closed due to high temperature, etc.)
     */
    void setDeviceStateChangedCallback(DeviceStateChangedCallback callback);

    /**
     * @brief Verify device authorization code
     *
     * @param authCode Authorization code
     * @return bool whether the activation is successful
     */
    bool activateAuthorization(const char *authCode);

    /**
     * @brief Write authorization code
     * @param[in] authCodeStr  Authorization code
     */
    void writeAuthorizationCode(const char *authCodeStr);

    /**
     * @brief Get the original parameter list of camera calibration saved in the device. The parameters in the list do not correspond to the current
     * open-current configuration. You need to select the parameters according to the actual situation, and may need to do scaling, mirroring and other
     * processing. Non-professional users are recommended to use the Pipeline::getCameraParam() interface.
     *
     * @return std::shared_ptr<CameraParamList> camera parameter list
     */
    std::shared_ptr<CameraParamList> getCalibrationCameraParamList();

    /**
     * @brief Get current depth work mode
     *
     * @return ob_depth_work_mode Current depth work mode
     */
    OBDepthWorkMode getCurrentDepthWorkMode();

    /**
     * @brief Switch depth work mode by OBDepthWorkMode. Prefer invoke switchDepthWorkMode(const char *modeName) to switch depth mode
     *        when known the complete name of depth work mode.
     * @param[in] workMode Depth work mode come from ob_depth_work_mode_list which return by ob_device_get_depth_work_mode_list
     */
    OBStatus switchDepthWorkMode(const OBDepthWorkMode &workMode);

    /**
     * @brief Switch depth work mode by work mode name.
     *
     * @param[in] modeName Depth work mode name which equals to OBDepthWorkMode.name
     */
    OBStatus switchDepthWorkMode(const char *modeName);

    /**
     * @brief Request support depth work mode list
     * @return OBDepthWorkModeList list of ob_depth_work_mode
     */
    std::shared_ptr<OBDepthWorkModeList> getDepthWorkModeList();

    /**
     * @brief Device restart
     * @attention The device will be disconnected and reconnected. After the device is disconnected, the access to the Device object interface may be abnormal.
     *   Please delete the object directly and obtain it again after the device is reconnected.
     */
    void reboot();

    /**
     * @brief Device restart delay mode
     * @attention The device will be disconnected and reconnected. After the device is disconnected, the access to the Device object interface may be abnormal.
     *   Please delete the object directly and obtain it again after the device is reconnected.
     * Support devices: Gemini2 L
     *
     * @param[in] delayMs Time unit：ms。delayMs == 0：No delay；delayMs > 0, Delay millisecond connect to host device after reboot
     */
    void reboot(uint32_t delayMs);

    /**
     * @brief get the current device synchronization configuration
     * @brief Device synchronization: including exposure synchronization function and multi-camera synchronization function of different sensors within a single
     * machine
     *
     * @return OBDeviceSyncConfig return the device synchronization configuration
     */
    OBDeviceSyncConfig getSyncConfig();

    /**
     * @brief Set the device synchronization configuration
     * @brief Used to configure the exposure synchronization function and multi-camera synchronization function of different sensors in a single machine
     *
     * @attention Calling this function will directly write the configuration to the device Flash, and it will still take effect after the device restarts. To
     * avoid affecting the Flash lifespan, do not update the configuration frequently.
     *
     * @param deviceSyncConfig Device synchronization configuration
     */
    void setSyncConfig(const OBDeviceSyncConfig &deviceSyncConfig);

    friend class Pipeline;
    friend class Recorder;
};

/**
 * @brief A class describing device information, representing the name, id, serial number and other basic information of an RGBD camera.
 */
class OB_EXTENSION_API DeviceInfo {
private:
    std::unique_ptr<DeviceInfoImpl> impl_;

public:
    DeviceInfo(std::unique_ptr<DeviceInfoImpl> impl);
    virtual ~DeviceInfo() noexcept;

    /**
     * @brief Get device name
     *
     * @return const char * return the device name
     */
    const char *name();

    /**
     * @brief Get the pid of the device
     *
     * @return int return the pid of the device
     */
    int pid();

    /**
     * @brief Get the vid of the device
     *
     * @return int return the vid of the device
     */
    int vid();

    /**
     * @brief Get system assigned uid for distinguishing between different devices
     *
     * @return const char * return the uid of the device
     */
    const char *uid();

    /**
     * @brief Get the serial number of the device
     *
     * @return const char * return the serial number of the device
     */
    const char *serialNumber();

    /**
     * @brief Get the version number of the firmware
     *
     * @return const char* return the version number of the firmware
     */
    const char *firmwareVersion();

    /**
     * @brief Get the USB connection type of the device (DEPRECATED)
     *
     * @return const char* the USB connection type of the device
     */
    DEPRECATED const char *usbType();

    /**
     * @brief Get the connection type of the device
     *
     * @return const char* the connection type of the device
     */
    const char *connectionType();

    /**
     * @brief Get the version number of the hardware
     *
     * @return const char* the version number of the hardware
     */
    const char *hardwareVersion();

    /**
     * @brief Get the minimum version number of the SDK supported by the device
     *
     * @return const char* the minimum SDK version number supported by the device
     */
    const char *supportedMinSdkVersion();

    /**
     * @brief Get the chip type name
     *
     * @return const char* the chip type name
     */
    const char *asicName();

    /**
     * @brief Get the device type
     *
     * @return OBDeviceType the device type
     */
    OBDeviceType deviceType();

    friend class Context;
    friend class DeviceList;
    friend class Pipeline;
};

/**
 * @brief Class representing a list of devices
 */
class OB_EXTENSION_API DeviceList {
private:
    std::unique_ptr<DeviceListImpl> impl_;

public:
    DeviceList(std::unique_ptr<DeviceListImpl> impl);
    ~DeviceList() noexcept;

    /**
     * @brief Get the number of devices in the list
     *
     * @return uint32_t the number of devices in the list
     */
    uint32_t deviceCount();

    /**
     * @brief Get the name of the device at the specified index (DEPRECATED)
     *
     * @param index the index of the device
     * @return int the name of the device
     */
    DEPRECATED const char *name(uint32_t index);

    /**
     * @brief Get the PID of the device at the specified index
     *
     * @param index the index of the device
     * @return int the PID of the device
     */
    int pid(uint32_t index);

    /**
     * @brief Get the VID of the device at the specified index
     *
     * @param index the index of the device
     * @return int the VID of the device
     */
    int vid(uint32_t index);

    /**
     * @brief Get the UID of the device at the specified index
     *
     * @param index the index of the device
     * @return const char* the UID of the device
     */
    const char *uid(uint32_t index);

    /**
     * @brief Get the serial number of the device at the specified index
     *
     * @param index the index of the device
     * @return const char* the serial number of the device
     */
    const char *serialNumber(uint32_t index);

    /**
     * @brief Get the device object at the specified index
     *
     * @attention If the device has already been acquired and created elsewhere, repeated acquisition will throw an exception
     *
     * @param index the index of the device to create
     * @return std::shared_ptr<Device> the device object
     */
    std::shared_ptr<Device> getDevice(uint32_t index);

    /**
     * @brief Get the device object with the specified serial number
     *
     * @attention If the device has already been acquired and created elsewhere, repeated acquisition will throw an exception
     *
     * @param serialNumber the serial number of the device to create
     * @return std::shared_ptr<Device> the device object
     */
    std::shared_ptr<Device> getDeviceBySN(const char *serialNumber);

    /**
     * @brief Get the specified device object from the device list by uid
     * @brief On Linux platform, the uid of the device is composed of bus-port-dev, for example 1-1.2-1. But the SDK will remove the dev number and only keep
     * the bus-port as the uid to create the device, for example 1-1.2, so that we can create a device connected to the specified USB port. Similarly, users can
     * also directly pass in bus-port as uid to create device.
     *
     * @attention If the device has been acquired and created elsewhere, repeated acquisition will throw an exception
     *
     * @param uid The uid of the device to be created
     * @return std::shared_ptr<Device> returns the device object
     */
    std::shared_ptr<Device> getDeviceByUid(const char *uid);
};

/**
 * @brief Class representing a list of camera parameters
 */
class OB_EXTENSION_API CameraParamList {
private:
    std::unique_ptr<CameraParamListImpl> impl_;

public:
    CameraParamList(std::unique_ptr<CameraParamListImpl> impl);
    ~CameraParamList() noexcept;

    /**
     * @brief Get the number of camera parameter groups
     *
     * @return uint32_t the number of camera parameter groups
     */
    uint32_t count();

    /**
     * @brief Get the camera parameters for the specified index
     *
     * @param index the index of the parameter group
     * @return OBCameraParam the corresponding group parameters
     */
    OBCameraParam getCameraParam(uint32_t index);
};

/**
 * @brief Class representing a list of OBDepthWorkMode
 */
class OB_EXTENSION_API OBDepthWorkModeList {
private:
    std::unique_ptr<OBDepthWorkModeListImpl> impl_;

public:
    OBDepthWorkModeList(std::unique_ptr<OBDepthWorkModeListImpl> impl_);
    ~OBDepthWorkModeList();

    /**
     * @brief Get the number of OBDepthWorkMode objects in the list
     *
     * @return uint32_t the number of OBDepthWorkMode objects in the list
     */
    uint32_t count();

    /**
     * @brief Get the OBDepthWorkMode object at the specified index
     *
     * @param index the index of the target OBDepthWorkMode object
     * @return OBDepthWorkMode the OBDepthWorkMode object at the specified index
     */
    OBDepthWorkMode getOBDepthWorkMode(uint32_t index);

    /**
     * @brief Get the name of the depth work mode at the specified index
     *
     * @param index the index of the depth work mode
     * @return std::string the name of the depth work mode
     */
    std::string getName(uint32_t index);

    /**
     * @brief Get the OBDepthWorkMode object at the specified index
     *
     * @param index the index of the target OBDepthWorkMode object
     * @return OBDepthWorkMode the OBDepthWorkMode object at the specified index
     */
    OBDepthWorkMode operator[](uint32_t index);
};

}  // namespace ob
