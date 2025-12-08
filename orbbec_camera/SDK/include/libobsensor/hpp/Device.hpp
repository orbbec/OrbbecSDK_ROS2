// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Device.hpp
 * @brief Device related types, including operations such as getting and creating a device, setting and obtaining device attributes, and obtaining sensors
 *
 */
#pragma once
#include "Types.hpp"

#include "libobsensor/h/Property.h"
#include "libobsensor/h/Device.h"
#include "libobsensor/h/Advanced.h"
#include "libobsensor/hpp/Filter.hpp"
#include "libobsensor/hpp/Sensor.hpp"

#include "Error.hpp"
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

namespace ob {

class DeviceInfo;
class SensorList;
class DevicePresetList;
class OBDepthWorkModeList;
class CameraParamList;
class DeviceFrameInterleaveList;
class PresetResolutionConfigList;

class Device {
public:
    /**
     * @brief Callback function for device firmware update progress
     *
     * @param[in] state The device firmware update status.
     * @param[in] message Status information.
     * @param[in] percent The percentage of the update progress.
     */
    typedef std::function<void(OBFwUpdateState state, const char *message, uint8_t percent)> DeviceFwUpdateCallback;

    /**
     * @brief Callback function for device status updates.
     *
     * @param[in] state The device status.
     * @param[in] message Status information.
     */
    typedef std::function<void(OBDeviceState state, const char *message)> DeviceStateChangedCallback;

protected:
    ob_device                 *impl_ = nullptr;
    DeviceStateChangedCallback deviceStateChangeCallback_;
    DeviceFwUpdateCallback     fwUpdateCallback_;

public:
    /**
     * @brief Describe the entity of the RGBD camera, representing a specific model of RGBD camera
     */
    explicit Device(ob_device_t *impl) : impl_(impl) {}

    Device(Device &&other) noexcept : impl_(other.impl_) {
        other.impl_ = nullptr;
    }

    Device &operator=(Device &&other) noexcept {
        if(this != &other) {
            ob_error *error = nullptr;
            ob_delete_device(impl_, &error);
            Error::handle(&error);
            impl_       = other.impl_;
            other.impl_ = nullptr;
        }
        return *this;
    }

    Device(const Device &)            = delete;
    Device &operator=(const Device &) = delete;

    virtual ~Device() noexcept {
        ob_error *error = nullptr;
        ob_delete_device(impl_, &error);
        Error::handle(&error, false);
    }

    ob_device_t *getImpl() const {
        return impl_;
    }

    /**
     * @brief Get device information
     *
     * @return std::shared_ptr<DeviceInfo> return device information
     */
    std::shared_ptr<DeviceInfo> getDeviceInfo() const {
        ob_error *error = nullptr;
        auto      info  = ob_device_get_device_info(impl_, &error);
        Error::handle(&error);
        return std::make_shared<DeviceInfo>(info);
    }

    /**
     * @brief Check if the extension information is exist
     *
     * @param[in] infoKey The key of the extension information
     *
     * @return bool Whether the extension information exists
     */
    bool isExtensionInfoExist(const std::string &infoKey) const {
        ob_error *error = nullptr;
        auto      exist = ob_device_is_extension_info_exist(impl_, infoKey.c_str(), &error);
        Error::handle(&error);
        return exist;
    }

    /**
     * @brief Get information about extensions obtained from SDK supported by the device
     *
     * @param[in] infoKey The key of the extension information
     *
     * @return const char* Returns extended information about the device
     */
    const char *getExtensionInfo(const std::string &infoKey) const {
        ob_error   *error = nullptr;
        const char *info  = ob_device_get_extension_info(impl_, infoKey.c_str(), &error);
        Error::handle(&error);
        return info;
    }

    /**
     * @brief Get device sensor list
     *
     * @return std::shared_ptr<SensorList> return the sensor list
     */
    std::shared_ptr<SensorList> getSensorList() const {
        ob_error *error = nullptr;
        auto      list  = ob_device_get_sensor_list(impl_, &error);
        Error::handle(&error);
        return std::make_shared<SensorList>(list);
    }

    /**
     * @brief Get specific type of sensor
     * if device not open, SDK will automatically open the connected device and return to the instance
     *
     * @return std::shared_ptr<Sensor> return the sensor example, if the device does not have the device,return nullptr
     */
    std::shared_ptr<Sensor> getSensor(OBSensorType type) const {
        ob_error *error  = nullptr;
        auto      sensor = ob_device_get_sensor(impl_, type, &error);
        Error::handle(&error);
        return std::make_shared<Sensor>(sensor);
    }

    /**
     * @brief Set int type of device property
     *
     * @param[in] propertyId Property id
     * @param[in] value Property value to be set
     */
    void setIntProperty(OBPropertyID propertyId, int32_t value) const {
        ob_error *error = nullptr;
        ob_device_set_int_property(impl_, propertyId, value, &error);
        Error::handle(&error);
    }

    /**
     * @brief Set float type of device property
     *
     * @param[in] propertyId Property id
     * @param[in] value Property value to be set
     */
    void setFloatProperty(OBPropertyID propertyId, float value) const {
        ob_error *error = nullptr;
        ob_device_set_float_property(impl_, propertyId, value, &error);
        Error::handle(&error);
    }

    /**
     * @brief Set bool type of device property
     *
     * @param[in] propertyId Property id
     * @param[in] value Property value to be set
     */
    void setBoolProperty(OBPropertyID propertyId, bool value) const {
        ob_error *error = nullptr;
        ob_device_set_bool_property(impl_, propertyId, value, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get int type of device property
     *
     * @param[in] propertyId Property id
     *
     * @return int32_t Property to get
     */
    int32_t getIntProperty(OBPropertyID propertyId) const {
        ob_error *error = nullptr;
        auto      value = ob_device_get_int_property(impl_, propertyId, &error);
        Error::handle(&error);
        return value;
    }

    /**
     * @brief Get float type of device property
     *
     * @param[in] propertyId Property id
     *
     * @return float Property to get
     */
    float getFloatProperty(OBPropertyID propertyId) const {
        ob_error *error = nullptr;
        auto      value = ob_device_get_float_property(impl_, propertyId, &error);
        Error::handle(&error);
        return value;
    }

    /**
     * @brief Get bool type of device property
     *
     * @param[in] propertyId Property id
     *
     * @return bool Property to get
     */
    bool getBoolProperty(OBPropertyID propertyId) const {
        ob_error *error = nullptr;
        auto      value = ob_device_get_bool_property(impl_, propertyId, &error);
        Error::handle(&error);
        return value;
    }

    /**
     * @brief Get int type device property range (including current value and default value)
     *
     * @param[in] propertyId Property id
     *
     * @return OBIntPropertyRange Property range
     */
    OBIntPropertyRange getIntPropertyRange(OBPropertyID propertyId) const {
        ob_error *error = nullptr;
        auto      range = ob_device_get_int_property_range(impl_, propertyId, &error);
        Error::handle(&error);
        return range;
    }

    /**
     * @brief Get float type device property range((including current value and default value)
     *
     * @param[in] propertyId Property id
     *
     * @return OBFloatPropertyRange Property range
     */
    OBFloatPropertyRange getFloatPropertyRange(OBPropertyID propertyId) const {
        ob_error *error = nullptr;
        auto      range = ob_device_get_float_property_range(impl_, propertyId, &error);
        Error::handle(&error);
        return range;
    }

    /**
     * @brief Get bool type device property range (including current value and default value)
     *
     * @param[in] propertyId The ID of the property
     *
     * @return OBBoolPropertyRange The range of the property
     */
    OBBoolPropertyRange getBoolPropertyRange(OBPropertyID propertyId) const {
        ob_error *error = nullptr;
        auto      range = ob_device_get_bool_property_range(impl_, propertyId, &error);
        Error::handle(&error);
        return range;
    }

    /**
     * @brief Set the structured data type of a device property
     *
     * @param[in] propertyId The ID of the property
     * @param[in] data The data to set
     * @param[in] dataSize The size of the data to set
     */
    void setStructuredData(OBPropertyID propertyId, const uint8_t *data, uint32_t dataSize) const {
        ob_error *error = nullptr;
        ob_device_set_structured_data(impl_, propertyId, data, dataSize, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get the structured data type of a device property
     *
     * @param[in] propertyId The ID of the property
     * @param[out] data The property data obtained
     * @param[out] dataSize The size of the data obtained
     */
    void getStructuredData(OBPropertyID propertyId, uint8_t *data, uint32_t *dataSize) const {
        ob_error *error = nullptr;
        ob_device_get_structured_data(impl_, propertyId, data, dataSize, &error);
        Error::handle(&error);
    }

    /**
     * @brief Set the customer data type of a device property
     *
     * @param[in] data The data to set
     * @param[in] dataSize The size of the data to set,the maximum length cannot exceed 65532 bytes.
     */
    void writeCustomerData(const void *data, uint32_t dataSize) {
        ob_error *error = nullptr;
        ob_device_write_customer_data(impl_, data, dataSize, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get the customer data type of a device property
     *
     * @param[in] data The property data obtained
     * @param[in] dataSize The size of the data obtained
     */
    void readCustomerData(void *data, uint32_t *dataSize) {
        ob_error *error = nullptr;
        ob_device_read_customer_data(impl_, data, dataSize, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get the number of properties supported by the device
     *
     * @return The number of supported properties
     */
    int getSupportedPropertyCount() const {
        ob_error *error = nullptr;
        auto      count = ob_device_get_supported_property_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the supported properties of the device
     *
     * @param[in] index The index of the property
     *
     * @return The type of supported property
     */
    OBPropertyItem getSupportedProperty(uint32_t index) const {
        ob_error *error = nullptr;
        auto      item  = ob_device_get_supported_property_item(impl_, index, &error);
        Error::handle(&error);
        return item;
    }

    /**
     * @brief Check if a property permission is supported
     *
     * @param[in] propertyId The ID of the property
     * @param[in] permission The read and write permissions to check
     *
     * @return Whether the property permission is supported
     */
    bool isPropertySupported(OBPropertyID propertyId, OBPermissionType permission) const {
        ob_error *error  = nullptr;
        auto      result = ob_device_is_property_supported(impl_, propertyId, permission, &error);
        Error::handle(&error);
        return result;
    }

    /**
     * @brief Check if the global timestamp is supported for the device
     *
     * @return Whether the global timestamp is supported
     */
    bool isGlobalTimestampSupported() const {
        ob_error *error  = nullptr;
        auto      result = ob_device_is_global_timestamp_supported(impl_, &error);
        Error::handle(&error);
        return result;
    }

    /**
     * @brief Enable or disable the global timestamp
     *
     * @param[in] enable Whether to enable the global timestamp
     */
    void enableGlobalTimestamp(bool enable) {
        ob_error *error = nullptr;
        ob_device_enable_global_timestamp(impl_, enable, &error);
        Error::handle(&error);
    }

    /**
     * @brief Update the device firmware
     *
     * @param[in] filePath Firmware path
     * @param[in] callback Firmware Update progress and status callback
     * @param[in] async Whether to execute asynchronously
     */
    void updateFirmware(const char *filePath, DeviceFwUpdateCallback callback, bool async = true) {
        ob_error *error   = nullptr;
        fwUpdateCallback_ = callback;
        ob_device_update_firmware(impl_, filePath, &Device::firmwareUpdateCallback, async, this, &error);
        Error::handle(&error);
    }

    /**
     * @brief Update the device firmware from data
     *
     * @param[in] firmwareData Firmware data
     * @param[in] firmwareDataSize Firmware data size
     * @param[in] callback Firmware Update progress and status callback
     * @param[in] async Whether to execute asynchronously
     */
    void updateFirmwareFromData(const uint8_t *firmwareData, uint32_t firmwareDataSize, DeviceFwUpdateCallback callback, bool async = true) {
        ob_error *error   = nullptr;
        fwUpdateCallback_ = callback;
        ob_device_update_firmware_from_data(impl_, firmwareData, firmwareDataSize, &Device::firmwareUpdateCallback, async, this, &error);
        Error::handle(&error);
    }

    /**
     * @brief Update the device optional depth presets
     *
     * @param[in] filePathList A list(2D array) of preset file paths, each up to OB_PATH_MAX characters.
     * @param[in] pathCount The number of the preset file paths.
     * @param[in] callback Preset update progress and status callback
     */
    void updateOptionalDepthPresets(const char filePathList[][OB_PATH_MAX], uint8_t pathCount, DeviceFwUpdateCallback callback) {
        ob_error *error   = nullptr;
        fwUpdateCallback_ = callback;
        ob_device_update_optional_depth_presets(impl_, filePathList, pathCount, &Device::firmwareUpdateCallback, this, &error);
        Error::handle(&error);
    }

    /**
     * @brief Set the device state changed callbacks
     *
     * @param[in] callback The callback function that is triggered when the device status changes (for example, the frame rate is automatically reduced or the
     * stream is closed due to high temperature, etc.)
     */
    void setDeviceStateChangedCallback(DeviceStateChangedCallback callback) {
        ob_error *error            = nullptr;
        deviceStateChangeCallback_ = callback;
        ob_device_set_state_changed_callback(impl_, &Device::deviceStateChangedCallback, this, &error);
        Error::handle(&error);
    }

    static void deviceStateChangedCallback(OBDeviceState state, const char *message, void *userData) {
        auto device = static_cast<Device *>(userData);
        device->deviceStateChangeCallback_(state, message);
    }

    /**
     * @brief Get current depth work mode
     *
     * @return ob_depth_work_mode Current depth work mode
     */
    OBDepthWorkMode getCurrentDepthWorkMode() const {
        ob_error *error = nullptr;
        auto      mode  = ob_device_get_current_depth_work_mode(impl_, &error);
        Error::handle(&error);
        return mode;
    }

    /**
     * @brief Get current depth mode name
     * @brief According the current preset name to return current depth mode name
     *
     * @return const char* return the current depth mode name.
     */
    const char *getCurrentDepthModeName() {
        ob_error *error = nullptr;
        auto      name  = ob_device_get_current_depth_work_mode_name(impl_, &error);
        Error::handle(&error);
        return name;
    }

    /**
     * @brief Switch depth work mode by OBDepthWorkMode. Prefer invoke switchDepthWorkMode(const char *modeName) to switch depth mode
     * when known the complete name of depth work mode.
     * @param[in] workMode Depth work mode come from ob_depth_work_mode_list which return by ob_device_get_depth_work_mode_list
     */
    OBStatus switchDepthWorkMode(const OBDepthWorkMode &workMode) const {
        ob_error *error  = nullptr;
        auto      status = ob_device_switch_depth_work_mode(impl_, &workMode, &error);
        Error::handle(&error);
        return status;
    }

    /**
     * @brief Switch depth work mode by work mode name.
     *
     * @param[in] modeName Depth work mode name which equals to OBDepthWorkMode.name
     */
    OBStatus switchDepthWorkMode(const char *modeName) const {
        ob_error *error  = nullptr;
        auto      status = ob_device_switch_depth_work_mode_by_name(impl_, modeName, &error);
        Error::handle(&error);
        return status;
    }

    /**
     * @brief Request support depth work mode list
     *
     * @return OBDepthWorkModeList list of ob_depth_work_mode
     */
    std::shared_ptr<OBDepthWorkModeList> getDepthWorkModeList() const {
        ob_error *error = nullptr;
        auto      list  = ob_device_get_depth_work_mode_list(impl_, &error);
        Error::handle(&error);
        return std::make_shared<OBDepthWorkModeList>(list);
    }

    /**
     * @brief Device restart
     * @attention The device will be disconnected and reconnected. After the device is disconnected, the access to the Device object interface may be abnormal.
     * Please delete the object directly and obtain it again after the device is reconnected.
     */
    void reboot() const {
        ob_error *error = nullptr;
        ob_device_reboot(impl_, &error);
        Error::handle(&error);
    }

    /**
     * @brief Device restart delay mode
     * @attention The device will be disconnected and reconnected. After the device is disconnected, the access to the Device object interface may be abnormal.
     *   Please delete the object directly and obtain it again after the device is reconnected.
     * Support devices: Gemini2 L
     *
     * @param[in] delayMs Time unit: ms. delayMs == 0: No delay; delayMs > 0, Delay millisecond connect to host device after reboot
     */
    void reboot(uint32_t delayMs) const {
        setIntProperty(OB_PROP_DEVICE_REBOOT_DELAY_INT, delayMs);
        reboot();
    }

    /**
     * @brief Enable or disable the device heartbeat.
     * @brief After enable the device heartbeat, the sdk will start a thread to send heartbeat signal to the device error every 3 seconds.
     *
     * @attention If the device does not receive the heartbeat signal for a long time, it will be disconnected and rebooted.
     *
     * @param[in] enable Whether to enable the device heartbeat.
     */
    void enableHeartbeat(bool enable) const {
        ob_error *error = nullptr;
        ob_device_enable_heartbeat(impl_, enable, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get the supported multi device sync mode bitmap of the device.
     * @brief For example, if the return value is 0b00001100, it means the device supports @ref OB_MULTI_DEVICE_SYNC_MODE_PRIMARY and @ref
     * OB_MULTI_DEVICE_SYNC_MODE_SECONDARY. User can check the supported mode by the code:
     * ```c
     * if(supported_mode_bitmap & OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN){
     * //support OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN
     * }
     * if(supported_mode_bitmap & OB_MULTI_DEVICE_SYNC_MODE_STANDALONE){
     * //support OB_MULTI_DEVICE_SYNC_MODE_STANDALONE
     * }
     * // and so on
     * ```
     * @return uint16_t return the supported multi device sync mode bitmap of the device.
     */
    uint16_t getSupportedMultiDeviceSyncModeBitmap() const {
        ob_error *error = nullptr;
        auto      mode  = ob_device_get_supported_multi_device_sync_mode_bitmap(impl_, &error);
        Error::handle(&error);
        return mode;
    }

    /**
     * @brief set the multi device sync configuration of the device.
     *
     * @param[in] config The multi device sync configuration.
     */
    void setMultiDeviceSyncConfig(const OBMultiDeviceSyncConfig &config) const {
        ob_error *error = nullptr;
        ob_device_set_multi_device_sync_config(impl_, &config, &error);
        Error::handle(&error);
    }

    /**
     * @brief get the multi device sync configuration of the device.
     *
     * @return OBMultiDeviceSyncConfig return the multi device sync configuration of the device.
     */
    OBMultiDeviceSyncConfig getMultiDeviceSyncConfig() const {
        ob_error *error  = nullptr;
        auto      config = ob_device_get_multi_device_sync_config(impl_, &error);
        Error::handle(&error);
        return config;
    }

    /**
     * @brief send the capture command to the device.
     * @brief The device will start one time image capture after receiving the capture command when it is in the @ref
     * OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING
     *
     * @attention The frequency of the user call this function multiplied by the number of frames per trigger should be less than the frame rate of the stream.
     * The number of frames per trigger can be set by @ref OBMultiDeviceSyncConfig::framesPerTrigger.
     * @attention For some models, receive and execute the capture command will have a certain delay and performance consumption, so the frequency of calling
     * this function should not be too high, please refer to the product manual for the specific supported frequency.
     * @attention If the device is not in the @ref OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING mode, device will ignore the capture command.
     */
    void triggerCapture() const {
        ob_error *error = nullptr;
        ob_device_trigger_capture(impl_, &error);
        Error::handle(&error);
    }

    /**
     * @brief set the timestamp reset configuration of the device.
     */
    void setTimestampResetConfig(const OBDeviceTimestampResetConfig &config) const {
        ob_error *error = nullptr;
        ob_device_set_timestamp_reset_config(impl_, &config, &error);
        Error::handle(&error);
    }

    /**
     * @brief get the timestamp reset configuration of the device.
     *
     * @return OBDeviceTimestampResetConfig return the timestamp reset configuration of the device.
     */
    OBDeviceTimestampResetConfig getTimestampResetConfig() const {
        ob_error *error  = nullptr;
        auto      config = ob_device_get_timestamp_reset_config(impl_, &error);
        Error::handle(&error);
        return config;
    }

    /**
     * @brief send the timestamp reset command to the device.
     * @brief The device will reset the timer for calculating the timestamp for output frames to 0 after receiving the timestamp reset command when the
     * timestamp reset function is enabled. The timestamp reset function can be enabled by call @ref ob_device_set_timestamp_reset_config.
     * @brief Before calling this function, user should call @ref ob_device_set_timestamp_reset_config to disable the timestamp reset function (It is not
     * required for some models, but it is still recommended to do so for code compatibility).
     *
     * @attention If the stream of the device is started, the timestamp of the continuous frames output by the stream will jump once after the timestamp reset.
     * @attention Due to the timer of device is not high-accuracy, the timestamp of the continuous frames output by the stream will drift after a long time.
     * User can call this function periodically to reset the timer to avoid the timestamp drift, the recommended interval time is 60 minutes.
     */
    void timestampReset() const {
        ob_error *error = nullptr;
        ob_device_timestamp_reset(impl_, &error);
        Error::handle(&error);
    }

    /**
     * @brief Alias for @ref timestampReset() since it is more accurate.
     */
    inline void timerReset() const {
        timestampReset();
    }

    /**
     * @brief synchronize the timer of the device with the host.
     * @brief After calling this function, the timer of the device will be synchronized with the host. User can call this function to multiple devices to
     * synchronize all timers of the devices.
     *
     * @attention If the stream of the device is started, the timestamp of the continuous frames output by the stream will may jump once after the timer
     * sync.
     * @attention Due to the timer of device is not high-accuracy, the timestamp of the continuous frames output by the stream will drift after a long time.
     * User can call this function periodically to synchronize the timer to avoid the timestamp drift, the recommended interval time is 60 minutes.
     *
     */
    void timerSyncWithHost() const {
        ob_error *error = nullptr;
        ob_device_timer_sync_with_host(impl_, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get current preset name
     * @brief The preset mean a set of parameters or configurations that can be applied to the device to achieve a specific effect or function.
     *
     * @return const char* return the current preset name, it should be one of the preset names returned by @ref getAvailablePresetList.
     */
    const char *getCurrentPresetName() const {
        ob_error   *error = nullptr;
        const char *name  = ob_device_get_current_preset_name(impl_, &error);
        Error::handle(&error);
        return name;
    }

    /**
     * @brief load the preset according to the preset name.
     * @attention After loading the preset, the settings in the preset will set to the device immediately. Therefore, it is recommended to re-read the device
     * settings to update the user program temporarily.
     * @param[in] presetName The preset name to set. The name should be one of the preset names returned by @ref getAvailablePresetList.
     */
    void loadPreset(const char *presetName) const {
        ob_error *error = nullptr;
        ob_device_load_preset(impl_, presetName, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get available preset list
     * @brief The available preset list usually defined by the device manufacturer and restores on the device.
     * @brief User can load the custom preset by calling @ref loadPresetFromJsonFile to append the available preset list.
     *
     * @return DevicePresetList return the available preset list.
     */
    std::shared_ptr<DevicePresetList> getAvailablePresetList() const {
        ob_error *error = nullptr;
        auto      list  = ob_device_get_available_preset_list(impl_, &error);
        Error::handle(&error);
        return std::make_shared<DevicePresetList>(list);
    }

    /**
     * @brief Load custom preset from file.
     * @brief After loading the custom preset, the settings in the custom preset will set to the device immediately.
     * @brief After loading the custom preset, the available preset list will be appended with the custom preset and named as the file name.
     *
     * @attention The user should ensure that the custom preset file is adapted to the device and the settings in the file are valid.
     * @attention It is recommended to re-read the device settings to update the user program temporarily after successfully loading the custom preset.
     *
     * @param[in] filePath The path of the custom preset file.
     */
    void loadPresetFromJsonFile(const char *filePath) const {
        ob_error *error = nullptr;
        ob_device_load_preset_from_json_file(impl_, filePath, &error);
        Error::handle(&error);
    }

    /**
     * @brief Load custom preset from data.
     * @brief After loading the custom preset, the settings in the custom preset will set to the device immediately.
     * @brief After loading the custom preset, the available preset list will be appended with the custom preset and named as the @p presetName.
     *
     * @attention The user should ensure that the custom preset data is adapted to the device and the settings in the data are valid.
     * @attention It is recommended to re-read the device settings to update the user program temporarily after successfully loading the custom preset.
     *
     * @param[in] presetName The name of the custome preset.
     * @param[in] data The custom preset data.
     * @param[in] size The size of the custom preset data.
     */
    void loadPresetFromJsonData(const char *presetName, const uint8_t *data, uint32_t size) {
        ob_error *error = nullptr;
        ob_device_load_preset_from_json_data(impl_, presetName, data, size, &error);
    }

    /**
     * @brief Export current device settings as a preset json data.
     * @brief After exporting the preset, a new preset named as the @p presetName will be added to the available preset list.
     *
     * @attention The memory of the data is allocated by the SDK, and will automatically be released by the SDK.
     * @attention The memory of the data will be reused by the SDK on the next call, so the user should copy the data to a new buffer if it needs to be
     * preserved.
     *
     * @param[in] presetName The name of the custome preset.
     * @param[out] data return the preset json data.
     * @param[out] dataSize return the size of the preset json data.
     */
    void exportSettingsAsPresetJsonData(const char *presetName, const uint8_t **data, uint32_t *dataSize) {
        ob_error *error = nullptr;
        ob_device_export_current_settings_as_preset_json_data(impl_, presetName, data, dataSize, &error);
    }

    /**
     * @brief Export current device settings as a preset json file.
     * @brief The exported preset file can be loaded by calling @ref loadPresetFromJsonFile to restore the device setting.
     * @brief After exporting the preset, a new preset named as the @p filePath will be added to the available preset list.
     *
     * @param[in] filePath The path of the preset file to be exported.
     */
    void exportSettingsAsPresetJsonFile(const char *filePath) const {
        ob_error *error = nullptr;
        ob_device_export_current_settings_as_preset_json_file(impl_, filePath, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get the current device status.
     *
     * @return OBDeviceState The device state information.
     */
    OBDeviceState getDeviceState() {
        OBDeviceState state = {};
        ob_error     *error = nullptr;
        state               = ob_device_get_device_state(impl_, &error);
        return state;
    }

    /**
     * @brief Send data to the device and receive data from the device.
     * @brief This is a factory and debug function, which can be used to send and receive data from the device. The data format is secret and belongs to the
     * device vendor.
     *
     * @attention The send and receive data buffer are managed by the caller, the receive data buffer should be allocated at 1024 bytes or larger.
     *
     * @param[in] sendData The data to be sent to the device.
     * @param[in] sendDataSize The size of the data to be sent to the device.
     * @param[out] receiveData The data received from the device.
     * @param[in,out] receiveDataSize The requeseted size of the data received from the device, and the actual size of the data received from the device.
     */
    void sendAndReceiveData(const uint8_t *sendData, uint32_t sendDataSize, uint8_t *receiveData, uint32_t *receiveDataSize) const {
        ob_error *error = nullptr;
        ob_device_send_and_receive_data(impl_, sendData, sendDataSize, receiveData, receiveDataSize, &error);
        Error::handle(&error);
    }

    /**
     * @brief Check if the device supports the frame interleave feature.
     *
     * @return bool Returns true if the device supports the frame interleave feature.
     */
    bool isFrameInterleaveSupported() const {
        ob_error *error = nullptr;
        bool      ret   = ob_device_is_frame_interleave_supported(impl_, &error);
        Error::handle(&error);
        return ret;
    }

    /**
     * @brief load the frame interleave according to frame interleave name.
     * @param[in] frameInterleaveName The frame interleave name to set. The name should be one of the frame interleave names returned by @ref
     * getAvailableFrameInterleaveList.
     */
    void loadFrameInterleave(const char *frameInterleaveName) const {
        ob_error *error = nullptr;
        ob_device_load_frame_interleave(impl_, frameInterleaveName, &error);
        Error::handle(&error);
    }

    /**
     * @brief Get available frame interleave list
     *
     * @return DeviceFrameInterleaveList return the available frame interleave list.
     */
    std::shared_ptr<DeviceFrameInterleaveList> getAvailableFrameInterleaveList() const {
        ob_error *error = nullptr;
        auto      list  = ob_device_get_available_frame_interleave_list(impl_, &error);
        Error::handle(&error);
        return std::make_shared<DeviceFrameInterleaveList>(list);
    }

    /**
     * @brief Get the available preset resolution config list
     *
     * @return PresetResolutionConfigList return the available preset resolution config list.
     */
    std::shared_ptr<PresetResolutionConfigList> getAvailablePresetResolutionConfigList() const {
        ob_error *error = nullptr;
        auto      list  = ob_device_get_available_preset_resolution_config_list(impl_, &error);
        Error::handle(&error);
        return std::make_shared<PresetResolutionConfigList>(list);
    }

private:
    static void firmwareUpdateCallback(ob_fw_update_state state, const char *message, uint8_t percent, void *userData) {
        auto device = static_cast<Device *>(userData);
        if(device && device->fwUpdateCallback_) {
            device->fwUpdateCallback_(state, message, percent);
        }
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    void deviceUpgrade(const char *filePath, DeviceFwUpdateCallback callback, bool async = true) {
        updateFirmware(filePath, callback, async);
    }

    void deviceUpgradeFromData(const uint8_t *firmwareData, uint32_t firmwareDataSize, DeviceFwUpdateCallback callback, bool async = true) {
        updateFirmwareFromData(firmwareData, firmwareDataSize, callback, async);
    }

    std::shared_ptr<CameraParamList> getCalibrationCameraParamList() {
        ob_error *error = nullptr;
        auto      impl  = ob_device_get_calibration_camera_param_list(impl_, &error);
        Error::handle(&error);
        return std::make_shared<CameraParamList>(impl);
    }

    void loadDepthFilterConfig(const char *filePath) {
        // In order to compile, some high-version compilers will warn that the function parameters are not used.
        (void)filePath;
    }
};

/**
 * @brief A class describing device information, representing the name, id, serial number and other basic information of an RGBD camera.
 */
class DeviceInfo {
private:
    ob_device_info_t *impl_ = nullptr;

public:
    explicit DeviceInfo(ob_device_info_t *impl) : impl_(impl) {}
    ~DeviceInfo() noexcept {
        ob_error *error = nullptr;
        ob_delete_device_info(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get device name
     *
     * @return const char * return the device name
     */
    const char *getName() const {
        ob_error   *error = nullptr;
        const char *name  = ob_device_info_get_name(impl_, &error);
        Error::handle(&error);
        return name;
    }

    /**
     * @brief Get the pid of the device
     *
     * @return int return the pid of the device
     */
    int getPid() const {
        ob_error *error = nullptr;
        int       pid   = ob_device_info_get_pid(impl_, &error);
        Error::handle(&error);
        return pid;
    }

    /**
     * @brief Get the vid of the device
     *
     * @return int return the vid of the device
     */
    int getVid() const {
        ob_error *error = nullptr;
        int       vid   = ob_device_info_get_vid(impl_, &error);
        Error::handle(&error);
        return vid;
    }

    /**
     * @brief Get system assigned uid for distinguishing between different devices
     *
     * @return const char * return the uid of the device
     */
    const char *getUid() const {
        ob_error   *error = nullptr;
        const char *uid   = ob_device_info_get_uid(impl_, &error);
        Error::handle(&error);
        return uid;
    }

    /**
     * @brief Get the serial number of the device
     *
     * @return const char * return the serial number of the device
     */
    const char *getSerialNumber() const {
        ob_error   *error = nullptr;
        const char *sn    = ob_device_info_get_serial_number(impl_, &error);
        Error::handle(&error);
        return sn;
    }

    /**
     * @brief Get the version number of the firmware
     *
     * @return const char* return the version number of the firmware
     */
    const char *getFirmwareVersion() const {
        ob_error   *error   = nullptr;
        const char *version = ob_device_info_get_firmware_version(impl_, &error);
        Error::handle(&error);
        return version;
    }

    /**
     * @brief Get the connection type of the device
     *
     * @return const char* the connection type of the device, currently supports: "USB", "USB1.0", "USB1.1", "USB2.0", "USB2.1", "USB3.0", "USB3.1",
     * "USB3.2", "Ethernet"
     */
    const char *getConnectionType() const {
        ob_error   *error = nullptr;
        const char *type  = ob_device_info_get_connection_type(impl_, &error);
        Error::handle(&error);
        return type;
    }

    /**
     * @brief Get the IP address of the device
     *
     * @attention Only valid for network devices, otherwise it will return "0.0.0.0".
     *
     * @return const char* the IP address of the device, such as "192.168.1.10"
     */
    const char *getIpAddress() const {
        ob_error   *error = nullptr;
        const char *ip    = ob_device_info_get_ip_address(impl_, &error);
        Error::handle(&error);
        return ip;
    }

    /**
     * @brief Get the version number of the hardware
     *
     * @return const char* the version number of the hardware
     */
    const char *getHardwareVersion() const {
        ob_error   *error   = nullptr;
        const char *version = ob_device_info_get_hardware_version(impl_, &error);
        Error::handle(&error);
        return version;
    }

    /**
     * @brief Get the minimum version number of the SDK supported by the device
     *
     * @return const char* the minimum SDK version number supported by the device
     */
    const char *getSupportedMinSdkVersion() const {
        ob_error   *error   = nullptr;
        const char *version = ob_device_info_get_supported_min_sdk_version(impl_, &error);
        Error::handle(&error);
        return version;
    }

    /**
     * @brief Get chip type name
     *
     * @return const char* the chip type name
     */
    const char *getAsicName() const {
        ob_error   *error = nullptr;
        const char *name  = ob_device_info_get_asicName(impl_, &error);
        Error::handle(&error);
        return name;
    }

    /**
     * @brief Get the device type
     *
     * @return OBDeviceType the device type
     */
    OBDeviceType getDeviceType() const {
        ob_error    *error = nullptr;
        OBDeviceType type  = ob_device_info_get_device_type(impl_, &error);
        Error::handle(&error);
        return type;
    }

    /**
     * @brief Get the subnet mask of the device
     *
     * @return const char* the subnet mask of the device, such as "255.255.255.0"
     */
    const char *getDeviceSubnetMask() const {
        ob_error   *error      = nullptr;
        const char *subnetMask = ob_device_info_get_subnet_mask(impl_, &error);
        Error::handle(&error);
        return subnetMask;
    }

    /**
     * @brief Get the gateway address of the device
     *
     * @return const char* the gateway address of the device, such as "192.168.1.1"
     */
    const char *getDeviceGateway() const {
        ob_error   *error      = nullptr;
        const char *subnetMask = ob_device_info_get_gateway(impl_, &error);
        Error::handle(&error);
        return subnetMask;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    const char *getDevicegateway() {
        return getDeviceGateway();
    }

    const char *name() const {
        return getName();
    }

    int pid() const {
        return getPid();
    }

    int vid() const {
        return getVid();
    }

    const char *uid() const {
        return getUid();
    }

    const char *serialNumber() const {
        return getSerialNumber();
    }

    const char *firmwareVersion() const {
        return getFirmwareVersion();
    }

    const char *connectionType() const {
        return getConnectionType();
    }

    const char *ipAddress() const {
        return getIpAddress();
    }

    const char *hardwareVersion() const {
        return getHardwareVersion();
    }

    const char *supportedMinSdkVersion() const {
        return getSupportedMinSdkVersion();
    }

    const char *asicName() const {
        return getAsicName();
    }

    OBDeviceType deviceType() const {
        return getDeviceType();
    }
};

/**
 * @brief Class representing a list of devices
 */
class DeviceList {
private:
    ob_device_list_t *impl_ = nullptr;

public:
    explicit DeviceList(ob_device_list_t *impl) : impl_(impl) {}
    ~DeviceList() noexcept {
        ob_error *error = nullptr;
        ob_delete_device_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of devices in the list
     *
     * @return uint32_t the number of devices in the list
     */
    uint32_t getCount() const {
        ob_error *error = nullptr;
        auto      count = ob_device_list_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the PID of the device at the specified index
     *
     * @param[in] index the index of the device
     *
     * @return int the PID of the device
     */
    int getPid(uint32_t index) const {
        ob_error *error = nullptr;
        auto      pid   = ob_device_list_get_device_pid(impl_, index, &error);
        Error::handle(&error);
        return pid;
    }

    /**
     * @brief Get the VID of the device at the specified index
     *
     * @param[in] index the index of the device
     *
     * @return int the VID of the device
     */
    int getVid(uint32_t index) const {
        ob_error *error = nullptr;
        auto      vid   = ob_device_list_get_device_vid(impl_, index, &error);
        Error::handle(&error);
        return vid;
    }

    /**
     * @brief Get the UID of the device at the specified index
     *
     * @param[in] index the index of the device
     *
     * @return const char* the UID of the device
     */
    const char *getUid(uint32_t index) const {
        ob_error *error = nullptr;
        auto      uid   = ob_device_list_get_device_uid(impl_, index, &error);
        Error::handle(&error);
        return uid;
    }

    /**
     * @brief Get the serial number of the device at the specified index
     *
     * @param[in] index the index of the device
     *
     * @return const char* the serial number of the device
     */
    const char *getSerialNumber(uint32_t index) const {
        ob_error *error = nullptr;
        auto      sn    = ob_device_list_get_device_serial_number(impl_, index, &error);
        Error::handle(&error);
        return sn;
    }

    /**
     * @brief Get the name of the device at the specified index in the device list.
     *
     * This function retrieves the name of the device at the given index in the device list.
     * If an error occurs during the operation, it will be handled by the Error::handle function.
     *
     * @param[in] index The index of the device in the device list.
     *
     * @return const char* The name of the device at the specified index.
     */
    const char *getName(uint32_t index) const {
        ob_error *error = nullptr;
        auto      name  = ob_device_list_get_device_name(impl_, index, &error);
        Error::handle(&error);
        return name;
    }

    /**
     * @brief Get device connection type
     *
     * @param[in] index device index
     *
     * @return const char* returns connection type, currently supports: "USB", "USB1.0", "USB1.1", "USB2.0", "USB2.1", "USB3.0", "USB3.1", "USB3.2", "Ethernet",
     * "GMSL2"
     */
    const char *getConnectionType(uint32_t index) const {
        ob_error *error = nullptr;
        auto      type  = ob_device_list_get_device_connection_type(impl_, index, &error);
        Error::handle(&error);
        return type;
    }

    /**
     * @brief get the ip address of the device at the specified index
     *
     * @attention Only valid for network devices, otherwise it will return "0.0.0.0".
     *
     * @param[in] index the index of the device
     *
     * @return const char* the ip address of the device
     */
    const char *getIpAddress(uint32_t index) const {
        ob_error *error = nullptr;
        auto      ip    = ob_device_list_get_device_ip_address(impl_, index, &error);
        Error::handle(&error);
        return ip;
    }

    /**
     * @brief get the subnet mask of the device at the specified index
     *
     * @attention Only valid for network devices, otherwise it will return "0.0.0.0".
     *
     * @param[in] index the index of the device
     *
     * @return const char* the subnet mask of the device
     */
    const char *getSubnetMask(uint32_t index) const {
        ob_error *error      = nullptr;
        auto      subnetMask = ob_device_list_get_device_subnet_mask(impl_, index, &error);
        Error::handle(&error);
        return subnetMask;
    }

    /**
     * @brief get the gateway of the device at the specified index
     *
     * @attention Only valid for network devices, otherwise it will return "0.0.0.0".
     *
     * @param[in] index the index of the device
     *
     * @return const char* the gateway of the device
     */
    const char *getGateway(uint32_t index) const {
        ob_error *error   = nullptr;
        auto      gateway = ob_device_list_get_device_gateway(impl_, index, &error);
        Error::handle(&error);
        return gateway;
    }

    /**
     * @brief Get the MAC address of the host network interface corresponding to the device at the specified index
     *
     * @attention Only valid for network devices, otherwise it will return "0:0:0:0:0:0".
     *
     * @param[in] index the index of the device
     *
     * @return const char* The MAC address of the host network interface associated with the device.
     */
    const char *getLocalMacAddress(uint32_t index) const {
        ob_error *error = nullptr;
        auto      mac   = ob_device_list_get_device_local_mac(impl_, index, &error);
        Error::handle(&error);
        return mac;
    }

    /**
     * @brief Get the IP address of the host network interface corresponding to the device at the specified index
     *
     * @attention Only valid for network devices, otherwise it will return "0.0.0.0".
     *
     * @param[in] index The index of the device
     *
     * @return const char* The IP address of the host network interface associated with the device.
     */
    const char *getLocalIP(uint32_t index) const {
        ob_error *error = nullptr;
        auto      ip    = ob_device_list_get_device_local_ip(impl_, index, &error);
        Error::handle(&error);
        return ip;
    }

    /**
     * @brief Get the subnet length of the host network interface corresponding to the device at the specified index
     *
     * @attention Only valid for network devices, otherwise it will return 0.
     *
     * @param[in] index The index of the device
     *
     * @return uint8_t The subnet length (0~32) of the host network interface associated with the device.
     */
    uint8_t getLocalSubnetLength(uint32_t index) const {
        ob_error *error      = nullptr;
        auto      subnetMask = ob_device_list_get_device_local_subnet_length(impl_, index, &error);
        Error::handle(&error);
        return subnetMask;
    }

    /**
     * @brief Get the gateway of the host network interface corresponding to the device at the specified index
     *
     * @attention Only valid for network devices, otherwise it will return "0.0.0.0".
     *
     * @param[in] index The index of the device
     *
     * @return const char* The gateway of the host network interface associated with the device.
     */
    const char *getLocalGateway(uint32_t index) const {
        ob_error *error   = nullptr;
        auto      gateway = ob_device_list_get_device_local_gateway(impl_, index, &error);
        Error::handle(&error);
        return gateway;
    }

    /**
     * @brief Get the device object at the specified index
     *
     * @attention If the device has already been acquired and created elsewhere, repeated acquisition will throw an exception
     *
     * @param[in] index the index of the device to create
     * @param[in] accessMode Device access mode. @ref ob_device_access_mode.
     *                       If the device does not support setting the Access Mode, the default OB_DEVICE_DEFAULT_ACCESS is used.
     *                       Applies only on first device creation or after release and re-creation; subsequent calls ignore it.
     *
     * @return std::shared_ptr<Device> the device object
     */
    std::shared_ptr<Device> getDevice(uint32_t index, OBDeviceAccessMode accessMode = OB_DEVICE_DEFAULT_ACCESS) const {
        ob_error *error  = nullptr;
        auto      device = ob_device_list_get_device_ex(impl_, index, accessMode, &error);
        Error::handle(&error);
        return std::make_shared<Device>(device);
    }

    /**
     * @brief Get the device object with the specified serial number
     *
     * @attention If the device has already been acquired and created elsewhere, repeated acquisition will throw an exception
     *
     * @param[in] serialNumber the serial number of the device to create
     * @param[in] accessMode Device access mode. @ref ob_device_access_mode.
     *                       If the device does not support setting the Access Mode, the default OB_DEVICE_DEFAULT_ACCESS is used.
     *                       Applies only on first device creation or after release and re-creation; subsequent calls ignore it.
     *
     * @return std::shared_ptr<Device> the device object
     */
    std::shared_ptr<Device> getDeviceBySN(const char *serialNumber, OBDeviceAccessMode accessMode = OB_DEVICE_DEFAULT_ACCESS) const {
        ob_error *error  = nullptr;
        auto      device = ob_device_list_get_device_by_serial_number_ex(impl_, serialNumber, accessMode, &error);
        Error::handle(&error);
        return std::make_shared<Device>(device);
    }

    /**
     * @brief Get the specified device object from the device list by uid
     * @brief On Linux platform, for usb device, the uid of the device is composed of bus-port-dev, for example 1-1.2-1. But the SDK will remove the dev number
     * and only keep the bus-port as the uid to create the device, for example 1-1.2, so that we can create a device connected to the specified USB port.
     * Similarly, users can also directly pass in bus-port as uid to create device.
     * @brief For GMSL device, the uid is GMSL port with "gmsl2-" prefix, for example gmsl2-1.
     *
     * @attention If the device has been acquired and created elsewhere, repeated acquisition will throw an exception
     *
     * @param[in] uid The uid of the device to be created
     * @param[in] accessMode Device access mode. @ref ob_device_access_mode.
     *                       If the device does not support setting the Access Mode, the default OB_DEVICE_DEFAULT_ACCESS is used.
     *                       Applies only on first device creation or after release and re-creation; subsequent calls ignore it.
     *
     * @return std::shared_ptr<Device> returns the device object
     */
    std::shared_ptr<Device> getDeviceByUid(const char *uid, OBDeviceAccessMode accessMode = OB_DEVICE_DEFAULT_ACCESS) const {
        ob_error *error  = nullptr;
        auto      device = ob_device_list_get_device_by_uid_ex(impl_, uid, accessMode, &error);
        Error::handle(&error);
        return std::make_shared<Device>(device);
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    uint32_t deviceCount() const {
        return getCount();
    }

    int pid(uint32_t index) const {
        return getPid(index);
    }

    int vid(uint32_t index) const {
        return getVid(index);
    }

    const char *uid(uint32_t index) const {
        return getUid(index);
    }

    const char *serialNumber(uint32_t index) const {
        return getSerialNumber(index);
    }

    const char *name(uint32_t index) const {
        return getName(index);
    }

    const char *connectionType(uint32_t index) const {
        return getConnectionType(index);
    }

    const char *ipAddress(uint32_t index) const {
        return getIpAddress(index);
    }
};

class OBDepthWorkModeList {
private:
    ob_depth_work_mode_list_t *impl_ = nullptr;

public:
    explicit OBDepthWorkModeList(ob_depth_work_mode_list_t *impl) : impl_(impl) {}
    ~OBDepthWorkModeList() {
        ob_error *error = nullptr;
        ob_delete_depth_work_mode_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of OBDepthWorkMode objects in the list
     *
     * @return uint32_t the number of OBDepthWorkMode objects in the list
     */
    uint32_t getCount() {
        ob_error *error = nullptr;
        auto      count = ob_depth_work_mode_list_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the OBDepthWorkMode object at the specified index
     *
     * @param[in] index the index of the target OBDepthWorkMode object
     *
     * @return OBDepthWorkMode the OBDepthWorkMode object at the specified index
     */
    OBDepthWorkMode getOBDepthWorkMode(uint32_t index) {
        ob_error *error = nullptr;
        auto      mode  = ob_depth_work_mode_list_get_item(impl_, index, &error);
        Error::handle(&error);
        return mode;
    }

    /**
     * @brief Get the OBDepthWorkMode object at the specified index
     *
     * @param[in] index the index of the target OBDepthWorkMode object
     *
     * @return OBDepthWorkMode the OBDepthWorkMode object at the specified index
     */
    OBDepthWorkMode operator[](uint32_t index) {
        return getOBDepthWorkMode(index);
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    uint32_t count() {
        return getCount();
    }
};

/**
 * @brief Class representing a list of device presets
 * @brief A device preset is a set of parameters or configurations that can be applied to the device to achieve a specific effect or function.
 */
class DevicePresetList {
private:
    ob_device_preset_list_t *impl_ = nullptr;

public:
    explicit DevicePresetList(ob_device_preset_list_t *impl) : impl_(impl) {}
    ~DevicePresetList() noexcept {
        ob_error *error = nullptr;
        ob_delete_preset_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of device presets in the list
     *
     * @return uint32_t the number of device presets in the list
     */
    uint32_t getCount() {
        ob_error *error = nullptr;
        auto      count = ob_device_preset_list_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the name of the device preset at the specified index
     *
     * @param[in] index the index of the device preset
     *
     * @return const char* the name of the device preset
     */
    const char *getName(uint32_t index) {
        ob_error   *error = nullptr;
        const char *name  = ob_device_preset_list_get_name(impl_, index, &error);
        Error::handle(&error);
        return name;
    }

    /**
     * @brief check if the preset list contains the special name preset.
     * @param[in] name The name of the preset
     *
     * @return bool Returns true if the special name is found in the preset list, otherwise returns false.
     */
    bool hasPreset(const char *name) {
        ob_error *error  = nullptr;
        auto      result = ob_device_preset_list_has_preset(impl_, name, &error);
        Error::handle(&error);
        return result;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    uint32_t count() {
        return getCount();
    }
};

/**
 * @brief Class representing a list of camera parameters
 */
class CameraParamList {
private:
    ob_camera_param_list_t *impl_ = nullptr;

public:
    explicit CameraParamList(ob_camera_param_list_t *impl) : impl_(impl) {}
    ~CameraParamList() noexcept {
        ob_error *error = nullptr;
        ob_delete_camera_param_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of camera parameters in the list.
     *
     * @return uint32_t the number of camera parameters in the list.
     */
    uint32_t getCount() {
        ob_error *error = nullptr;
        auto      count = ob_camera_param_list_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the camera parameters for the specified index
     *
     * @param[in] index the index of the parameter group
     *
     * @return OBCameraParam the corresponding group parameters
     */
    OBCameraParam getCameraParam(uint32_t index) {
        ob_error *error = nullptr;
        auto      param = ob_camera_param_list_get_param(impl_, index, &error);
        Error::handle(&error);
        return param;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    uint32_t count() {
        return getCount();
    }
};

/**
 * @brief Class representing a list of device Frame Interleave
 */
class DeviceFrameInterleaveList {
private:
    ob_device_frame_interleave_list_t *impl_ = nullptr;

public:
    explicit DeviceFrameInterleaveList(ob_device_frame_interleave_list_t *impl) : impl_(impl) {}
    ~DeviceFrameInterleaveList() noexcept {
        ob_error *error = nullptr;
        ob_delete_frame_interleave_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of device frame interleave in the list
     *
     * @return uint32_t the number of device frame interleave  in the list
     */
    uint32_t getCount() {
        ob_error *error = nullptr;
        auto      count = ob_device_frame_interleave_list_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the name of the device frame interleave at the specified index
     *
     * @param[in] index the index of the device frame interleave
     *
     * @return const char* the name of the device frame interleave
     */
    const char *getName(uint32_t index) {
        ob_error   *error = nullptr;
        const char *name  = ob_device_frame_interleave_list_get_name(impl_, index, &error);
        Error::handle(&error);
        return name;
    }

    /**
     * @brief check if the frame interleave list contains the special name frame interleave.
     *
     * @param[in] name The name of the frame interleave
     *
     * @return bool Returns true if the special name is found in the frame interleave list, otherwise returns false.
     */
    bool hasFrameInterleave(const char *name) {
        ob_error *error  = nullptr;
        auto      result = ob_device_frame_interleave_list_has_frame_interleave(impl_, name, &error);
        Error::handle(&error);
        return result;
    }
};

/**
 * @brief Class representing a list of preset resolution config list
 */
class PresetResolutionConfigList {
private:
    ob_preset_resolution_config_list_t *impl_ = nullptr;

public:
    explicit PresetResolutionConfigList(ob_preset_resolution_config_list_t *impl) : impl_(impl) {}
    ~PresetResolutionConfigList() noexcept {
        ob_error *error = nullptr;
        ob_delete_preset_resolution_config_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of device preset resolution ratio in the list
     */
    uint32_t getCount() {
        ob_error *error = nullptr;
        auto      count = ob_device_preset_resolution_config_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /*
     *  @brief Get the device preset resolution ratio at the specified index

     *  @param[in] index the index of the device preset resolution ratio
     *
     *  @return OBPresetResolutionConfig the corresponding device preset resolution ratio
     */
    OBPresetResolutionConfig getPresetResolutionRatioConfig(uint32_t index) {
        ob_error *error  = nullptr;
        auto      config = ob_device_preset_resolution_config_list_get_item(impl_, index, &error);
        Error::handle(&error);
        return config;
    }
};

}  // namespace ob
