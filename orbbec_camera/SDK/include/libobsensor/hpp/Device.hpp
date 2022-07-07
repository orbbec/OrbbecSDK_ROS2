/**
 * \if English
 * @file Device.hpp
 * @brief Device related types, including operations such as getting and creating a device, setting and obtaining device attributes, and obtaining sensors
 * \else
 * @file Device.hpp
 * @brief 设备相关类型，包括获取创建设备，设置及获取设备属性，获取传感器等操作
 * \endif
 *
 */
#pragma once
#include "Types.hpp"

#include "libobsensor/h/Property.h"

#include <memory>
#include <string>

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

class OB_EXTENSION_API Device {
private:
    std::unique_ptr<DeviceImpl> impl_;

public:
    /**
     * \if English
     * @brief Describe the entity of the RGBD camera, representing a specific model of RGBD camera
     * \else
     * @brief 描述RGBD相机的实体，代表一个具体型号的RGBD相机
     * \endif
     */
    Device(std::unique_ptr<DeviceImpl> impl);
    virtual ~Device() noexcept;
    /**
     * \if English
     * @brief Get device information
     *
     * @return std::shared_ptr<DeviceInfo> returns device information
     * \else
     * @brief 获取设备信息
     *
     * @return std::shared_ptr<DeviceInfo> 返回设备的信息
     * \endif
     */
    std::shared_ptr<DeviceInfo> getDeviceInfo();

    /**
     * \if English
     * @brief Get device sensor list
     *
     * @return std::shared_ptr<SensorList> returns the sensor list
     * \else
     * @brief 获取设备传感器列表
     *
     * @return std::shared_ptr<SensorList> 返回传感器列表
     * \endif
     */
    std::shared_ptr<SensorList> getSensorList();

    /**
     * \if English
     * @brief Get specific type of sensor
     * if device not open, SDK will automatically open the connected device and return to the instance
     *
     * @return std::shared_ptr<Sensor> eturns the sensor example, if the device does not have the device,returns nullptr
     * \else
     * @brief 获取指定类型传感器
     * 如果设备没有打开传感器，在SDK内部会自动打开设备并返回实例
     *
     * @return std::shared_ptr<Sensor> 返回传感器示例，如果设备没有该设备，返回nullptr
     * \endif
     */
    std::shared_ptr<Sensor> getSensor(OBSensorType type);
    /**
     * \if English
     * @brief Set int type of device property
     *
     * @param propertyId Property id
     * @param property Property to be set
     * \else
     * @brief 设置int类型的设备属性
     *
     * @param propertyId 属性id
     * @param property 要设置的属性
     * \endif
     */
    void setIntProperty(OBPropertyID propertyId, int32_t property);

    /**
     * \if English
     * @brief Set float type of device property
     *
     * @param propertyId Property id
     * @param property Property to be set
     * \else
     * @brief 设置float类型的设备属性
     *
     * @param propertyId 属性id
     * @param property 要设置的属性
     * \endif
     */
    void setFloatProperty(OBPropertyID propertyId, float property);

    /**
     * \if English
     * @brief Set bool type of device property
     *
     * @param propertyId Property id
     * @param property Property to be set
     * \else
     * @brief 设置bool类型的设备属性
     *
     * @param propertyId 属性id
     * @param property 要设置的属性
     * \endif
     */
    void setBoolProperty(OBPropertyID propertyId, bool property);

    /**
     * \if English
     * @brief Get int type of device property
     *
     * @param propertyId Property id
     * @return int32_t Property to get
     * \else
     * @brief 获取int类型的设备属性
     *
     * @param propertyId 属性id
     * @return int32_t 获取的属性数据
     * \endif
     */
    int32_t getIntProperty(OBPropertyID propertyId);

    /**
     * \if English
     * @brief Get float type of device property
     *
     * @param propertyId Property id
     * @return float Property to get
     * \else
     * @brief 获取float类型的设备属性
     *
     * @param propertyId 属性id
     * @return float 获取的属性数据
     * \endif
     */
    float getFloatProperty(OBPropertyID propertyId);

    /**
     * \if English
     * @brief Get bool type of device property
     *
     * @param propertyId Property id
     * @return bool Property to get
     * \else
     * @brief 获取bool类型的设备属性
     *
     * @param propertyId 属性id
     * @return bool 获取的属性数据
     * \endif
     */
    bool getBoolProperty(OBPropertyID propertyId);

    /**
     * \if English
     * @brief Get int type device property range (ncluding current valueand default value)
     *
     * @param propertyId Property id
     * @return OBIntPropertyRange Property range
     * \else
     * @brief 获取int类型的设备属性的范围(包括当前值和默认值)
     *
     * @param propertyId 属性id
     * @return OBIntPropertyRange 属性的范围
     * \endif
     */
    OBIntPropertyRange getIntPropertyRange(OBPropertyID propertyId);

    /**
     * \if English
     * @brief Get float type device property range((including current valueand default value)
     *
     * @param propertyId Property id
     * @return OBFloatPropertyRange Property range
     * \else
     * @brief 获取float类型的设备属性的范围(包括当前值和默认值)
     *
     * @param propertyId 属性id
     * @return OBFloatPropertyRange 属性的范围
     * \endif
     */
    OBFloatPropertyRange getFloatPropertyRange(OBPropertyID propertyId);

    /**
     * \if English
     * @brief Get bool type device property range (including current value anddefault value)
     *
     * @param propertyId Property id
     * @return OBBoolPropertyRange Property range
     * \else
     * @brief 获取bool类型的设备属性的范围(包括当前值和默认值)
     *
     * @param propertyId 属性id
     * @return OBBoolPropertyRange 属性的范围
     * \endif
     */
    OBBoolPropertyRange getBoolPropertyRange(OBPropertyID propertyId);

    /**
     * \if English
     * @brief AHB register write
     *
     * @param reg Register to be written 
     * @param mask  The mask to be writen
     * @param value The value to be written
     * \else
     * @brief AHB写寄存器
     *
     * @param reg 要写入的寄存器
     * @param mask 要写入的掩码
     * @param value 要写入的值
     * \endif
     */
    void writeAHB(uint32_t reg, uint32_t mask, uint32_t value);

    /**
     * \if English
     * @brief AHB AHB register read
     *
     * @param reg Register to be read
     * @param mask The mask to be read
     * @param value The value to be returned
     * \else
     * @brief AHB读寄存器
     *
     * @param reg 要读取的寄存器
     * @param mask 要读取的掩码
     * @param value 读取的值返回
     * \endif
     */
    void readAHB(uint32_t reg, uint32_t mask, uint32_t *value);

    /**
     * \if English
     * @brief I2C register write
     *
     * @param reg I2C module ID to be written
     * @param reg Register to be written
     * @param mask The mask to be written
     * @param value he value to be written
     * \else
     * @brief I2C写寄存器
     *
     * @param reg 要写入的I2C模块ID
     * @param reg 要写入的寄存器
     * @param mask 要写入的掩码
     * @param value 要写入的值
     * \endif
     */
    void writeI2C(uint32_t moduleId, uint32_t reg, uint32_t mask, uint32_t value);

    /**
     * \if English
     * @brief I2C registers read
     *
     * @param reg I2C module ID to be read
     * @param reg Register to be read
     * @param mask The mask to be read
     * @param value The value to be returned
     * \else
     * @brief I2C读寄存器
     *
     * @param reg 要读取的I2C模块ID
     * @param reg 要读取的寄存器
     * @param mask 要读取的掩码
     * @param value 读取的值返回
     * \endif
     */
    void readI2C(uint32_t moduleId, uint32_t reg, uint32_t mask, uint32_t *value);

    /**
     * \if English
     * @brief Set the properties of writing to Flash
     *
     * @param offset flash offset address
     * @param data Property data to be written
     * @param dataSize  The size of the property to be written
     * @param callback Write flash progress callback
     * @param async    Whether to execute asynchronously
     * \else
     * @brief 设置写入Flash的属性
     *
     * @param offset flash 偏移地址
     * @param data 要写入的属性数据
     * @param dataSize 要写入的属性大小
     * @param callback 写flash进度回调
     * @param async    是否异步执行
     * \endif
     */
    void writeFlash(uint32_t offset, const void *data, uint32_t dataSize, SetDataCallback callback, bool async = false);

    /**
     * \if English
     * @brief Read Flash property
     *
     * @param offset flash offset address
     * @param data Property data to be read
     * @param dataSize  The size of the property to get
     * @param callback Read data returned by flash and progress callback
     * @param async    Whether to execute asynchronously
     * \else
     * @brief 读取Flash的属性
     *
     * @param offset flash 偏移地址
     * @param data 读取的属性数据
     * @param dataSize 获取的属性大小
     * @param callback 读flash返回的数据及进度回调
     * @param async    是否异步执行
     * \endif
     */
    void readFlash(uint32_t offset, uint32_t dataSize, GetDataCallback callback, bool async = false);

    /**
     * \if English
     * @brief Set raw data type of device property [Asynchronous callback]
     *
     * @param propertyId Property id
     * @param data Property data to be set
     * @param dataSize The size of the property data to be set
     * @param callback rawdata set progress callback
     * @param async    Whether to execute asynchronously
     * \else
     * @brief 设置raw data类型的设备属性数据[异步回调]
     *
     * @param propertyId 属性id
     * @param data 要设置的属性数据
     * @param dataSize 要设置的属性数据大小
     * @param callback rawdata设置进度回调
     * @param async    是否异步执行
     * \endif
     */
    void setRawData(OBPropertyID propertyId, const void *data, uint32_t dataSize, SetDataCallback callback, bool async = false);

    /**
     * \if English
     * @brief Get raw data type of device property [Asynchronous callback]
     *
     * @param propertyId Property id
     * @param data Property data obtained
     * @param dataSize Get the size of the property
     * @param callback  Get the returned data and progress callback
     * @param async    Whether to execute asynchronously
     * \else
     * @brief 获取raw data类型的设备属性数据[异步回调]
     *
     * @param propertyId 属性id
     * @param data 获取的属性数据
     * @param dataSize 获取的属性大小
     * @param callback 获取返回的数据及进度回调
     * @param async    是否异步执行
     * \endif
     */
    void getRawData(OBPropertyID propertyId, GetDataCallback callback, bool async = false);

    /**
	 * \if English
     * @brief Set structured data type of device property
     *
     * @param propertyId Property id
     * @param data Property data to be set
     * @param dataSize The size of the attribute to be set
     * \else
     * @brief 设置structured data类型的设备属性
     *
     * @param propertyId 属性id
     * @param data 要设置的属性数据
     * @param dataSize 要设置的属性大小
     * \endif
     */
    void setStructuredData(OBPropertyID propertyId, const void *data, uint32_t dataSize);

    /**
     * \if English
     * @brief Get structured data type of device property
     *
     * @param propertyId Property id
     * @param data Property data obtained
     * @param dataSize Get the size of the attribute
     * \else
     * @brief 获取structured data类型的设备属性
     *
     * @param propertyId 属性id
     * @param data 获取的属性数据
     * @param dataSize 获取的属性大小
     * \endif
     */	
    void getStructuredData(OBPropertyID propertyId, void *data, uint32_t *dataSize);

    /**
     * \if English
     * @brief Get number of devices supported property
     *
     * @return uint32_t returns the number of supported attributes
     * \else
     * @brief 获取设备支持的属性的数量
     *
     * @return uint32_t 返回支持的属性的数量
     * \endif
     */
    uint32_t getSupportedPropertyCount();

    /**
     * \if English
     * @brief Get device supported properties
     *
     * @param uint32_t Property index
     * @return OBPropertyItem returns the type of supported properties
     * \else
     * @brief 获取设备支持的属性
     *
     * @param uint32_t 属性的index
     * @return OBPropertyItem 返回支持的属性的类型
     * \endif
     */
    OBPropertyItem getSupportedProperty(uint32_t index);

    /**
	 * \if English
	 * @brief Judge property permission support
     *
     * @param propertyId Property id
     * @param permission Types of read and write permissions that need to be interpreted
     * @return bool returns whether it is supported
	 * \else
     * @brief 判断属性权限支持情况
     *
     * @param propertyId 属性id
     * @param permission 需要判读的读写权限类型
     * @return bool 返回是否支持
	 * \endif
     */
    bool isPropertySupported(OBPropertyID propertyId, OBPermissionType permission);

    /**
     * \if English
     * @brief Synchronize device time (synchronize local systemtime to device)
     * @return uint64_t command (round trip time， rtt）
     * \else
     * @brief 同步设备时间（向设备授时，同步本地系统时间到设备）
     * @return uint64_t 命令往返时间延时（round trip time， rtt）
     * \endif
     */
    uint64_t syncDeviceTime();

    /**
     * \if English
     * @brief Upgrade the device firmware
     *
     * @param filePath Firmware path
     * @param callback  Firmware upgrade progress and status callback
     * @param async    Whether to execute asynchronously
     * \else
     * @brief 升级设备固件
     *
     * @param filePath 固件的路径
     * @param callback 固件升级进度及状态回调
     * @param async    是否异步执行
     * \endif
     */
    void deviceUpgrade(const char *filePath, DeviceUpgradeCallback callback, bool async = true);

    /**
     * \if English
     * @brief Send files to the specified path on the device side [Asynchronouscallback]
     *
     * @param filePath Original file path
     * @param dstPath  Accept the save path on the device side
     * @param callback File transfer callback
     * @param async    Whether to execute asynchronously
     * \else
     * @brief 发送文件到设备端指定路径[异步回调]
     *
     * @param filePath 原文件路径
     * @param dstPath 设备端接受保存路径
     * @param callback 文件传输回调
     * @param async    是否异步执行
     * \endif
     */
    void sendFile(const char *filePath, const char *dstPath, SendFileCallback callback, bool async = true);

    /**
     * \if English
     * @brief Get the current state
     * @return OBDeviceState device state information
     * \else
     * @brief 获取当前设备状态
     * @return OBDeviceState 设备状态信息
     * \endif
     */
    OBDeviceState getDeviceState();

    /**
     * \if English
     * @brief Set the device state changed callbacks
     *
     * @param callback The callback function that is triggered when the device status changes (for example, the frame rate is automatically reduced or the stream is closed due to high temperature, etc.)
     * \else
     * @brief 设置设备状态改变回调函数
     *
     * @param callback 设备状态改变（如，由于温度过高自动降低帧率或关流等）时触发的回调函数
     * \endif
     */
    void setDeviceStateChangedCallback(DeviceStateChangedCallback callback);

    /**
     * \if English
     * @brief Verify device authorization code
     * @param authCode Authorization code
     * @return bool whether the activation is successfu
     * \else
     * @brief 验证设备授权码
     * @param authCode 授权码
     * @return bool 激活是否成功
     * \endif
     */
    bool activateAuthorization(const char *authCode);

    /**
     * \if English
     * @brief Write authorization code
     * @param authCode  Authorization code
     * \else
     * @brief 写入设备授权码
     * @param authCode 授权码
     * \endif
     */
    void writeAuthorizationCode(const char *authCodeStr);

    /**
	 * \if English
	 * @brief Get the original parameter list of camera calibration saved in the device. The parameters in the list do not correspond to the current open-current configuration.
     * You need to select the parameters according to the actual situation, and may need to do scaling, mirroring and other processing. Non-professional users are recommended to use the Pipeline::getCameraParam() interface.
     *
     * @return std::shared_ptr<CameraParamList> camera parameter list
	 * \else
     * @brief 获取设备内保存的相机标定的原始参数列表，列表内参数不与当前开流配置相对应，
     * 需要自行根据实际情况选用参数并可能需要做缩放、镜像等处理。非专业用户建议使用Pipeline::getCameraParam()接口。
     *
     * @return std::shared_ptr<CameraParamList> 相机参数列表
	 * \endif
     */
    std::shared_ptr<CameraParamList> getCalibrationCameraParamList();

    /**
	 * \if English
	 * @brief Device restart
     * @attention The device will be disconnected and reconnected. After the device is disconnected, the access to the Device object interface may be abnormal. 
     *   Please delete the object directly and obtain it again after the device is reconnected.
	 * \else
     * @brief 设备重启
     * @attention 设备会掉线重连，设备掉线后对Device对象接口访问可能会发生异常，请直接删除该对象，
     *   待设备重连后可重新获取。
	 * \endif
     */
    void reboot();

    friend class Pipeline;
    friend class Recorder;
};

/**
 * \if English
 * @brief A class describing device information, representing the name, id, serial number and other basic information of an RGBD camera.
 * \else
 * @brief 描述设备信息的类，代表一个RGBD相机的名称，id，序列号等其他设备自身基本信息。
 * \endif
 *
 */
class OB_EXTENSION_API DeviceInfo {
private:
    std::unique_ptr<DeviceInfoImpl> impl_;

public:
    DeviceInfo(std::unique_ptr<DeviceInfoImpl> impl);
    virtual ~DeviceInfo() noexcept;
    /**
     * \if English
     * @brief Get device name
     *
     * @return const char * returns the device name
     * \else
     * @brief 获取设备名称
     *
     * @return const char * 返回设备名称
     * \endif
     */
    const char *name();

    /**
     * \if English
     * @brief Get the pid of the device
     *
     * @return int returns the pid of the device
     * \else
     * @brief 获取设备的pid
     *
     * @return int 返回设备的pid
     * \endif
     */
    int pid();

    /**
     * \if English
     * @brief Get the vid of the device
     *
     * @return int returns the vid of the device
     * \else
     * @brief 获取设备的vid
     *
     * @return int 返回设备的vid
     * \endif
     */
    int vid();

    /**
     * \if English
     * @brief Get system assigned uid for distinguishing between different devices
     *
     * @return const char * returns the uid of the device
     * \else
     * @brief 获取设备的uid，该uid标识设备接入os操作系统时，给当前设备分派的唯一id，用来区分不同的设备
     *
     * @return const char * 返回设备的uid
     * \endif
     */
    const char *uid();

    /**
     * \if English
     * @brief Get the serial number of the device
     *
     * @return const char * returns the serial number of the device
     * \else
     * @brief 获取设备的序列号
     *
     * @return const char * 返回设备的序列号
     * \endif
     */
    const char *serialNumber();

    /**
     * \if English
     * @brief Get the version number of the firmware
     *
     * @return const char* returns the version number of the firmware
     * \else
     * @brief 获取固件的版本号
     *
     * @return const char* 返回固件的版本号
     * \endif
     */
    const char *firmwareVersion();

    /**
     * \if English
     * @brief Get usb connection type
     *
     * @return const char* returns usb connection type
     * \else
     * @brief 获取usb连接类型
     *
     * @return const char* 返回usb连接类型
     * \endif
     */
    DEPRECATED const char *usbType();

    /**
     * \if English
     * @brief Get device connection type
     *
     * @return const char* returns connection type
     * \else
     * @brief 获取设备连接类型
     *
     * @return const char* 返回连接类型
     * \endif
     */
    const char *connectionType();

    /**
	 * \if English
	 * @brief Get the version number of the hardware
     *
     * @return const char* returns the version number of the hardware
	 * \else
     * @brief 获取硬件的版本号
     *
     * @return const char* 返回硬件的版本号
	 * \endif
     */
    const char *hardwareVersion();

    /**
	 * \if English
	 * @brief Get the minimum version number of the SDK supported by the device
     *
     * @return const char* returns the minimum SDK version number supported by the device
	 * \else
     * @brief 获取设备支持的SDK最小版本号
     *
     * @return const char* 返回设备支持的SDK最小版本号
	 * \endif
     */
    const char *supportedMinSdkVersion();

    /**
	 * \if English
	 * @brief Get chip type name
     *
     * @return const char* returns the chip type name
	 * \else
     * @brief 获取芯片类型名称
     *
     * @return const char* 返回芯片类型名称
	 * \endif
     */
    const char *asicName();

    /**
	 * \if English
	 * @brief Get device type
     *
     * @return OBDeviceType returns the device type
	 * \else
     * @brief 获取设备类型
     *
     * @return OBDeviceType 返回设备类型
	 * \endif
     */
    OBDeviceType deviceType();

    friend class Context;
    friend class DeviceList;
    friend class Pipeline;
};

class OB_EXTENSION_API DeviceList {
private:
    std::unique_ptr<DeviceListImpl> impl_;

public:
    DeviceList(std::unique_ptr<DeviceListImpl> impl);
    ~DeviceList() noexcept;

    /**
     * \if English
     * @brief Get device count
     *
     * @return uint32_t returns the number of devices
     * \else
     * @brief 获取设备数量
     *
     * @return uint32_t 返回设备的数量
     * \endif
     */
    uint32_t deviceCount();

    /**
     * \if English
     * @brief Get the name of the specified device
     *
     * @param index Device index
     * @return int returns the name of the device
     * \else
     * @brief 获取指定设备的名称
     *
     * @param index 设备索引
     * @return int 返回设备的名称
     * \endif
     */
    const char *name(uint32_t index);

    /**
     * \if English
     * @brief Get the pid of the specified device
     *
     * @param index Device index
     * @return int returns the pid of the device
     * \else
     * @brief 获取指定设备的pid
     *
     * @param index 设备索引
     * @return int 返回设备的pid
     * \endif
     */
    int pid(uint32_t index);

    /**
     * \if English
     * @brief Get the vid of the specified device
     *
     * @param index Device index
     * @return int returns the vid of the device
     * \else
     * @brief 获取指定设备的vid
     *
     * @param index 设备索引
     * @return int 返回设备的vid
     * \endif
     */
    int vid(uint32_t index);

    /**
     * \if English
     * @brief Get the uid of the specified device
     *
     * @param index Device index
     * @return const char * returns the uid of the device
     * \else
     * @brief 获取指定设备的uid
     *
     * @param index 设备索引
     * @return const char * 返回设备的uid
	 * \endif
     */
    const char *uid(uint32_t index);

    /**
	 * \if English
     * @brief Get the serial number of the specified device
     *
     * @param index device index
     * @return const char * returns the serial number of the device
	 * \else
	 * @brief 获取指定设备的序列号
     *
     * @param index 设备索引
     * @return const char * 返回设备的序列号
	 * \endif
     */
    const char *serialNumber(uint32_t index);

    /**
	 * \if English
     * @brief Get the specified device object from the device list
     * @attention If the device has been acquired and created elsewhere, repeated acquisition will throw an exception
     * @param index index of the device to create
     * @return std::shared_ptr<Device> returns the device object
	 * \else
     * @brief 从设备列表中获取指定设备对象,
     * @attention 如果设备有在其他地方被获取创建，重复获取将会抛异常
     * @param index 要创建设备的索引
     * @return std::shared_ptr<Device> 返回设备对象
	 * \endif
     */
    std::shared_ptr<Device> getDevice(uint32_t index);

    /**
	 * \if English
     * @brief Get the specified device object from the device list
     * @attention If the device has been acquired and created elsewhere, repeated acquisition will throw an exception
     * @param serialNumber The serial number of the device to be created
     * @return std::shared_ptr<Device> returns the device object
	 * \else
     * @brief 从设备列表中获取指定设备对象
     * @attention 如果设备有在其他地方被获取创建，重复获取将会抛异常
     * @param serialNumber 要创建设备的序列号
     * @return std::shared_ptr<Device> 返回设备对象
	 * \endif
     */
    std::shared_ptr<Device> getDeviceBySN(const char *serialNumber);

    /**
	 * \if English
	 * @brief Get the specified device object from the device list
     * @attention If the device has been acquired and created elsewhere, repeated acquisition will throw an exception
     * @param uid If the device has been acquired and created elsewhere, repeated acquisition will throw an exception
     * @return std::shared_ptr<Device> If the device has been acquired and created elsewhere, repeated acquisition will throw an exception
	 * \else
     * @brief 从设备列表中获取指定设备对象
     * @attention 如果设备有在其他地方被获取创建，重复获取将会抛异常
     * @param uid 要创建设备的uid
     * @return std::shared_ptr<Device> 返回设备对象
	 * \endif
     */
    std::shared_ptr<Device> getDeviceByUid(const char *uid);
};

class OB_EXTENSION_API CameraParamList {
private:
    std::unique_ptr<CameraParamListImpl> impl_;

public:
    CameraParamList(std::unique_ptr<CameraParamListImpl> impl);
    ~CameraParamList() noexcept;
    /**
	 * \if English
	 * @brief Number of camera parameter groups
     *
     * @return uint32_t  returns the number of camera parameter groups
	 * \else
     * @brief 相机参数组数
     *
     * @return uint32_t  返回相机参数组数
	 * \endif
     */
    uint32_t count();

    /**
	 * \if English
	 * @brief Get camera parameters
     *
     * @param index parameter index
     * @return OBCameraParam returns the corresponding group parameters
	 * \else
     * @brief 获取相机参数
     *
     * @param index 参数索引
     * @return OBCameraParam 返回对应组参数
	 * \endif
     */
    OBCameraParam getCameraParam(uint32_t index);
};

}  // namespace ob
