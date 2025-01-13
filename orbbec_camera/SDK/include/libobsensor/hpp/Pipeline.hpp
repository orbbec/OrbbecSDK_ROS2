/**
 * @file Pipeline.hpp
 * @brief The SDK's advanced API type can quickly implement switching streaming and frame synchronization
 * operations.
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
     * @brief Pipeline is a high-level interface for applications, algorithms related RGBD data streams. Pipeline can provide alignment inside and synchronized
     * FrameSet. Pipeline() no parameter version, which opens the first device in the list of devices connected to the OS by default. If the application has
     * obtained the device through the DeviceList, opening the Pipeline() at this time will throw an exception that the device has been created.
     */
    Pipeline();

    /**
     * @brief
     * Pipeline(std::shared_ptr< Device > device ) Function for multi-device operations. Multiple devices need to be obtained through DeviceList, and the device
     * and pipeline are bound through this interface.
     */
    Pipeline(std::shared_ptr<Device> device);

    /**
     * @brief Construct a pipeline for playback of recorded stream files
     *
     * @param filename The file path of the recorded stream file to be played back
     */
    Pipeline(const char *filename);

    /**
     * @brief Destroy the pipeline object
     */
    ~Pipeline() noexcept;

    /**
     * @brief Start the pipeline with configuration parameters
     *
     * @param config The parameter configuration of the pipeline
     */
    void start(std::shared_ptr<Config> config);

    /**
     * @brief Start the pipeline with default configuration parameters
     */
    void start();

    /**
     * @brief Start the pipeline and set the frameset data callback
     *
     * @param config The configuration of the pipeline
     * @param callback The callback to be triggered when all frame data in the frameset arrives
     */
    void start(std::shared_ptr<Config> config, FrameSetCallback callback);

    /**
     * @brief Stop the pipeline
     */
    void stop();

    /**
     * @brief get Enabled Stream Profile List,It must be called after the pipeline is started, otherwise it will return an empty list
     */
    std::shared_ptr<StreamProfileList> getEnabledStreamProfileList();
    /**
     * @brief Get the pipeline configuration parameters
     * @brief Returns the default configuration if the user has not configured it
     *
     * @return std::shared_ptr<Config> The configured parameters
     */
    std::shared_ptr<Config> getConfig();

    /**
     * @brief Wait for frameset data
     *
     * @param timeout_ms The waiting timeout in milliseconds
     * @return std::shared_ptr<FrameSet> The waiting frameset data
     */
    std::shared_ptr<FrameSet> waitForFrames(uint32_t timeout_ms = 1000);

    /**
     * @brief Get the device object
     *
     * @return std::shared_ptr<Device> The device object
     */
    std::shared_ptr<Device> getDevice();

    /**
     * @brief Get the playback object
     *
     * @return std::shared_ptr<Playback> The playback object
     */
    std::shared_ptr<Playback> getPlayback();

    /**
     * @brief Get the stream profile of the specified sensor
     *
     * @param sensorType The type of sensor
     * @return std::shared_ptr<StreamProfileList> The stream profile list
     */
    std::shared_ptr<StreamProfileList> getStreamProfileList(OBSensorType sensorType);

    /**
     * @brief Turn on frame synchronization
     */
    void enableFrameSync();

    /**
     * @brief Turn off frame synchronization
     */
    void disableFrameSync();

    /**
     * @brief Get the camera parameters
     *
     * @note If D2C is enabled, it will return the camera parameters after D2C. If not, it will return the default parameters.
     *
     * @return OBCameraParam The camera parameters
     */
    OBCameraParam getCameraParam();

    /**
     * @brief Get camera parameters by entering color and depth resolution
     * @attention If D2C is enabled, it will return the camera parameters after D2C, if not, it will return to the default parameters
     *
     * @param colorWidth Width of color resolution
     * @param colorHeight High of color resolution
     * @param depthWidth Width of depth resolution
     * @param depthHeight High of depth resolution
     *
     * @return  OBCameraParam returns camera parameters
     */
    OBCameraParam getCameraParamWithProfile(uint32_t colorWidth, uint32_t colorHeight, uint32_t depthWidth, uint32_t depthHeight);

    /**
     * @brief Get the calibration parameters
     *
     * @param config The configured parameters
     *
     * @return OBCalibrationParam The calibration parameters
     */
    OBCalibrationParam getCalibrationParam(std::shared_ptr<Config> config);

    /**
     * @brief Return a list of D2C-enabled depth sensor resolutions corresponding to the input color sensor resolution
     *
     * @param colorProfile The input color sensor resolution
     * @param alignMode The input align mode
     * @return std::shared_ptr<StreamProfileList> A list of depth sensor resolutions
     */
    std::shared_ptr<StreamProfileList> getD2CDepthProfileList(std::shared_ptr<StreamProfile> colorProfile, OBAlignMode alignMode);

    /**
     * @brief Get the valid area between the minimum distance and maximum distance after D2C
     *
     * @param minimumDistance The minimum working distance
     * @param maximumDistance The maximum working distance (optional)
     * @return OBRect The area information valid after D2C at the working distance
     */
    OBRect getD2CValidArea(uint32_t minimumDistance, uint32_t maximumDistance = 0);

    /**
     * @brief Dynamically switch the corresponding config configuration
     *
     * @param config The updated config configuration
     */
    void switchConfig(std::shared_ptr<Config> config);

    /**
     * @brief Start recording
     *
     * @param filename The name of the record file
     */
    void startRecord(const char *filename);

    /**
     * @brief Stop recording
     */
    void stopRecord();
};

/**
 * @brief Config class for configuring pipeline parameters
 *
 * The Config class provides an interface for configuring pipeline parameters.
 */
class OB_EXTENSION_API Config {
private:
    std::unique_ptr<ConfigImpl> impl_;

public:
    /**
     * @brief Construct a new Config object
     */
    Config();

    /**
     * @brief Destroy the Config object
     */
    ~Config() noexcept;

    /**
     * @brief Enable a stream to be used in the pipeline
     *
     * @param streamProfile The stream configuration to be enabled
     */
    void enableStream(std::shared_ptr<StreamProfile> streamProfile);

    /**
     * @deprecated Use enableStream(std::shared_ptr<StreamProfile> streamProfile) instead
     * @brief Enable all streams to be used in the pipeline
     */
    void enableAllStream();

    /**
     * @brief Enable a video stream to be used in the pipeline.
     *
     * This function allows users to enable a video stream with customizable parameters.
     * If no parameters are specified, the stream will be enabled with default resolution settings.
     * Users who wish to set custom resolutions should refer to the product manual, as available resolutions vary by camera model.
     *
     * @param type The video stream type.
     * @param width The video stream width (default is OB_WIDTH_ANY, which selects the default resolution).
     * @param height The video stream height (default is OB_HEIGHT_ANY, which selects the default resolution).
     * @param fps The video stream frame rate (default is OB_FPS_ANY, which selects the default frame rate).
     * @param format The video stream format (default is OB_FORMAT_ANY, which selects the default format).
     */
    void enableVideoStream(ob_stream_type type, int width = OB_WIDTH_ANY, int height = OB_HEIGHT_ANY, int fps = OB_FPS_ANY, OBFormat format = OB_FORMAT_ANY);

    /**
     * @brief Enable an accelerometer stream to be used in the pipeline.
     *
     * This function allows users to enable an accelerometer stream with customizable parameters.
     * If no parameters are specified, the stream will be enabled with default settings.
     * Users who wish to set custom full-scale ranges or sample rates should refer to the product manual, as available settings vary by device model.
     *
     * @param fullScaleRange The full-scale range of the accelerometer (default is OB_ACCEL_FULL_SCALE_RANGE_ANY, which selects the default range).
     * @param sampleRate The sample rate of the accelerometer (default is OB_ACCEL_SAMPLE_RATE_ANY, which selects the default rate).
     */
    void enableAccelStream(ob_accel_full_scale_range fullScaleRange = OB_ACCEL_FULL_SCALE_RANGE_ANY,
                           ob_accel_sample_rate      sampleRate     = OB_ACCEL_SAMPLE_RATE_ANY);

    /**
     * @brief Enable a gyroscope stream to be used in the pipeline.
     *
     * This function allows users to enable a gyroscope stream with customizable parameters.
     * If no parameters are specified, the stream will be enabled with default settings.
     * Users who wish to set custom full-scale ranges or sample rates should refer to the product manual, as available settings vary by device model.
     *
     * @param fullScaleRange The full-scale range of the gyroscope (default is OB_GYRO_FULL_SCALE_RANGE_ANY, which selects the default range).
     * @param sampleRate The sample rate of the gyroscope (default is OB_GYRO_SAMPLE_RATE_ANY, which selects the default rate).
     */
    void enableGyroStream(ob_gyro_full_scale_range fullScaleRange = OB_GYRO_FULL_SCALE_RANGE_ANY, ob_gyro_sample_rate sampleRate = OB_GYRO_SAMPLE_RATE_ANY);

    /**
     * @brief Disable a stream to be used in the pipeline
     *
     * @param streamType The stream configuration to be disabled
     */
    void disableStream(OBStreamType streamType);

    /**
     * @brief Disable all streams to be used in the pipeline
     */
    void disableAllStream();

    /**
     * @brief Get the Enabled Stream Profile List
     *
     * @return std::shared_ptr<StreamProfileList>
     */
    std::shared_ptr<StreamProfileList> getEnabledStreamProfileList() const;

    /**
     * @brief Set the alignment mode
     *
     * @param mode The alignment mode
     */
    void setAlignMode(OBAlignMode mode);

    /**
     * @brief Set whether the depth needs to be scaled after setting D2C
     *
     * @param enable Whether scaling is required
     */
    void setDepthScaleRequire(bool enable);

    /**
     * @brief Set the D2C target resolution
     * @brief The D2C target resolution is applicable to cases where the color stream is not enabled using the OrbbecSDK and the depth needs to be D2C.
     *
     * @note When you use OrbbecSDK to enable the color stream, you also use this interface to set the D2C target resolution. The configuration of the
     * enabled Color stream is preferred for D2C.
     *
     * @param d2cTargetWidth The D2C target width resolution
     * @param d2cTargetHeight The D2C target height resolution
     */
    void setD2CTargetResolution(uint32_t d2cTargetWidth, uint32_t d2cTargetHeight);

    /**
     * @brief Set the frame aggregation output mode for the pipeline configuration
     * @brief The processing strategy when the FrameSet generated by the frame aggregation function does not contain the frames of all opened streams (which
     * can be caused by different frame rates of each stream, or by the loss of frames of one stream): drop directly or output to the user.
     *
     * @param mode The frame aggregation output mode to be set (default mode is @ref OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION)
     */
    void setFrameAggregateOutputMode(OBFrameAggregateOutputMode mode);

    friend class Pipeline;
};

}  // namespace ob
