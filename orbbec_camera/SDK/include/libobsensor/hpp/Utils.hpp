/**
 * @file Utils.hpp
 * @brief The SDK utils class
 *
 */
#pragma once

#include "Types.hpp"

namespace ob {
class Device;

class OB_EXTENSION_API CoordinateTransformHelper {
public:
    /**
     * @brief Transform a 3d point of a source coordinate system into a 3d point of the target coordinate system.
     *
     * @param calibrationParam Device calibration param,see pipeline::getCalibrationParam
     * @param sourcePoint3f Source 3d point value
     * @param sourceSensorType Source sensor type
     * @param targetSensorType Target sensor type
     * @param targetPoint3f Target 3d point value
     *
     * @return bool  Transform result
     */
    static bool calibration3dTo3d(const OBCalibrationParam calibrationParam, const OBPoint3f sourcePoint3f, const OBSensorType sourceSensorType,
                                  const OBSensorType targetSensorType, OBPoint3f *targetPoint3f);

    /**
     * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 3d point of the target coordinate system.
     *
     * @param calibrationParam Device calibration param,see pipeline::getCalibrationParam
     * @param sourcePoint2f Source 2d point value
     * @param sourceDepthPixelValue The depth of sourcePoint2f in millimeters
     * @param sourceSensorType Source sensor type
     * @param targetSensorType Target sensor type
     * @param targetPoint3f Target 3d point value
     *
     * @return bool Transform result
     */
    static bool calibration2dTo3d(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                  const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint3f *targetPoint3f);

    /**
     * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 3d point of the target coordinate system.
     * @brief This function uses undistortion, which may result in longer processing time.
     *
     * @param calibrationParam Device calibration param,see pipeline::getCalibrationParam
     * @param sourcePoint2f Source 2d point value
     * @param sourceDepthPixelValue The depth of sourcePoint2f in millimeters
     * @param sourceSensorType Source sensor type
     * @param targetSensorType Target sensor type
     * @param targetPoint3f Target 3d point value
     *
     * @return bool Transform result
     */
    static bool calibration2dTo3dUndistortion(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                              const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint3f *targetPoint3f);

    /**
     * @brief Transform a 3d point of a source coordinate system into a 2d pixel coordinate of the target camera.
     *
     * @param calibrationParam Device calibration param,see pipeline::getCalibrationParam
     * @param sourcePoint3f Source 3d point value
     * @param sourceSensorType Source sensor type
     * @param targetSensorType Target sensor type
     * @param targetPoint2f Target 2d point value
     *
     * @return bool Transform result
     */
    static bool calibration3dTo2d(const OBCalibrationParam calibrationParam, const OBPoint3f sourcePoint3f, const OBSensorType sourceSensorType,
                                  const OBSensorType targetSensorType, OBPoint2f *targetPoint2f);

    /**
     * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 2d pixel coordinate of the target camera
     *
     * @param calibrationParam Device calibration param,see pipeline::getCalibrationParam
     * @param sourcePoint2f Source 2d point value
     * @param sourceDepthPixelValue The depth of sourcePoint2f in millimeters
     * @param sourceSensorType Source sensor type
     * @param targetSensorType Target sensor type
     * @param targetPoint2f Target 2d point value
     *
     * @return bool Transform result
     */
    static bool calibration2dTo2d(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                  const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint2f *targetPoint2f);

    /**
     * @brief Transforms the depth frame into the geometry of the color camera.
     *
     * @param device Device handle
     * @param depthFrame Input depth frame
     * @param targetColorCameraWidth Target color camera width
     * @param targetColorCameraHeight Target color camera height
     *
     * @return std::shared_ptr<ob::Frame> Transformed depth frame
     */
    static std::shared_ptr<ob::Frame> transformationDepthFrameToColorCamera(std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Frame> depthFrame,
                                                                            uint32_t targetColorCameraWidth, uint32_t targetColorCameraHeight);

    /**
     * @brief Init transformation tables
     *
     * @param calibrationParam Device calibration param,see pipeline::getCalibrationParam
     * @param sensorType sensor type
     * @param data input data,needs to be allocated externally.During initialization, the external allocation size is 'dataSize', for example, dataSize = 1920 *
     * 1080 * 2*sizeof(float) (1920 * 1080 represents the image resolution, and 2 represents two LUTs, one for x-coordinate and one for y-coordinate).
     * @param dataSize input data size
     * @param xyTables output xy tables
     *
     * @return bool Transform result
     */
    static bool transformationInitXYTables(const OBCalibrationParam calibrationParam, const OBSensorType sensorType, float *data, uint32_t *dataSize,
                                           OBXYTables *xyTables);

    /**
     * @brief Transform depth image to point cloud data
     *
     * @param xyTables input xy tables,see CoordinateTransformHelper::transformationInitXYTables
     * @param depthImageData input depth image data
     * @param pointCloudData output point cloud data
     *
     */
    static void transformationDepthToPointCloud(OBXYTables *xyTables, const void *depthImageData, void *pointCloudData);

    /**
     * @brief Transform depth image to RGBD point cloud data
     *
     * @param xyTables input xy tables,see CoordinateTransformHelper::transformationInitXYTables
     * @param depthImageData input depth image data
     * @param colorImageData input color image data (only RGB888 support)
     * @param pointCloudData output RGBD point cloud data
     *
     */
    static void transformationDepthToRGBDPointCloud(OBXYTables *xyTables, const void *depthImageData, const void *colorImageData, void *pointCloudData);
};
}  // namespace ob
