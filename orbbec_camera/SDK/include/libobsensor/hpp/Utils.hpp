// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Utils.hpp
 * @brief The SDK utils class
 *
 */
#pragma once
#include "libobsensor/h/Utils.h"
#include "Device.hpp"
#include "Types.hpp"
#include "Frame.hpp"

#include <memory>

namespace ob {
class Device;

class CoordinateTransformHelper {
public:
    /**
     * @brief Transform a 3d point of a source coordinate system into a 3d point of the target coordinate system.
     *
     * @param[in] source_point3f Source 3d point value
     * @param[in] extrinsic Transformation matrix from source to target
     * @param[out] target_point3f Target 3d point value
     *
     * @return bool Transform result
     */
    static bool transformation3dto3d(const OBPoint3f source_point3f, OBExtrinsic extrinsic, OBPoint3f *target_point3f) {
        ob_error *error  = NULL;
        bool      result = ob_transformation_3d_to_3d(source_point3f, extrinsic, target_point3f, &error);
        Error::handle(&error);
        return result;
    }

    /**
     * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 3d point of the target coordinate system.
     *
     * @param[in] source_intrinsic Source intrinsic parameters
     * @param[in] source_point2f Source 2d point value
     * @param[in] source_depth_pixel_value The depth of sourcePoint2f in millimeters
     * @param[in] extrinsic Transformation matrix from source to target
     * @param[out] target_point3f Target 3d point value
     *
     * @return bool Transform result
     */
    static bool transformation2dto3d(const OBPoint2f source_point2f, const float source_depth_pixel_value, const OBCameraIntrinsic source_intrinsic,
                                     OBExtrinsic extrinsic, OBPoint3f *target_point3f) {
        ob_error *error  = NULL;
        bool      result = ob_transformation_2d_to_3d(source_point2f, source_depth_pixel_value, source_intrinsic, extrinsic, target_point3f, &error);
        Error::handle(&error);
        return result;
    }

    /**
     * @brief Transform a 3d point of a source coordinate system into a 2d pixel coordinate of the target camera.
     *
     * @param[in] source_point3f Source 3d point value
     * @param[in] target_intrinsic Target intrinsic parameters
     * @param[in] target_distortion Target distortion parameters
     * @param[in] extrinsic Transformation matrix from source to target
     * @param[out] target_point2f Target 2d point value
     *
     * @return bool Transform result
     */
    static bool transformation3dto2d(const OBPoint3f source_point3f, const OBCameraIntrinsic target_intrinsic, const OBCameraDistortion target_distortion,
                                     OBExtrinsic extrinsic, OBPoint2f *target_point2f) {
        ob_error *error  = NULL;
        bool      result = ob_transformation_3d_to_2d(source_point3f, target_intrinsic, target_distortion, extrinsic, target_point2f, &error);
        Error::handle(&error);
        return result;
    }

    /**
     * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 2d pixel coordinate of the target camera
     *
     * @param[in] source_intrinsic Source intrinsic parameters
     * @param[in] source_distortion Source distortion parameters
     * @param[in] source_point2f Source 2d point value
     * @param[in] source_depth_pixel_value The depth of sourcePoint2f in millimeters
     * @param[in] target_intrinsic Target intrinsic parameters
     * @param[in] target_distortion Target distortion parameters
     * @param[in] extrinsic Transformation matrix from source to target
     * @param[out] target_point2f Target 2d point value
     *
     * @return bool Transform result
     */
    static bool transformation2dto2d(const OBPoint2f source_point2f, const float source_depth_pixel_value, const OBCameraIntrinsic source_intrinsic,
                                     const OBCameraDistortion source_distortion, const OBCameraIntrinsic target_intrinsic,
                                     const OBCameraDistortion target_distortion, OBExtrinsic extrinsic, OBPoint2f *target_point2f) {
        ob_error *error  = NULL;
        bool      result = ob_transformation_2d_to_2d(source_point2f, source_depth_pixel_value, source_intrinsic, source_distortion, target_intrinsic,
                                                      target_distortion, extrinsic, target_point2f, &error);
        Error::handle(&error);
        return result;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    static bool calibration3dTo3d(const OBCalibrationParam calibrationParam, const OBPoint3f sourcePoint3f, const OBSensorType sourceSensorType,
                                  const OBSensorType targetSensorType, OBPoint3f *targetPoint3f) {
        ob_error *error  = NULL;
        bool      result = ob_calibration_3d_to_3d(calibrationParam, sourcePoint3f, sourceSensorType, targetSensorType, targetPoint3f, &error);
        Error::handle(&error);
        return result;
    }

    static bool calibration2dTo3d(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                  const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint3f *targetPoint3f) {
        ob_error *error = NULL;
        bool      result =
            ob_calibration_2d_to_3d(calibrationParam, sourcePoint2f, sourceDepthPixelValue, sourceSensorType, targetSensorType, targetPoint3f, &error);
        Error::handle(&error);
        return result;
    }

    static bool calibration3dTo2d(const OBCalibrationParam calibrationParam, const OBPoint3f sourcePoint3f, const OBSensorType sourceSensorType,
                                  const OBSensorType targetSensorType, OBPoint2f *targetPoint2f) {
        ob_error *error  = NULL;
        bool      result = ob_calibration_3d_to_2d(calibrationParam, sourcePoint3f, sourceSensorType, targetSensorType, targetPoint2f, &error);
        Error::handle(&error);
        return result;
    }

    static bool calibration2dTo2d(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                  const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint2f *targetPoint2f) {
        ob_error *error = NULL;
        bool      result =
            ob_calibration_2d_to_2d(calibrationParam, sourcePoint2f, sourceDepthPixelValue, sourceSensorType, targetSensorType, targetPoint2f, &error);
        Error::handle(&error);
        return result;
    }

    static std::shared_ptr<ob::Frame> transformationDepthFrameToColorCamera(std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Frame> depthFrame,
                                                                            uint32_t targetColorCameraWidth, uint32_t targetColorCameraHeight) {
        ob_error *error = NULL;

        // unsafe operation, need to cast const to non-const
        auto unConstImpl = const_cast<ob_frame *>(depthFrame->getImpl());

        auto result = transformation_depth_frame_to_color_camera(device->getImpl(), unConstImpl, targetColorCameraWidth, targetColorCameraHeight, &error);
        Error::handle(&error);
        return std::make_shared<ob::Frame>(result);
    }

    static bool transformationInitXYTables(const OBCalibrationParam calibrationParam, const OBSensorType sensorType, float *data, uint32_t *dataSize,
                                           OBXYTables *xyTables) {
        ob_error *error  = NULL;
        bool      result = transformation_init_xy_tables(calibrationParam, sensorType, data, dataSize, xyTables, &error);
        Error::handle(&error);
        return result;
    }

    static void transformationDepthToPointCloud(OBXYTables *xyTables, const void *depthImageData, void *pointCloudData) {
        ob_error *error = NULL;
        transformation_depth_to_pointcloud(xyTables, depthImageData, pointCloudData, &error);
        Error::handle(&error, false);
    }

    static void transformationDepthToRGBDPointCloud(OBXYTables *xyTables, const void *depthImageData, const void *colorImageData, void *pointCloudData) {
        ob_error *error = NULL;
        transformation_depth_to_rgbd_pointcloud(xyTables, depthImageData, colorImageData, pointCloudData, &error);
        Error::handle(&error, false);
    }
};

class PointCloudHelper {
public:
    /**
     * @brief save point cloud to ply file.
     *
     * @param[in] fileName Point cloud save path
     * @param[in] frame Point cloud frame
     * @param[in] saveBinary Binary or textual,true: binary, false: textual
     * @param[in] useMesh Save mesh or not, true: save as mesh, false: not save as mesh
     * @param[in] meshThreshold Distance threshold for creating faces in point cloud,default value :50
     *
     * @return bool save point cloud result
     */
    static bool savePointcloudToPly(const char *fileName, std::shared_ptr<ob::Frame> frame, bool saveBinary, bool useMesh, float meshThreshold) {
        ob_error *error       = NULL;
        auto      unConstImpl = const_cast<ob_frame *>(frame->getImpl());
        bool      result      = ob_save_pointcloud_to_ply(fileName, unConstImpl, saveBinary, useMesh, meshThreshold, &error);
        Error::handle(&error, false);
        return result;
    }

    /**
     * @brief Save LiDAR point cloud to PLY file.
     *
     * @param[in] fileName Point cloud save path
     * @param[in] frame LiDAR point cloud frame
     * @param[in] saveBinary Binary or textual,true: binary, false: textual
     * @return bool save LiDAR point cloud result
     */
    static bool saveLiDARPointcloudToPly(const char *fileName, std::shared_ptr<ob::LiDARPointsFrame> frame, bool saveBinary) {
        ob_error *error       = NULL;
        auto      unConstImpl = const_cast<ob_frame *>(frame->getImpl());
        bool      result      = ob_save_lidar_pointcloud_to_ply(fileName, unConstImpl, saveBinary, &error);
        Error::handle(&error, false);
        return result;
    }
};
}  // namespace ob
