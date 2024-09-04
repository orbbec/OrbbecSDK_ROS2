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

class  CoordinateTransformHelper {
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
        ob_error *error = NULL;
        bool result =  ob_transformation_3d_to_3d(source_point3f, extrinsic, target_point3f, &error);
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
        ob_error *error = NULL;
        bool result = ob_transformation_2d_to_3d(source_point2f, source_depth_pixel_value, source_intrinsic, extrinsic, target_point3f, &error);
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
        ob_error *error = NULL;
        bool result = ob_transformation_3d_to_2d(source_point3f, target_intrinsic, target_distortion, extrinsic, target_point2f, &error);
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
    static bool transformation2dto2d(const OBPoint2f source_point2f, const float source_depth_pixel_value,const OBCameraIntrinsic source_intrinsic, 
                                const OBCameraDistortion source_distortion,  const OBCameraIntrinsic target_intrinsic,
                                const OBCameraDistortion target_distortion,OBExtrinsic extrinsic, OBPoint2f *target_point2f)  {
        ob_error *error = NULL;
        bool result = ob_transformation_2d_to_2d(source_point2f, source_depth_pixel_value, source_intrinsic, source_distortion, target_intrinsic, 
                                            target_distortion, extrinsic, target_point2f, &error);
        Error::handle(&error);
        return result;
    }

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
                                                                            uint32_t targetColorCameraWidth, uint32_t targetColorCameraHeight) {
        ob_error *error = NULL;

        // unsafe operation, need to cast const to non-const
        auto unConstImpl = const_cast<ob_frame *>(depthFrame->getImpl());

        auto result = transformation_depth_frame_to_color_camera(device->getImpl() , unConstImpl, targetColorCameraWidth, targetColorCameraHeight, &error);
        Error::handle(&error);  
        return std::make_shared<ob::Frame>(result);
    }

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
                                           OBXYTables *xyTables) {
        ob_error *error = NULL;
        bool result = transformation_init_xy_tables(calibrationParam, sensorType, data, dataSize, xyTables, &error);
        Error::handle(&error);
        return result;
    }

    /**
     * @brief Transform depth image to point cloud data
     *
     * @param xyTables input xy tables,see CoordinateTransformHelper::transformationInitXYTables
     * @param depthImageData input depth image data
     * @param pointCloudData output point cloud data
     *
     */
    static void transformationDepthToPointCloud(OBXYTables *xyTables, const void *depthImageData, void *pointCloudData) {
        ob_error *error = NULL;
        transformation_depth_to_pointcloud(xyTables, depthImageData, pointCloudData, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Transform depth image to RGBD point cloud data
     *
     * @param xyTables input xy tables,see CoordinateTransformHelper::transformationInitXYTables
     * @param depthImageData input depth image data
     * @param colorImageData input color image data (only RGB888 support)
     * @param pointCloudData output RGBD point cloud data
     *
     */
    static void transformationDepthToRGBDPointCloud(OBXYTables *xyTables, const void *depthImageData, const void *colorImageData, void *pointCloudData){
        ob_error *error = NULL;
        transformation_depth_to_rgbd_pointcloud(xyTables, depthImageData, colorImageData, pointCloudData, &error); 
        Error::handle(&error, false);  
    }


    // \deprecated This function is deprecated and will be removed in a future version.  
    //             Use the transformation3dto3d instead.  
    OB_DEPRECATED static bool calibration3dTo3d(const OBCalibrationParam calibrationParam, const OBPoint3f sourcePoint3f, const OBSensorType sourceSensorType,
                                  const OBSensorType targetSensorType, OBPoint3f *targetPoint3f) {
        ob_error *error = NULL;
        bool result = ob_calibration_3d_to_3d(calibrationParam, sourcePoint3f, sourceSensorType, targetSensorType, targetPoint3f, &error);
        Error::handle(&error);
        return result;
    }
    // \deprecated This function is deprecated and will be removed in a future version.  
    //             Use the transformation2dto3d instead. 
    OB_DEPRECATED static bool calibration2dTo3d(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                  const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint3f *targetPoint3f) {
        ob_error *error = NULL;
        bool result = ob_calibration_2d_to_3d(calibrationParam, sourcePoint2f, sourceDepthPixelValue, sourceSensorType, targetSensorType, targetPoint3f, &error);
        Error::handle(&error);
        return result;
    }

    // \deprecated This function is deprecated and will be removed in a future version.  
    //             Use the transformation3dto2d instead.
    OB_DEPRECATED static bool calibration3dTo2d(const OBCalibrationParam calibrationParam, const OBPoint3f sourcePoint3f, const OBSensorType sourceSensorType,
                                  const OBSensorType targetSensorType, OBPoint2f *targetPoint2f) {
        ob_error *error = NULL;
        bool result = ob_calibration_3d_to_2d(calibrationParam, sourcePoint3f, sourceSensorType, targetSensorType, targetPoint2f, &error);
        Error::handle(&error);
        return result;
    }
    // \deprecated This function is deprecated and will be removed in a future version.  
    //             Use the transformation2dto2d instead.
    OB_DEPRECATED static bool calibration2dTo2d(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                  const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint2f *targetPoint2f) {
        ob_error *error = NULL;
        bool result = ob_calibration_2d_to_2d(calibrationParam, sourcePoint2f, sourceDepthPixelValue, sourceSensorType, targetSensorType, targetPoint2f, &error);
        Error::handle(&error);
        return result;
    }
};
}  // namespace ob
