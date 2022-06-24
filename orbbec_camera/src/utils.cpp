/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2022 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#include "orbbec_camera/utils.h"
namespace orbbec_camera {
sensor_msgs::msg::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                                 OBCameraDistortion distortion) {
  sensor_msgs::msg::CameraInfo info;
  info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info.width = intrinsic.width;
  info.height = intrinsic.height;
  info.d.resize(5, 0.0);
  info.d[0] = distortion.k1;
  info.d[1] = distortion.k2;
  info.d[2] = distortion.k3;
  info.d[3] = distortion.k4;
  info.d[4] = distortion.k5;

  info.k.fill(0.0);
  info.k[0] = intrinsic.fx;
  info.k[2] = intrinsic.cx;
  info.k[4] = intrinsic.fy;
  info.k[5] = intrinsic.cy;
  info.k[8] = 1.0;

  info.r.fill(0.0);
  info.r[0] = 1;
  info.r[4] = 1;
  info.r[8] = 1;

  info.p.fill(0.0);
  info.p[0] = info.k[0];
  info.p[2] = info.k[2];
  info.p[5] = info.k[4];
  info.p[6] = info.k[5];
  info.p[10] = 1.0;

  return info;
}

void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
  size_t point_size = frame->dataSize() / sizeof(OBColorPoint);
  FILE *fp = fopen(fileName.c_str(), "wb+");
  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", point_size);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "property uchar red\n");
  fprintf(fp, "property uchar green\n");
  fprintf(fp, "property uchar blue\n");
  fprintf(fp, "end_header\n");

  auto *point = (OBColorPoint *)frame->data();
  for (size_t i = 0; i < point_size; i++) {
    fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", point->x, point->y, point->z, (int)point->r,
            (int)point->g, (int)point->b);
    point++;
  }

  fflush(fp);
  fclose(fp);
}

void savePointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
  size_t point_size = frame->dataSize() / sizeof(OBPoint);
  FILE *fp = fopen(fileName.c_str(), "wb+");
  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", point_size);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "end_header\n");

  auto *points = (OBPoint *)frame->data();
  for (size_t i = 0; i < point_size; i++) {
    fprintf(fp, "%.3f %.3f %.3f\n", points->x, points->y, points->z);
    points++;
  }

  fflush(fp);
  fclose(fp);
}

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) {
  Eigen::Matrix3f m;
  // We need to be careful about the order, as RS2 rotation matrix is
  // column-major, while Eigen::Matrix3f expects row-major.
  m << rotation[0], rotation[3], rotation[6], rotation[1], rotation[4], rotation[7], rotation[2],
      rotation[5], rotation[8];
  Eigen::Quaternionf q(m);
  return {q.x(), q.y(), q.z(), q.w()};
}
std::ostream &operator<<(std::ostream &os, const OBCameraParam &rhs) {
  auto depth_intrinsic = rhs.depthIntrinsic;
  auto rgb_intrinsic = rhs.rgbIntrinsic;
  os << "=====depth intrinsic=====\n";
  os << "fx : " << depth_intrinsic.fx << "\n";
  os << "fy : " << depth_intrinsic.fy << "\n";
  os << "cx : " << depth_intrinsic.cx << "\n";
  os << "cy : " << depth_intrinsic.cy << "\n";
  os << "width : " << depth_intrinsic.width << "\n";
  os << "height : " << depth_intrinsic.height << "\n";
  os << "=====rgb intrinsic=====\n";
  os << "fx : " << rgb_intrinsic.fx << "\n";
  os << "fy : " << rgb_intrinsic.fy << "\n";
  os << "cx : " << rgb_intrinsic.cx << "\n";
  os << "cy : " << rgb_intrinsic.cy << "\n";
  os << "width : " << rgb_intrinsic.width << "\n";
  os << "height : " << rgb_intrinsic.height << "\n";
  return os;
}

orbbec_camera_msgs::msg::Extrinsics obExtrinsicsToMsg(const OBD2CTransform &extrinsics,
                                                      const std::string &frame_id) {
  orbbec_camera_msgs::msg::Extrinsics msg;
  for (int i = 0; i < 9; ++i) {
    msg.rotation[i] = extrinsics.rot[i];
    if (i < 3) {
      msg.translation[i] = extrinsics.trans[i];
    }
  }

  msg.header.frame_id = frame_id;
  return msg;
}

rclcpp::Time frameTimeStampToROSTime(uint64_t ms) {
  auto total = static_cast<uint64_t>(ms * 1e6);
  uint64_t sec = total / 1000000000;
  uint64_t nano_sec = total % 1000000000;
  rclcpp::Time stamp(sec, nano_sec);
  return stamp;
}
std::string getObSDKVersion() {
  std::string major = std::to_string(ob::Version::getMajor());
  std::string minor = std::to_string(ob::Version::getMinor());
  std::string patch = std::to_string(ob::Version::getPatch());
  std::string version = major + "." + minor + "." + patch;
  return version;
}
OBFormat OBFormatFromString(const std::string &format) {
  std::string fixed_format;
  std::transform(format.begin(), format.end(), std::back_inserter(fixed_format),
                 [](const auto ch) { return std::isalpha(ch) ? toupper(ch) : ch; });
  if (fixed_format == "YUYV") {
    return OB_FORMAT_YUYV;
  } else if (fixed_format == "YUYV2") {
    return OB_FORMAT_YUY2;
  } else if (fixed_format == "UYVY") {
    return OB_FORMAT_UYVY;
  } else if (fixed_format == "NV12") {
    return OB_FORMAT_NV12;
  } else if (fixed_format == "NV21") {
    return OB_FORMAT_NV21;
  } else if (fixed_format == "H264") {
    return OB_FORMAT_H264;
  } else if (fixed_format == "H265") {
    return OB_FORMAT_H265;
  } else if (fixed_format == "Y16") {
    return OB_FORMAT_Y16;
  } else if (fixed_format == "Y8") {
    return OB_FORMAT_Y8;
  } else if (fixed_format == "Y10") {
    return OB_FORMAT_Y10;
  } else if (fixed_format == "Y11") {
    return OB_FORMAT_Y11;
  } else if (fixed_format == "Y12") {
    return OB_FORMAT_Y12;
  } else if (fixed_format == "GRAY") {
    return OB_FORMAT_GRAY;
  } else if (fixed_format == "HEVC") {
    return OB_FORMAT_HEVC;
  } else if (fixed_format == "I420") {
    return OB_FORMAT_I420;
  } else if (fixed_format == "ACCEL") {
    return OB_FORMAT_ACCEL;
  } else if (fixed_format == "GYRO") {
    return OB_FORMAT_GYRO;
  } else if (fixed_format == "POINT") {
    return OB_FORMAT_POINT;
  } else if (fixed_format == "RGB_POINT") {
    return OB_FORMAT_RGB_POINT;
  } else if (fixed_format == "REL") {
    return OB_FORMAT_RLE;
  } else if (fixed_format == "RGB888") {
    return OB_FORMAT_RGB888;
  } else if (fixed_format == "BGR") {
    return OB_FORMAT_BGR;
  } else if (fixed_format == "Y14") {
    return OB_FORMAT_Y14;
  } else {
    return OB_FORMAT_UNKNOWN;
  }
}

}  // namespace orbbec_camera
