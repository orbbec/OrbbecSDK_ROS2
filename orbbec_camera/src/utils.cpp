/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include <regex>
#include "orbbec_camera/utils.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "orbbec_camera/constants.h"
namespace orbbec_camera {
sensor_msgs::msg::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                                 OBCameraDistortion distortion, int width) {
  (void)width;
  sensor_msgs::msg::CameraInfo info;
  info.distortion_model = getDistortionModels(distortion);
  info.width = intrinsic.width;
  info.height = intrinsic.height;
  info.d.resize(8, 0.0);
  info.d[0] = distortion.k1;
  info.d[1] = distortion.k2;
  info.d[2] = distortion.p1;
  info.d[3] = distortion.p2;
  info.d[4] = distortion.k3;
  info.d[5] = distortion.k4;
  info.d[6] = distortion.k5;
  info.d[7] = distortion.k6;
  bool all_zero = std::all_of(info.d.begin(), info.d.end(), [](double val) { return val == 0.0; });
  info.roi.do_rectify = all_zero;

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

void saveRGBPointsToPly(const std::shared_ptr<ob::Frame> &frame, const std::string &fileName) {
  size_t point_size = frame->getDataSize() / sizeof(OBColorPoint);
  FILE *fp = fopen(fileName.c_str(), "wb+");
  std::shared_ptr<int> fp_guard(nullptr, [&fp](int *) {
    fflush(fp);
    fclose(fp);
  });
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

  const auto *points = (OBColorPoint *)frame->getData();
  CHECK_NOTNULL(points);
  for (size_t i = 0; i < point_size; i++) {
    fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", points[i].x, points[i].y, points[i].z,
            (int)points[i].r, (int)points[i].g, (int)points[i].b);
  }
}

void saveRGBPointCloudMsgToPly(const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
                               const std::string &fileName) {
  FILE *fp = fopen(fileName.c_str(), "wb+");
  CHECK_NOTNULL(fp);
  CHECK_NOTNULL(msg);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  // First, count the actual number of valid points
  size_t valid_points = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      ++valid_points;
    }
  }

  // Reset the iterators
  iter_x = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "x");
  iter_y = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "y");
  iter_z = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(*msg, "r");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(*msg, "g");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(*msg, "b");

  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", valid_points);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "property uchar red\n");
  fprintf(fp, "property uchar green\n");
  fprintf(fp, "property uchar blue\n");
  fprintf(fp, "end_header\n");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", *iter_x, *iter_y, *iter_z, (int)*iter_r,
              (int)*iter_g, (int)*iter_b);
    }
  }

  fflush(fp);
  fclose(fp);
}

void saveDepthPointsToPly(const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
                          const std::string &fileName) {
  FILE *fp = fopen(fileName.c_str(), "wb+");
  CHECK_NOTNULL(fp);
  CHECK_NOTNULL(msg);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  // First, count the actual number of valid points
  size_t valid_points = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      ++valid_points;
    }
  }

  // Reset the iterators
  iter_x = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "x");
  iter_y = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "y");
  iter_z = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "z");

  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", valid_points);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "end_header\n");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      fprintf(fp, "%.3f %.3f %.3f\n", *iter_x, *iter_y, *iter_z);
    }
  }

  fflush(fp);
  fclose(fp);
}

void savePointsToPly(const std::shared_ptr<ob::Frame> &frame, const std::string &fileName) {
  size_t point_size = frame->getDataSize() / sizeof(OBPoint);
  FILE *fp = fopen(fileName.c_str(), "wb+");
  std::shared_ptr<int> fp_guard(nullptr, [&fp](int *) {
    fflush(fp);
    fclose(fp);
  });
  CHECK_NOTNULL(fp);
  CHECK_NOTNULL(frame);
  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", point_size);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "end_header\n");

  const auto *points = (OBPoint *)frame->getData();
  CHECK_NOTNULL(points);
  for (size_t i = 0; i < point_size; i++) {
    fprintf(fp, "%.3f %.3f %.3f\n", points[i].x, points[i].y, points[i].z);
  }
}

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) {
  Eigen::Matrix3f m;
  m << rotation[0], rotation[1], rotation[2], rotation[3], rotation[4], rotation[5], rotation[6],
      rotation[7], rotation[8];
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
      msg.translation[i] = extrinsics.trans[i] / 1000.0;
    }
  }

  msg.header.frame_id = frame_id;
  return msg;
}

rclcpp::Time fromMsToROSTime(uint64_t ms) {
  auto total = static_cast<uint64_t>(ms * 1e6);
  uint64_t sec = total / 1000000000;
  uint64_t nano_sec = total % 1000000000;
  rclcpp::Time stamp(sec, nano_sec);
  return stamp;
}

rclcpp::Time fromUsToROSTime(uint64_t us) {
  auto total = static_cast<uint64_t>(us * 1e3);
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
  if (format.empty()) {
    return OB_FORMAT_UNKNOWN;
  }
  std::string fixed_format;
  std::transform(format.begin(), format.end(), std::back_inserter(fixed_format),
                 [](const auto ch) { return std::isalpha(ch) ? toupper(ch) : ch; });
  std::cout << "OBFormatFromString: " << fixed_format << std::endl;
  if (fixed_format == "MJPG") {
    return OB_FORMAT_MJPG;
  } else if (fixed_format == "MJPEG") {
    return OB_FORMAT_MJPEG;
  } else if (fixed_format == "YUYV") {
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
  } else if (fixed_format == "RGB888" || fixed_format == "RGB") {
    return OB_FORMAT_RGB888;
  } else if (fixed_format == "BGR") {
    return OB_FORMAT_BGR;
  } else if (fixed_format == "Y14") {
    return OB_FORMAT_Y14;
  } else if (fixed_format == "BGRA") {
    return OB_FORMAT_BGRA;
  } else if (fixed_format == "COMPRESSED") {
    return OB_FORMAT_COMPRESSED;
  } else if (fixed_format == "RVL") {
    return OB_FORMAT_RVL;
  } else if (fixed_format == "Z16") {
    return OB_FORMAT_Z16;
  } else if (fixed_format == "YV12") {
    return OB_FORMAT_YV12;
  } else if (fixed_format == "BA81") {
    return OB_FORMAT_BA81;
  } else if (fixed_format == "RGBA") {
    return OB_FORMAT_RGBA;
  } else if (fixed_format == "BYR2") {
    return OB_FORMAT_BYR2;
  } else if (fixed_format == "RW16") {
    return OB_FORMAT_RW16;
  } else if (fixed_format == "Y12C4") {
    return OB_FORMAT_Y12C4;
  } else if (fixed_format == "LIDAR_POINT") {
    return OB_FORMAT_LIDAR_POINT;
  } else if (fixed_format == "LIDAR_SPHERE_POINT") {
    return OB_FORMAT_LIDAR_SPHERE_POINT;
  } else if (fixed_format == "LIDAR_SCAN") {
    return OB_FORMAT_LIDAR_SCAN;
  } else if (fixed_format == "LIDAR_CALIBRATION") {
    return OB_FORMAT_LIDAR_CALIBRATION;
  }
  //   else if (fixed_format == "DISP16") {
  //     return OB_FORMAT_DISP16;
  //   }
  else {
    return OB_FORMAT_UNKNOWN;
  }
}

OBLiDARScanRate OBScanRateFromInt(const int rate) {
  std::cout << "OBScanRateFromInt: " << rate << std::endl;
  if (rate == 5) {
    return OB_LIDAR_SCAN_5HZ;
  } else if (rate == 10) {
    return OB_LIDAR_SCAN_10HZ;
  } else if (rate == 15) {
    return OB_LIDAR_SCAN_15HZ;
  } else if (rate == 20) {
    return OB_LIDAR_SCAN_20HZ;
  } else if (rate == 25) {
    return OB_LIDAR_SCAN_25HZ;
  } else if (rate == 30) {
    return OB_LIDAR_SCAN_30HZ;
  } else if (rate == 40) {
    return OB_LIDAR_SCAN_40HZ;
  } else {
    return OB_LIDAR_SCAN_UNKNOWN;
  }
}

std::string OBFormatToString(const OBFormat &format) {
  switch (format) {
    case OB_FORMAT_MJPG:
      return "MJPG";
    case OB_FORMAT_YUYV:
      return "YUYV";
    case OB_FORMAT_YUY2:
      return "YUYV2";
    case OB_FORMAT_UYVY:
      return "UYVY";
    case OB_FORMAT_NV12:
      return "NV12";
    case OB_FORMAT_NV21:
      return "NV21";
    case OB_FORMAT_H264:
      return "H264";
    case OB_FORMAT_H265:
      return "H265";
    case OB_FORMAT_Y16:
      return "Y16";
    case OB_FORMAT_Y8:
      return "Y8";
    case OB_FORMAT_Y10:
      return "Y10";
    case OB_FORMAT_Y11:
      return "Y11";
    case OB_FORMAT_Y12:
      return "Y12";
    case OB_FORMAT_GRAY:
      return "GRAY";
    case OB_FORMAT_HEVC:
      return "HEVC";
    case OB_FORMAT_I420:
      return "I420";
    case OB_FORMAT_ACCEL:
      return "ACCEL";
    case OB_FORMAT_GYRO:
      return "GYRO";
    case OB_FORMAT_POINT:
      return "POINT";
    case OB_FORMAT_RGB_POINT:
      return "RGB_POINT";
    case OB_FORMAT_RLE:
      return "REL";
    case OB_FORMAT_RGB888:
      return "RGB888";
    case OB_FORMAT_BGR:
      return "BGR";
    case OB_FORMAT_Y14:
      return "Y14";
    case OB_FORMAT_BGRA:
      return "BGRA";
    case OB_FORMAT_COMPRESSED:
      return "COMPRESSED";
    case OB_FORMAT_RVL:
      return "RVL";
    case OB_FORMAT_Z16:
      return "Z16";
    case OB_FORMAT_YV12:
      return "YV12";
    case OB_FORMAT_BA81:
      return "BA81";
    case OB_FORMAT_RGBA:
      return "RGBA";
    case OB_FORMAT_BYR2:
      return "BYR2";
    case OB_FORMAT_RW16:
      return "RW16";
    case OB_FORMAT_Y12C4:
      return "Y12C4";
    // case OB_FORMAT_DISP16:
    //   return "DISP16";
    default:
      return "UNKNOWN";
  }
}

std::ostream &operator<<(std::ostream &os, const OBFormat &rhs) {
  os << OBFormatToString(rhs);
  return os;
}

std::string ObDeviceTypeToString(const OBDeviceType &type) {
  switch (type) {
    case OBDeviceType::OB_STRUCTURED_LIGHT_BINOCULAR_CAMERA:
      return "structured light binocular camera";
    case OBDeviceType::OB_STRUCTURED_LIGHT_MONOCULAR_CAMERA:
      return "structured light monocular camera";
    case OBDeviceType::OB_TOF_CAMERA:
      return "tof camera";
    default:
      // 处理其他未预见的情况
      break;
  }
  return "unknown technology camera";
}

rmw_qos_profile_t getRMWQosProfileFromString(const std::string &str_qos) {
  std::string upper_str_qos = str_qos;
  std::transform(upper_str_qos.begin(), upper_str_qos.end(), upper_str_qos.begin(), ::toupper);
  if (upper_str_qos == "SYSTEM_DEFAULT") {
    return rmw_qos_profile_system_default;
  } else if (upper_str_qos == "DEFAULT") {
    return rmw_qos_profile_default;
  } else if (upper_str_qos == "PARAMETER_EVENTS") {
    return rmw_qos_profile_parameter_events;
  } else if (upper_str_qos == "SERVICES_DEFAULT") {
    return rmw_qos_profile_services_default;
  } else if (upper_str_qos == "PARAMETERS") {
    return rmw_qos_profile_parameters;
  } else if (upper_str_qos == "SENSOR_DATA") {
    return rmw_qos_profile_sensor_data;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("astra_camera"),
                        "Invalid QoS profile: " << upper_str_qos << ". Using default QoS profile.");
    return rmw_qos_profile_default;
  }
}

bool isOpenNIDevice(int pid) {
  static const std::vector<int> OPENNI_DEVICE_PIDS = {
      0x0300, 0x0301, 0x0400, 0x0401, 0x0402, 0x0403, 0x0404, 0x0407, 0x0601, 0x060b, 0x060e,
      0x060f, 0x0610, 0x0613, 0x0614, 0x0616, 0x0617, 0x0618, 0x061b, 0x062b, 0x062c, 0x062d,
      0x0632, 0x0633, 0x0634, 0x0635, 0x0636, 0x0637, 0x0638, 0x0639, 0x063a, 0x0650, 0x0651,
      0x0654, 0x0655, 0x0656, 0x0657, 0x0658, 0x0659, 0x065a, 0x065b, 0x065c, 0x065d, 0x0698,
      0x0699, 0x069a, 0x055c, 0x065e, 0x069a, 0x069f, 0x06a0, 0x069e};

  return std::any_of(OPENNI_DEVICE_PIDS.begin(), OPENNI_DEVICE_PIDS.end(),
                     [pid](int pid_openni) { return pid == pid_openni; });
}

OB_DEPTH_PRECISION_LEVEL depthPrecisionLevelFromString(
    const std::string &depth_precision_level_str) {
  if (depth_precision_level_str == "1mm") {
    return OB_PRECISION_1MM;
  } else if (depth_precision_level_str == "0.8mm") {
    return OB_PRECISION_0MM8;
  } else if (depth_precision_level_str == "0.4mm") {
    return OB_PRECISION_0MM4;
  } else if (depth_precision_level_str == "0.2mm") {
    return OB_PRECISION_0MM2;
  } else if (depth_precision_level_str == "0.1mm") {
    return OB_PRECISION_0MM1;
  } else {
    return OB_PRECISION_0MM8;
  }
}

float depthPrecisionFromString(const std::string &depth_precision_level_str) {
  // covert 0.8mm to 0.8
  if (depth_precision_level_str.size() < 2) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("utils"),
                       "Invalid depth precision level: " << depth_precision_level_str
                                                         << ". Using default precision level 1mm");
    return 1.0;
  }
  std::string depth_precision_level_str_num =
      depth_precision_level_str.substr(0, depth_precision_level_str.size() - 2);
  return std::stof(depth_precision_level_str_num);
}

OBMultiDeviceSyncMode OBSyncModeFromString(const std::string &mode) {
  if (mode == "FREE_RUN") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
  } else if (mode == "STANDALONE") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_STANDALONE;
  } else if (mode == "PRIMARY") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_PRIMARY;
  } else if (mode == "SECONDARY") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SECONDARY;
  } else if (mode == "SECONDARY_SYNCED") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED;
  } else if (mode == "SOFTWARE_TRIGGERING") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING;
  } else if (mode == "HARDWARE_TRIGGERING") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING;
  } else {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
  }
}

OB_SAMPLE_RATE sampleRateFromString(std::string &sample_rate) {
  // covert to lower case
  std::transform(sample_rate.begin(), sample_rate.end(), sample_rate.begin(), ::tolower);
  if (sample_rate == "1.5625hz") {
    return OB_SAMPLE_RATE_1_5625_HZ;
  } else if (sample_rate == "3.125hz") {
    return OB_SAMPLE_RATE_3_125_HZ;
  } else if (sample_rate == "6.25hz") {
    return OB_SAMPLE_RATE_6_25_HZ;
  } else if (sample_rate == "12.5hz") {
    return OB_SAMPLE_RATE_12_5_HZ;
  } else if (sample_rate == "25hz") {
    return OB_SAMPLE_RATE_25_HZ;
  } else if (sample_rate == "50hz") {
    return OB_SAMPLE_RATE_50_HZ;
  } else if (sample_rate == "100hz") {
    return OB_SAMPLE_RATE_100_HZ;
  } else if (sample_rate == "200hz") {
    return OB_SAMPLE_RATE_200_HZ;
  } else if (sample_rate == "500hz") {
    return OB_SAMPLE_RATE_500_HZ;
  } else if (sample_rate == "1khz") {
    return OB_SAMPLE_RATE_1_KHZ;
  } else if (sample_rate == "2khz") {
    return OB_SAMPLE_RATE_2_KHZ;
  } else if (sample_rate == "4khz") {
    return OB_SAMPLE_RATE_4_KHZ;
  } else if (sample_rate == "8khz") {
    return OB_SAMPLE_RATE_8_KHZ;
  } else if (sample_rate == "16khz") {
    return OB_SAMPLE_RATE_16_KHZ;
  } else if (sample_rate == "32khz") {
    return OB_SAMPLE_RATE_32_KHZ;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("utils"), "Unknown OB_SAMPLE_RATE: " << sample_rate);
    return OB_SAMPLE_RATE_100_HZ;
  }
}

std::string sampleRateToString(const OB_SAMPLE_RATE &sample_rate) {
  switch (sample_rate) {
    case OB_SAMPLE_RATE_1_5625_HZ:
      return "1.5625hz";
    case OB_SAMPLE_RATE_3_125_HZ:
      return "3.125hz";
    case OB_SAMPLE_RATE_6_25_HZ:
      return "6.25hz";
    case OB_SAMPLE_RATE_12_5_HZ:
      return "12.5hz";
    case OB_SAMPLE_RATE_25_HZ:
      return "25hz";
    case OB_SAMPLE_RATE_50_HZ:
      return "50hz";
    case OB_SAMPLE_RATE_100_HZ:
      return "100hz";
    case OB_SAMPLE_RATE_200_HZ:
      return "200hz";
    case OB_SAMPLE_RATE_500_HZ:
      return "500hz";
    case OB_SAMPLE_RATE_1_KHZ:
      return "1khz";
    case OB_SAMPLE_RATE_2_KHZ:
      return "2khz";
    case OB_SAMPLE_RATE_4_KHZ:
      return "4khz";
    case OB_SAMPLE_RATE_8_KHZ:
      return "8khz";
    case OB_SAMPLE_RATE_16_KHZ:
      return "16khz";
    case OB_SAMPLE_RATE_32_KHZ:
      return "32khz";
    default:
      return "100hz";
  }
}

std::ostream &operator<<(std::ostream &os, const OB_SAMPLE_RATE &rhs) {
  os << sampleRateToString(rhs);
  return os;
}

OB_GYRO_FULL_SCALE_RANGE fullGyroScaleRangeFromString(std::string &full_scale_range) {
  std::transform(full_scale_range.begin(), full_scale_range.end(), full_scale_range.begin(),
                 ::tolower);
  if (full_scale_range == "16dps") {
    return OB_GYRO_FS_16dps;
  } else if (full_scale_range == "31dps") {
    return OB_GYRO_FS_31dps;
  } else if (full_scale_range == "62dps") {
    return OB_GYRO_FS_62dps;
  } else if (full_scale_range == "125dps") {
    return OB_GYRO_FS_125dps;
  } else if (full_scale_range == "250dps") {
    return OB_GYRO_FS_250dps;
  } else if (full_scale_range == "500dps") {
    return OB_GYRO_FS_500dps;
  } else if (full_scale_range == "1000dps") {
    return OB_GYRO_FS_1000dps;
  } else if (full_scale_range == "2000dps") {
    return OB_GYRO_FS_2000dps;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("utils"),
                        "Unknown OB_GYRO_FULL_SCALE_RANGE: " << full_scale_range);
    return OB_GYRO_FS_2000dps;
  }
}

std::string fullGyroScaleRangeToString(const OB_GYRO_FULL_SCALE_RANGE &full_scale_range) {
  switch (full_scale_range) {
    case OB_GYRO_FS_16dps:
      return "16dps";
    case OB_GYRO_FS_31dps:
      return "31dps";
    case OB_GYRO_FS_62dps:
      return "62dps";
    case OB_GYRO_FS_125dps:
      return "125dps";
    case OB_GYRO_FS_250dps:
      return "250dps";
    case OB_GYRO_FS_500dps:
      return "500dps";
    case OB_GYRO_FS_1000dps:
      return "1000dps";
    case OB_GYRO_FS_2000dps:
      return "2000dps";
    default:
      return "16dps";
  }
}

std::ostream &operator<<(std::ostream &os, const OB_GYRO_FULL_SCALE_RANGE &rhs) {
  os << fullGyroScaleRangeToString(rhs);
  return os;
}

OBAccelFullScaleRange fullAccelScaleRangeFromString(std::string &full_scale_range) {
  std::transform(full_scale_range.begin(), full_scale_range.end(), full_scale_range.begin(),
                 ::tolower);
  if (full_scale_range == "2g") {
    return OB_ACCEL_FS_2g;
  } else if (full_scale_range == "4g") {
    return OB_ACCEL_FS_4g;
  } else if (full_scale_range == "8g") {
    return OB_ACCEL_FS_8g;
  } else if (full_scale_range == "16g") {
    return OB_ACCEL_FS_16g;
  } else if (full_scale_range == "3g") {
    return OB_ACCEL_FS_3g;
  } else if (full_scale_range == "6g") {
    return OB_ACCEL_FS_6g;
  } else if (full_scale_range == "12g") {
    return OB_ACCEL_FS_12g;
  } else if (full_scale_range == "24g") {
    return OB_ACCEL_FS_24g;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("utils"),
                        "Unknown OB_ACCEL_FULL_SCALE_RANGE: " << full_scale_range);
    return OB_ACCEL_FS_16g;
  }
}

std::string fullAccelScaleRangeToString(const OBAccelFullScaleRange &full_scale_range) {
  switch (full_scale_range) {
    case OB_ACCEL_FS_2g:
      return "2g";
    case OB_ACCEL_FS_4g:
      return "4g";
    case OB_ACCEL_FS_8g:
      return "8g";
    case OB_ACCEL_FS_16g:
      return "16g";
    case OB_ACCEL_FS_3g:
      return "3g";
    case OB_ACCEL_FS_6g:
      return "6g";
    case OB_ACCEL_FS_12g:
      return "12g";
    case OB_ACCEL_FS_24g:
      return "24g";
    default:
      return "2g";
  }
}

std::ostream &operator<<(std::ostream &os, const OBAccelFullScaleRange &rhs) {
  os << fullAccelScaleRangeToString(rhs);
  return os;
}

std::string parseUsbPort(const std::string &line) {
  std::string port_id;
  std::regex usb_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*",
                       std::regex_constants::ECMAScript);
  std::smatch base_match;
  bool found_usb = std::regex_match(line, base_match, usb_regex);

  if (found_usb) {
    port_id = base_match[1].str();
    std::cout << "USB port_id: " << port_id << std::endl;

    if (base_match[2].str().empty()) {
      std::regex end_regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
      bool found_end = std::regex_match(port_id, base_match, end_regex);

      if (found_end) {
        port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
        std::cout << "Modified USB port_id: " << port_id << std::endl;
      }
    }

    return port_id;
  }

  std::regex gmsl_regex("(gmsl[0-9]+)(?:-[0-9]+)*(-[0-9]+)$", std::regex_constants::ECMAScript);
  bool found_gmsl = std::regex_match(line, base_match, gmsl_regex);

  if (found_gmsl) {
    port_id = base_match[1].str() + base_match[2].str();
    std::cout << "Parsed GMSL Port ID: " << port_id << std::endl;
    return port_id;
  }

  return "";
}

bool isValidJPEG(const std::shared_ptr<ob::ColorFrame> &frame) {
  if (frame->getDataSize() < 2) {  // Checking both start and end markers, so minimal size is 4
    return false;
  }

  const auto *data = static_cast<const uint8_t *>(frame->getData());

  // Check for JPEG start marker
  if (data[0] != 0xFF || data[1] != 0xD8) {
    return false;
  }
  return true;
}

std::string metaDataTypeToString(const OBFrameMetadataType &meta_data_type) {
  switch (meta_data_type) {
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_TIMESTAMP:
      return "frame_timestamp";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP:
      return "sensor_timestamp";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_FRAME_NUMBER:
      return "frame_number";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AUTO_EXPOSURE:
      return "auto_exposure";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_EXPOSURE:
      return "exposure";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_GAIN:
      return "gain";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AUTO_WHITE_BALANCE:
      return "auto_white_balance";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_WHITE_BALANCE:
      return "white_balance";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_BRIGHTNESS:
      return "brightness";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_CONTRAST:
      return "contrast";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_SATURATION:
      return "saturation";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_SHARPNESS:
      return "sharpness";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_BACKLIGHT_COMPENSATION:
      return "backlight_compensation";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_HUE:
      return "hue";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_GAMMA:
      return "gamma";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_POWER_LINE_FREQUENCY:
      return "power_line_frequency";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_LOW_LIGHT_COMPENSATION:
      return "low_light_compensation";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_MANUAL_WHITE_BALANCE:
      return "manual_white_balance";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_ACTUAL_FRAME_RATE:
      return "actual_frame_rate";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_FRAME_RATE:
      return "frame_rate";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AE_ROI_LEFT:
      return "ae_roi_left";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AE_ROI_TOP:
      return "ae_roi_top";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AE_ROI_RIGHT:
      return "ae_roi_right";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AE_ROI_BOTTOM:
      return "ae_roi_bottom";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_EXPOSURE_PRIORITY:
      return "exposure_priority";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_NAME:
      return "hdr_sequence_name";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_SIZE:
      return "hdr_sequence_size";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX:
      return "hdr_sequence_index";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_LASER_POWER:
      return "frame_laser_power";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_LASER_POWER_MODE:
      return "frame_laser_power_mode";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_EMITTER_MODE:
      return "frame_emitter_mode";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_GPIO_INPUT_DATA:
      return "gpio_input_data";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_DISPARITY_SEARCH_OFFSET:
      return "disparity_search_offset";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_DISPARITY_SEARCH_RANGE:
      return "disparity search range";
    default:
      return "unknown_field";
  }
}

std::ostream &operator<<(std::ostream &os, const OBFrameMetadataType &rhs) {
  os << metaDataTypeToString(rhs);
  return os;
}

OBHoleFillingMode holeFillingModeFromString(const std::string &hole_filling_mode) {
  if (hole_filling_mode == "FILL_TOP") {
    return OB_HOLE_FILL_TOP;
  } else if (hole_filling_mode == "FILL_NEAREST") {
    return OB_HOLE_FILL_NEAREST;
  } else if (hole_filling_mode == "FILL_FAREST") {
    return OB_HOLE_FILL_FAREST;
  } else {
    return OB_HOLE_FILL_NEAREST;
  }
}

bool isGemini2R(int pid) {
  if (pid == GEMINI2R_PID || pid == GEMINI2RL_PID) {
    return true;
  }
  if (pid == GEMINI2R_PID2 || pid == GEMINI2RL_PID2) {
    return true;
  }
  return false;
}

OBStreamType obStreamTypeFromString(const std::string &stream_type) {
  std::string upper_stream_type = stream_type;
  std::transform(upper_stream_type.begin(), upper_stream_type.end(), upper_stream_type.begin(),
                 ::toupper);
  if (upper_stream_type == "VIDEO") {
    return OB_STREAM_VIDEO;
  } else if (upper_stream_type == "IR") {
    return OB_STREAM_IR;
  } else if (upper_stream_type == "COLOR") {
    return OB_STREAM_COLOR;
  } else if (upper_stream_type == "DEPTH") {
    return OB_STREAM_DEPTH;
  } else if (upper_stream_type == "ACCEL") {
    return OB_STREAM_ACCEL;
  } else if (upper_stream_type == "GYRO") {
    return OB_STREAM_GYRO;
  } else if (upper_stream_type == "IR_LEFT") {
    return OB_STREAM_IR_LEFT;
  } else if (upper_stream_type == "IR_RIGHT") {
    return OB_STREAM_IR_RIGHT;
  } else if (upper_stream_type == "RAW_PHASE") {
    return OB_STREAM_RAW_PHASE;
  } else {
    return OB_STREAM_UNKNOWN;
  }
}

UndistortedImageResult undistortImage(const cv::Mat &image, const OBCameraIntrinsic &intrinsic,
                                      const OBCameraDistortion &distortion) {
  UndistortedImageResult result;
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix.at<double>(0, 0) = intrinsic.fx;
  camera_matrix.at<double>(1, 1) = intrinsic.fy;
  camera_matrix.at<double>(0, 2) = intrinsic.cx;
  camera_matrix.at<double>(1, 2) = intrinsic.cy;

  // Create the distortion coefficients matrix using the extended distortion model
  cv::Mat dist_coeffs = (cv::Mat_<float>(8, 1) << distortion.k1, distortion.k2, distortion.p1,
                         distortion.p2, distortion.k3, distortion.k4, distortion.k5, distortion.k6);
  cv::Size image_size(image.cols, image.rows);
  cv::Mat new_camera_matrix =
      cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, 0.0, image_size);
  // Undistort the image using the new camera matrix
  cv::undistort(image, result.image, camera_matrix, dist_coeffs, new_camera_matrix);
  // Update the intrinsic parameters with the new camera matrix
  result.new_intrinsic = intrinsic;  // Copy original values first
  result.new_intrinsic.fx = new_camera_matrix.at<double>(0, 0);
  result.new_intrinsic.fy = new_camera_matrix.at<double>(1, 1);
  result.new_intrinsic.cx = new_camera_matrix.at<double>(0, 2);
  result.new_intrinsic.cy = new_camera_matrix.at<double>(1, 2);
  result.new_intrinsic.width = image.cols;
  result.new_intrinsic.height = image.rows;
  return result;
}
std::string getDistortionModels(OBCameraDistortion distortion) {
  switch (distortion.model) {
    case OB_DISTORTION_NONE:
      return sensor_msgs::distortion_models::PLUMB_BOB;
    case OB_DISTORTION_MODIFIED_BROWN_CONRADY:
      return sensor_msgs::distortion_models::PLUMB_BOB;
    case OB_DISTORTION_INVERSE_BROWN_CONRADY:
      return sensor_msgs::distortion_models::PLUMB_BOB;
    case OB_DISTORTION_BROWN_CONRADY:
      return sensor_msgs::distortion_models::PLUMB_BOB;
    case OB_DISTORTION_BROWN_CONRADY_K6:
      return sensor_msgs::distortion_models::PLUMB_BOB;
    case OB_DISTORTION_KANNALA_BRANDT4:
      return sensor_msgs::distortion_models::EQUIDISTANT;
    default:
      return sensor_msgs::distortion_models::PLUMB_BOB;
  }
}

std::string calcMD5(const std::string &data) {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_len = 0;

  EVP_MD_CTX *ctx = EVP_MD_CTX_new();
  EVP_DigestInit_ex(ctx, EVP_md5(), nullptr);
  EVP_DigestUpdate(ctx, data.data(), data.size());
  EVP_DigestFinal_ex(ctx, digest, &digest_len);
  EVP_MD_CTX_free(ctx);

  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < digest_len; ++i) ss << std::setw(2) << (int)digest[i];
  return ss.str();
}
double getScanAngleIncrement(OBLiDARScanRate fps) {
  switch (fps) {
    case OB_LIDAR_SCAN_15HZ:
      return deg2rad(0.075);
    case OB_LIDAR_SCAN_20HZ:
      return deg2rad(0.1);
    case OB_LIDAR_SCAN_25HZ:
      return deg2rad(0.125);
    case OB_LIDAR_SCAN_30HZ:
      return deg2rad(0.15);
    case OB_LIDAR_SCAN_40HZ:
      return deg2rad(0.2);
    default:
      return deg2rad(0.1);
  }
}

double deg2rad(double deg) { return deg * M_PI / 180.0; }

double rad2deg(double rad) {
  double angle_degrees = rad * (180.0 / M_PI);
  if (angle_degrees < 0) {
    angle_degrees += 360.0;
  }
  return angle_degrees;
}

}  // namespace orbbec_camera
