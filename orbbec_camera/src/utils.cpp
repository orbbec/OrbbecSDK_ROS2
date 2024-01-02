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
namespace orbbec_camera {
sensor_msgs::msg::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                                 OBCameraDistortion distortion, int width) {
  (void)width;
  sensor_msgs::msg::CameraInfo info;
  info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
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
  size_t point_size = frame->dataSize() / sizeof(OBColorPoint);
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

  const auto *points = (OBColorPoint *)frame->data();
  CHECK_NOTNULL(points);
  for (size_t i = 0; i < point_size; i++) {
    fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", points[i].x, points[i].y, points[i].z,
            (int)points[i].r, (int)points[i].g, (int)points[i].b);
  }
}

void saveRGBPointCloudMsgToPly(const sensor_msgs::msg::PointCloud2 &msg,
                               const std::string &fileName) {
  FILE *fp = fopen(fileName.c_str(), "wb+");
  CHECK_NOTNULL(fp);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  // First, count the actual number of valid points
  size_t valid_points = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      ++valid_points;
    }
  }

  // Reset the iterators
  iter_x = sensor_msgs::PointCloud2ConstIterator<float>(msg, "x");
  iter_y = sensor_msgs::PointCloud2ConstIterator<float>(msg, "y");
  iter_z = sensor_msgs::PointCloud2ConstIterator<float>(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(msg, "r");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(msg, "g");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(msg, "b");

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

void saveDepthPointsToPly(const sensor_msgs::msg::PointCloud2 &msg, const std::string &fileName) {
  FILE *fp = fopen(fileName.c_str(), "wb+");
  CHECK_NOTNULL(fp);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  // First, count the actual number of valid points
  size_t valid_points = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      ++valid_points;
    }
  }

  // Reset the iterators
  iter_x = sensor_msgs::PointCloud2ConstIterator<float>(msg, "x");
  iter_y = sensor_msgs::PointCloud2ConstIterator<float>(msg, "y");
  iter_z = sensor_msgs::PointCloud2ConstIterator<float>(msg, "z");

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
  size_t point_size = frame->dataSize() / sizeof(OBPoint);
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
  fprintf(fp, "end_header\n");

  const auto *points = (OBPoint *)frame->data();
  CHECK_NOTNULL(points);
  for (size_t i = 0; i < point_size; i++) {
    fprintf(fp, "%.3f %.3f %.3f\n", points[i].x, points[i].y, points[i].z);
  }
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
  } else {
    return OB_FORMAT_UNKNOWN;
  }
}

std::string ObDeviceTypeToString(const OBDeviceType &type) {
  switch (type) {
    case OBDeviceType::OB_STRUCTURED_LIGHT_BINOCULAR_CAMERA:
      return "structured light binocular camera";
    case OBDeviceType::OB_STRUCTURED_LIGHT_MONOCULAR_CAMERA:
      return "structured light monocular camera";
    case OBDeviceType::OB_TOF_CAMERA:
      return "tof camera";
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
      0x0699, 0x069a, 0x055c, 0x065e, 0x069a, 0x069f, 0x06a0};

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
    default:
      return "2g";
  }
}

std::string parseUsbPort(const std::string &line) {
  std::string port_id;
  std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*",
                        std::regex_constants::ECMAScript);
  std::smatch base_match;
  bool found = std::regex_match(line, base_match, self_regex);
  if (found) {
    port_id = base_match[1].str();
    if (base_match[2].str().empty())  // This is libuvc string. Remove counter is exists.
    {
      std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
      bool found_end = std::regex_match(port_id, base_match, end_regex);
      if (found_end) {
        port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
      }
    }
  }
  return port_id;
}

bool isValidJPEG(const std::shared_ptr<ob::ColorFrame> &frame) {
  if (frame->dataSize() < 2) {  // Checking both start and end markers, so minimal size is 4
    return false;
  }

  const auto *data = static_cast<const uint8_t *>(frame->data());

  // Check for JPEG start marker
  if (data[0] != 0xFF || data[1] != 0xD8) {
    return false;
  }
  return true;
}

}  // namespace orbbec_camera
