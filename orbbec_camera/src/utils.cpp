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
  uint64_t  nano_sec = total % 1000000000;
  rclcpp::Time stamp(sec, nano_sec);
  return stamp;
}
}  // namespace orbbec_camera
