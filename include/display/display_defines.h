#pragma once

namespace Display {
using LaserDataMap = std::map<int, std::vector<Eigen::Vector2f>>;
using LaserColorMap = std::map<int, QColor>;
using ParticlePointsType = std::vector<Eigen::Vector3f>;
using RangeVec = Eigen::Vector4f;  // xmin,ymin,xmax,ymax
using RobotPose = Eigen::Vector3f;
using RegionDataMap = std::map<std::string, std::vector<RangeVec>>;
using TagDataMap = std::map<std::string, RobotPose>;
#ifndef RAD2DEG_ZEG
#define RAD2DEG_ZEG 57.295779513082
#endif
#ifndef PI
#define PI 3.141592653589793
#endif
#ifndef DEG2RAD_ZEG
#define DEG2RAD_ZEG 0.017453292519943
#endif
inline double deg2rad(double angle) { return angle * DEG2RAD_ZEG; }
inline double rad2deg(double angle) { return angle * RAD2DEG_ZEG; }
inline Eigen::Vector3d absoluteSum(const Eigen::Vector3d& p,
                                   const Eigen::Vector3d& d) {
  Eigen::Quaterniond q1 = Eigen::AngleAxisd(p[2], Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  Eigen::Vector3d t1 = Eigen::Vector3d(p[0], p[1], 0.0);
  Eigen::Vector3d p1 = Eigen::Vector3d(d[0], d[1], 0.0);
  Eigen::Vector3d pw;
  pw = q1 * p1 + t1;
  pw[2] += d[2];
  return pw;
}
}  // namespace Display
