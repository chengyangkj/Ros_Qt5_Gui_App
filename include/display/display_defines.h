#pragma once

namespace Display {
using LaserDataMap = std::map<int, std::vector<Eigen::Vector2f>>;
using LaserData = std::vector<Eigen::Vector2f>;
using LaserColorMap = std::map<int, QColor>;
using ParticlePointsType = std::vector<Eigen::Vector3f>;
using RangeVec = Eigen::Vector4f;  // xmin,ymin,xmax,ymax
using Pose3f = Eigen::Vector3f;
using Point2f = Eigen::Vector2f;
using PathData = std::vector<Point2f>;
using RegionDataMap = std::map<std::string, std::vector<RangeVec>>;
using TagDataMap = std::map<std::string, Pose3f>;
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
}  // namespace Display
