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
using Color =Eigen::Vector3f;
}  // namespace Display
