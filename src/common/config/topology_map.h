#pragma once
#include "json_struct/json_struct.h"
#include "point/point_type.h"

JS_ENUM(PointType, NavGoal);
JS_ENUM_DECLARE_STRING_PARSER(PointType)
struct TopologyMap {
  struct PointInfo {
    double x;
    double y;
    double theta;
    std::string name;
    PointType type{PointType::NavGoal};
    JS_OBJ(x, y, theta, name, type);
    PointInfo() {}
    PointInfo(double _x, double _y, double _theta, std::string _name)
        : x(_x), y(_y), theta(_theta), name(_name) {}
    PointInfo(basic::RobotPose pose, std::string _name)
        : x(pose.x), y(pose.y), theta(pose.theta), name(_name) {}
    basic::RobotPose ToRobotPose() { return basic::RobotPose(x, y, theta); }
    bool FromRobotPose(const basic::RobotPose &pose) {
      x = pose.x;
      y = pose.y;
      theta = pose.theta;
      return true;
    }
  };
  std::string map_name;
  std::vector<PointInfo> points;
  JS_OBJ(map_name, points);
  void AddPoint(const PointInfo &point) { points.push_back(point); }
  void RemovePoint(const std::string &name) {
    auto it =
        std::find_if(points.begin(), points.end(), [&](const PointInfo &point) {
          return point.name == name;
        });
    if (it != points.end()) {
      points.erase(it);
    }
  }
  void UpdatePoint(const std::string &name, const PointInfo &point) {
    auto it =
        std::find_if(points.begin(), points.end(), [&](const PointInfo &point) {
          return point.name == name;
        });
    if (it != points.end()) {
      *it = point;
    }
  }
  PointInfo GetPoint(const std::string &name) {
    auto it =
        std::find_if(points.begin(), points.end(), [&](const PointInfo &point) {
          return point.name == name;
        });
    if (it != points.end()) {
      return *it;
    }
    // 如果找不到点，返回一个空的PointInfo对象
    return PointInfo();
  }
  std::vector<PointInfo> GetPoints() { return points; }
};
