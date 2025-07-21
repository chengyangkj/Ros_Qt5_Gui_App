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
  
  struct RouteInfo {
    std::string from;
    std::string to;
    std::string route_id;       // 路径ID，用于显示和管理
    JS_OBJ(from, to, route_id);
    RouteInfo() {}
    RouteInfo(const std::string &_from, const std::string &_to)
        : from(_from), to(_to) {
      route_id = from + "->" + to;
    }
  };
  
  std::string map_name;
  std::vector<PointInfo> points;
  std::vector<RouteInfo> routes;
  JS_OBJ(map_name, points, routes);
  
  void AddPoint(const PointInfo &point) { points.push_back(point); }
  void RemovePoint(const std::string &name) {
    auto it =
        std::find_if(points.begin(), points.end(), [&](const PointInfo &point) {
          return point.name == name;
        });
    if (it != points.end()) {
      points.erase(it);
    }
    
    // 同时删除相关的路径
    routes.erase(std::remove_if(routes.begin(), routes.end(),
                               [&](const RouteInfo &route) {
                                 return route.from == name || route.to == name;
                               }), routes.end());
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
  
  // 路径管理方法
  void AddRoute(const RouteInfo &route) {
    // 检查是否已存在相同的路径
    auto it = std::find_if(routes.begin(), routes.end(),
                          [&](const RouteInfo &r) {
                            return r.from == route.from && r.to == route.to;
                          });
    if (it == routes.end()) {
      routes.push_back(route);
    }
  }
  
  void RemoveRoute(const std::string &route_id) {
    routes.erase(std::remove_if(routes.begin(), routes.end(),
                               [&](const RouteInfo &route) {
                                 return route.route_id == route_id;
                               }), routes.end());
  }
  
  RouteInfo GetRoute(const std::string &route_id) {
    auto it = std::find_if(routes.begin(), routes.end(),
                          [&](const RouteInfo &route) {
                            return route.route_id == route_id;
                          });
    if (it != routes.end()) {
      return *it;
    }
    return RouteInfo();
  }
  
  std::vector<RouteInfo> GetRoutes() { return routes; }
  
  bool HasRoute(const std::string &from, const std::string &to) {
    auto it = std::find_if(routes.begin(), routes.end(),
                          [&](const RouteInfo &route) {
                            return route.from == from && route.to == to;
                          });
    return it != routes.end();
  }
  
  // 判断两个点之间是否为双向连接（同时存在from->to和to->from两个路径）
  bool IsBidirectional(const std::string &from, const std::string &to) {
    return HasRoute(from, to) && HasRoute(to, from);
  }
};
