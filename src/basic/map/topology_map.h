#pragma once
#include <nlohmann/json.hpp>
#include "point/point_type.h"
#include <set>
#include <map>

enum class PointType {
  NavGoal
};

// nlohmann/json 序列化支持
NLOHMANN_JSON_SERIALIZE_ENUM(PointType, {
  {PointType::NavGoal, "NavGoal"}
});


struct TopologyMap {
  // 路径属性结构体
  struct RouteInfo {
    std::string controller{"FollowPath"};
    double speed_limit{1.0};  // 默认最大速度1.0
    RouteInfo() = default;
    RouteInfo(std::string ctrl) 
        :  controller(ctrl) {}
    RouteInfo(std::string ctrl, double speed) 
        :  controller(ctrl), speed_limit(speed) {}
    
  };
  
  struct PointInfo {
    double x;
    double y;
    double theta;
    std::string name;
    PointType type{PointType::NavGoal};
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

  //属性
  struct PropertyInfo {
    std::vector<std::string> support_controllers{"FollowPath","Backward","MPPI"};
  };
  
  std::string map_name;
  PropertyInfo map_property;
  std::vector<PointInfo> points;

  //from->to
  std::map<std::string, std::map<std::string, RouteInfo>> routes;

  
  void AddPoint(const PointInfo &point) { points.push_back(point); }
  void RemovePoint(const std::string &name) {
    auto it =
        std::find_if(points.begin(), points.end(), [&](const PointInfo &point) {
          return point.name == name;
        });
    if (it != points.end()) {
      points.erase(it);
    }
    
    // 删除相关的路径
    routes.erase(name);  // 删除以该点为起点的所有路径
    // 删除以该点为终点的所有路径
    for (auto &route : routes) {
      route.second.erase(name);
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

  void UpdatePointName(const std::string &old_name, const std::string &new_name) {
    auto it = std::find_if(points.begin(), points.end(), [&](const PointInfo &point) {
      return point.name == old_name;
    });
    if (it != points.end()) {
      it->name = new_name;
    }
    
    // 更新路径中的点名称
    if (routes.count(old_name) > 0) {
      auto destinations = routes[old_name];
      routes.erase(old_name);
      routes[new_name] = destinations;
    }
    
    // 更新其他路径中的终点名称
    for (auto &route : routes) {
      if (route.second.count(old_name) > 0) {
        auto route_info = route.second[old_name];
        route.second.erase(old_name);
        route.second[new_name] = route_info;
      }
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
    return PointInfo();
  }
  
  std::vector<PointInfo> GetPoints() { return points; }
  
  // 路径管理方法
  void AddRoute(const std::string &from, const std::string &to) {
    if (from == to) return;  // 防止自环
    routes[from][to] = RouteInfo("FollowPath");
  }
  
  void RemoveRoute(const std::string &route_id) {
    size_t arrow_pos = route_id.find("->");
    if (arrow_pos != std::string::npos) {
      std::string from = route_id.substr(0, arrow_pos);
      std::string to = route_id.substr(arrow_pos + 2);
      RemoveRoute(from, to);
    }
  }
  
  void RemoveRoute(const std::string &from, const std::string &to) {
    if (routes.count(from) > 0) {
      routes[from].erase(to);
      if (routes[from].empty()) {
        routes.erase(from);
      }
    }
  }
  
  bool HasRoute(const std::string &from, const std::string &to) {
    return routes.count(from) > 0 && routes[from].count(to) > 0;
  }
  
  bool IsBidirectional(const std::string &from, const std::string &to) {
    return HasRoute(from, to) && HasRoute(to, from);
  }
  
  // 路径属性管理方法
  RouteInfo GetRouteInfo(const std::string &route_id) {
    size_t arrow_pos = route_id.find("->");
    if (arrow_pos != std::string::npos) {
      std::string from = route_id.substr(0, arrow_pos);
      std::string to = route_id.substr(arrow_pos + 2);
      if (routes.count(from) > 0 && routes[from].count(to) > 0) {
        return routes[from][to];
      }
    }
    return RouteInfo(); // 返回默认属性
  }
  
  void SetRouteInfo(const std::string &route_id, const RouteInfo &info) {
    size_t arrow_pos = route_id.find("->");
    if (arrow_pos != std::string::npos) {
      std::string from = route_id.substr(0, arrow_pos);
      std::string to = route_id.substr(arrow_pos + 2);
      routes[from][to] = info;
    }
  }
  
  std::vector<std::pair<std::string, std::string>> GetRoutes() {
    std::vector<std::pair<std::string, std::string>> result;
    for (const auto &from_routes : routes) {
      for (const auto &route : from_routes.second) {
        result.emplace_back(from_routes.first, route.first);
      }
    }
    return result;
  }
};

// 嵌套结构体的 nlohmann/json 序列化支持
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TopologyMap::PointInfo, x, y, theta, name, type);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TopologyMap::RouteInfo, controller, speed_limit);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TopologyMap::PropertyInfo, support_controllers);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TopologyMap, map_name, map_property,points, routes);