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
  
  std::string map_name;
  std::vector<PointInfo> points;
  std::map<std::string, std::set<std::string>> routes;
  
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
        route.second.erase(old_name);
        route.second.insert(new_name);
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
    routes[from].insert(to);
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
  
  std::vector<std::pair<std::string, std::string>> GetRoutes() {
    std::vector<std::pair<std::string, std::string>> result;
    for (const auto &from_routes : routes) {
      for (const auto &to : from_routes.second) {
        result.emplace_back(from_routes.first, to);
      }
    }
    return result;
  }
};

// 嵌套结构体的 nlohmann/json 序列化支持
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TopologyMap::PointInfo, x, y, theta, name, type);

// TopologyMap 的 nlohmann/json 序列化支持
inline void to_json(nlohmann::json& j, const TopologyMap& map) {
  // 将路径转换为数组格式以保持兼容性
  std::vector<std::pair<std::string, std::string>> routes_vec;
  for (const auto &from_routes : map.routes) {
    for (const auto &to : from_routes.second) {
      routes_vec.emplace_back(from_routes.first, to);
    }
  }
  
  j = nlohmann::json{
    {"map_name", map.map_name},
    {"points", map.points},
    {"routes", routes_vec}
  };
}

inline void from_json(const nlohmann::json& j, TopologyMap& map) {
  j.at("map_name").get_to(map.map_name);
  j.at("points").get_to(map.points);
  
  map.routes.clear();
  auto routes_array = j.at("routes").get<std::vector<std::pair<std::string, std::string>>>();
  for (const auto &route : routes_array) {
    map.routes[route.first].insert(route.second);
  }
}
