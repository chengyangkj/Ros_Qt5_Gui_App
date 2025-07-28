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

// 路径方向枚举
enum class RouteDirection {
  Forward,    // 前进
  Backward    // 后退
};

// nlohmann/json 序列化支持
NLOHMANN_JSON_SERIALIZE_ENUM(RouteDirection, {
  {RouteDirection::Forward, "Forward"},
  {RouteDirection::Backward, "Backward"}
});

// 控制器类型枚举
enum class ControllerType {
  MPPI,       // MPPI控制器
  DWB         // DWB控制器
};

// nlohmann/json 序列化支持
NLOHMANN_JSON_SERIALIZE_ENUM(ControllerType, {
  {ControllerType::MPPI, "MPPI"},
  {ControllerType::DWB, "DWB"}
});

struct TopologyMap {
  // 路径属性结构体
  struct RouteInfo {
    RouteDirection direction{RouteDirection::Forward};
    ControllerType controller{ControllerType::MPPI};
    
    RouteInfo() = default;
    RouteInfo(RouteDirection dir, ControllerType ctrl) 
        : direction(dir), controller(ctrl) {}
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
  
  std::string map_name;
  std::vector<PointInfo> points;
  std::map<std::string, std::set<std::string>> routes;
  // 路径属性映射：route_id -> RouteInfo
  std::map<std::string, RouteInfo> route_properties;
  
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
    
    // 删除相关的路径属性
    std::vector<std::string> routes_to_remove;
    for (const auto &prop : route_properties) {
      size_t arrow_pos = prop.first.find("->");
      if (arrow_pos != std::string::npos) {
        std::string from = prop.first.substr(0, arrow_pos);
        std::string to = prop.first.substr(arrow_pos + 2);
        if (from == name || to == name) {
          routes_to_remove.push_back(prop.first);
        }
      }
    }
    for (const auto &route_id : routes_to_remove) {
      route_properties.erase(route_id);
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
    
    // 更新路径属性中的点名称
    std::map<std::string, RouteInfo> new_properties;
    for (const auto &prop : route_properties) {
      size_t arrow_pos = prop.first.find("->");
      if (arrow_pos != std::string::npos) {
        std::string from = prop.first.substr(0, arrow_pos);
        std::string to = prop.first.substr(arrow_pos + 2);
        
        std::string new_from = (from == old_name) ? new_name : from;
        std::string new_to = (to == old_name) ? new_name : to;
        std::string new_route_id = new_from + "->" + new_to;
        
        new_properties[new_route_id] = prop.second;
      }
    }
    route_properties = new_properties;
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
    
    // 为新路径设置默认属性
    std::string route_id = from + "->" + to;
    if (route_properties.count(route_id) == 0) {
      route_properties[route_id] = RouteInfo();
    }
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
    
    // 删除路径属性
    std::string route_id = from + "->" + to;
    route_properties.erase(route_id);
  }
  
  bool HasRoute(const std::string &from, const std::string &to) {
    return routes.count(from) > 0 && routes[from].count(to) > 0;
  }
  
  bool IsBidirectional(const std::string &from, const std::string &to) {
    return HasRoute(from, to) && HasRoute(to, from);
  }
  
  // 路径属性管理方法
  RouteInfo GetRouteInfo(const std::string &route_id) {
    if (route_properties.count(route_id) > 0) {
      return route_properties[route_id];
    }
    return RouteInfo(); // 返回默认属性
  }
  
  void SetRouteInfo(const std::string &route_id, const RouteInfo &info) {
    route_properties[route_id] = info;
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
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TopologyMap::RouteInfo, direction, controller);

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
    {"routes", routes_vec},
    {"route_properties", map.route_properties}
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
  
  // 读取路径属性（如果存在）
  if (j.contains("route_properties")) {
    j.at("route_properties").get_to(map.route_properties);
  }
}
