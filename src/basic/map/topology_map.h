#pragma once
#include <nlohmann/json.hpp>
#include "point/point_type.h"
#include <set>

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
  
  struct RouteInfo {
    std::string from;
    std::string to;
    RouteInfo() {}
    RouteInfo(const std::string &_from, const std::string &_to)
        : from(_from), to(_to) {}
    
    // 为std::set提供比较操作符
    bool operator<(const RouteInfo &other) const {
      if (from != other.from) {
        return from < other.from;
      }
      return to < other.to;
    }
    
    // 提供相等比较操作符
    bool operator==(const RouteInfo &other) const {
      return from == other.from && to == other.to;
    }
    
    // 生成route_id用于显示（保持兼容性）
    std::string GetRouteId() const {
      return from + "->" + to;
    }
  };
  
  std::string map_name;
  std::vector<PointInfo> points;
  std::set<RouteInfo> routes;
  
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
    for (auto it = routes.begin(); it != routes.end();) {
      if (it->from == name || it->to == name) {
        it = routes.erase(it);
      } else {
        ++it;
      }
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
    
    // 更新路径中的点名称，由于set元素是const的，需要先删除再插入
    std::set<RouteInfo> updated_routes;
    for (const auto &route : routes) {
      RouteInfo new_route = route;
      if (route.from == old_name) {
        new_route.from = new_name;
      }
      if (route.to == old_name) {
        new_route.to = new_name;
      }
      updated_routes.insert(new_route);
    }
    routes = std::move(updated_routes);
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
    // 防止自环路径（from和to相同）
    if (route.from == route.to) {
      return;
    }
    // set会自动处理重复路径的去重
    routes.insert(route);
  }
  
  void RemoveRoute(const std::string &route_id) {
    // 解析route_id得到from和to（格式：from->to）
    size_t arrow_pos = route_id.find("->");
    if (arrow_pos != std::string::npos) {
      std::string from = route_id.substr(0, arrow_pos);
      std::string to = route_id.substr(arrow_pos + 2);
      RouteInfo route_to_remove(from, to);
      routes.erase(route_to_remove);
    }
  }
  
  void RemoveRoute(const std::string &from, const std::string &to) {
    RouteInfo route_to_remove(from, to);
    routes.erase(route_to_remove);
  }
  
  RouteInfo GetRoute(const std::string &route_id) {
    // 解析route_id得到from和to（格式：from->to）
    size_t arrow_pos = route_id.find("->");
    if (arrow_pos != std::string::npos) {
      std::string from = route_id.substr(0, arrow_pos);
      std::string to = route_id.substr(arrow_pos + 2);
      RouteInfo route_to_find(from, to);
      auto it = routes.find(route_to_find);
      if (it != routes.end()) {
        return *it;
      }
    }
    return RouteInfo();
  }
  
  RouteInfo GetRoute(const std::string &from, const std::string &to) {
    RouteInfo route_to_find(from, to);
    auto it = routes.find(route_to_find);
    if (it != routes.end()) {
      return *it;
    }
    return RouteInfo();
  }
  
  // 检查是否存在特定路径
  bool HasRoute(const std::string &from, const std::string &to) {
    RouteInfo route_to_find(from, to);
    return routes.find(route_to_find) != routes.end();
  }
  
  // 检查两点之间是否为双向连接
  bool IsBidirectional(const std::string &from, const std::string &to) {
    return HasRoute(from, to) && HasRoute(to, from);
  }
  
  std::vector<RouteInfo> GetRoutes() { 
    return std::vector<RouteInfo>(routes.begin(), routes.end()); 
  }
};

// 嵌套结构体的 nlohmann/json 序列化支持
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TopologyMap::PointInfo, x, y, theta, name, type);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TopologyMap::RouteInfo, from, to);

// TopologyMap 的 nlohmann/json 序列化支持
inline void to_json(nlohmann::json& j, const TopologyMap& map) {
  j = nlohmann::json{
    {"map_name", map.map_name},
    {"points", map.points},
    {"routes", std::vector<TopologyMap::RouteInfo>(map.routes.begin(), map.routes.end())}
  };
}

inline void from_json(const nlohmann::json& j, TopologyMap& map) {
  j.at("map_name").get_to(map.map_name);
  j.at("points").get_to(map.points);
  
  std::vector<TopologyMap::RouteInfo> routes_vector;
  j.at("routes").get_to(routes_vector);
  map.routes = std::set<TopologyMap::RouteInfo>(routes_vector.begin(), routes_vector.end());
}
