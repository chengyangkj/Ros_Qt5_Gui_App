#pragma once
#include <map>
#include "json_struct/json_struct.h"
namespace Config {
struct DisplayConfig {
  std::string display_name;
  std::string topic;
  bool enable = true;
  DisplayConfig() = default;
  DisplayConfig(std::string display_name, std::string topic, bool enable)
      : display_name(display_name), topic(topic), enable(enable) {}
  JS_OBJ(display_name, topic, enable);
};
struct TopologyMapConfig {
  std::string map_name = "./default_topology_map.json";
  JS_OBJ(map_name);
};
struct ImageDisplayConfig {
  std::string location;
  std::string topic;
  bool enable = true;

  JS_OBJ(location, topic, enable);
};
struct Point {
  double x;
  double y;
  JS_OBJ(x, y);
};
struct RobotShapedConfig {
  std::vector<Point> shaped_points;
  bool is_ellipse{false};
  std::string color{"0x00000FF"};
  float opacity{0.5};
  JS_OBJ(shaped_points, is_ellipse, color, opacity);
};
struct ConfigRoot {
  std::vector<DisplayConfig> display_config;
  TopologyMapConfig topology_map_config;
  std::vector<ImageDisplayConfig> images;
  RobotShapedConfig robot_shape_config;
  JS_OBJ(display_config, images, topology_map_config, robot_shape_config);
};

}  // namespace Config
