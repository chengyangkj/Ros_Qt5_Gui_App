#pragma once
#include <map>
#include <nlohmann/json.hpp>
namespace Config {
struct DisplayConfig {
  std::string display_name;
  std::string topic;
  bool enable = true;
  bool visible = true;
  DisplayConfig() = default;
  DisplayConfig(std::string display_name, std::string topic, bool enable)
      : display_name(display_name), topic(topic), enable(enable) {}
  DisplayConfig(std::string display_name, std::string topic, bool enable, bool visible)
      : display_name(display_name), topic(topic), enable(enable), visible(visible) {}
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DisplayConfig, display_name, topic, enable, visible);
struct TopologyMapConfig {
  std::string map_name = "./default_topology_map.json";
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(TopologyMapConfig, map_name);

struct ImageDisplayConfig {
  std::string location;
  std::string topic;
  bool enable = true;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ImageDisplayConfig, location, topic, enable);

struct Point {
  double x;
  double y;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Point, x, y);
struct RobotShapedConfig {
  std::vector<Point> shaped_points;
  bool is_ellipse{false};
  std::string color{"0x00000FF"};
  float opacity{0.5};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(RobotShapedConfig, shaped_points, is_ellipse, color, opacity);

struct ConfigRoot {
  std::vector<DisplayConfig> display_config;
  TopologyMapConfig topology_map_config;
  std::vector<ImageDisplayConfig> images;
  RobotShapedConfig robot_shape_config;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigRoot, display_config, images, topology_map_config, robot_shape_config);

}  // namespace Config
