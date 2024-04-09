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
struct ConfigRoot {
  std::vector<DisplayConfig> display_config;
  TopologyMapConfig topology_map_config;
  std::vector<ImageDisplayConfig> images;
  JS_OBJ(display_config, images, topology_map_config);
};

}  // namespace Config
