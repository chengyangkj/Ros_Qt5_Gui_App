#pragma once
#include <string>
class PluginManager {
 private:
  /* data */
 public:
  PluginManager(/* args */);
  ~PluginManager();
  bool LoadPlugins(const std::string &plugin_path);
  void UnloadPlugins();
};
