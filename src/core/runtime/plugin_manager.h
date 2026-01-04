#pragma once
#include <memory>
#include <string>
#include <vector>

class PluginManager {
 public:
  PluginManager();
  ~PluginManager();
  
  PluginManager(const PluginManager&) = delete;
  PluginManager& operator=(const PluginManager&) = delete;
  
  bool LoadPlugins(const std::string& plugin_path);
  void UnloadPlugins();
  
  bool IsLoaded() const { return loaded_; }

 private:
  bool loaded_{false};
  std::vector<std::string> loaded_plugin_paths_;
};
