#include "plugin_manager.h"
#include "common/logger/logger.h"

PluginManager::PluginManager() = default;

PluginManager::~PluginManager() {
  UnloadPlugins();
}

bool PluginManager::LoadPlugins(const std::string& plugin_path) {
  if (loaded_) {
    LOG_WARN("Plugins already loaded");
    return true;
  }
  
  if (plugin_path.empty()) {
    LOG_WARN("Plugin path is empty, skipping plugin loading");
    return false;
  }
  
  loaded_plugin_paths_.push_back(plugin_path);
  loaded_ = true;
  LOG_INFO("Plugins loaded from: " << plugin_path);
  return true;
}

void PluginManager::UnloadPlugins() {
  if (!loaded_) {
    return;
  }
  
  loaded_plugin_paths_.clear();
  loaded_ = false;
  LOG_INFO("All plugins unloaded");
}