#pragma once
#include "topology_map.h"
#include <QSettings>
#ifndef GET_TOPIC_NAME
#define GET_TOPIC_NAME(frame_name)                                             \
  Config::ConfigManager::Instacnce()->GetTopicName(frame_name)
#endif

#ifndef SET_DEFAULT_TOPIC_NAME
#define SET_DEFAULT_TOPIC_NAME(frame_name, topic_name)                         \
  Config::ConfigManager::Instacnce()->SetDefaultTopicName(frame_name,          \
                                                          topic_name);
#endif

#ifndef SET_DEFAULT_CONFIG
#define SET_DEFAULT_CONFIG(key, value)                                         \
  Config::ConfigManager::Instacnce()->SetDefaultConfig(key, value);
#endif

namespace Config {

class ConfigManager {
private:
  QString config_path_ = "./config.ini";

public:
  ConfigManager(/* args */);
  ~ConfigManager();
  void Init(const QString &config_path);
  static ConfigManager *Instacnce() {
    static ConfigManager config;
    return &config;
  }
  std::string GetTopicName(const std::string &frame_name);
  void SetDefaultConfig(const std::string &name, const std::string &value);
  void SetDefaultTopicName(const std::string &frame_name,
                           const std::string &topic_name);
  bool GetConfig(const std::string &key, std::string &value);
  bool SetConfig(const std::string &key, const std::string &value);
  bool ReadTopologyMap(const std::string &map_path, TopologyMap &map);
  bool WriteTopologyMap(const std::string &map_path,
                        const TopologyMap &topology_map);
};
} // namespace Config