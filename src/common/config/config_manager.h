#pragma once
#include <mutex>
#include "config_define.h"
#include "topology_map.h"
#include "macros.h"

#ifndef GET_TOPIC_NAME
  #define GET_TOPIC_NAME(frame_name) \
    Config::ConfigManager::Instance()->GetTopicName(frame_name)
#endif

#ifndef SET_DEFAULT_TOPIC_NAME
  #define SET_DEFAULT_TOPIC_NAME(frame_name, topic_name)                \
    Config::ConfigManager::Instance()->SetDefaultTopicName(frame_name, \
                                                            topic_name);
#endif

#ifndef SET_DEFAULT_KEY_VALUE
  #define SET_DEFAULT_KEY_VALUE(key, value)                             \
    Config::ConfigManager::Instance()->SetDefaultKeyValue(key, value);
#endif

#ifndef SET_KEY_VALUE
  #define SET_KEY_VALUE(key, value)                                     \
    Config::ConfigManager::Instance()->SetConfigValue(key, value);
#endif

#ifndef GET_CONFIG_VALUE
  #define GET_CONFIG_VALUE(key, default_value)                          \
    Config::ConfigManager::Instance()->GetConfigValue(key, default_value)
#endif

namespace Config {

class ConfigManager {
 private:
  std::string config_path_ = "./config.json";
  ConfigRoot config_root_;  // 配置文件根节点
  std::mutex mutex_;

  bool ReadRootConfig();
  bool StoreConfigUnlocked();  // 内部版本，不获取锁（假设调用者已持有锁）

 public:
  ~ConfigManager();
  bool StoreConfig();
  void Init(const std::string &config_path);
  static bool writeStringToFile(const std::string &filePath,
                                const std::string &content);

  std::string GetTopicName(const std::string &frame_name);
  void SetDefaultConfig(const std::string &name, const std::string &value);
  void SetDefaultTopicName(const std::string &frame_name,
                           const std::string &topic_name);
  std::string GetConfigValue(const std::string &key, const std::string &default_value = "");
  void SetConfigValue(const std::string &key, const std::string &value);
  void SetDefaultKeyValue(const std::string &key, const std::string &value);
  ConfigRoot &GetRootConfig() { return config_root_; }
  bool ReadTopologyMap(const std::string &map_path, TopologyMap &map);
  bool WriteTopologyMap(const std::string &map_path,
                        const TopologyMap &topology_map);

  DEFINE_SINGLETON(ConfigManager)
};
}  // namespace Config