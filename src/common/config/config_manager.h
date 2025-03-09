#pragma once
#include <mutex>
#include "config_define.h"
#include "topology_map.h"
#ifndef GET_TOPIC_NAME
  #define GET_TOPIC_NAME(frame_name) \
    Config::ConfigManager::Instacnce()->GetTopicName(frame_name)
#endif

#ifndef SET_DEFAULT_TOPIC_NAME
  #define SET_DEFAULT_TOPIC_NAME(frame_name, topic_name)                \
    Config::ConfigManager::Instacnce()->SetDefaultTopicName(frame_name, \
                                                            topic_name);
#endif

namespace Config {

class ConfigManager {
 private:
  std::string config_path_ = "./config.json";
  ConfigRoot config_root_;  // 配置文件根节点
  std::mutex mutex_;
  ConfigManager(/* args */);
  // 禁用拷贝构造函数和赋值运算符
  ConfigManager(const ConfigManager &) = delete;
  ConfigManager &operator=(const ConfigManager &) = delete;
  bool ReadRootConfig();

 public:
  ~ConfigManager();
  bool StoreConfig();
  void Init(const std::string &config_path);
  static bool writeStringToFile(const std::string &filePath,
                                const std::string &content);
  static ConfigManager *Instacnce();
  std::string GetTopicName(const std::string &frame_name);
  void SetDefaultConfig(const std::string &name, const std::string &value);
  void SetDefaultTopicName(const std::string &frame_name,
                           const std::string &topic_name);
  ConfigRoot &GetRootConfig() { return config_root_; }
  bool ReadTopologyMap(const std::string &map_path, TopologyMap &map);
  bool WriteTopologyMap(const std::string &map_path,
                        const TopologyMap &topology_map);
};
}  // namespace Config