#pragma once
#include <QSettings>

#ifndef GET_TOPIC_NAME
#define GET_TOPIC_NAME(frame_name)                                             \
  Config::configManager::Instacnce()->GetTopicName(frame_name)
#endif

#ifndef SET_DEFAULT_TOPIC_NAME
#define SET_DEFAULT_TOPIC_NAME(frame_name, topic_name)                         \
  Config::configManager::Instacnce()->SetDefaultTopicName(frame_name,          \
                                                          topic_name);
#endif

#ifndef SET_DEFAULT_CONFIG
#define SET_DEFAULT_CONFIG(key, value)                                         \
  Config::configManager::Instacnce()->SetDefaultConfig(key, value);
#endif

namespace Config {

class configManager {
private:
  QString config_path_ = "./config.ini";

public:
  configManager(/* args */);
  ~configManager();
  void Init(const QString &config_path);
  static configManager *Instacnce() {
    static configManager config;
    return &config;
  }
  std::string GetTopicName(const std::string &frame_name);
  void SetDefaultConfig(const std::string &name, const std::string &value);
  void SetDefaultTopicName(const std::string &frame_name,
                           const std::string &topic_name);
};
} // namespace Config