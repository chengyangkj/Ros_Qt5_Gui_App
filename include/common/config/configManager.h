#pragma once
#include <QSettings>

#ifndef GET_TOPIC_NAME
#define GET_TOPIC_NAME(frame_name)                                             \
  Config::configManager::Instacnce()->GetTopicName(frame_name)
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
};
} // namespace Config