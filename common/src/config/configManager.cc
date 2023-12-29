#include "config/configManager.h"
#include <QFile>
namespace Config {
// #define CHECK_DEFALUT
configManager::configManager(/* args */) { Init(config_path_); }
void configManager::Init(const QString &config_path) {
  QFile file(config_path);
  config_path_ = config_path;
  QSettings setting(config_path_, QSettings::IniFormat);
  // init
  if (!file.exists()) {
    setting.setValue("core/version", "");
  }
}
configManager::~configManager() {}
std::string configManager::GetTopicName(const std::string &frame_name) {
  QSettings setting(config_path_, QSettings::IniFormat);
  return setting.value(QString::fromStdString(frame_name) + "/Topic", "")
      .toString()
      .toStdString();
}

void configManager::SetDefaultConfig(const std::string &name,
                                     const std::string &value) {
  QSettings setting(config_path_, QSettings::IniFormat);
  if (!setting.contains(QString::fromStdString(name))) {
    setting.setValue(QString::fromStdString(name),
                     QString::fromStdString(value));
  }
}
void configManager::SetDefaultTopicName(const std::string &frame_name,
                                        const std::string &topic_name) {
  SetDefaultConfig(frame_name + "/Topic", topic_name);
}

} // namespace Config