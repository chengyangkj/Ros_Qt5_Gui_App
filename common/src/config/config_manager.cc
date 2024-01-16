#include "config/config_manager.h"
#include <QFile>
#include <fstream>
namespace Config {
// #define CHECK_DEFALUT
ConfigManager::ConfigManager(/* args */) { Init(config_path_); }
void ConfigManager::Init(const QString &config_path) {
  QFile file(config_path);
  config_path_ = config_path;
  QSettings setting(config_path_, QSettings::IniFormat);
  // init
  if (!file.exists()) {
    setting.setValue("core/version", "");
  }
}
ConfigManager::~ConfigManager() {}
std::string ConfigManager::GetTopicName(const std::string &frame_name) {
  QSettings setting(config_path_, QSettings::IniFormat);
  return setting.value(QString::fromStdString(frame_name) + "/Topic", "")
      .toString()
      .toStdString();
}

void ConfigManager::SetDefaultConfig(const std::string &name,
                                     const std::string &value) {
  QSettings setting(config_path_, QSettings::IniFormat);
  if (!setting.contains(QString::fromStdString(name))) {
    setting.setValue(QString::fromStdString(name),
                     QString::fromStdString(value));
  }
}
void ConfigManager::SetDefaultTopicName(const std::string &frame_name,
                                        const std::string &topic_name) {
  SetDefaultConfig(frame_name + "/Topic", topic_name);
}
std::string ConfigManager::GetConfig(const std::string &key) {
  std::string value;
  QSettings setting(config_path_, QSettings::IniFormat);
  value =
      setting.value(QString::fromStdString(key), "").toString().toStdString();
  return value;
}
bool ConfigManager::SetConfig(const std::string &key,
                              const std::string &value) {
  QSettings setting(config_path_, QSettings::IniFormat);
  setting.setValue(QString::fromStdString(key), QString::fromStdString(value));
  return true;
}
bool ConfigManager::ReadTopologyMap(const std::string &map_path,
                                    TopologyMap &map) {
  std::ifstream file(map_path);
  std::string json((std::istreambuf_iterator<char>(file)),
                   std::istreambuf_iterator<char>());
  file.close();
  JS::ParseContext parseContext(json);
  // JS::ParseContext has the member
  if (parseContext.parseTo(map) != JS::Error::NoError) {
    std::string errorStr = parseContext.makeErrorString();
    fprintf(stderr, "Error parsing struct %s\n", errorStr.c_str());
    return false;
  }
  return true;
}
bool ConfigManager::WriteTopologyMap(const std::string &map_path,
                                     const TopologyMap &topology_map) {
  std::ofstream file(map_path);
  if (!file.is_open()) {
    fprintf(stderr, "Error opening file %s\n", map_path.c_str());
    return false;
  }
  std::string pretty_json = JS::serializeStruct(topology_map);
  file << pretty_json;
  file.close();
  return true;
}

} // namespace Config