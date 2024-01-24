#include "config/config_manager.h"
#include "boost/filesystem.hpp"
#include <QFile>
#include <fstream>
namespace Config {
// #define CHECK_DEFALUT
ConfigManager::ConfigManager(/* args */) { Init(config_path_); }
void ConfigManager::Init(const std::string &config_path) {
  config_path_ = config_path;
  // 配置不存在 写入默认配置
  if (!boost::filesystem::exists(config_path_)) {
    std::string pretty_json = JS::serializeStruct(topology_map);
    writeStringToFile(map_path, pretty_json);
  }
  std::ifstream file(config_path_);
  std::string json((std::istreambuf_iterator<char>(file)),
                   std::istreambuf_iterator<char>());
  file.close();
  JS::ParseContext parseContext(json);
  // JS::ParseContext has the member
  if (parseContext.parseTo(config_root_) != JS::Error::NoError) {
    std::string errorStr = parseContext.makeErrorString();
    fprintf(stderr, "Error parsing config.json error: %s\n", errorStr.c_str());
    std::exit(1);
  }
}
ConfigManager::~ConfigManager() {
  std::string pretty_json = JS::serializeStruct(topology_map);
  writeStringToFile(map_path, pretty_json);
}
std::string ConfigManager::GetTopicName(const std::string &frame_name) {
  auto iter = std::find_if(config_root_.display_config.begin(),
                           config_root_.display_config.end(),
                           [&frame_name](const auto &item) {
                             return item.display_name == frame_name;
                           });
  if (iter == config_root_.display_config.end()) {
    return "";
  }
  return iter->topic;
}

void ConfigManager::SetDefaultTopicName(const std::string &frame_name,
                                        const std::string &topic_name) {
  auto iter = std::find_if(config_root_.display_config.begin(),
                           config_root_.display_config.end(),
                           [&frame_name](const auto &item) {
                             return item.display_name == frame_name;
                           });
  if (iter == config_root_.display_config.end()) {
    config_root_.display_config.push_back(
        DisplayConfig(frame_name, topic_name, true));
  } else if (iter->topic == "") {
    iter->topic = topic_name;
  }
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
bool writeStringToFile(const std::string &filePath,
                       const std::string &content) {
  boost::filesystem::path path(filePath);
  if (!boost::filesystem::exists(path.parent_path())) {
    boost::filesystem::create_directories(path.parent_path()); // 创建路径
  }

  std::ofstream outputFile(filePath);
  if (outputFile) {
    outputFile << content; // 写入内容
    outputFile.close();    // 关闭文件
    return true;
  } else {
    std::cerr << "无法创建文件 " << filePath << std::endl;
    return false;
  }
}
bool ConfigManager::WriteTopologyMap(const std::string &map_path,
                                     const TopologyMap &topology_map) {

  std::string pretty_json = JS::serializeStruct(topology_map);
  return writeStringToFile(map_path, pretty_json);
}

} // namespace Config