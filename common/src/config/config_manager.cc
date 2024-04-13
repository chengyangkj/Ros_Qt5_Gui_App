#include "config/config_manager.h"
#include <QFile>
#include <boost/dll.hpp>
#include <fstream>

#include "boost/filesystem.hpp"
namespace Config {
bool ConfigManager::writeStringToFile(const std::string &filePath,
                                      const std::string &content) {
  boost::filesystem::path path(filePath);
  if (!boost::filesystem::exists(path.parent_path())) {
    boost::filesystem::create_directories(path.parent_path());  // 创建路径
  }

  std::ofstream outputFile(filePath);
  if (outputFile) {
    outputFile << content;  // 写入内容
    outputFile.close();     // 关闭文件
    return true;
  } else {
    std::cerr << "无法创建文件 " << filePath << std::endl;
    return false;
  }
}
ConfigManager *ConfigManager::Instacnce() {
  static ConfigManager config;
  return &config;
}
// #define CHECK_DEFALUT
ConfigManager::ConfigManager(/* args */) { Init(config_path_); }
void ConfigManager::Init(const std::string &config_path) {
  config_path_ = config_path;
  // 配置不存在 写入默认配置
  if (!boost::filesystem::exists(config_path_)) {
    std::string pretty_json = JS::serializeStruct(config_root_);

    writeStringToFile(config_path_, pretty_json);
  }
  ReadRootConfig();
}
ConfigManager::~ConfigManager() {
  std::string pretty_json = JS::serializeStruct(config_root_);
  std::cout << "write json" << std::endl;
  writeStringToFile(config_path_, pretty_json);
}
bool ConfigManager::ReadRootConfig() {
  std::lock_guard<std::mutex> lock(mutex_);
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
  return true;
}
bool ConfigManager::StoreConfig() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string pretty_json = JS::serializeStruct(config_root_);
  writeStringToFile(config_path_, pretty_json);
  return true;
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
  StoreConfig();
}

bool ConfigManager::ReadTopologyMap(const std::string &map_path,
                                    TopologyMap &map) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string fullPath = map_path;
  boost::filesystem::path inputPathObj(map_path);
  // 判断路径是否为相对路径
  if (inputPathObj.is_relative()) {
    // 获取当前可执行程序的路径
    boost::filesystem::path executablePath = boost::dll::program_location();

    // 使用 absolute 函数将相对路径转换为绝对路径
    boost::filesystem::path absolutePath = boost::filesystem::absolute(inputPathObj, executablePath.parent_path());

    // 将路径转换为字符串
    fullPath = absolutePath.string();
  }

  std::ifstream file(fullPath);
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
  std::lock_guard<std::mutex> lock(mutex_);
  std::string fullPath = map_path;
  boost::filesystem::path inputPathObj(map_path);
  // 判断路径是否为相对路径
  if (inputPathObj.is_relative()) {
    // 获取当前可执行程序的路径
    boost::filesystem::path executablePath = boost::dll::program_location();

    // 使用 absolute 函数将相对路径转换为绝对路径
    boost::filesystem::path absolutePath = boost::filesystem::absolute(inputPathObj, executablePath.parent_path());

    // 将路径转换为字符串
    fullPath = absolutePath.string();
  }
  std::string pretty_json = JS::serializeStruct(topology_map);
  return writeStringToFile(fullPath, pretty_json);
}

}  // namespace Config