#include "channel_manager.h"
#include <boost/dll.hpp>
#include <iostream>
#include <algorithm>
#include "config/config_manager.h"
namespace fs = boost::filesystem;
ChannelManager::ChannelManager() {}
ChannelManager::~ChannelManager() {}

std::string ChannelManager::GetChannelPath(const std::string &channel_type) {
  std::string libDir = boost::dll::program_location().parent_path().string() + "/lib/";
  std::string libPrefix = "libchannel_";
  std::string libSuffix;

#ifdef __WIN32
  libSuffix = ".dll";
#elif __APPLE__
  libSuffix = ".dylib";
#elif __linux__
  libSuffix = ".so";
#endif

  return libDir + libPrefix + channel_type + libSuffix;
}

bool ChannelManager::OpenChannelAuto() {
  if (channel_ptr_) {
    CloseChannel();
  }

  // 从配置读取通道类型
  auto &config = Config::ConfigManager::Instance()->GetRootConfig();
  std::string channel_type = config.channel_config.channel_type.empty() ? "auto" : config.channel_config.channel_type;

  if (channel_type == "auto") {
    auto channel_list = DiscoveryAllChannel();
    if (channel_list.size() == 0) {
      std::cout << "No channel found in lib directory" << std::endl;
      return false;
    }
    
    // 如果有多个 channel，过滤掉 ROSBridge
    std::vector<std::string> filtered_list;
    if (channel_list.size() > 1) {
      for (const auto& channel_path : channel_list) {
        // 检查路径中是否包含 "rosbridge"（不区分大小写）
        std::string lower_path = channel_path;
        std::transform(lower_path.begin(), lower_path.end(), lower_path.begin(), ::tolower);
        if (lower_path.find("rosbridge") == std::string::npos) {
          filtered_list.push_back(channel_path);
        }
      }
      // 如果过滤后还有 channel，使用过滤后的列表
      if (filtered_list.size() > 0) {
        channel_list = filtered_list;
      }
    }
    
    std::cout << "Auto select channel: " << channel_list[0] << std::endl;
    return OpenChannel(channel_list[0]);
  } else if (channel_type == "ros2" || channel_type == "ros1" || channel_type == "rosbridge") {
    std::string channel_path = GetChannelPath(channel_type);
    std::cout << "Open channel from config: " << channel_path << std::endl;
    return OpenChannel(channel_path);
  } else {
    std::cout << "Unknown channel type: " << channel_type << ", fallback to auto" << std::endl;
    auto channel_list = DiscoveryAllChannel();
    if (channel_list.size() == 0) {
      return false;
    }
    return OpenChannel(channel_list[0]);
  }
}
std::vector<std::string> ChannelManager::DiscoveryAllChannel() {
  std::vector<std::string> res;

  std::string libDir = boost::dll::program_location().parent_path().string() + "/lib/";

  for (fs::directory_entry &entry : fs::directory_iterator(libDir)) {
    if (fs::is_regular_file(entry.path())) {
      std::string fileName = entry.path().filename().string();
#ifdef __WIN32
      if (entry.path().extension() == ".dll" &&
          fileName.find("channel_") == 0) {
        std::cout << "find channel:" << entry.path() << std::endl;
        res.push_back(entry.path());
      }
#elif __APPLE__
      if (entry.path().extension() == ".dylib" &&
          fileName.find("libchannel_") == 0) {
        std::cout << "find channel:" << entry.path() << std::endl;
        res.push_back(entry.path());
      }
#elif __linux__
      if (entry.path().extension() == ".so" &&
          fileName.find("libchannel_") == 0) {
        std::cout << "find channel:" << entry.path() << std::endl;
        res.push_back(entry.path().string());
      }
#endif
    }
  }
  return res;
}
bool ChannelManager::OpenChannel(const std::string &path) {
  try {
    library_channel_ = new boost::dll::shared_library(path);
    // 动态链接库加载成功
    // 在这里可以使用 QLibrary 提供的函数指针来访问动态链接库中的函数
    typedef VirtualChannelNode *(*GetChannelInstanceFunc)();
    GetChannelInstanceFunc func_get =
        (GetChannelInstanceFunc)library_channel_->get<VirtualChannelNode *()>(
            "GetChannelInstance");  // 取出该符号
    channel_ptr_ = func_get();
    if (channel_ptr_ == nullptr) {
      std::cout << "get channel instance failed!" << std::endl;
      return false;
    }
    if (!channel_ptr_->Init()) {
      std::cout << "channel init failed!" << std::endl;
      return false;
    } else {
      std::cout << "open channel:" << channel_ptr_->Name() << std::endl;
      return true;
    }

  } catch (const boost::system::system_error &e) {
    std::cout << "Failed to load dynamic library: " << e.what() << std::endl;
    return false;
  }

  return true;
}
void ChannelManager::CloseChannel() {
  channel_ptr_->ShutDown();
  delete channel_ptr_;
  channel_ptr_ = nullptr;
}
VirtualChannelNode *ChannelManager::GetChannel() {
  [[unlikely]] if (channel_ptr_ == nullptr) {
    std::cout << "error channel is nullptr exit!" << std::endl;
    exit(1);
  }
  return channel_ptr_;
}