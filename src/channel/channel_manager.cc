#include "channel_manager.h"
#include <algorithm>
#include <boost/dll.hpp>
#include "config/config_manager.h"
#include "logger/logger.h"
namespace fs = boost::filesystem;

namespace {

std::string ChannelTypeFromFilename(const std::string &filename) {
#ifdef _WIN32
  static const char kPrefix[] = "channel_";
#else
  static const char kPrefix[] = "libchannel_";
#endif
  fs::path p(filename);
  std::string stem = p.stem().string();
  const size_t plen = sizeof(kPrefix) - 1;
  if (stem.size() >= plen && stem.compare(0, plen, kPrefix) == 0) {
    return stem.substr(plen);
  }
  return stem;
}

}  // namespace

std::string ChannelManager::ChannelTypeFromLibraryPath(const std::string &library_path) {
  return ChannelTypeFromFilename(fs::path(library_path).filename().string());
}

std::string ChannelManager::NormalizeStoredChannelType(const std::string &raw) {
  std::string s = raw;
  static const std::string kLib = "libchannel_";
  static const std::string kCh = "channel_";
  while (s.size() >= kLib.size() && s.compare(0, kLib.size(), kLib) == 0) {
    s = s.substr(kLib.size());
  }
  while (s.size() > kCh.size() && s.compare(0, kCh.size(), kCh) == 0) {
    s = s.substr(kCh.size());
  }
  return s;
}

ChannelManager::ChannelManager() {}
ChannelManager::~ChannelManager() {}

std::string ChannelManager::GetChannelPath(const std::string &channel_type) {
  fs::path libDir = boost::dll::program_location().parent_path() / "lib";
  std::string libPrefix;
  std::string libSuffix;

#ifdef _WIN32
  libPrefix = "channel_";
  libSuffix = ".dll";
#elif __APPLE__
  libPrefix = "libchannel_";
  libSuffix = ".dylib";
#elif __linux__
  libPrefix = "libchannel_";
  libSuffix = ".so";
#endif

  fs::path libPath = libDir / (libPrefix + channel_type + libSuffix);
  return libPath.string();
}

bool ChannelManager::OpenChannelAuto() {
  if (channel_ptr_) {
    CloseChannel();
  }

  auto &config = Config::ConfigManager::Instance()->GetRootConfig();
  std::string channel_type = config.channel_config.channel_type.empty() ? "auto" : config.channel_config.channel_type;
  if (channel_type != "auto") {
    channel_type = NormalizeStoredChannelType(channel_type);
  }

  if (channel_type == "auto") {
    auto channel_list = DiscoveryChannelTypes();
    if (channel_list.size() == 0) {
      LOG_ERROR("No channel found in lib directory");
      return false;
    }

    std::vector<std::string> filtered_list;
    if (channel_list.size() > 1) {
      for (const auto &t : channel_list) {
        std::string lower_t = t;
        std::transform(lower_t.begin(), lower_t.end(), lower_t.begin(), ::tolower);
        if (lower_t.find("rosbridge") == std::string::npos) {
          filtered_list.push_back(t);
        }
      }
      if (filtered_list.size() > 0) {
        channel_list = filtered_list;
      }
    }

    std::string channel_path = GetChannelPath(channel_list[0]);
    LOG_INFO("Auto select channel: " << channel_path);
    return OpenChannel(channel_path);
  } else {
    std::string channel_path = GetChannelPath(channel_type);
    LOG_INFO("Open channel from config: " << channel_path);
    return OpenChannel(channel_path);
  }
}

std::vector<std::string> ChannelManager::DiscoveryChannelTypes() {
  std::vector<std::string> res;

  fs::path libDir = boost::dll::program_location().parent_path() / "lib";

  if (!fs::exists(libDir) || !fs::is_directory(libDir)) {
    return res;
  }

  for (fs::directory_entry &entry : fs::directory_iterator(libDir)) {
    if (!fs::is_regular_file(entry.path())) {
      continue;
    }
    std::string fileName = entry.path().filename().string();
#ifdef _WIN32
    if (entry.path().extension() == ".dll" && fileName.find("channel_") == 0) {
      std::string t = ChannelTypeFromLibraryPath(entry.path().string());
      LOG_INFO("find channel: " << entry.path().string() << " type=" << t);
      res.push_back(t);
    }
#elif __APPLE__
    if (entry.path().extension() == ".dylib" && fileName.find("libchannel_") == 0) {
      std::string t = ChannelTypeFromLibraryPath(entry.path().string());
      LOG_INFO("find channel: " << entry.path().string() << " type=" << t);
      res.push_back(t);
    }
#elif __linux__
    if (entry.path().extension() == ".so" && fileName.find("libchannel_") == 0) {
      std::string t = ChannelTypeFromLibraryPath(entry.path().string());
      LOG_INFO("find channel: " << entry.path().string() << " type=" << t);
      res.push_back(t);
    }
#endif
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
      LOG_ERROR("get channel instance failed!");
      return false;
    }
    if (!channel_ptr_->Init()) {
      LOG_ERROR("channel init failed!");
      return false;
    } else {
      LOG_INFO("open channel: " << channel_ptr_->Name());
      return true;
    }

  } catch (const boost::system::system_error &e) {
    LOG_ERROR("Failed to load dynamic library: " << e.what());
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
    LOG_ERROR("error channel is nullptr exit!");
    exit(1);
  }
  return channel_ptr_;
}