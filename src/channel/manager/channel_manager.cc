#include "channel_manager.h"
#include <boost/dll.hpp>

#include <iostream>
namespace fs = boost::filesystem;
ChannelManager::ChannelManager() {}
ChannelManager::~ChannelManager() {}
bool ChannelManager::OpenChannelAuto() {
  auto channel_list = DiscoveryAllChannel();
  if (channel_list.size() == 0)
    return false;
  // 如果已经打开了channel，则关闭
  if (channel_ptr_) {
    CloseChannel();
  }

  return OpenChannel(channel_list[0]);
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
void ChannelManager::SendMessage(const MsgId &msg_id, const std::any &msg) {
  if (channel_ptr_ != nullptr) {
    channel_ptr_->SendMessage(msg_id, msg);
  }
}
VirtualChannelNode *ChannelManager::GetChannel() {
  [[unlikely]] if (channel_ptr_ == nullptr) {
    std::cout << "error channel is nullptr exit!" << std::endl;
    exit(1);
  }
  return channel_ptr_;
}