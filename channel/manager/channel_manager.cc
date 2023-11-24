#include "channel_manager.h"

#include <iostream>
ChannelManager::ChannelManager() {}
ChannelManager::~ChannelManager() {}
bool ChannelManager::OpenChannel(const QString &path) {
  library_channel_ = new QLibrary(path);
  if (library_channel_->load()) {
    // 动态链接库加载成功
    // 在这里可以使用 QLibrary 提供的函数指针来访问动态链接库中的函数
    typedef VirtualChannelNode *(*GetChannelInstanceFunc)();
    GetChannelInstanceFunc func_get =
        (GetChannelInstanceFunc)library_channel_->resolve(
            "GetChannelInstance"); //取出该符号
    channel_ptr_ = func_get();
    if (channel_ptr_ == nullptr) {
      std::cout << "get channel instance failed!" << std::endl;
      return false;
    }
  } else {
    // 动态链接库加载失败
    // 可以通过 myLib.errorString() 获取错误信息进行调试
    std::cout << "load channel instance failed!:"
              << library_channel_->errorString().toStdString() << std::endl;
    return false;
  }
  return true;
}
void ChannelManager::CloseChannel() {}
void ChannelManager::SendMessage(const Msg::MsgId &msg_id,
                                 const std::any &msg) {
  if (channel_ptr_ != nullptr) {
    channel_ptr_->SendMessage(msg_id, msg);
  }
}