/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-27 09:58:26
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-06 12:29:09
 * @FilePath: /ros_qt5_gui_app/include/channel/base/comm_instance.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <memory>
#include <string>
#include <vector>
#include <boost/dll/shared_library.hpp>
#include "virtual_channel_node.h"

class ChannelManager {
 public:
  explicit ChannelManager();
  ~ChannelManager();
  
  ChannelManager(const ChannelManager&) = delete;
  ChannelManager& operator=(const ChannelManager&) = delete;
  
  bool OpenChannel(const std::string& path);
  bool OpenChannelAuto();
  std::vector<std::string> DiscoveryAllChannel();
  VirtualChannelNode* GetChannel();
  void CloseChannel();
  
  bool IsChannelOpen() const { return channel_ptr_ != nullptr; }

 private:
  std::string GetChannelPath(const std::string& channel_type);
  
  std::unique_ptr<VirtualChannelNode> channel_ptr_;
  std::unique_ptr<boost::dll::shared_library> library_channel_;
};