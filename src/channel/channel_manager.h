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
#include <any>
#include <boost/dll/import.hpp>
#include <boost/filesystem.hpp>
#include "msg/msg_info.h"
#include "virtual_channel_node.h"
class ChannelManager {
 private:
  VirtualChannelNode *channel_ptr_{nullptr};
  boost::dll::shared_library *library_channel_;
  std::string GetChannelPath(const std::string &channel_type);

 public:
  explicit ChannelManager();
  ~ChannelManager();
  /// @brief 传入channel so路径，打开对应的通信channel
  /// @param name
  /// @return
  bool OpenChannel(const std::string &name);
  /// @brief 根据配置自动选择channel类型并打开，默认auto
  /// @return
  bool OpenChannelAuto();
  /// @brief 查找lib路径下所有channel
  /// @return channel list
  std::vector<std::string> DiscoveryAllChannel();
  VirtualChannelNode *GetChannel();
  void CloseChannel();
};