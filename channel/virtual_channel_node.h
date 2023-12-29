/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-25 16:20:39
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-07 14:41:45
 * @FilePath:
 * /ros_qt5_gui_app/include/channel/base/virtual_communcation_node.h
 */
#pragma once
#include "msg/msg_info.h"
#include "occupancy_map.h"
#include "point_type.h"
#include <any>
#include <thread>
#include <atomic>
using namespace basic;
class VirtualChannelNode {

private:
  std::thread process_thread_;

public:
  VirtualChannelNode(/* args */) {}
  void RegisterOnDataCallback(
      std::function<void(const MsgId &id, const std::any &data)> &&func) {
    OnDataCallback = func;
  }
  bool Init() {
    if (Start()) {
      std::cout << "start channel success" << std::endl;
      run_flag_ = true;
      process_thread_ = std::thread([this]() {
        while (run_flag_) {
          Process();
          std::this_thread::sleep_for(
              std::chrono::milliseconds(1000 / loop_rate_));
        }
      });
      return true;
    }
    return false;
  }
  void ShutDown() {
    run_flag_ = false;
    Stop();
    process_thread_.join();
  }
  virtual ~VirtualChannelNode() {}
  virtual void Process() {}
  virtual bool Start() = 0;
  virtual bool Stop() = 0;
  virtual std::string Name() = 0;
  virtual void SendMessage(const MsgId &msg_id, const std::any &msg) = 0;

public:
  std::function<void(const MsgId &id, const std::any &data)> OnDataCallback;
  int loop_rate_{30};
  std::atomic<bool> run_flag_{false};
};
