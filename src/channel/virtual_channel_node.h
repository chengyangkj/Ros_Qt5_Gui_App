/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-25 16:20:39
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-07 14:41:45
 * @FilePath:
 * /ros_qt5_gui_app/include/channel/base/virtual_communcation_node.h
 */
#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include "basic/msg/msg_info.h"
#include "basic/map/occupancy_map.h"
#include "basic/map/topology_map.h"
#include "basic/point/point_type.h"
#include "core/framework/framework.h"

class VirtualChannelNode {
 public:
  VirtualChannelNode() = default;
  virtual ~VirtualChannelNode() = default;
  
  VirtualChannelNode(const VirtualChannelNode&) = delete;
  VirtualChannelNode& operator=(const VirtualChannelNode&) = delete;
  
  bool Init() {
    if (!Start()) {
      return false;
    }
    
    run_flag_ = true;
    process_thread_ = std::thread([this]() {
      const auto period = std::chrono::milliseconds(1000 / loop_rate_);
      while (run_flag_) {
        Process();
        std::this_thread::sleep_for(period);
      }
    });
    
    return true;
  }
  
  void ShutDown() {
    run_flag_ = false;
    Stop();
    if (process_thread_.joinable()) {
      process_thread_.join();
    }
  }
  
  virtual void Process() {}
  virtual bool Start() = 0;
  virtual bool Stop() = 0;
  virtual std::string Name() = 0;
  
  virtual bool IsConnecting() const { return false; }
  virtual bool IsConnectionFailed() const { return false; }
  virtual std::string GetConnectionError() const { return ""; }

  int loop_rate_{30};
  std::atomic<bool> run_flag_{false};

 private:
  std::thread process_thread_;
};
