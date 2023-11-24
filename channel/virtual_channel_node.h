/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-25 16:20:39
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-07 14:41:45
 * @FilePath:
 * /ros_qt5_gui_app/include/channel/base/virtual_communcation_node.h
 */
#pragma once
#include "cost_map.h"
#include "msg/msg_info.h"
#include "occupancy_map.h"
#include "point_type.h"
#include <any>
#include <thread>
using namespace basic;
using namespace Msg;
class VirtualChannelNode {

public:
  basic::OccupancyMap occ_map_;

private:
  std::thread process_thread_;

public:
  VirtualChannelNode(/* args */) {
    process_thread_ = std::thread([this]() { Process(); });
  }
  void RegisterOnDataCallback(
      std::function<void(const MsgId &id, const std::any &data)> &&func) {
    OnDataCallback = func;
  }
  virtual ~VirtualChannelNode() { process_thread_.join(); }
  virtual void Process() {}

public:
  virtual void PubRelocPose(const RobotPose &pose){};
  virtual void PubNavGoal(const RobotPose &pose){};
  virtual void PubRobotSpeed(const RobotSpeed &speed){};
  std::function<void(const MsgId &id, const std::any &data)> OnDataCallback;

  virtual void SendMessage(const Msg::MsgId &msg_id, const std::any &msg){};
};
