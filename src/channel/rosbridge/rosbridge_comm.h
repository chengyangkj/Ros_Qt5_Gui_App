#ifndef ROSBRIDGE_COMM_H
#define ROSBRIDGE_COMM_H

#include "virtual_channel_node.h"
#include "include/ros_bridge.h"
#include "include/ros_topic.h"
#include "include/client/socket_websocket_connection.h"
#include "include/messages/rosbridge_publish_msg.h"
#include "include/types.h"
#include "algorithm.h"
#include "point_type.h"
#include "core/framework/framework.h"
#include "config/config_manager.h"
#include "logger/logger.h"
#include "msg/msg_info.h"
#include "tf2_rosbridge.h"
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <atomic>
#include <mutex>

using namespace rosbridge2cpp;

class RosbridgeComm : public VirtualChannelNode {
 public:
  RosbridgeComm();
  ~RosbridgeComm() override = default;

 public:
  bool Start() override;
  bool Stop() override;
  void Process() override;
  std::string Name() override { return "ROSBridge"; };
  void PubRelocPose(const basic::RobotPose &pose);
  void PubNavGoal(const basic::RobotPose &pose);
  void PubRobotSpeed(const basic::RobotSpeed &speed);
  void PubTopologyMapUpdate(const TopologyMap &topology_map);
  
  bool IsConnecting() const override { return connecting_; }
  bool IsConnectionFailed() const override { return connection_failed_; }
  std::string GetConnectionError() const override {
    std::lock_guard<std::mutex> lock(error_msg_mutex_);
    return connection_error_msg_;
  }

 private:
  void MapCallback(const ROSBridgePublishMsg &msg);
  void LocalCostMapCallback(const ROSBridgePublishMsg &msg);
  void GlobalCostMapCallback(const ROSBridgePublishMsg &msg);
  void LaserCallback(const ROSBridgePublishMsg &msg);
  void PathCallback(const ROSBridgePublishMsg &msg);
  void LocalPathCallback(const ROSBridgePublishMsg &msg);
  void BatteryCallback(const ROSBridgePublishMsg &msg);
  void OdomCallback(const ROSBridgePublishMsg &msg);
  void RobotFootprintCallback(const ROSBridgePublishMsg &msg);
  void TopologyMapCallback(const ROSBridgePublishMsg &msg);
  void ImageCallback(const ROSBridgePublishMsg &msg, const std::string &location);
  void TfCallback(const ROSBridgePublishMsg &msg);

  basic::RobotPose GetTransform(const std::string &from, const std::string &to);
  void GetRobotPose();

 private:
  std::unique_ptr<SocketWebSocketConnection> websocket_connection_;
  std::unique_ptr<ROSBridge> ros_bridge_;
  
  std::unordered_map<std::string, std::unique_ptr<ROSTopic>> publishers_;
  std::unordered_map<std::string, std::unique_ptr<ROSTopic>> subscribers_;
  std::unordered_map<std::string, ROSCallbackHandle<FunVrROSPublishMsg>> callback_handles_;
  
  std::unordered_map<std::string, TransformData> tf_cache_;
  std::mutex tf_cache_mutex_;
  TF2Rosbridge tf2_;
  
  basic::OccupancyMap occ_map_;
  basic::RobotPose m_currPose;
  std::atomic_bool init_flag_{false};
  std::atomic_bool connecting_{false};
  std::atomic_bool connection_failed_{false};
  std::string connection_error_msg_;
  mutable std::mutex error_msg_mutex_;
  std::thread connection_thread_;
  
  std::string rosbridge_ip_;
  int rosbridge_port_;
  
  void ConnectAsync();
  void ReconnectLoop();
  
  std::atomic_bool reconnect_enabled_{true};
  std::atomic_bool reconnecting_{false};
  std::thread reconnect_thread_;
  std::mutex reconnect_mutex_;
};

#endif  // ROSBRIDGE_COMM_H

