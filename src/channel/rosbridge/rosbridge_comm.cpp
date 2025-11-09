/**
 * @file rosbridge_comm.cpp
 * @brief ROSBridge通信通道实现
 * @details 通过WebSocket连接ROS Bridge服务器，实现ROS话题的订阅和发布
 */

#include "rosbridge_comm.h"
#include "include/ros_time.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <algorithm>

/**
 * @brief 构造函数，初始化默认配置
 */
RosbridgeComm::RosbridgeComm() {
  // 设置默认话题名称
  SET_DEFAULT_TOPIC_NAME(DISPLAY_GOAL, "/goal_pose")
  SET_DEFAULT_TOPIC_NAME(MSG_ID_SET_RELOC_POSE, "/initialpose")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_MAP, "/map")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_LOCAL_COST_MAP, "/local_costmap/costmap")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_GLOBAL_COST_MAP, "/global_costmap/costmap")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_LASER, "/scan")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_GLOBAL_PATH, "/plan")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_LOCAL_PATH, "/local_plan")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_ROBOT, "/odom")
  SET_DEFAULT_TOPIC_NAME(MSG_ID_SET_ROBOT_SPEED, "/cmd_vel")
  SET_DEFAULT_TOPIC_NAME(MSG_ID_BATTERY_STATE, "/battery")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_ROBOT_FOOTPRINT, "/local_costmap/published_footprint")
  SET_DEFAULT_TOPIC_NAME(DISPLAY_TOPOLOGY_MAP, "/map/topology")
  SET_DEFAULT_TOPIC_NAME(MSG_ID_TOPOLOGY_MAP_UPDATE, "/map/topology/update")
  
  // 设置默认键值配置
  SET_DEFAULT_KEY_VALUE("BaseFrameId", "base_link")
  
  // 设置默认通道配置
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  if (config.channel_config.channel_type.empty()) {
    config.channel_config.channel_type = "auto";
  }
  if (config.channel_config.rosbridge_config.ip.empty()) {
    config.channel_config.rosbridge_config.ip = "127.0.0.1";
  }
  if (config.channel_config.rosbridge_config.port.empty()) {
    config.channel_config.rosbridge_config.port = "9090";
  }
  
  // 设置默认图像配置
  if (Config::ConfigManager::Instacnce()->GetRootConfig().images.empty()) {
    Config::ConfigManager::Instacnce()->GetRootConfig().images.push_back(
        Config::ImageDisplayConfig{.location = "front",
                                   .topic = "/camera/front/image_raw",
                                   .enable = true});
  }
  Config::ConfigManager::Instacnce()->StoreConfig();
}

/**
 * @brief 启动ROSBridge连接并初始化订阅者和发布者
 * @return 成功返回true，失败返回false
 */
bool RosbridgeComm::Start() {
  // 从配置读取ROSBridge服务器地址和端口
  auto &config = Config::ConfigManager::Instacnce()->GetRootConfig();
  rosbridge_ip_ = config.channel_config.rosbridge_config.ip.empty() ? "127.0.0.1" : config.channel_config.rosbridge_config.ip;
  rosbridge_port_ = std::stoi(config.channel_config.rosbridge_config.port.empty() ? "9090" : config.channel_config.rosbridge_config.port);
  
  connection_failed_ = false;
  connecting_ = true;
  {
    std::lock_guard<std::mutex> lock(error_msg_mutex_);
    connection_error_msg_.clear();
  }
  
  // 启动异步连接线程
  connection_thread_ = std::thread(&RosbridgeComm::ConnectAsync, this);
  
  return true;
}

void RosbridgeComm::ConnectAsync() {
  // 创建WebSocket连接
  websocket_connection_ = std::make_unique<SocketWebSocketConnection>();
  websocket_connection_->RegisterErrorCallback([this](TransportError err) {
    std::lock_guard<std::mutex> lock(error_msg_mutex_);
    if (err == TransportError::R2C_CONNECTION_CLOSED) {
      connection_error_msg_ = "ROSBridge 连接已关闭";
      LOG_ERROR("ROSBridge connection closed");
    } else if (err == TransportError::R2C_SOCKET_ERROR) {
      connection_error_msg_ = "ROSBridge 网络连接错误";
      LOG_ERROR("ROSBridge socket error");
    }
    connection_failed_ = true;
    connecting_ = false;
  });
  
  // 创建ROSBridge实例
  ros_bridge_ = std::make_unique<ROSBridge>(*websocket_connection_,false);
  
  // 连接到ROSBridge服务器
  LOG_INFO("Attempting to connect to ROSBridge server at " << rosbridge_ip_ << ":" << rosbridge_port_);
  if (!ros_bridge_->Init(rosbridge_ip_, rosbridge_port_)) {
    std::lock_guard<std::mutex> lock(error_msg_mutex_);
    connection_error_msg_ = "无法连接到 ROSBridge 服务器 " + rosbridge_ip_ + ":" + std::to_string(rosbridge_port_) + 
                            "\n\n请检查：\n"
                            "1. ROSBridge 服务器是否正在运行\n"
                            "2. IP 地址和端口是否正确\n"
                            "3. 网络连接是否正常";
    LOG_ERROR("Failed to connect to ROSBridge server!");
    connection_failed_ = true;
    connecting_ = false;
    return;
  }
  
  LOG_INFO("Successfully connected to ROSBridge server!");
  connecting_ = false;
  
  // ========== 订阅ROS话题 ==========
  
  // 地图话题订阅
  auto map_topic = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(DISPLAY_MAP), "nav_msgs/OccupancyGrid", 1);
  callback_handles_[GET_TOPIC_NAME(DISPLAY_MAP)] = map_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { MapCallback(msg); });
  subscribers_[GET_TOPIC_NAME(DISPLAY_MAP)] = std::move(map_topic);
  
  // 局部代价地图话题订阅
  auto local_cost_map_topic = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(DISPLAY_LOCAL_COST_MAP), "nav_msgs/OccupancyGrid", 1);
  callback_handles_[GET_TOPIC_NAME(DISPLAY_LOCAL_COST_MAP)] = local_cost_map_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { LocalCostMapCallback(msg); });
  subscribers_[GET_TOPIC_NAME(DISPLAY_LOCAL_COST_MAP)] = std::move(local_cost_map_topic);
  
  // 全局代价地图话题订阅
  auto global_cost_map_topic = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(DISPLAY_GLOBAL_COST_MAP), "nav_msgs/OccupancyGrid", 1);
  callback_handles_[GET_TOPIC_NAME(DISPLAY_GLOBAL_COST_MAP)] = global_cost_map_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { GlobalCostMapCallback(msg); });
  subscribers_[GET_TOPIC_NAME(DISPLAY_GLOBAL_COST_MAP)] = std::move(global_cost_map_topic);
  
  // 激光扫描话题订阅
  auto laser_topic = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(DISPLAY_LASER), "sensor_msgs/LaserScan", 20);
  callback_handles_[GET_TOPIC_NAME(DISPLAY_LASER)] = laser_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { LaserCallback(msg); });
  subscribers_[GET_TOPIC_NAME(DISPLAY_LASER)] = std::move(laser_topic);
  
  // 电池状态话题订阅
  auto battery_topic = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(MSG_ID_BATTERY_STATE), "sensor_msgs/BatteryState", 1);
  callback_handles_[GET_TOPIC_NAME(MSG_ID_BATTERY_STATE)] = battery_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { BatteryCallback(msg); });
  subscribers_[GET_TOPIC_NAME(MSG_ID_BATTERY_STATE)] = std::move(battery_topic);
  
  // 全局路径话题订阅
  auto global_path_topic = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(DISPLAY_GLOBAL_PATH), "nav_msgs/Path", 20);
  callback_handles_[GET_TOPIC_NAME(DISPLAY_GLOBAL_PATH)] = global_path_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { PathCallback(msg); });
  subscribers_[GET_TOPIC_NAME(DISPLAY_GLOBAL_PATH)] = std::move(global_path_topic);
  
  // 局部路径话题订阅
  auto local_path_topic = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(DISPLAY_LOCAL_PATH), "nav_msgs/Path", 20);
  callback_handles_[GET_TOPIC_NAME(DISPLAY_LOCAL_PATH)] = local_path_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { LocalPathCallback(msg); });
  subscribers_[GET_TOPIC_NAME(DISPLAY_LOCAL_PATH)] = std::move(local_path_topic);
  
  // 里程计话题订阅
  auto odom_topic = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(DISPLAY_ROBOT), "nav_msgs/Odometry", 20);
  callback_handles_[GET_TOPIC_NAME(DISPLAY_ROBOT)] = odom_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { OdomCallback(msg); });
  subscribers_[GET_TOPIC_NAME(DISPLAY_ROBOT)] = std::move(odom_topic);
  
  // 机器人足迹话题订阅
  auto robot_footprint_topic = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(DISPLAY_ROBOT_FOOTPRINT), "geometry_msgs/PolygonStamped", 20);
  callback_handles_[GET_TOPIC_NAME(DISPLAY_ROBOT_FOOTPRINT)] = robot_footprint_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { RobotFootprintCallback(msg); });
  subscribers_[GET_TOPIC_NAME(DISPLAY_ROBOT_FOOTPRINT)] = std::move(robot_footprint_topic);
  
  // 拓扑地图话题订阅
  auto topology_map_topic = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(DISPLAY_TOPOLOGY_MAP), "topology_msgs/TopologyMap", 1);
  callback_handles_[GET_TOPIC_NAME(DISPLAY_TOPOLOGY_MAP)] = topology_map_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { TopologyMapCallback(msg); });
  subscribers_[GET_TOPIC_NAME(DISPLAY_TOPOLOGY_MAP)] = std::move(topology_map_topic);
  
  // 图像话题订阅（动态配置）
  for (auto one_image_display : Config::ConfigManager::Instacnce()->GetRootConfig().images) {
    if (!one_image_display.enable) continue;
    LOG_INFO("image location:" << one_image_display.location << " topic:" << one_image_display.topic);
    auto image_topic = std::make_unique<ROSTopic>(*ros_bridge_, one_image_display.topic, "sensor_msgs/Image", 1);
    std::string location = one_image_display.location;
    callback_handles_[one_image_display.topic] = image_topic->Subscribe(
        [this, location](const ROSBridgePublishMsg &msg) { ImageCallback(msg, location); });
    subscribers_[one_image_display.topic] = std::move(image_topic);
  }
  
  // TF变换话题订阅
  auto tf_topic = std::make_unique<ROSTopic>(*ros_bridge_, "/tf", "tf2_msgs/TFMessage", 100);
  callback_handles_["/tf"] = tf_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { TfCallback(msg); });
  subscribers_["/tf"] = std::move(tf_topic);
  
  // TF静态变换话题订阅
  auto tf_static_topic = std::make_unique<ROSTopic>(*ros_bridge_, "/tf_static", "tf2_msgs/TFMessage", 100);
  callback_handles_["/tf_static"] = tf_static_topic->Subscribe(
      [this](const ROSBridgePublishMsg &msg) { TfCallback(msg); });
  subscribers_["/tf_static"] = std::move(tf_static_topic);
  
  // ========== 发布ROS话题 ==========
  
  // 导航目标点发布者
  auto nav_goal_publisher = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(DISPLAY_GOAL), "geometry_msgs/PoseStamped", 10);
  nav_goal_publisher->Advertise();
  publishers_[GET_TOPIC_NAME(DISPLAY_GOAL)] = std::move(nav_goal_publisher);
  
  // 重定位位姿发布者
  auto reloc_pose_publisher = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(MSG_ID_SET_RELOC_POSE), "geometry_msgs/PoseWithCovarianceStamped", 10);
  reloc_pose_publisher->Advertise();
  publishers_[GET_TOPIC_NAME(MSG_ID_SET_RELOC_POSE)] = std::move(reloc_pose_publisher);
  
  // 机器人速度发布者
  auto speed_publisher = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(MSG_ID_SET_ROBOT_SPEED), "geometry_msgs/Twist", 10);
  speed_publisher->Advertise();
  publishers_[GET_TOPIC_NAME(MSG_ID_SET_ROBOT_SPEED)] = std::move(speed_publisher);
  
  // 拓扑地图更新发布者
  auto topology_map_update_publisher = std::make_unique<ROSTopic>(*ros_bridge_, GET_TOPIC_NAME(MSG_ID_TOPOLOGY_MAP_UPDATE), "topology_msgs/TopologyMap", 1);
  topology_map_update_publisher->Advertise();
  publishers_[GET_TOPIC_NAME(MSG_ID_TOPOLOGY_MAP_UPDATE)] = std::move(topology_map_update_publisher);
  
  // ========== 订阅内部消息总线 ==========
  
  SUBSCRIBE(MSG_ID_SET_NAV_GOAL_POSE, [this](const RobotPose& pose) {
    LOG_INFO("recv nav goal pose:" << pose);
    PubNavGoal(pose);
  });
  
  SUBSCRIBE(MSG_ID_SET_RELOC_POSE, [this](const RobotPose& pose) {
    LOG_INFO("recv reloc pose:" << pose);
    PubRelocPose(pose);
  });
  
  SUBSCRIBE(MSG_ID_SET_ROBOT_SPEED, [this](const RobotSpeed& speed) {
    LOG_INFO("recv robot speed:" << speed);
    PubRobotSpeed(speed);
  });
  
  SUBSCRIBE(MSG_ID_TOPOLOGY_MAP_UPDATE, [this](const TopologyMap& topology_map) {
    LOG_INFO("recv topology map update:" << topology_map.map_name);
    PubTopologyMapUpdate(topology_map);
  });
  
  init_flag_ = true;
}

/**
 * @brief 停止ROSBridge连接并清理资源
 * @return 成功返回true
 */
bool RosbridgeComm::Stop() {
  init_flag_ = false;
  connecting_ = false;
  
  if (connection_thread_.joinable()) {
    connection_thread_.join();
  }
  
  subscribers_.clear();
  publishers_.clear();
  callback_handles_.clear();
  ros_bridge_.reset();
  websocket_connection_.reset();
  return true;
}

/**
 * @brief 处理循环，定期更新机器人位姿
 */
void RosbridgeComm::Process() {
  if (init_flag_ && ros_bridge_ && ros_bridge_->IsHealthy()) {
    GetRobotPose();
  }
}

/**
 * @brief 获取机器人位姿并发布
 */
void RosbridgeComm::GetRobotPose() {
  std::string base_frame = GET_CONFIG_VALUE("BaseFrameId", "base_link");
  auto pose = GetTransform( "map",base_frame);
  PUBLISH(MSG_ID_ROBOT_POSE, pose);
}

/**
 * @brief TF变换回调函数，更新TF缓存
 * @param msg ROSBridge消息
 */
void RosbridgeComm::TfCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull()) return;
  
  const auto &msg_json = msg.msg_json_;
  if (!msg_json.HasMember("transforms")) return;
  
  const auto &transforms = msg_json["transforms"];
  if (!transforms.IsArray()) return;
  
  std::lock_guard<std::mutex> lock(tf_cache_mutex_);
  
  // 遍历所有变换并更新缓存
  for (rapidjson::SizeType i = 0; i < transforms.Size(); i++) {
    const auto &transform_stamped = transforms[i];
    if (!transform_stamped.HasMember("header") || !transform_stamped.HasMember("child_frame_id") || 
        !transform_stamped.HasMember("transform")) {
      LOG_INFO("TfCallback: skipping invalid transform at index " << i);
      continue;
    }
    
    std::string child_frame = transform_stamped["child_frame_id"].GetString();
    const auto &transform = transform_stamped["transform"];
    
    if (!transform.HasMember("translation") || !transform.HasMember("rotation")) {
      LOG_INFO("TfCallback: skipping transform with missing translation/rotation for child_frame: " << child_frame);
      continue;
    }
    
    // 提取平移和旋转信息
    const auto &translation = transform["translation"];
    const auto &rotation = transform["rotation"];
    
    double x = translation["x"].GetDouble();
    double y = translation["y"].GetDouble();
    double qx = rotation.HasMember("x") ? rotation["x"].GetDouble() : 0.0;
    double qy = rotation.HasMember("y") ? rotation["y"].GetDouble() : 0.0;
    double qz = rotation.HasMember("z") ? rotation["z"].GetDouble() : 0.0;
    double qw = rotation.HasMember("w") ? rotation["w"].GetDouble() : 1.0;
    
    // 将四元数转换为欧拉角（yaw）
    double theta = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    // 获取父坐标系名称
    std::string parent_frame = "unknown";
    if (transform_stamped["header"].HasMember("frame_id")) {
      parent_frame = transform_stamped["header"]["frame_id"].GetString();
    }
    
    // LOG_INFO("TfCallback: transform - parent: [" << parent_frame << "] -> child: [" << child_frame 
    //         << "] (x: " << x << ", y: " << y << ", theta: " << theta << ")");
    
    // 存储变换数据
    TransformData tf_data;
    tf_data.x = x;
    tf_data.y = y;
    tf_data.theta = theta;
    tf_data.parent_frame = parent_frame;
    
    tf_cache_[child_frame] = tf_data;
  }
  
  // 更新 TF2Rosbridge 图结构
  tf2_.UpdateTF(tf_cache_);
}

/**
 * @brief 获取两个坐标系之间的变换
 * @param from 源坐标系
 * @param to 目标坐标系
 * @return 变换后的位姿
 */
basic::RobotPose RosbridgeComm::GetTransform(const std::string &from, const std::string &to) {
  std::lock_guard<std::mutex> lock(tf_cache_mutex_);
  return tf2_.LookUpForTransform(from, to);
}

/**
 * @brief 地图回调函数，处理占用栅格地图消息
 * @param msg ROSBridge消息
 */
void RosbridgeComm::MapCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull()) return;


  
  const auto &msg_json = msg.msg_json_;
  if (!msg_json.HasMember("info") || !msg_json.HasMember("data")) return;
  
  const auto &info = msg_json["info"];
  if (!info.HasMember("origin") || !info.HasMember("width") || 
      !info.HasMember("height") || !info.HasMember("resolution")) return;
  
  // 提取地图元信息
  const auto &origin = info["origin"]["position"];
  double origin_x = origin["x"].GetDouble();
  double origin_y = origin["y"].GetDouble();
  int width = info["width"].GetInt();
  int height = info["height"].GetInt();
  double resolution = info["resolution"].GetDouble();
  
  // 创建占用栅格地图
  basic::OccupancyMap new_map(height, width, Eigen::Vector3d(origin_x, origin_y, 0), resolution);
  
  // 填充地图数据
  const auto &data = msg_json["data"];
  if (data.IsArray()) {
    rapidjson::SizeType max_size = static_cast<rapidjson::SizeType>(width * height);
    for (rapidjson::SizeType i = 0; i < data.Size() && i < max_size; i++) {
      int x = static_cast<int>(i / width);
      int y = static_cast<int>(i % width);
      int8_t value = data[i].GetInt();
      new_map(x, y) = value;
    }
  }
  new_map.SetFlip();
  
  occ_map_ = new_map;
  PUBLISH(MSG_ID_OCCUPANCY_MAP, new_map);
}

/**
 * @brief 局部代价地图回调函数
 * @param msg ROSBridge消息
 */
void RosbridgeComm::LocalCostMapCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull() || occ_map_.cols == 0 || occ_map_.rows == 0) return;
  
  const auto &msg_json = msg.msg_json_;
  if (!msg_json.HasMember("info") || !msg_json.HasMember("data")) return;
  
  const auto &info = msg_json["info"];
  if (!info.HasMember("origin") || !info.HasMember("width") || 
      !info.HasMember("height") || !info.HasMember("resolution")) return;
  
  // 提取原点信息（包含位置和方向）
  const auto &origin = info["origin"];
  double origin_x = origin["position"]["x"].GetDouble();
  double origin_y = origin["position"]["y"].GetDouble();
  double origin_theta = 0.0;
  if (origin.HasMember("orientation")) {
    const auto &orientation = origin["orientation"];
    double qx = orientation.HasMember("x") ? orientation["x"].GetDouble() : 0.0;
    double qy = orientation.HasMember("y") ? orientation["y"].GetDouble() : 0.0;
    double qz = orientation.HasMember("z") ? orientation["z"].GetDouble() : 0.0;
    double qw = orientation.HasMember("w") ? orientation["w"].GetDouble() : 1.0;
    origin_theta = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  }
  
  int width = info["width"].GetInt();
  int height = info["height"].GetInt();
  double resolution = info["resolution"].GetDouble();
  
  // 创建代价地图
  basic::OccupancyMap cost_map(height, width, Eigen::Vector3d(origin_x, origin_y, 0), resolution);
  
  // 填充代价地图数据
  const auto &data = msg_json["data"];
  if (data.IsArray()) {
    rapidjson::SizeType max_size = static_cast<rapidjson::SizeType>(width * height);
    for (rapidjson::SizeType i = 0; i < data.Size() && i < max_size; i++) {
      int x = static_cast<int>(i / width);
      int y = static_cast<int>(i % width);
      int8_t value = data[i].GetInt();
      cost_map(x, y) = value;
    }
  }
  cost_map.SetFlip();
  
  // 将局部代价地图叠加到全局地图上
  basic::OccupancyMap sized_cost_map = occ_map_;
  basic::RobotPose origin_pose;
  origin_pose.x = origin_x;
  origin_pose.y = origin_y + cost_map.heightMap();
  origin_pose.theta = origin_theta;
  
  // 计算原点在地图坐标系中的位置
  double map_o_x, map_o_y;
  occ_map_.xy2OccPose(origin_pose.x, origin_pose.y, map_o_x, map_o_y);
  sized_cost_map.map_data.setZero();
  
  // 将局部代价地图数据复制到全局地图对应位置
  for (int x = 0; x < occ_map_.rows; x++)
    for (int y = 0; y < occ_map_.cols; y++) {
      if (x > map_o_x && y > map_o_y && y < map_o_y + cost_map.rows &&
          x < map_o_x + cost_map.cols) {
        sized_cost_map(x, y) = cost_map(x - map_o_x, y - map_o_y);
      } else {
        sized_cost_map(x, y) = 0;
      }
    }
  PUBLISH(MSG_ID_LOCAL_COST_MAP, sized_cost_map);
}

/**
 * @brief 全局代价地图回调函数
 * @param msg ROSBridge消息
 */
void RosbridgeComm::GlobalCostMapCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull()) return;
  
  const auto &msg_json = msg.msg_json_;
  if (!msg_json.HasMember("info") || !msg_json.HasMember("data")) return;
  
  const auto &info = msg_json["info"];
  if (!info.HasMember("origin") || !info.HasMember("width") || 
      !info.HasMember("height") || !info.HasMember("resolution")) return;
  
  // 提取地图元信息
  const auto &origin = info["origin"]["position"];
  double origin_x = origin["x"].GetDouble();
  double origin_y = origin["y"].GetDouble();
  int width = info["width"].GetInt();
  int height = info["height"].GetInt();
  double resolution = info["resolution"].GetDouble();
  
  // 创建代价地图
  basic::OccupancyMap cost_map(height, width, Eigen::Vector3d(origin_x, origin_y, 0), resolution);
  
  // 填充地图数据
  const auto &data = msg_json["data"];
  if (data.IsArray()) {
    rapidjson::SizeType max_size = static_cast<rapidjson::SizeType>(width * height);
    for (rapidjson::SizeType i = 0; i < data.Size() && i < max_size; i++) {
      int x = static_cast<int>(i / width);
      int y = static_cast<int>(i % width);
      int8_t value = data[i].GetInt();
      cost_map(x, y) = value;
    }
  }
  cost_map.SetFlip();
  PUBLISH(MSG_ID_GLOBAL_COST_MAP, cost_map);
}

/**
 * @brief 激光扫描回调函数
 * @param msg ROSBridge消息
 */
void RosbridgeComm::LaserCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull()) return;
  
  const auto &msg_json = msg.msg_json_;
  if (!msg_json.HasMember("angle_min") || !msg_json.HasMember("angle_max") || 
      !msg_json.HasMember("angle_increment") || !msg_json.HasMember("ranges")) return;
  
  // 提取角度参数
  double angle_min = msg_json["angle_min"].GetDouble();
  double angle_increment = msg_json["angle_increment"].GetDouble();
  
  // 转换激光扫描数据为点云
  basic::LaserScan laser_points;
  const auto &ranges = msg_json["ranges"];
  if (ranges.IsArray()) {
    for (rapidjson::SizeType i = 0; i < ranges.Size(); i++) {
      double dist = ranges[i].GetDouble();
      // 跳过无效距离值
      if (std::isinf(dist)) continue;
      
      // 计算当前点的角度和坐标
      double angle = angle_min + i * angle_increment;
      double x = dist * std::cos(angle);
      double y = dist * std::sin(angle);
      
      basic::Point p;
      p.x = x;
      p.y = y;
      laser_points.push_back(p);
    }
  }
  laser_points.id = 0;
  PUBLISH(MSG_ID_LASER_SCAN, laser_points);
}

/**
 * @brief 全局路径回调函数
 * @param msg ROSBridge消息
 */
void RosbridgeComm::PathCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull()) return;
  
  const auto &msg_json = msg.msg_json_;
  if (!msg_json.HasMember("poses")) return;
  
  // 提取路径点
  basic::RobotPath path;
  const auto &poses = msg_json["poses"];
  if (poses.IsArray()) {
    for (rapidjson::SizeType i = 0; i < poses.Size(); i++) {
      const auto &pose = poses[i]["pose"]["position"];
      basic::Point point;
      point.x = pose["x"].GetDouble();
      point.y = pose["y"].GetDouble();
      path.push_back(point);
    }
  }
  PUBLISH(MSG_ID_GLOBAL_PATH, path);
}

/**
 * @brief 局部路径回调函数
 * @param msg ROSBridge消息
 */
void RosbridgeComm::LocalPathCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull()) return;
  
  const auto &msg_json = msg.msg_json_;
  if (!msg_json.HasMember("poses")) return;
  
  // 提取路径点
  basic::RobotPath path;
  const auto &poses = msg_json["poses"];
  if (poses.IsArray()) {
    for (rapidjson::SizeType i = 0; i < poses.Size(); i++) {
      const auto &pose = poses[i]["pose"]["position"];
      basic::Point point;
      point.x = pose["x"].GetDouble();
      point.y = pose["y"].GetDouble();
      path.push_back(point);
    }
  }
  PUBLISH(MSG_ID_LOCAL_PATH, path);
}

/**
 * @brief 电池状态回调函数
 * @param msg ROSBridge消息
 */
void RosbridgeComm::BatteryCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull()) return;
  
  const auto &msg_json = msg.msg_json_;
  std::map<std::string, std::string> map;
  
  // 提取电池百分比和电压
  if (msg_json.HasMember("percentage")) {
    map["percent"] = std::to_string(msg_json["percentage"].GetDouble());
  }
  if (msg_json.HasMember("voltage")) {
    map["voltage"] = std::to_string(msg_json["voltage"].GetDouble());
  }
  PUBLISH(MSG_ID_BATTERY_STATE, map);
}

/**
 * @brief 里程计回调函数
 * @param msg ROSBridge消息
 */
void RosbridgeComm::OdomCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull()) return;
  
  const auto &msg_json = msg.msg_json_;
  if (!msg_json.HasMember("pose") || !msg_json.HasMember("twist")) return;
  
  basic::RobotState state;
  
  // 提取速度信息
  const auto &twist = msg_json["twist"]["twist"];
  state.vx = twist.HasMember("linear") ? twist["linear"]["x"].GetDouble() : 0.0;
  state.vy = twist.HasMember("linear") ? twist["linear"]["y"].GetDouble() : 0.0;
  state.w = twist.HasMember("angular") ? twist["angular"]["z"].GetDouble() : 0.0;
  
  // 提取位置信息
  const auto &pose = msg_json["pose"]["pose"];
  state.x = pose["position"]["x"].GetDouble();
  state.y = pose["position"]["y"].GetDouble();
  
  // 提取方向信息（四元数转欧拉角）
  if (pose.HasMember("orientation")) {
    const auto &orientation = pose["orientation"];
    double qx = orientation.HasMember("x") ? orientation["x"].GetDouble() : 0.0;
    double qy = orientation.HasMember("y") ? orientation["y"].GetDouble() : 0.0;
    double qz = orientation.HasMember("z") ? orientation["z"].GetDouble() : 0.0;
    double qw = orientation.HasMember("w") ? orientation["w"].GetDouble() : 1.0;
    state.theta = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  }
  
  PUBLISH(MSG_ID_ODOM_POSE, state);
}

/**
 * @brief 机器人足迹回调函数
 * @param msg ROSBridge消息
 */
void RosbridgeComm::RobotFootprintCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull()) return;
  
  const auto &msg_json = msg.msg_json_;
  if (!msg_json.HasMember("polygon") || !msg_json["polygon"].HasMember("points")) return;
  
  // 提取机器人足迹多边形点
  basic::RobotPath footprint;
  const auto &points = msg_json["polygon"]["points"];
  if (points.IsArray()) {
    for (rapidjson::SizeType i = 0; i < points.Size(); i++) {
      basic::Point p;
      p.x = points[i]["x"].GetDouble();
      p.y = points[i]["y"].GetDouble();
      footprint.push_back(p);
    }
  }
  PUBLISH(MSG_ID_ROBOT_FOOTPRINT, footprint);
}

/**
 * @brief 拓扑地图回调函数
 * @param msg ROSBridge消息
 */
void RosbridgeComm::TopologyMapCallback(const ROSBridgePublishMsg &msg) {
  if (msg.msg_json_.IsNull()) return;
  
  const auto &msg_json = msg.msg_json_;
  TopologyMap topology_map;
  
  // 提取地图名称
  if (msg_json.HasMember("map_name")) {
    topology_map.map_name = msg_json["map_name"].GetString();
  }
  
  // 提取地图属性
  if (msg_json.HasMember("map_property")) {
    const auto &map_property = msg_json["map_property"];
    
    // 提取支持的控制器列表
    if (map_property.HasMember("support_controllers")) {
      const auto &controllers = map_property["support_controllers"];
      if (controllers.IsArray()) {
        for (rapidjson::SizeType i = 0; i < controllers.Size(); i++) {
          std::string controller = controllers[i].GetString();
          if (std::find(topology_map.map_property.support_controllers.begin(),
                       topology_map.map_property.support_controllers.end(),
                       controller) == topology_map.map_property.support_controllers.end()) {
            topology_map.map_property.support_controllers.push_back(controller);
          }
        }
      }
    }
    
    // 提取支持的目标检查器列表
    if (map_property.HasMember("support_goal_checkers")) {
      const auto &goal_checkers = map_property["support_goal_checkers"];
      if (goal_checkers.IsArray()) {
        for (rapidjson::SizeType i = 0; i < goal_checkers.Size(); i++) {
          std::string goal_checker = goal_checkers[i].GetString();
          if (std::find(topology_map.map_property.support_goal_checkers.begin(),
                       topology_map.map_property.support_goal_checkers.end(),
                       goal_checker) == topology_map.map_property.support_goal_checkers.end()) {
            topology_map.map_property.support_goal_checkers.push_back(goal_checker);
          }
        }
      }
    }
  }
  
  // 提取拓扑点信息
  if (msg_json.HasMember("points")) {
    const auto &points = msg_json["points"];
    if (points.IsArray()) {
      for (rapidjson::SizeType i = 0; i < points.Size(); i++) {
        const auto &point_msg = points[i];
        TopologyMap::PointInfo point_info;
        point_info.name = point_msg["name"].GetString();
        point_info.x = point_msg["x"].GetDouble();
        point_info.y = point_msg["y"].GetDouble();
        point_info.theta = point_msg["theta"].GetDouble();
        point_info.type = static_cast<PointType>(point_msg["type"].GetInt());
        topology_map.points.push_back(point_info);
      }
    }
  }
  
  // 提取路由信息
  if (msg_json.HasMember("routes")) {
    const auto &routes = msg_json["routes"];
    if (routes.IsArray()) {
      for (rapidjson::SizeType i = 0; i < routes.Size(); i++) {
        const auto &route_msg = routes[i];
        TopologyMap::RouteInfo route_info;
        route_info.controller = route_msg["route_info"]["controller"].GetString();
        route_info.speed_limit = route_msg["route_info"]["speed_limit"].GetDouble();
        route_info.goal_checker = route_msg["route_info"]["goal_checker"].GetString();
        topology_map.routes[route_msg["from_point"].GetString()][route_msg["to_point"].GetString()] = route_info;
      }
    }
  }
  
  LOG_INFO("recv topology map:" << topology_map.map_name);
  PUBLISH(MSG_ID_TOPOLOGY_MAP, topology_map);
}

/**
 * @brief 图像回调函数，处理图像消息
 * @param msg ROSBridge消息
 * @param location 图像位置标识
 */
void RosbridgeComm::ImageCallback(const ROSBridgePublishMsg &msg, const std::string &location) {
  if (msg.msg_json_.IsNull()) return;
  
  const auto &msg_json = msg.msg_json_;
  if (!msg_json.HasMember("encoding") || !msg_json.HasMember("data")) return;
  
  std::string encoding = msg_json["encoding"].GetString();
  const auto &data = msg_json["data"];
  
  cv::Mat conversion_mat_;
  
  if (data.IsArray()) {
    // 提取图像数据
    std::vector<uint8_t> image_data;
    for (rapidjson::SizeType i = 0; i < data.Size(); i++) {
      image_data.push_back(static_cast<uint8_t>(data[i].GetInt()));
    }
    
    int width = msg_json.HasMember("width") ? msg_json["width"].GetInt() : 0;
    int height = msg_json.HasMember("height") ? msg_json["height"].GetInt() : 0;
    
    // 根据编码格式处理图像
    if (encoding == "rgb8" || encoding == "RGB8") {
      // RGB8格式，直接使用
      cv::Mat img(height, width, CV_8UC3, image_data.data());
      conversion_mat_ = img.clone();
    } else if (encoding == "8UC1" || encoding == "mono8") {
      // 灰度图，转换为RGB
      cv::Mat img(height, width, CV_8UC1, image_data.data());
      cv::cvtColor(img, conversion_mat_, CV_GRAY2RGB);
    } else if (encoding == "16UC1" || encoding == "32FC1") {
      // 16位或32位深度图，需要归一化
      double min = 0;
      double max = 10;
      
      if (encoding == "16UC1") {
        // 16位深度图处理
        std::vector<uint16_t> image_data_16;
        for (rapidjson::SizeType i = 0; i < data.Size(); i++) {
          image_data_16.push_back(static_cast<uint16_t>(data[i].GetInt()));
        }
        cv::Mat img(height, width, CV_16UC1, image_data_16.data());
        max *= 1000;  // 16位深度值范围调整
        cv::Mat img_scaled_8u;
        cv::Mat(img - min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
      } else {
        // 32位浮点深度图处理
        std::vector<float> image_data_32;
        for (rapidjson::SizeType i = 0; i < data.Size(); i++) {
          image_data_32.push_back(static_cast<float>(data[i].GetDouble()));
        }
        cv::Mat img(height, width, CV_32FC1, image_data_32.data());
        cv::Mat img_scaled_8u;
        cv::Mat(img - min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
      }
    } else {
      LOG_ERROR("Unsupported image encoding: " << encoding);
      return;
    }
  }
  
  PUBLISH(MSG_ID_IMAGE, (std::pair<std::string, cv::Mat>(location, conversion_mat_)));
}

/**
 * @brief 发布重定位位姿
 * @param pose 机器人位姿
 */
void RosbridgeComm::PubRelocPose(const RobotPose &pose) {
  rapidjson::Document msg;
  msg.SetObject();
  auto &allocator = msg.GetAllocator();
  
  // 构建消息头
  rapidjson::Value header(rapidjson::kObjectType);
  header.AddMember("frame_id", rapidjson::Value("map", allocator), allocator);
  
  // 设置当前时间戳
  ROSTime now = ROSTime::now();
  rapidjson::Value stamp(rapidjson::kObjectType);
  stamp.AddMember("secs", static_cast<uint64_t>(now.sec_), allocator);
  stamp.AddMember("nsecs", static_cast<uint64_t>(now.nsec_), allocator);
  header.AddMember("stamp", stamp, allocator);
  
  msg.AddMember("header", header, allocator);
  
  // 构建位姿信息
  rapidjson::Value pose_value(rapidjson::kObjectType);
  
  // 位置
  rapidjson::Value position(rapidjson::kObjectType);
  position.AddMember("x", pose.x, allocator);
  position.AddMember("y", pose.y, allocator);
  position.AddMember("z", 0.0, allocator);
  pose_value.AddMember("position", position, allocator);
  
  // 方向（四元数）
  rapidjson::Value orientation(rapidjson::kObjectType);
  double qw = std::cos(pose.theta / 2.0);
  double qz = std::sin(pose.theta / 2.0);
  orientation.AddMember("x", 0.0, allocator);
  orientation.AddMember("y", 0.0, allocator);
  orientation.AddMember("z", qz, allocator);
  orientation.AddMember("w", qw, allocator);
  pose_value.AddMember("orientation", orientation, allocator);
  
  // 协方差矩阵（36个元素）
  rapidjson::Value covariance(rapidjson::kArrayType);
  for (int i = 0; i < 36; i++) {
    covariance.PushBack(0.0, allocator);
  }

  rapidjson::Value pose_with_covariance(rapidjson::kObjectType);
  pose_with_covariance.AddMember("pose", pose_value, allocator);
  pose_with_covariance.AddMember("covariance", covariance, allocator);

  msg.AddMember("pose", pose_with_covariance, allocator);
  
  // 发布消息
  auto it = publishers_.find(GET_TOPIC_NAME(MSG_ID_SET_RELOC_POSE));
  if (it != publishers_.end()) {
    it->second->Publish(msg);
  }
}

/**
 * @brief 发布导航目标点
 * @param pose 目标位姿
 */
void RosbridgeComm::PubNavGoal(const RobotPose &pose) {
  rapidjson::Document msg;
  msg.SetObject();
  auto &allocator = msg.GetAllocator();
  
  // 构建消息头
  rapidjson::Value header(rapidjson::kObjectType);
  header.AddMember("frame_id", rapidjson::Value("map", allocator), allocator);
  
  // 设置当前时间戳
  ROSTime now = ROSTime::now();
  rapidjson::Value stamp(rapidjson::kObjectType);
  stamp.AddMember("secs", static_cast<uint64_t>(now.sec_), allocator);
  stamp.AddMember("nsecs", static_cast<uint64_t>(now.nsec_), allocator);
  header.AddMember("stamp", stamp, allocator);
  
  msg.AddMember("header", header, allocator);
  
  // 构建位姿信息
  rapidjson::Value pose_value(rapidjson::kObjectType);

  // 位置
  rapidjson::Value position(rapidjson::kObjectType);
  position.AddMember("x", pose.x, allocator);
  position.AddMember("y", pose.y, allocator);
  position.AddMember("z", 0.0, allocator);
  pose_value.AddMember("position", position, allocator);

  // 方向（四元数）
  rapidjson::Value orientation(rapidjson::kObjectType);
  double qw = std::cos(pose.theta / 2.0);
  double qz = std::sin(pose.theta / 2.0);
  orientation.AddMember("x", 0.0, allocator);
  orientation.AddMember("y", 0.0, allocator);
  orientation.AddMember("z", qz, allocator);
  orientation.AddMember("w", qw, allocator);
  pose_value.AddMember("orientation", orientation, allocator);

  msg.AddMember("pose", pose_value, allocator);
  
  // 发布消息
  auto it = publishers_.find(GET_TOPIC_NAME(DISPLAY_GOAL));
  if (it != publishers_.end()) {
    it->second->Publish(msg);
  }
}

/**
 * @brief 发布机器人速度
 * @param speed 速度信息
 */
void RosbridgeComm::PubRobotSpeed(const RobotSpeed &speed) {
  rapidjson::Document msg;
  msg.SetObject();
  auto &allocator = msg.GetAllocator();
  
  // 线速度
  rapidjson::Value linear(rapidjson::kObjectType);
  linear.AddMember("x", speed.vx, allocator);
  linear.AddMember("y", speed.vy, allocator);
  linear.AddMember("z", 0.0, allocator);
  
  // 角速度
  rapidjson::Value angular(rapidjson::kObjectType);
  angular.AddMember("x", 0.0, allocator);
  angular.AddMember("y", 0.0, allocator);
  angular.AddMember("z", speed.w, allocator);
  
  msg.AddMember("linear", linear, allocator);
  msg.AddMember("angular", angular, allocator);
  
  // 发布消息
  auto it = publishers_.find(GET_TOPIC_NAME(MSG_ID_SET_ROBOT_SPEED));
  if (it != publishers_.end()) {
    it->second->Publish(msg);
  }
}

/**
 * @brief 发布拓扑地图更新
 * @param topology_map 拓扑地图数据
 */
void RosbridgeComm::PubTopologyMapUpdate(const TopologyMap &topology_map) {
  rapidjson::Document msg;
  msg.SetObject();
  auto &allocator = msg.GetAllocator();
  
  // 地图名称
  msg.AddMember("map_name", rapidjson::Value(topology_map.map_name.c_str(), allocator), allocator);
  
  // 地图属性
  rapidjson::Value map_property(rapidjson::kObjectType);
  
  // 支持的控制器列表
  rapidjson::Value support_controllers(rapidjson::kArrayType);
  for (const auto& controller : topology_map.map_property.support_controllers) {
    support_controllers.PushBack(rapidjson::Value(controller.c_str(), allocator), allocator);
  }
  map_property.AddMember("support_controllers", support_controllers, allocator);
  
  // 支持的目标检查器列表
  rapidjson::Value support_goal_checkers(rapidjson::kArrayType);
  for (const auto& goal_checker : topology_map.map_property.support_goal_checkers) {
    support_goal_checkers.PushBack(rapidjson::Value(goal_checker.c_str(), allocator), allocator);
  }
  map_property.AddMember("support_goal_checkers", support_goal_checkers, allocator);
  
  msg.AddMember("map_property", map_property, allocator);
  
  // 拓扑点列表
  rapidjson::Value points(rapidjson::kArrayType);
  for (const auto& point : topology_map.points) {
    rapidjson::Value point_value(rapidjson::kObjectType);
    point_value.AddMember("name", rapidjson::Value(point.name.c_str(), allocator), allocator);
    point_value.AddMember("x", point.x, allocator);
    point_value.AddMember("y", point.y, allocator);
    point_value.AddMember("theta", point.theta, allocator);
    point_value.AddMember("type", static_cast<int>(point.type), allocator);
    points.PushBack(point_value, allocator);
  }
  msg.AddMember("points", points, allocator);
  
  // 路由信息
  rapidjson::Value routes(rapidjson::kArrayType);
  for (const auto& from_routes : topology_map.routes) {
    for (const auto& route : from_routes.second) {
      rapidjson::Value route_value(rapidjson::kObjectType);
      route_value.AddMember("from_point", rapidjson::Value(from_routes.first.c_str(), allocator), allocator);
      route_value.AddMember("to_point", rapidjson::Value(route.first.c_str(), allocator), allocator);
      
      rapidjson::Value route_info(rapidjson::kObjectType);
      route_info.AddMember("controller", rapidjson::Value(route.second.controller.c_str(), allocator), allocator);
      route_info.AddMember("speed_limit", route.second.speed_limit, allocator);
      route_info.AddMember("goal_checker", rapidjson::Value(route.second.goal_checker.c_str(), allocator), allocator);
      route_value.AddMember("route_info", route_info, allocator);
      
      routes.PushBack(route_value, allocator);
    }
  }
  msg.AddMember("routes", routes, allocator);
  
  // 发布消息
  auto it = publishers_.find(GET_TOPIC_NAME(MSG_ID_TOPOLOGY_MAP_UPDATE));
  if (it != publishers_.end()) {
    it->second->Publish(msg);
  }
}
