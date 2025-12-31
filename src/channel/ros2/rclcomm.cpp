/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-27 14:47:24
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 09:14:17
 * @FilePath: /ros_qt5_gui_app/src/channel/ros1/rosnode.cpp
 * @Description: ros2通讯类
 */
#include "rclcomm.h"
#include <fstream>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include "config/config_manager.h"
#include "logger/logger.h"
#include "core/framework/framework.h"
#include "msg/msg_info.h"
rclcomm::rclcomm() {
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
  SET_DEFAULT_KEY_VALUE("BaseFrameId", "base_link")
  if (Config::ConfigManager::Instance()->GetRootConfig().images.empty()) {
    Config::ConfigManager::Instance()->GetRootConfig().images.push_back(
        Config::ImageDisplayConfig{.location = "front",
                                   .topic = "/camera/front/image_raw",
                                   .enable = true});
  }
  Config::ConfigManager::Instance()->StoreConfig();
}
bool rclcomm::Start() {
  rclcpp::init(0, nullptr);
  m_executor = new rclcpp::executors::MultiThreadedExecutor;

  node = rclcpp::Node::make_shared("ros_qt5_gui_app");
  m_executor->add_node(node);
  callback_group_laser =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_other =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub1_obt = rclcpp::SubscriptionOptions();
  sub1_obt.callback_group = callback_group_other;
  auto sub_laser_obt = rclcpp::SubscriptionOptions();
  sub_laser_obt.callback_group = callback_group_laser;

  nav_goal_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      GET_TOPIC_NAME(DISPLAY_GOAL), 10);
  reloc_pose_publisher_ =
      node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          GET_TOPIC_NAME(MSG_ID_SET_RELOC_POSE), 10);
  speed_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>(
      GET_TOPIC_NAME(MSG_ID_SET_ROBOT_SPEED), 10);
  map_subscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      GET_TOPIC_NAME(DISPLAY_MAP),
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&rclcomm::map_callback, this, std::placeholders::_1), sub1_obt);
  local_cost_map_subscriber_ =
      node->create_subscription<nav_msgs::msg::OccupancyGrid>(
          GET_TOPIC_NAME(DISPLAY_LOCAL_COST_MAP),
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          std::bind(&rclcomm::localCostMapCallback, this,
                    std::placeholders::_1),
          sub1_obt);
  global_cost_map_subscriber_ =
      node->create_subscription<nav_msgs::msg::OccupancyGrid>(
          GET_TOPIC_NAME(DISPLAY_GLOBAL_COST_MAP),
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          std::bind(&rclcomm::globalCostMapCallback, this,
                    std::placeholders::_1),
          sub1_obt);

  laser_scan_subscriber_ =
      node->create_subscription<sensor_msgs::msg::LaserScan>(
          GET_TOPIC_NAME(DISPLAY_LASER), 20,
          std::bind(&rclcomm::laser_callback, this, std::placeholders::_1),
          sub_laser_obt);
  battery_state_subscriber_ =
      node->create_subscription<sensor_msgs::msg::BatteryState>(
          GET_TOPIC_NAME(MSG_ID_BATTERY_STATE), 1,
          std::bind(&rclcomm::BatteryCallback, this, std::placeholders::_1),
          sub1_obt);
  global_path_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
      GET_TOPIC_NAME(DISPLAY_GLOBAL_PATH), 20,
      std::bind(&rclcomm::path_callback, this, std::placeholders::_1),
      sub1_obt);
  local_path_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
      GET_TOPIC_NAME(DISPLAY_LOCAL_PATH), 20,
      std::bind(&rclcomm::local_path_callback, this, std::placeholders::_1),
      sub1_obt);
  odometry_subscriber_ = node->create_subscription<nav_msgs::msg::Odometry>(
      GET_TOPIC_NAME(DISPLAY_ROBOT), 20,
      std::bind(&rclcomm::odom_callback, this, std::placeholders::_1),
      sub1_obt);
  robot_footprint_subscriber_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      GET_TOPIC_NAME(DISPLAY_ROBOT_FOOTPRINT), 20,
      std::bind(&rclcomm::robotFootprintCallback, this, std::placeholders::_1),
      sub1_obt);
  topology_map_subscriber_ = node->create_subscription<topology_msgs::msg::TopologyMap>(
      GET_TOPIC_NAME(DISPLAY_TOPOLOGY_MAP), 
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&rclcomm::topologyMapCallback, this, std::placeholders::_1),
      sub1_obt);
  topology_map_update_publisher_ = node->create_publisher<topology_msgs::msg::TopologyMap>(
      GET_TOPIC_NAME(MSG_ID_TOPOLOGY_MAP_UPDATE), 
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  for (auto one_image_display : Config::ConfigManager::Instance()->GetRootConfig().images) {
    LOG_INFO("image location:" << one_image_display.location << "topic:" << one_image_display.topic);
    image_subscriber_list_.emplace_back(
        node->create_subscription<sensor_msgs::msg::Image>(
            one_image_display.topic, 1, [this, one_image_display](const sensor_msgs::msg::Image::SharedPtr msg) {
              cv::Mat conversion_mat_;
              try {
                // 深拷贝转换为opencv类型
                cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
                    msg, sensor_msgs::image_encodings::RGB8);
                conversion_mat_ = cv_ptr->image;
              } catch (cv_bridge::Exception &e) {
                try {
                  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
                  if (msg->encoding == "CV_8UC3") {
                    // assuming it is rgb
                    conversion_mat_ = cv_ptr->image;
                  } else if (msg->encoding == "8UC1") {
                    // convert gray to rgb
                    cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
                  } else if (msg->encoding == "16UC1" ||
                             msg->encoding == "32FC1") {
                    double min = 0;
                    double max = 10;
                    if (msg->encoding == "16UC1") max *= 1000;
                    // if (ui_.dynamic_range_check_box->isChecked()) {
                    //   // dynamically adjust range based on min/max in image
                    //   cv::minMaxLoc(cv_ptr->image, &min, &max);
                    //   if (min == max) {
                    //     // completely homogeneous images are displayed in gray
                    //     min = 0;
                    //     max = 2;
                    //   }
                    // }
                    cv::Mat img_scaled_8u;
                    cv::Mat(cv_ptr->image - min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
                    cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
                  } else {
                    LOG_ERROR("image from " << msg->encoding
                                            << " to 'rgb8' an exception was thrown (%s)"
                                            << e.what());
                    return;
                  }
                } catch (cv_bridge::Exception &e) {
                  LOG_ERROR(
                      "image from "
                      << msg->encoding
                      << " to 'rgb8' an exception was thrown (%s)" << e.what());

                  return;
                }
              }
              PUBLISH(MSG_ID_IMAGE, (std::pair<std::string, cv::Mat>(one_image_display.location, conversion_mat_)));
            }));
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock(), std::chrono::seconds(10));
  transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  SUBSCRIBE(MSG_ID_SET_NAV_GOAL_POSE, [this](const RobotPose& pose) {
    std::cout << "recv nav goal pose:" << pose << std::endl;
    PubNavGoal(pose);
  });
  SUBSCRIBE(MSG_ID_SET_RELOC_POSE, [this](const RobotPose& pose) {
    std::cout << "recv reloc pose:" << pose << std::endl;
    PubRelocPose(pose);
  });
  SUBSCRIBE(MSG_ID_SET_ROBOT_SPEED, [this](const RobotSpeed& speed) {
    std::cout << "recv robot speed:" << speed << std::endl;
    PubRobotSpeed(speed);
  });
  SUBSCRIBE(MSG_ID_TOPOLOGY_MAP_UPDATE, [this](const TopologyMap& topology_map) {
    std::cout << "recv topology map update:" << topology_map.map_name << std::endl;
    topology_msgs::msg::TopologyMap ros_msg = ConvertToRosMsg(topology_map);
    topology_map_update_publisher_->publish(ros_msg);
  });
  
  init_flag_ = true;
  return true;
}

bool rclcomm::Stop() {
  rclcpp::shutdown();
  return true;
}

void rclcomm::BatteryCallback(
    const sensor_msgs::msg::BatteryState::SharedPtr msg) {
  std::map<std::string, std::string> map;
  map["percent"] = std::to_string(msg->percentage);
  map["voltage"] = std::to_string(msg->voltage);
  PUBLISH(MSG_ID_BATTERY_STATE, map);
}

void rclcomm::getRobotPose() {
  std::string base_frame = Config::ConfigManager::Instance()->GetConfigValue("BaseFrameId", "base_link");
  auto pose = getTransform(base_frame, "map");
  PUBLISH(MSG_ID_ROBOT_POSE, pose);
}
/**
 * @description: 获取坐标变化
 * @param {string} from 要变换的坐标系
 * @param {string} to 基坐标系
 * @return {basic::RobotPose}from变换到to坐标系下，需要变换的坐标
 */
basic::RobotPose rclcomm::getTransform(std::string from, std::string to) {
  basic::RobotPose ret;
  try {
    if (!tf_buffer_->canTransform(to, from, tf2::TimePointZero, std::chrono::milliseconds(100))) {
      return ret;
    }
    geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(to, from, tf2::TimePointZero, std::chrono::milliseconds(100));
    geometry_msgs::msg::Quaternion msg_quat = transform.transform.rotation;
    // 转换类型
    tf2::Quaternion q;
    tf2::fromMsg(msg_quat, q);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    // x y
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;

    ret.x = x;
    ret.y = y;
    ret.theta = yaw;

  } catch (tf2::TransformException &ex) {
    // LOG_ERROR("getTransform error from:" << from << " to:" << to
    //                                      << " error:" << ex.what());
  }
  return ret;
}
void rclcomm::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  basic::RobotState state;
  state.vx = (double)msg->twist.twist.linear.x;
  state.vy = (double)msg->twist.twist.linear.y;
  state.w = (double)msg->twist.twist.angular.z;
  state.x = (double)msg->pose.pose.position.x;
  state.y = (double)msg->pose.pose.position.y;

  geometry_msgs::msg::Quaternion msg_quat = msg->pose.pose.orientation;
  // 转换类型
  tf2::Quaternion q;
  tf2::fromMsg(msg_quat, q);
  tf2::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  state.theta = yaw;
  PUBLISH(MSG_ID_ODOM_POSE, state);
}
void rclcomm::local_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  try {
    if (!tf_buffer_->canTransform("map", msg->header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(100))) {
      return;
    }
    geometry_msgs::msg::PointStamped point_map_frame;
    geometry_msgs::msg::PointStamped point_odom_frame;
    basic::RobotPath path;
    for (int i = 0; i < msg->poses.size(); i++) {
      double x = msg->poses.at(i).pose.position.x;
      double y = msg->poses.at(i).pose.position.y;
      point_odom_frame.point.x = x;
      point_odom_frame.point.y = y;
      point_odom_frame.header.frame_id = msg->header.frame_id;
      point_odom_frame.header.stamp = msg->header.stamp;
      tf_buffer_->transform(point_odom_frame, point_map_frame, "map", std::chrono::milliseconds(100));
      basic::Point point;
      point.x = point_map_frame.point.x;
      point.y = point_map_frame.point.y;
      path.push_back(point);
    }
    PUBLISH(MSG_ID_LOCAL_PATH, path);
  } catch (tf2::TransformException &ex) {
  }
}

/// @brief loop for rate
void rclcomm::Process() {
  if (rclcpp::ok()) {
    m_executor->spin_some();
    getRobotPose();
  }
  // std::cout << "loop" << std::endl;
}

void rclcomm::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  try {
    if (!tf_buffer_->canTransform("map", msg->header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(100))) {
      return;
    }
    geometry_msgs::msg::PointStamped point_map_frame;
    geometry_msgs::msg::PointStamped point_odom_frame;
    basic::RobotPath path;
    for (int i = 0; i < msg->poses.size(); i++) {
      double x = msg->poses.at(i).pose.position.x;
      double y = msg->poses.at(i).pose.position.y;
      point_odom_frame.point.x = x;
      point_odom_frame.point.y = y;
      point_odom_frame.header.frame_id = msg->header.frame_id;
      point_odom_frame.header.stamp = msg->header.stamp;
      tf_buffer_->transform(point_odom_frame, point_map_frame, "map", std::chrono::milliseconds(100));
      basic::Point point;
      point.x = point_map_frame.point.x;
      point.y = point_map_frame.point.y;
      path.push_back(point);
    }
    PUBLISH(MSG_ID_GLOBAL_PATH, path);
  } catch (tf2::TransformException &ex) {
  }
}

void rclcomm::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // qDebug()<<"订阅到激光话题";
  // std::cout<<"recv laser"<<std::endl;
  double angle_min = msg->angle_min;
  double angle_max = msg->angle_max;
  double angle_increment = msg->angle_increment;
  try {
    //        geometry_msgs::msg::TransformStamped laser_transform =
    //        tf_buffer_->lookupTransform("map","base_scan",tf2::TimePointZero);
    geometry_msgs::msg::PointStamped point_base_frame;
    geometry_msgs::msg::PointStamped point_laser_frame;
    basic::LaserScan laser_points;
    for (int i = 0; i < msg->ranges.size(); i++) {
      // 计算当前偏移角度
      double angle = angle_min + i * angle_increment;
      double dist = msg->ranges[i];
      if (isinf(dist))
        continue;
      double x = dist * cos(angle);
      double y = dist * sin(angle);
      point_laser_frame.point.x = x;
      point_laser_frame.point.y = y;
      point_laser_frame.header.frame_id = msg->header.frame_id;
      std::string base_frame = Config::ConfigManager::Instance()->GetConfigValue("BaseFrameId", "base_link");
      tf_buffer_->transform(point_laser_frame, point_base_frame, base_frame);
      basic::Point p;
      p.x = point_base_frame.point.x;
      p.y = point_base_frame.point.y;
      laser_points.push_back(p);
    }
    laser_points.id = 0;
    PUBLISH(MSG_ID_LASER_SCAN, laser_points);
  } catch (tf2::TransformException &ex) {
    // tf_buffer_->lookupTransform("map", "base_scan", tf2::TimePointZero);
  }
}

void rclcomm::globalCostMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  int width = msg->info.width;
  int height = msg->info.height;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;
  basic::OccupancyMap cost_map(height, width,
                               Eigen::Vector3d(origin_x, origin_y, 0),
                               msg->info.resolution);
  for (int i = 0; i < msg->data.size(); i++) {
    int x = int(i / width);
    int y = i % width;
    cost_map(x, y) = msg->data[i];
  }
  cost_map.SetFlip();
  PUBLISH(MSG_ID_GLOBAL_COST_MAP, cost_map);
}
void rclcomm::localCostMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (occ_map_.cols == 0 || occ_map_.rows == 0)
    return;
  int width = msg->info.width;
  int height = msg->info.height;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(msg->info.origin.orientation, q);
  tf2::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  double origin_theta = yaw;
  basic::OccupancyMap cost_map(height, width,
                               Eigen::Vector3d(origin_x, origin_y, 0),
                               msg->info.resolution);
  for (int i = 0; i < msg->data.size(); i++) {
    int x = (int)i / width;
    int y = i % width;
    cost_map(x, y) = msg->data[i];
  }
  cost_map.SetFlip();
  basic::OccupancyMap sized_cost_map = occ_map_;
  basic::RobotPose origin_pose;
  try {
    // 坐标变换 将局部代价地图的基础坐标转换为map下 进行绘制显示
    geometry_msgs::msg::PoseStamped pose_map_frame;
    geometry_msgs::msg::PoseStamped pose_curr_frame;
    pose_curr_frame.pose.position.x = origin_x;
    pose_curr_frame.pose.position.y = origin_y;
    q.setRPY(0, 0, origin_theta);
    pose_curr_frame.pose.orientation = tf2::toMsg(q);
    pose_curr_frame.header.frame_id = msg->header.frame_id;
    tf_buffer_->transform(pose_curr_frame, pose_map_frame, "map");
    tf2::fromMsg(pose_map_frame.pose.orientation, q);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    origin_pose.x = pose_map_frame.pose.position.x;
    origin_pose.y = pose_map_frame.pose.position.y + cost_map.heightMap();
    origin_pose.theta = yaw;
  } catch (tf2::TransformException &ex) {
    LOG_ERROR("getTransform localCostMapCallback error:" << ex.what());
  }

  double map_o_x, map_o_y;
  occ_map_.xy2OccPose(origin_pose.x, origin_pose.y, map_o_x, map_o_y);
  sized_cost_map.map_data.setZero();
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
void rclcomm::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;
  int width = msg->info.width;
  int height = msg->info.height;
  double resolution = msg->info.resolution;
  basic::OccupancyMap new_map(
      height, width, Eigen::Vector3d(origin_x, origin_y, 0), resolution);

  for (int i = 0; i < msg->data.size(); i++) {
    int x = int(i / width);
    int y = i % width;
    new_map(x, y) = msg->data[i];
  }
  new_map.SetFlip();
  
  occ_map_ = new_map;
  PUBLISH(MSG_ID_OCCUPANCY_MAP, new_map);
}

void rclcomm::PubRelocPose(const RobotPose &pose) {
  geometry_msgs::msg::PoseWithCovarianceStamped geo_pose;
  geo_pose.header.frame_id = "map";
  geo_pose.header.stamp = node->get_clock()->now();
  geo_pose.pose.pose.position.x = pose.x;
  geo_pose.pose.pose.position.y = pose.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.theta);
  geo_pose.pose.pose.orientation = tf2::toMsg(q);
  reloc_pose_publisher_->publish(geo_pose);
}
void rclcomm::PubNavGoal(const RobotPose &pose) {
  geometry_msgs::msg::PoseStamped geo_pose;
  geo_pose.header.frame_id = "map";
  geo_pose.header.stamp = node->get_clock()->now();
  geo_pose.pose.position.x = pose.x;
  geo_pose.pose.position.y = pose.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.theta);
  geo_pose.pose.orientation = tf2::toMsg(q);
  nav_goal_publisher_->publish(geo_pose);
}
void rclcomm::PubRobotSpeed(const RobotSpeed &speed) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = speed.vx;
  twist.linear.y = speed.vy;
  twist.linear.z = 0;

  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = speed.w;

  // Publish it and resolve any remaining callbacks
  speed_publisher_->publish(twist);
}

void rclcomm::robotFootprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
  try {
    geometry_msgs::msg::PointStamped point_map_frame;
    geometry_msgs::msg::PointStamped point_footprint_frame;
    basic::RobotPath footprint;
    
    for (const auto& point : msg->polygon.points) {
      point_footprint_frame.point.x = point.x;
      point_footprint_frame.point.y = point.y;
      point_footprint_frame.header.frame_id = msg->header.frame_id;
      
      tf_buffer_->transform(point_footprint_frame, point_map_frame, "map");
      
      basic::Point p;
      p.x = point_map_frame.point.x;
      p.y = point_map_frame.point.y;
      footprint.push_back(p);
    }
    
    PUBLISH(MSG_ID_ROBOT_FOOTPRINT, footprint);
  } catch (tf2::TransformException &ex) {
    LOG_ERROR("robotFootprintCallback transform error: " << ex.what());
  }
}

TopologyMap rclcomm::ConvertFromRosMsg(const topology_msgs::msg::TopologyMap::SharedPtr msg) {
  TopologyMap topology_map;
  
  topology_map.map_name = msg->map_name;
  
  for (const auto& controller : msg->map_property.support_controllers) {
    if(std::find(topology_map.map_property.support_controllers.begin(), topology_map.map_property.support_controllers.end(), controller) == topology_map.map_property.support_controllers.end()) {
      topology_map.map_property.support_controllers.push_back(controller);
    }
    LOG_INFO("support controller:" << controller);
  }

  for (const auto& goal_checker : msg->map_property.support_goal_checkers) {
    if(std::find(topology_map.map_property.support_goal_checkers.begin(), topology_map.map_property.support_goal_checkers.end(), goal_checker) == topology_map.map_property.support_goal_checkers.end()) {
      topology_map.map_property.support_goal_checkers.push_back(goal_checker);
    }
    LOG_INFO("support goal checker:" << goal_checker);
  }
  
  for (const auto& point_msg : msg->points) {
    TopologyMap::PointInfo point_info;
    point_info.name = point_msg.name;
    point_info.x = point_msg.x;
    point_info.y = point_msg.y;
    point_info.theta = point_msg.theta;
    point_info.type = static_cast<PointType>(point_msg.type);
    topology_map.points.push_back(point_info);
  }
  
  for (const auto& route_msg : msg->routes) {
    TopologyMap::RouteInfo route_info;
    route_info.controller = route_msg.route_info.controller;
    route_info.speed_limit = route_msg.route_info.speed_limit;
    route_info.goal_checker = route_msg.route_info.goal_checker;
    topology_map.routes[route_msg.from_point][route_msg.to_point] = route_info;
  }
  
  return topology_map;
}

topology_msgs::msg::TopologyMap rclcomm::ConvertToRosMsg(const TopologyMap& topology_map) {
  topology_msgs::msg::TopologyMap msg;
  
  msg.map_name = topology_map.map_name;
  
  for (const auto& controller : topology_map.map_property.support_controllers) {
    if(std::find(msg.map_property.support_controllers.begin(), msg.map_property.support_controllers.end(), controller) == msg.map_property.support_controllers.end()) {
      msg.map_property.support_controllers.push_back(controller);
    }
  }

  for (const auto& goal_checker : topology_map.map_property.support_goal_checkers) {
    if(std::find(msg.map_property.support_goal_checkers.begin(), msg.map_property.support_goal_checkers.end(), goal_checker) == msg.map_property.support_goal_checkers.end()) {
      msg.map_property.support_goal_checkers.push_back(goal_checker);
    }
  }
  
  for (const auto& point : topology_map.points) {
    topology_msgs::msg::TopologyMapPointInfo point_msg;
    point_msg.name = point.name;
    point_msg.x = point.x;
    point_msg.y = point.y;
    point_msg.theta = point.theta;
    point_msg.type = static_cast<uint8_t>(point.type);
    msg.points.push_back(point_msg);
  }
  
  for (const auto& from_routes : topology_map.routes) {
    for (const auto& route : from_routes.second) {
      topology_msgs::msg::RouteConnection route_msg;
      route_msg.from_point = from_routes.first;
      route_msg.to_point = route.first;
      route_msg.route_info.controller = route.second.controller;
      route_msg.route_info.speed_limit = route.second.speed_limit;
      route_msg.route_info.goal_checker = route.second.goal_checker;
      msg.routes.push_back(route_msg);
    }
  }
  
  return msg;
}

void rclcomm::topologyMapCallback(const topology_msgs::msg::TopologyMap::SharedPtr msg) {
  TopologyMap topology_map = ConvertFromRosMsg(msg);
  LOG_INFO("recv topology map:" << topology_map.map_name);
  for (const auto& point : topology_map.points) {
    LOG_INFO("point:" << point.name << " x:" << point.x << " y:" << point.y << " theta:" << point.theta);
  }
  for (const auto& route : topology_map.routes) {
    for (const auto& route_info : route.second) {
      LOG_INFO("route:" << route.first << " -> " << route_info.first << " controller:" << route_info.second.controller << " speed_limit:" << route_info.second.speed_limit);
    }
  }
  PUBLISH(MSG_ID_TOPOLOGY_MAP, topology_map);
}
