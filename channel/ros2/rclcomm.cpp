/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-27 14:47:24
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 09:14:17
 * @FilePath: /ros_qt5_gui_app/src/channel/ros1/rosnode.cpp
 * @Description: ros2通讯类
 */
#include "rclcomm.h"
#include "config/configManager.h"
// #include "logger/logger.h"
#include <fstream>
rclcomm::rclcomm() {}
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

  navGoalPublisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      GET_TOPIC_NAME("NavGoal"), 10);
  initPosePublisher_ =
      node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          GET_TOPIC_NAME("Reloc"), 10);
  _publisher =
      node->create_publisher<std_msgs::msg::Int32>("ros2_qt_dmeo_publish", 10);
  speed_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>(
      GET_TOPIC_NAME("Speed"), 10);
  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      GET_TOPIC_NAME("Map"),
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&rclcomm::map_callback, this, std::placeholders::_1), sub1_obt);
  m_localCostMapSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      GET_TOPIC_NAME("LocalCostMap"),
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&rclcomm::localCostMapCallback, this, std::placeholders::_1),
      sub1_obt);
  m_globalCostMapSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      GET_TOPIC_NAME("GlobalCostMap"),
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&rclcomm::globalCostMapCallback, this, std::placeholders::_1),
      sub1_obt);
  m_laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      GET_TOPIC_NAME("LaserScan"), 20,
      std::bind(&rclcomm::laser_callback, this, std::placeholders::_1),
      sub_laser_obt);
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      GET_TOPIC_NAME("GlobalPlan"), 20,
      std::bind(&rclcomm::path_callback, this, std::placeholders::_1),
      sub1_obt);
  _local_path_sub = node->create_subscription<nav_msgs::msg::Path>(
      GET_TOPIC_NAME("LocalPlan"), 20,
      std::bind(&rclcomm::local_path_callback, this, std::placeholders::_1),
      sub1_obt);
  m_odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      GET_TOPIC_NAME("Odometry"), 20,
      std::bind(&rclcomm::odom_callback, this, std::placeholders::_1),
      sub1_obt);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  init_flag_ = true;
  return true;
}
bool rclcomm::Stop() {
  rclcpp::shutdown();
  return true;
}
void rclcomm::SendMessage(const Msg::MsgId &msg_id, const std::any &msg) {
  switch (msg_id) {
  case Msg::MsgId::kSetNavGoalPose: {
    auto pose = std::any_cast<basic::RobotPose>(msg);
    std::cout << "recv nav goal pose:" << pose << std::endl;

    PubNavGoal(pose);

  } break;
  case Msg::MsgId::kSetRelocPose: {
    auto pose = std::any_cast<basic::RobotPose>(msg);
    std::cout << "recv reloc pose:" << pose << std::endl;
    PubRelocPose(pose);

  } break;

  default:
    break;
  }
}
void rclcomm::getRobotPose() {
  OnDataCallback(MsgId::kRobotPose, getTrasnsform("base_link", "map"));
}
/**
 * @description: 获取坐标变化
 * @param {string} from 要变换的坐标系
 * @param {string} to 基坐标系
 * @return {basic::RobotPose}from变换到to坐标系下，需要变换的坐标
 */
basic::RobotPose rclcomm::getTrasnsform(std::string from, std::string to) {
  try {
    geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(to, from, tf2::TimePointZero);
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
    basic::RobotPose ret;
    ret.x = x;
    ret.y = y;
    ret.theta = yaw;
    return ret;
  } catch (tf2::TransformException &ex) {

    // LOGGER_ERROR("getTrasnsform error from:" << from << " to:" << to
    //                                          << " error:" << ex.what());
  }
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
  OnDataCallback(MsgId::kOdomPose, state);
}
void rclcomm::local_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  try {
    //        geometry_msgs::msg::TransformStamped laser_transform =
    //        tf_buffer_->lookupTransform("map","base_scan",tf2::TimePointZero);
    geometry_msgs::msg::PointStamped point_map_frame;
    geometry_msgs::msg::PointStamped point_odom_frame;
    basic::RobotPath path;
    for (int i = 0; i < msg->poses.size(); i++) {
      double x = msg->poses.at(i).pose.position.x;
      double y = msg->poses.at(i).pose.position.y;
      point_odom_frame.point.x = x;
      point_odom_frame.point.y = y;
      point_odom_frame.header.frame_id = msg->header.frame_id;
      tf_buffer_->transform(point_odom_frame, point_map_frame, "map");
      basic::Point point;
      point.x = point_map_frame.point.x;
      point.y = point_map_frame.point.y;
      path.push_back(point);
      //        qDebug()<<"x:"<<x<<" y:"<<y<<" trans:"<<point.x()<<"
      //        "<<point.y();
    }
    OnDataCallback(MsgId::kLocalPath, path);
  } catch (tf2::TransformException &ex) {
  }
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
void rclcomm::Process() {
  while (rclcpp::ok()) {
    m_executor->spin_some();
    getRobotPose();
    // std::cout << "loop" << std::endl;
  }
}

void rclcomm::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  basic::RobotPath path;
  for (int i = 0; i < msg->poses.size(); i++) {
    double x = msg->poses.at(i).pose.position.x;
    double y = msg->poses.at(i).pose.position.y;
    basic::Point point;
    point.x = x;
    point.y = y;
    path.push_back(point);
    //        qDebug()<<"x:"<<x<<" y:"<<y<<" trans:"<<point.x()<<"
    //        "<<point.y();
  }
  OnDataCallback(MsgId::kGlobalPath, path);
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
    geometry_msgs::msg::PointStamped point_map_frame;
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
      tf_buffer_->transform(point_laser_frame, point_map_frame, "base_link");
      basic::Point p;
      p.x = point_map_frame.point.x;
      p.y = point_map_frame.point.y;
      laser_points.push_back(p);
    }
    laser_points.id = 0;
    // basic::RobotPose pose = getTrasnsform(msg->header.frame_id,
    // "base_link"); std::cout << "get transform" << pose.x << " " << pose.y
    // << " " << pose.theta
    //           << std::endl;
    OnDataCallback(MsgId::kLaserScan, laser_points);
  } catch (tf2::TransformException &ex) {
  }
}

void rclcomm::globalCostMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  int width = msg->info.width;
  int height = msg->info.height;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;
  basic::CostMap cost_map(height, width, Eigen::Vector3d(origin_x, origin_y, 0),
                          msg->info.resolution);
  for (int i = 0; i < msg->data.size(); i++) {
    int x = int(i / width);
    int y = i % width;
    cost_map(x, y) = msg->data[i];
  }
  cost_map.SetFlip();
  OnDataCallback(MsgId::kGlobalCostMap, cost_map);
}
void rclcomm::localCostMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
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
  basic::CostMap cost_map(height, width, Eigen::Vector3d(origin_x, origin_y, 0),
                          msg->info.resolution);
  for (int i = 0; i < msg->data.size(); i++) {
    int x = (int)i / width;
    int y = i % width;
    cost_map(x, y) = msg->data[i];
  }
  cost_map.SetFlip();
  //      map_image.save("/home/chengyangkj/test.jpg");
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

    basic::RobotPose localCostmapPose;
    localCostmapPose.x = pose_map_frame.pose.position.x;
    localCostmapPose.y = pose_map_frame.pose.position.y + cost_map.heightMap();
    localCostmapPose.theta = yaw;
    // emit emitUpdateLocalCostMap(cost_map, localCostmapPose);
  } catch (tf2::TransformException &ex) {
  }
}
void rclcomm::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;
  int width = msg->info.width;
  int height = msg->info.height;
  m_resolution = msg->info.resolution;
  occ_map_ = basic::OccupancyMap(
      height, width, Eigen::Vector3d(origin_x, origin_y, 0), m_resolution);

  for (int i = 0; i < msg->data.size(); i++) {
    int x = int(i / width);
    int y = i % width;
    occ_map_(x, y) = msg->data[i];
  }
  occ_map_.SetFlip();
  OnDataCallback(MsgId::kOccupancyMap, occ_map_);
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
  initPosePublisher_->publish(geo_pose);
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
  navGoalPublisher_->publish(geo_pose);
}
