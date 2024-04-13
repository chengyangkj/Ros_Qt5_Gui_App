/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-27 14:47:24
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-07-28 10:21:19
 * @FilePath: /ros_qt5_gui_app/src/channel/ros1/RosNode.cpp
 * @Description: ros1通讯类
 */
#include "rosnode.h"
#include <opencv2/opencv.hpp>
#include "config/config_manager.h"
RosNode::RosNode(/* args */) {
  SET_DEFAULT_TOPIC_NAME("NavGoal", "/move_base_simple/goal")
  SET_DEFAULT_TOPIC_NAME("Reloc", "/initialpose")
  SET_DEFAULT_TOPIC_NAME("Map", "/map")
  SET_DEFAULT_TOPIC_NAME("LocalCostMap", "/move_base/local_costmap/costmap")
  SET_DEFAULT_TOPIC_NAME("GlobalCostMap", "/move_base/global_costmap/costmap")
  SET_DEFAULT_TOPIC_NAME("LaserScan", "/scan")
  SET_DEFAULT_TOPIC_NAME("GlobalPlan", "/move_base/DWAPlannerROS/global_plan")
  SET_DEFAULT_TOPIC_NAME("LocalPlan", "/move_base/DWAPlannerROS/local_plan")
  SET_DEFAULT_TOPIC_NAME("Odometry", "/odom")
  SET_DEFAULT_TOPIC_NAME("Speed", "/cmd_vel")
  SET_DEFAULT_TOPIC_NAME("Battery", "/battery")
  SET_DEFAULT_TOPIC_NAME("MoveBaseStatus", "/move_base/status")
  if (Config::ConfigManager::Instacnce()->GetRootConfig().images.empty()) {
    Config::ConfigManager::Instacnce()->GetRootConfig().images.push_back(
        Config::ImageDisplayConfig{.location = "front",
                                   .topic = "/camera/rgb/image_raw",
                                   .enable = true});

  }
  Config::ConfigManager::Instacnce()->StoreConfig();
  std::cout << "ros node start" << std::endl;
}
basic::RobotPose Convert(const geometry_msgs::Pose &pose) {
  RobotPose robot_pose;

  // 提取位置信息
  robot_pose.x = pose.position.x;
  robot_pose.y = pose.position.y;

  // 提取姿态信息
  tf::Quaternion quat(pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w);
  double r, p;
  tf::Matrix3x3(quat).getRPY(r, p, robot_pose.theta);

  return robot_pose;
}

RosNode::~RosNode() {}
/// @brief loop for rate
void RosNode::Process() {
  if (ros::ok()) {
    GetRobotPose();
    ros::spinOnce();
  }
}
bool RosNode::Start() {
  int argc = 0;
  ros::init(argc, nullptr, "ros_qt5_gui_app", ros::init_options::AnonymousName);
  while (!ros::master::check()) {
    LOG_ERROR("wait ros master........");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  ros::start();
  init();
  return true;
}
void RosNode::init() {
  // 设置默认的topic名称
  ros::NodeHandle nh;
  nav_goal_publisher_ =
      nh.advertise<geometry_msgs::PoseStamped>(GET_TOPIC_NAME("NavGoal"), 10);
  reloc_pose_publisher_ =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          GET_TOPIC_NAME("Reloc"), 10);
  speed_publisher_ =
      nh.advertise<geometry_msgs::Twist>(GET_TOPIC_NAME("Speed"), 10);

  map_subscriber_ =
      nh.subscribe(GET_TOPIC_NAME("Map"), 1, &RosNode::MapCallback, this);
  local_cost_map_subscriber_ = nh.subscribe(
      GET_TOPIC_NAME("LocalCostMap"), 1, &RosNode::LocalCostMapCallback, this);
  global_cost_map_subscriber_ =
      nh.subscribe(GET_TOPIC_NAME("GlobalCostMap"), 1,
                   &RosNode::GlobalCostMapCallback, this);
  laser_scan_subscriber_ = nh.subscribe(GET_TOPIC_NAME("LaserScan"), 1,
                                        &RosNode::LaserScanCallback, this);
  global_path_subscriber_ = nh.subscribe(GET_TOPIC_NAME("GlobalPlan"), 1,
                                         &RosNode::GlobalPathCallback, this);
  local_path_subscriber_ = nh.subscribe(GET_TOPIC_NAME("LocalPlan"), 1,
                                        &RosNode::LocalPathCallback, this);
  odometry_subscriber_ = nh.subscribe(GET_TOPIC_NAME("Odometry"), 1,
                                      &RosNode::OdometryCallback, this);
  battery_subscriber_ = nh.subscribe(GET_TOPIC_NAME("Battery"), 1,
                                     &RosNode::BatteryCallback, this);

  for (auto one_image_display : Config::ConfigManager::Instacnce()->GetRootConfig().images) {
    LOG_INFO("image location:" << one_image_display.location << " topic:" << one_image_display.topic);
    image_subscriber_list_.emplace_back(nh.subscribe(
        one_image_display.topic, 1,
        boost::function<void(const sensor_msgs::ImageConstPtr &)>(
            [this, one_image_display](const sensor_msgs::ImageConstPtr &msg) {
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

              OnDataCallback(MsgId::kImage, std::pair<std::string, std::shared_ptr<cv::Mat>>(one_image_display.location, std::make_shared<cv::Mat>(conversion_mat_)));
            })));
  }

  // move_base_status_subscriber_ = nh.subscribe(GET_TOPIC_NAME("MoveBaseStatus"), 1,
  //                                             &RosNode::BatteryCallback, this);
  tf_listener_ = new tf::TransformListener();
}
bool RosNode::Stop() {
  ros::shutdown();
  return true;
}
void RosNode::SendMessage(const MsgId &msg_id, const std::any &msg) {
  switch (msg_id) {
    case MsgId::kSetNavGoalPose: {
      auto pose = std::any_cast<basic::RobotPose>(msg);
      std::cout << "recv nav goal pose:" << pose << std::endl;

      PubNavGoal(pose);

    } break;
    case MsgId::kSetRelocPose: {
      auto pose = std::any_cast<basic::RobotPose>(msg);
      std::cout << "recv reloc pose:" << pose << std::endl;
      PubRelocPose(pose);

    } break;
    case MsgId::kSetRobotSpeed: {
      auto speed = std::any_cast<basic::RobotSpeed>(msg);
      std::cout << "recv speed pose:" << speed << std::endl;
      PubRobotSpeed(speed);

    } break;
    default:
      break;
  }
}

void RosNode::BatteryCallback(sensor_msgs::BatteryState::ConstPtr battery) {
  std::map<std::string, std::string> map;
  map["percent"] = std::to_string(battery->percentage);
  map["voltage"] = std::to_string(battery->voltage);
  OnDataCallback(MsgId::kBatteryState, map);
}
// void RosNode::MbStatusCallback(actionlib_msgs::GoalStatusArray::ConstPtr msg) {

// }
void RosNode::OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  basic::RobotState state =
      static_cast<basic::RobotState>(Convert(msg->pose.pose));
  state.vx = (double)msg->twist.twist.linear.x;
  state.vy = (double)msg->twist.twist.linear.y;
  state.w = (double)msg->twist.twist.angular.z;
  OnDataCallback(MsgId::kOdomPose, state);
}
void RosNode::MapCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;
  int width = msg->info.width;
  int height = msg->info.height;
  double resolution = msg->info.resolution;
  occ_map_ = basic::OccupancyMap(
      height, width, Eigen::Vector3d(origin_x, origin_y, 0), resolution);

  for (int i = 0; i < msg->data.size(); i++) {
    int x = int(i / width);
    int y = i % width;
    occ_map_(x, y) = msg->data[i];
  }
  occ_map_.SetFlip();
  OnDataCallback(MsgId::kOccupancyMap, occ_map_);
}
void RosNode::LocalCostMapCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
  if (occ_map_.cols == 0 || occ_map_.rows == 0)
    return;
  int width = msg->info.width;
  int height = msg->info.height;
  double origin_x = msg->info.origin.position.x;
  double origin_y = msg->info.origin.position.y;
  double origin_theta = 0;
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
    geometry_msgs::PointStamped pose_map_frame;
    geometry_msgs::PointStamped pose_curr_frame;
    pose_curr_frame.point.x = origin_x;
    pose_curr_frame.point.y = origin_y;
    pose_curr_frame.header.frame_id = msg->header.frame_id;
    tf_listener_->transformPoint("map", pose_curr_frame, pose_map_frame);

    origin_pose.x = pose_map_frame.point.x;
    origin_pose.y = pose_map_frame.point.y + cost_map.heightMap();
    origin_pose.theta = 0;
  } catch (tf2::TransformException &ex) {
    // LOG_ERROR("getTrasnsform localCostMapCallback error:" << ex.what());
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
  OnDataCallback(MsgId::kLocalCostMap, sized_cost_map);
}
void RosNode::GlobalCostMapCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
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
  OnDataCallback(MsgId::kGlobalCostMap, cost_map);
}
// 激光雷达点云话题回调
void RosNode::LaserScanCallback(sensor_msgs::LaserScanConstPtr msg) {
  double angle_min = msg->angle_min;
  double angle_max = msg->angle_max;
  double angle_increment = msg->angle_increment;
  try {
    geometry_msgs::PointStamped point_base_frame;
    geometry_msgs::PointStamped point_laser_frame;
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

      tf_listener_->transformPoint("base_link", point_laser_frame,
                                   point_base_frame);
      basic::Point p;
      p.x = point_base_frame.point.x;
      p.y = point_base_frame.point.y;
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
void RosNode::GlobalPathCallback(nav_msgs::Path::ConstPtr msg) {
  try {
    //        geometry_msgs::msg::TransformStamped laser_transform =
    //        tf_buffer_->lookupTransform("map","base_scan",tf2::TimePointZero);
    geometry_msgs::PointStamped point_map_frame;
    geometry_msgs::PointStamped point_odom_frame;
    basic::RobotPath path;
    for (int i = 0; i < msg->poses.size(); i++) {
      double x = msg->poses.at(i).pose.position.x;
      double y = msg->poses.at(i).pose.position.y;
      point_odom_frame.point.x = x;
      point_odom_frame.point.y = y;
      point_odom_frame.header.frame_id = msg->header.frame_id;
      tf_listener_->transformPoint("map", point_odom_frame, point_map_frame);
      basic::Point point;
      point.x = point_map_frame.point.x;
      point.y = point_map_frame.point.y;
      path.push_back(point);
    }
    OnDataCallback(MsgId::kGlobalPath, path);
  } catch (tf2::TransformException &ex) {
  }
}
void RosNode::LocalPathCallback(nav_msgs::Path::ConstPtr msg) {
  try {
    //        geometry_msgs::msg::TransformStamped laser_transform =
    //        tf_buffer_->lookupTransform("map","base_scan",tf2::TimePointZero);
    geometry_msgs::PointStamped point_map_frame;
    geometry_msgs::PointStamped point_odom_frame;
    basic::RobotPath path;
    for (int i = 0; i < msg->poses.size(); i++) {
      double x = msg->poses.at(i).pose.position.x;
      double y = msg->poses.at(i).pose.position.y;
      point_odom_frame.point.x = x;
      point_odom_frame.point.y = y;
      point_odom_frame.header.frame_id = msg->header.frame_id;
      tf_listener_->transformPoint("map", point_odom_frame, point_map_frame);
      basic::Point point;
      point.x = point_map_frame.point.x;
      point.y = point_map_frame.point.y;
      path.push_back(point);
    }
    OnDataCallback(MsgId::kLocalPath, path);
  } catch (tf2::TransformException &ex) {
  }
}
void RosNode::PubRelocPose(const RobotPose &pose) {
  geometry_msgs::PoseWithCovarianceStamped geo_pose;
  geo_pose.header.frame_id = "map";
  geo_pose.header.stamp = ros::Time(0);
  geo_pose.pose.pose.position.x = pose.x;
  geo_pose.pose.pose.position.y = pose.y;
  geometry_msgs::Quaternion q;  // 初始化四元数（geometry_msgs类型）
  q = tf::createQuaternionMsgFromRollPitchYaw(
      0, 0, pose.theta);  // 欧拉角转四元数（geometry_msgs::Quaternion）
  geo_pose.pose.pose.orientation = q;
  reloc_pose_publisher_.publish(geo_pose);
}
void RosNode::PubNavGoal(const RobotPose &pose) {
  geometry_msgs::PoseStamped geo_pose;
  geo_pose.header.frame_id = "map";
  geo_pose.header.stamp = ros::Time(0);
  geo_pose.pose.position.x = pose.x;
  geo_pose.pose.position.y = pose.y;
  geometry_msgs::Quaternion q;  // 初始化四元数（geometry_msgs类型）
  q = tf::createQuaternionMsgFromRollPitchYaw(
      0, 0, pose.theta);  // 欧拉角转四元数（geometry_msgs::Quaternion）
  geo_pose.pose.orientation = q;
  nav_goal_publisher_.publish(geo_pose);
}
void RosNode::PubRobotSpeed(const RobotSpeed &speed) {
  geometry_msgs::Twist twist;
  twist.linear.x = speed.vx;
  twist.linear.y = speed.vy;
  twist.linear.z = 0;

  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = speed.w;

  // Publish it and resolve any remaining callbacks
  speed_publisher_.publish(twist);
}
void RosNode::GetRobotPose() {
  OnDataCallback(MsgId::kRobotPose, GetTrasnsform("base_link", "map"));
}
/**
 * @description: 获取坐标变化
 * @param {string} from 要变换的坐标系
 * @param {string} to 基坐标系
 * @return {basic::RobotPose}from变换到to坐标系下，需要变换的坐标
 */
basic::RobotPose RosNode::GetTrasnsform(std::string from, std::string to) {
  basic::RobotPose ret;
  try {
    tf::StampedTransform transform;
    tf_listener_->lookupTransform(to, from, ros::Time(0), transform);
    tf::Quaternion quat = transform.getRotation();

    // 将四元数转换为欧拉角
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // x y
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    ret.x = x;
    ret.y = y;
    ret.theta = yaw;

  } catch (tf2::TransformException &ex) {
    LOG_ERROR("getTrasnsform error from:" << from << " to:" << to
                                          << " error:" << ex.what());
  }
  return ret;
}