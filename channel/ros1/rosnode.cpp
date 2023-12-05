/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-27 14:47:24
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-07-28 10:21:19
 * @FilePath: /ros_qt5_gui_app/src/channel/ros1/RosNode.cpp
 * @Description: ros1通讯类
 */
#include "rosnode.h"
#include "config/configManager.h"
RosNode::RosNode(/* args */) { std::cout << "ros node start" << std::endl; }

RosNode::~RosNode() {}
/// @brief loop for rate
void RosNode::Process() {
  if (ros::ok()) {
    // updateRobotPose();
    std::cout << "process" << std::endl;
  }
}
bool RosNode::Start() {
  int argc = 0;
  ros::init(argc, nullptr, "ros_qt5_gui_app", ros::init_options::AnonymousName);
  if (!ros::master::check()) {
    std::cout << "ros master not running" << std::endl;
    return false;
  }
  ros::start();
  init();
  return true;
}
void RosNode::init() {
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
  laser_scan_subscriber_ = nh.subscribe(GET_TOPIC_NAME("LaserScan"), 20,
                                        &RosNode::LaserScanCallback, this);
  global_path_subscriber_ = nh.subscribe(
      GET_TOPIC_NAME("GlobalPlan"), 20,&RosNode::GlobalPathCallback, this);
  local_path_subscriber_ = nh.subscribe(GET_TOPIC_NAME("LocalPlan"), 20,
                                        &RosNode::LocalPathCallback, this);
  odometry_subscriber_ = nh.subscribe(GET_TOPIC_NAME("Odometry"), 200,
                                      &RosNode::OdometryCallback, this);
}
bool RosNode::Stop() {
  ros::shutdown();
  return true;
}
void RosNode::SendMessage(const MsgId &msg_id, const std::any &msg) {}
void RosNode::OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {}
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
void RosNode::LocalCostMapCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {}
void RosNode::GlobalCostMapCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {}
// 激光雷达点云话题回调
void RosNode::LaserScanCallback(sensor_msgs::LaserScanConstPtr laser_msg) {
  geometry_msgs::PointStamped laser_point;
  geometry_msgs::PointStamped map_point;
  laser_point.header.frame_id = laser_msg->header.frame_id;
  std::vector<float> ranges = laser_msg->ranges;
  // // 转换到二维XY平面坐标系下;
  // for (int i = 0; i < ranges.size(); i++) {
  //   // scan_laser坐标系下
  //   double angle = laser_msg->angle_min + i * laser_msg->angle_increment;
  //   double X = ranges[i] * cos(angle);
  //   double Y = ranges[i] * sin(angle);
  //   laser_point.point.x = X;
  //   laser_point.point.y = Y;
  //   laser_point.point.z = 0.0;
  //   // change to map frame
  //   try {
  //     m_Laserlistener->transformPoint(map_frame, laser_point, map_point);
  //   } catch (tf::TransformException &ex) {
  //     try {
  //       m_robotPoselistener->waitForTransform(map_frame, base_frame,
  //                                             ros::Time(0),
  //                                             ros::Duration(0.4));
  //       m_Laserlistener->waitForTransform(map_frame, laser_frame,
  //       ros::Time(0),
  //                                         ros::Duration(0.4));
  //     } catch (tf::TransformException &ex) {
  //       log(Error, ("laser tf transform: " +
  //       QString(ex.what())).toStdString());
  //     }
  //   }
  //   // 转化为图元坐标系
  //   QPointF roboPos =
  //       transWordPoint2Scene(QPointF(map_point.point.x, map_point.point.y));

  // }
}
void RosNode::GlobalPathCallback(nav_msgs::Path::ConstPtr path){}
void RosNode::LocalPathCallback(nav_msgs::Path::ConstPtr path){}