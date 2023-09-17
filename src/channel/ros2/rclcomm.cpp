/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-27 14:47:24
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-09-17 09:47:18
 * @FilePath: /ros_qt5_gui_app/src/channel/ros1/rosnode.cpp
 * @Description: ros2通讯类
 */
#include "channel/ros2/rclcomm.h"
#include <fstream>
rclcomm::rclcomm() {
  int argc = 0;
  char **argv = NULL;
  rclcpp::init(argc, argv);
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

  _navPosePublisher =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
  _initPosePublisher =
      node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", 10);
  _publisher =
      node->create_publisher<std_msgs::msg::Int32>("ros2_qt_dmeo_publish", 10);
  _subscription = node->create_subscription<std_msgs::msg::Int32>(
      "ros2_qt_dmeo_publish", 10,
      std::bind(&rclcomm::recv_callback, this, std::placeholders::_1),
      sub1_obt);
  m_map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&rclcomm::map_callback, this, std::placeholders::_1), sub1_obt);
  m_localCostMapSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&rclcomm::localCostMapCallback, this, std::placeholders::_1),
      sub1_obt);
  m_globalCostMapSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&rclcomm::globalCostMapCallback, this, std::placeholders::_1),
      sub1_obt);
  m_laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 20,
      std::bind(&rclcomm::laser_callback, this, std::placeholders::_1),
      sub_laser_obt);
  m_path_sub = node->create_subscription<nav_msgs::msg::Path>(
      "/plan", 20,
      std::bind(&rclcomm::path_callback, this, std::placeholders::_1),
      sub1_obt);
  _local_path_sub = node->create_subscription<nav_msgs::msg::Path>(
      "/local_plan", 20,
      std::bind(&rclcomm::local_path_callback, this, std::placeholders::_1),
      sub1_obt);
  m_odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20,
      std::bind(&rclcomm::odom_callback, this, std::placeholders::_1),
      sub1_obt);
  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  m_transform_listener =
      std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}
void rclcomm::getRobotPose() {
  try {
    geometry_msgs::msg::TransformStamped transform =
        m_tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
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
    m_currPose.x = x;
    m_currPose.y = y;
    m_currPose.theta = yaw;
    emit emitUpdateRobotPose(m_currPose);
  } catch (tf2::TransformException &ex) {
    qDebug() << "robot pose transform error:" << ex.what();
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
  emit emitOdomInfo(state);
}
void rclcomm::local_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
  try {
    //        geometry_msgs::msg::TransformStamped laser_transform =
    //        m_tf_buffer->lookupTransform("map","base_scan",tf2::TimePointZero);
    geometry_msgs::msg::PointStamped point_map_frame;
    geometry_msgs::msg::PointStamped point_odom_frame;
    basic::RobotPath path;
    for (int i = 0; i < msg->poses.size(); i++) {
      double x = msg->poses.at(i).pose.position.x;
      double y = msg->poses.at(i).pose.position.y;
      point_odom_frame.point.x = x;
      point_odom_frame.point.y = y;
      point_odom_frame.header.frame_id = msg->header.frame_id;
      m_tf_buffer->transform(point_odom_frame, point_map_frame, "map");
      basic::Point point;
      point.x = point_map_frame.point.x;
      point.y = point_map_frame.point.y;
      path.push_back(point);
      //        qDebug()<<"x:"<<x<<" y:"<<y<<" trans:"<<point.x()<<"
      //        "<<point.y();
    }
    emit emitUpdateLocalPath(path);
  } catch (tf2::TransformException &ex) {
    qDebug() << "local path transform error:" << ex.what();
  }
}
void rclcomm::run() {
  std_msgs::msg::Int32 pub_msg;
  pub_msg.data = 0;
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    pub_msg.data++;
    m_executor->spin_some();
    getRobotPose();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
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
    //        qDebug()<<"x:"<<x<<" y:"<<y<<" trans:"<<point.x()<<" "<<point.y();
  }
  emit emitUpdatePath(path);
}
void rclcomm::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // qDebug()<<"订阅到激光话题";
  // std::cout<<"recv laser"<<std::endl;
  double angle_min = msg->angle_min;
  double angle_max = msg->angle_max;
  double angle_increment = msg->angle_increment;
  try {
    //        geometry_msgs::msg::TransformStamped laser_transform =
    //        m_tf_buffer->lookupTransform("map","base_scan",tf2::TimePointZero);
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
      m_tf_buffer->transform(point_laser_frame, point_map_frame, "map");
      basic::Point p;
      p.x = point_map_frame.point.x;
      p.y = point_map_frame.point.y;
      laser_points.push_back(p);
    }
    laser_points.id = 0;
    emit emitUpdateLaserPoint(laser_points);
  } catch (tf2::TransformException &ex) {
    qDebug() << "laser pose transform error:" << ex.what();
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
  emit emitUpdateGlobalCostMap(cost_map);
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
    m_tf_buffer->transform(pose_curr_frame, pose_map_frame, "map");
    tf2::fromMsg(pose_map_frame.pose.orientation, q);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    basic::RobotPose localCostmapPose;
    localCostmapPose.x = pose_map_frame.pose.position.x;
    localCostmapPose.y = pose_map_frame.pose.position.y;
    localCostmapPose.theta = yaw;
    //      std::cout<<"odomInMapPose:"<<pose_map_frame.pose.position.x<<"
    //      "<<pose_map_frame.pose.position.y<<" curr:"<<m_currPose.x<<"
    //      "<<m_currPose.y<<std::endl; std::cout<<"origin x:"<<origin_x<<"
    //      y:"<<origin_y<<" theta:"<<origin_theta<<std::endl;
    emit emitUpdateLocalCostMap(
        cost_map, localCostmapPose);
  } catch (tf2::TransformException &ex) {
    qDebug() << "local cost map pose transform error:" << ex.what();
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
  emit emitUpdateMap(occ_map_);
}
void rclcomm::recv_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  //     qDebug()<<msg->data;
  emit emitTopicData("i am listen from topic:" +
                     QString::fromStdString(std::to_string(msg->data)));
}
void rclcomm::pub2DPose(QPointF start, QPointF end) {
  auto start_pose = transScenePoint2Word(basic::Point(start.x(), start.y()));
  auto end_pose = transScenePoint2Word(basic::Point(end.x(), end.y()));
  double angle = atan2(end_pose.y - start_pose.y, end_pose.x - start_pose.x);
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node->get_clock()->now();
  pose.pose.pose.position.x = start_pose.x;
  pose.pose.pose.position.y = start_pose.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  pose.pose.pose.orientation = tf2::toMsg(q);
  _initPosePublisher->publish(pose);
}
void rclcomm::pub2DGoal(QPointF start, QPointF end) {
  auto start_pose = transScenePoint2Word(basic::Point(start.x(), start.y()));
  auto end_pose = transScenePoint2Word(basic::Point(end.x(), end.y()));
  double angle = atan2(end_pose.y - start_pose.y, end_pose.x - start_pose.x);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node->get_clock()->now();
  pose.pose.position.x = start_pose.x;
  pose.pose.position.y = start_pose.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  pose.pose.orientation = tf2::toMsg(q);
  _navPosePublisher->publish(pose);
}
