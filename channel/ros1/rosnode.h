#pragma once
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>  //图像编码格式

#include <tf/transform_listener.h>
#include "virtual_channel_node.h"
class RosNode : public VirtualChannelNode {
 private:
  /* data */
 public:
  RosNode(/* args */);
  ~RosNode() override;

  void Process() override;
  bool Start() override;
  bool Stop() override;
  std::string Name() { return "ROS1"; };
  void SendMessage(const MsgId &msg_id, const std::any &msg) override;

 private:
  void init();
  void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void MapCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
  void LocalCostMapCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
  void GlobalCostMapCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
  void LaserScanCallback(sensor_msgs::LaserScan::ConstPtr laser_msg);
  void GlobalPathCallback(nav_msgs::Path::ConstPtr path);
  void LocalPathCallback(nav_msgs::Path::ConstPtr path);
  void BatteryCallback(sensor_msgs::BatteryState::ConstPtr path);
  // void MbStatusCallback(actionlib_msgs::GoalStatusArray::ConstPtr msg);
  void PubRobotSpeed(const RobotSpeed &speed);
  void PubNavGoal(const RobotPose &pose);
  void PubRelocPose(const RobotPose &pose);
  void GetRobotPose();
  basic::RobotPose GetTrasnsform(std::string from, std::string to);

 private:
  ros::Publisher nav_goal_publisher_;
  ros::Publisher speed_publisher_;
  ros::Publisher reloc_pose_publisher_;

  ros::Subscriber odometry_subscriber_;
  ros::Subscriber map_subscriber_;
  ros::Subscriber local_cost_map_subscriber_;
  ros::Subscriber global_cost_map_subscriber_;
  ros::Subscriber laser_scan_subscriber_;
  ros::Subscriber global_path_subscriber_;
  ros::Subscriber local_path_subscriber_;
  ros::Subscriber battery_subscriber_;
  ros::Subscriber move_base_status_subscriber_;
  std::vector<ros::Subscriber> image_subscriber_list_;
  basic::OccupancyMap occ_map_;

  tf::TransformListener *tf_listener_;
};