/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-20 15:46:29
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-07 14:16:09
 * @FilePath: /ros_qt5_gui_app/include/rclcomm.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef RCLCOMM_H
#define RCLCOMM_H
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include "algorithm.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "point_type.h"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "virtual_channel_node.h"
class rclcomm : public VirtualChannelNode {
 public:
  rclcomm();
  ~rclcomm() override = default;

 private:
  void recv_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void localCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void globalCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void BatteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void getRobotPose();
  void local_path_callback(const nav_msgs::msg::Path::SharedPtr msg);

 public:
  bool Start() override;
  bool Stop() override;
  void Process() override;
  std::string Name() override { return "ROS2"; };
  void PubRelocPose(const RobotPose &pose);
  void PubNavGoal(const RobotPose &pose);
  void PubRobotSpeed(const RobotSpeed &speed);
  basic::RobotPose getTrasnsform(std::string from, std::string to);
  void SendMessage(const MsgId &msg_id, const std::any &msg) override;

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      reloc_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      nav_goal_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      local_cost_map_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      global_cost_map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr
      battery_state_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscriber_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscriber_list_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::shared_ptr<rclcpp::Node> node;
  basic::OccupancyMap occ_map_;
  basic::RobotPose m_currPose;
  rclcpp::executors::MultiThreadedExecutor *m_executor;
  rclcpp::CallbackGroup::SharedPtr callback_group_laser;
  rclcpp::CallbackGroup::SharedPtr callback_group_other;
  std::atomic_bool init_flag_{false};

 private:
};

#endif  // RCLCOMM_H
