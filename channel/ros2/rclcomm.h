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

#include "algorithm.h"
#include "base/virtual_comm_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "point_type.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>
class rclcomm : public VirtualCommNode {
  Q_OBJECT
public:
  rclcomm();
  ~rclcomm() override = default;
  void run() override;

private:
  void recv_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void localCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void globalCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void getRobotPose();
  void local_path_callback(const nav_msgs::msg::Path::SharedPtr msg);

public slots:
  void pub2DPose(const RobotPose &pose) override;
  void pub2DGoal(const RobotPose &pose) override;
  void pubSpeed(const RobotSpeed &speed) override;
  basic::RobotPose getTrasnsform(std::string from, std::string to);

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _publisher;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initPosePublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      navGoalPublisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _subscription;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      m_localCostMapSub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      m_globalCostMapSub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _local_path_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::shared_ptr<rclcpp::Node> node;

  double m_resolution;
  basic::RobotPose m_currPose;
  rclcpp::executors::MultiThreadedExecutor *m_executor;
  rclcpp::CallbackGroup::SharedPtr callback_group_laser;
  rclcpp::CallbackGroup::SharedPtr callback_group_other;

private:
signals:
  void emitTopicData(QString);
};

#endif // RCLCOMM_H
