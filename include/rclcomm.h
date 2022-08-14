#ifndef RCLCOMM_H
#define RCLCOMM_H

#include <QDebug>
#include <QImage>
#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>

#include "RobotAlgorithm.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
class rclcomm : public QThread {
  Q_OBJECT
 public:
  rclcomm();
  void run() override;
  QPointF transWordPoint2Scene(QPointF point);
  QPointF transScenePoint2Word(QPointF point);

 private:
  void recv_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void localCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void globalCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  QImage rotateMapWithY(QImage map);
  void getRobotPose();
 public slots:
  void pub2DPose(QPointF, QPointF);
  void pub2DGoal(QPointF, QPointF);

 private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      _initPosePublisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      _navPosePublisher;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _subscription;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      m_localCostMapSub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      m_globalCostMapSub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_path_sub;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_transform_listener;
  std::shared_ptr<rclcpp::Node> node;
  QPointF m_wordOrigin;
  double m_resolution;
  RobotPose m_currPose;
  rclcpp::executors::MultiThreadedExecutor *m_executor;
  rclcpp::CallbackGroup::SharedPtr callback_group_laser;
  rclcpp::CallbackGroup::SharedPtr callback_group_other;
 signals:
  void emitTopicData(QString);
  void emitUpdateMap(QImage img);
  void emitUpdateRobotPose(RobotPose pose);
  void emitUpdateLaserPoint(QPolygonF points);
  void emitUpdatePath(QPolygonF points);
  void emitUpdateLocalCostMap(QImage img);
  void emitUpdateGlobalCostMap(QImage img);
};

#endif  // RCLCOMM_H
