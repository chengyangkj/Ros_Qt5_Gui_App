/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-07-25 16:20:39
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-07-27 14:07:54
 * @FilePath:
 * /ROS2_Qt5_Gui_App/include/communication/base/virtual_communcation_node.h
 */
#pragma once
#include <QObject>
#include <QThread>

#include "QPointF"
#include "basic/map/cost_map.h"
#include "basic/map/occupancy_map.h"
#include "basic/point_type.h"
using namespace basic;
class VirtualCommNode : public QThread {
  Q_OBJECT
 public:
  basic::OccupancyMap occ_map_;

 public:
  VirtualCommNode(/* args */) {}
  ~VirtualCommNode() {}
  basic::Point transWordPoint2Scene(basic::Point point) {
    basic::Point ret;
    int x, y;
    occ_map_.xy2scene(point.x, point.y, x, y);
    ret.x = x;
    ret.y = y;
    return ret;
  }
  basic::Point transScenePoint2Word(basic::Point point) {
    basic::Point ret;
    occ_map_.scene2xy(point.x, point.y, ret.x, ret.y);

    return ret;
  }
 signals:
  void emitUpdateMap(OccupancyMap map);
  void emitUpdateLocalCostMap(CostMap img, basic::RobotPose pose);
  void emitUpdateGlobalCostMap(CostMap img);
  void emitUpdateRobotPose(basic::RobotPose pose);
  void emitUpdateLaserPoint(LaserScan laser_scan);
  void emitUpdatePath(RobotPath points);
  void emitUpdateLocalPath(RobotPath points);
  void emitOdomInfo(basic::RobotState state);
 public slots:
  void slotPub2DPose(QPointF start, QPointF end) { pub2DPose(start, end); }
  void slotPub2DGoal(QPointF start, QPointF end) { pub2DGoal(start, end); }

 public:
  virtual void pub2DPose(QPointF start, QPointF end) = 0;
  virtual void pub2DGoal(QPointF start, QPointF end) = 0;
};
