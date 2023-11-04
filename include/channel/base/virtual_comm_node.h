/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-25 16:20:39
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-07 14:41:45
 * @FilePath:
 * /ros_qt5_gui_app/include/channel/base/virtual_communcation_node.h
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
  virtual ~VirtualCommNode() {}
  basic::Point transWordPoint2Scene(basic::Point point) {
    basic::Point ret;
    double x, y;
    occ_map_.xy2occPose(point.x, point.y, x, y);
    ret.x = x;
    ret.y = y;
    return ret;
  }
  basic::Point transScenePoint2Word(basic::Point point) {
    basic::Point ret;
    occ_map_.occPose2xy(point.x, point.y, ret.x, ret.y);

    return ret;
  }
signals:
  void emitUpdateMap(OccupancyMap map);
  void emitUpdateLocalCostMap(CostMap img, RobotPose pose);
  void emitUpdateGlobalCostMap(CostMap img);
  void emitUpdateRobotPose(RobotPose pose);
  void emitUpdateLaserPoint(LaserScan laser_scan); //发布车身坐标系下的激光点
  void emitUpdatePath(RobotPath points);
  void emitUpdateLocalPath(RobotPath points);
  void emitOdomInfo(RobotState state);

public slots:
  virtual void pub2DPose(Eigen::Vector3f pose) = 0;
  virtual void pub2DGoal(Eigen::Vector3f pose) = 0;
  virtual void pubSpeed(Eigen::Vector3f speed) = 0;
};
