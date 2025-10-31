/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-29 14:21:31
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 09:46:05
 * @FilePath:
 * ////include/display/virtual_display.h
 * @Description: 图层的虚拟类
 */
#pragma once
#ifndef DISPLAY_VIRTUAL_DISPLAY_H_
#define DISPLAY_VIRTUAL_DISPLAY_H_
#include <math.h>
#include <Eigen/Dense>
#include <QCursor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QObject>
#include <QPainter>
#include <QVector2D>
#include <QVector3D>
#include <QtCore>
#include <any>
#include <algorithm>
#include <iostream>
#include <mutex>
#include "msg/msg_info.h"

#include "display_defines.h"
#include "occupancy_map.h"
#include "point_type.h"
using namespace basic;
#define GetAnyData(type, data, res_data)    \
  {                                         \
    try {                                   \
      res_data = std::any_cast<type>(data); \
    } catch (const std::bad_any_cast &e) {  \
      LOG_ERROR("GetAnyData error: " << e.what());        \
    }                                       \
  }

namespace Display {
#define DISPLAY_ROBOT ToString(MsgId::kRobotPose)
#define DISPLAY_MAP ToString(MsgId::kOccupancyMap)
#define DISPLAY_LOCAL_COST_MAP ToString(MsgId::kLocalCostMap)
#define DISPLAY_GLOBAL_COST_MAP ToString(MsgId::kGlobalCostMap)
#define DISPLAY_GLOBAL_PATH ToString(MsgId::kGlobalPath)
#define DISPLAY_LOCAL_PATH ToString(MsgId::kLocalPath)
#define DISPLAY_LASER ToString(MsgId::kLaserScan)
#define DISPLAY_PARTICLE "Particle"
#define DISPLAY_REGION "Region"
#define DISPLAY_TAG "Tag"
#define DISPLAY_GOAL "GoalPose"
#define DISPLAY_TOPOLINE "TopologyLine"
#define DISPLAY_ROBOT_FOOTPRINT ToString(MsgId::kRobotFootprint)
#define DISPLAY_TOPOLOGY_MAP ToString(MsgId::kTopologyMap)



class FactoryDisplay;
class VirtualDisplay : public QObject, public QGraphicsItem {
  Q_OBJECT
 
 friend class FactoryDisplay;
 public:
  std::string display_type_{"null"};
  QPointF pressed_pose_;
  QPointF start_pose_;
  QPointF end_pose_;
  bool is_mouse_press_;
  Qt::MouseButton pressed_button_{Qt::MouseButton::NoButton};
  double scale_value_ = 1;
  bool move_enable_{false};
  bool is_rotate_event_{false};
  bool enable_scale_{true};
  QRectF bounding_rect_;
  QTransform transform_;
  double rotate_value_{0};
  OccupancyMap map_data_;
  QPointF rotate_center_;
  bool enable_rotate_{false};
  std::string parent_name_;
  RobotPose pose_in_parent_{0, 0, 0};
  RobotPose curr_scene_pose_{0, 0, 0};
  VirtualDisplay *parent_ptr_;
  std::vector<VirtualDisplay *> children_;
  bool is_moving_{false};
  std::string display_name_;
  std::mutex display_name_mutex_;
  
  // 状态相关的互斥锁
  std::mutex state_mutex_;  // 保护 scale_value_, rotate_value_, move_enable_, is_moving_
  std::mutex pose_mutex_;   // 保护 curr_scene_pose_, pose_in_parent_
  std::mutex map_data_mutex_;  // 保护 map_data_
  std::mutex geometry_mutex_;  // 保护 bounding_rect_
  std::mutex metadata_mutex_;  // 保护 display_type_, parent_name_, children_

  double min_scale_value_{0.1};
  double max_scale_value_{20};
 signals:
  void signalCursorPose(QPointF pose);
  void signalPoseUpdate(const RobotPose &pose);
  void signalItemChange(GraphicsItemChange change, const QVariant &value);
  void signalPositionChanged(); // 新增：位置变化信号

 public:
  VirtualDisplay(const std::string &display_type, const int &z_value,
                 const std::string &parent_name, std::string display_name = "");
  virtual ~VirtualDisplay();
  bool UpdateDisplay(const std::any &data) { return UpdateData(data); }
  VirtualDisplay *SetRotateEnable(const bool &enable) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    enable_rotate_ = enable;
    return this;
  }
  double GetScaleValue() { 
    std::lock_guard<std::mutex> lock(state_mutex_);
    return scale_value_; 
  }
  VirtualDisplay *SetScaleEnable(const bool &enable) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    enable_scale_ = enable;
    return this;
  }
  VirtualDisplay *SetMoveEnable(const bool &enable) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      move_enable_ = enable;
    }
    if (enable) {
      // 只有响应的图层才响应鼠标事件
      setAcceptHoverEvents(true);
      setAcceptedMouseButtons(Qt::AllButtons);
      setAcceptDrops(true);
      setFlag(ItemAcceptsInputMethod, true);
      setFlag(ItemSendsGeometryChanges, true);
    }
    update();
    return this;
  }
  bool GetMoveEnable() { 
    std::lock_guard<std::mutex> lock(state_mutex_);
    return move_enable_; 
  }

  void AddChild(VirtualDisplay *child) { 
    std::lock_guard<std::mutex> lock(metadata_mutex_);
    children_.push_back(child); 
  }

  void RemoveChild(VirtualDisplay *child) {
    std::lock_guard<std::mutex> lock(metadata_mutex_);
    children_.erase(std::remove(children_.begin(), children_.end(), child),
                    children_.end());
  }

  std::vector<VirtualDisplay *> GetChildren() { 
    std::lock_guard<std::mutex> lock(metadata_mutex_);
    return children_; 
  }

  void UpdateMap(const OccupancyMap &map) { 
    std::lock_guard<std::mutex> lock(map_data_mutex_);
    map_data_ = map;
  }

  double GetRotate() { 
    std::lock_guard<std::mutex> lock(state_mutex_);
    return rotate_value_; 
  }
  
  virtual bool UpdateData(const std::any &data) = 0;
  virtual bool SetDisplayConfig(const std::string &config_name,
                                const std::any &config_data);
  bool SetScaled(const double &value);
  bool SetRotate(const double &value);
  void SetBoundingRect(QRectF rect) { 
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    bounding_rect_ = rect; 
  }
  QPointF GetOriginPose() { 
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    return bounding_rect_.topLeft(); 
  }
  QPointF GetOriginPoseScene() { return mapToScene(GetOriginPose()); }
  QPointF PoseToScene(QPointF pose) {  //将坐标转换为scene(以中心为原点)
    return mapToScene((pose + GetOriginPose()));
  }
  void CenterOnScene(QPointF pose);
  bool IsMoving() { 
    std::lock_guard<std::mutex> lock(state_mutex_);
    return is_moving_; 
  }
  void UpdatePose(const RobotPose &pose) { SetPoseInParent(pose); }
  void SetPoseInParent(const RobotPose &pose);
  RobotPose GetCurrentScenePose() { 
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return curr_scene_pose_; 
  }
  RobotPose GetPoseInParent() { 
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return pose_in_parent_; 
  }
  std::string GetParentName() { 
    std::lock_guard<std::mutex> lock(metadata_mutex_);
    return parent_name_; 
  }
  std::string GetDisplayName() { 
    std::lock_guard<std::mutex> lock(display_name_mutex_);
    return display_name_; }
  //设置原点在全局的坐标
  void SetOriginPoseInScene(const QPointF &pose) {
    setPos(pose - GetOriginPose());
  }
  void MovedBy(const qreal &x, const qreal &y);
  QRectF boundingRect() const override { return bounding_rect_; }
  std::string GetDisplayType();
  void SetDisplayType(const std::string &display_type);
  void Update();

 private:
  void SetDisplayName(const std::string &name) { 
    std::lock_guard<std::mutex> lock(display_name_mutex_);
    display_name_ = name; 
  }
  void wheelEvent(QGraphicsSceneWheelEvent *event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
  void hoverMoveEvent(QGraphicsSceneHoverEvent *event) override;
  virtual QVariant itemChange(GraphicsItemChange change,
                              const QVariant &value) override;
 private slots:
  void parentItemChange(GraphicsItemChange change, const QVariant &value);
};
}  // namespace Display
#endif  // DISPLAY_VIRTUAL_DISPLAY_H_