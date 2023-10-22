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
#include <iostream>

#include "basic/map/occupancy_map.h"
#include "basic/point_type.h"
#include "display_defines.h"
using namespace basic;
#define GetAnyData(type, data, res_data)                                       \
  {                                                                            \
    try {                                                                      \
      res_data = std::any_cast<type>(data);                                    \
    } catch (const std::bad_any_cast &e) {                                     \
      std::cout << e.what() << '\n';                                           \
    }                                                                          \
  }
namespace Display {

class DisplayGroup : public QObject, public QGraphicsItemGroup {
  Q_OBJECT

public:
  std::string display_name_{"null"};
  QPointF pressed_pose_;
  QPointF start_pose_;
  QPointF end_pose_;
  bool is_mouse_press_;
  Qt::MouseButton pressed_button_{Qt::MouseButton::NoButton};
  QCursor *curr_cursor_ = nullptr;
  QCursor *move_cursor_ = nullptr;
  QCursor *pose_cursor_ = nullptr;
  QCursor *goal_cursor_ = nullptr;
  double scale_value_ = 1;
  bool enable_mouse_event_{false};
  bool is_rotate_event_{false};
  bool enable_scale_{true};
  QRectF bounding_rect_;
  QTransform transform_;
  double rotate_value_{0};
  OccupancyMap map_data_;
  QPointF rotate_center_;
  bool align_to_map_{true};
  bool enable_rotate_{false};
  std::string group_name_;
signals:
  void displayUpdated(std::string name);
  void displaySetScaled(std::string name, double value);
  void updateCursorPose(std::string display_name, QPointF pose);
  void scenePoseChanged(std::string display_name, QPointF pose);
  void displaySetRotate(std::string name, double value);

public:
  DisplayGroup(const std::string &display_name);
  virtual ~DisplayGroup();
  void mouseMoveOnRotate(const QPointF &oldPoint, const QPointF &mousePos);

  DisplayGroup *SetAlignToMap(const bool &align) {
    align_to_map_ = align;
    return this;
  }
  DisplayGroup *SetRotateEnable(const bool &enable) {
    enable_rotate_ = enable;
    return this;
  }
  DisplayGroup *SetScaleEnable(const bool &enable) {
    enable_scale_ = enable;
    return this;
  }
  bool GetAlignToMap() { return align_to_map_; }
  void UpdateMap(OccupancyMap map) { map_data_ = map; }
  // 设置当前图层是否响应鼠标事件
  void SetEnableMosuleEvent(const bool &response) {
    enable_mouse_event_ = response;
    if (response) {
      // 只有响应的图层才响应鼠标事件
      setAcceptHoverEvents(true);
      setAcceptedMouseButtons(Qt::AllButtons);
      setAcceptDrops(true);
      setFlag(ItemAcceptsInputMethod, true);
    }
  }
  double GetRotate() { return rotate_value_; }
  bool SetScaled(const double &value);
  bool SetRotate(const double &value);
  void SetBoundingRect(QRectF rect) { bounding_rect_ = rect; }
  QPointF GetOriginPose() { return bounding_rect_.topLeft(); }
  QPointF GetOriginPoseScene() { return mapToScene(GetOriginPose()); }
  QPointF PoseToScene(QPointF pose) { //将坐标转换为scene(以中心为原点)
    return mapToScene((pose + GetOriginPose()));
  }
  //设置原点在全局的坐标
  void SetOriginPoseInScene(const QPointF &pose) {
    setPos(pose - GetOriginPose());
  }

  QRectF boundingRect() const override { return bounding_rect_; }
  std::string GetDisplayName();
  void SetDisplayName(const std::string &display_name);
  void Update();
  void wheelEvent(QGraphicsSceneWheelEvent *event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
  void hoverMoveEvent(QGraphicsSceneHoverEvent *event) override;
};
} // namespace Display