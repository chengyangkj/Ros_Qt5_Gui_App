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
      std::cout << e.what() << '\n';        \
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
#define DISPLAY_SHAPE "RobotShape"
enum MapEditMode {
  kStop = 0,
  kNormal,        // 正常编辑模式
  kErase,         // 橡皮擦模式
  kDrawLine,      // 绘制线段模式
  kAddPoint,      // 添加点位模式
  kRegion,        // 添加区域模式
  kDrawWithPen,   //画笔
  kLinkTopology,  // 拓扑图
};
class VirtualDisplay : public QObject, public QGraphicsItem {
  Q_OBJECT

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
  double min_scale_value_{0.1};
  double max_scale_value_{20};
 signals:
  void signalCursorPose(QPointF pose);
  void signalPoseUpdate(const RobotPose &pose);
  void signalItemChange(GraphicsItemChange change, const QVariant &value);

 public:
  VirtualDisplay(const std::string &display_type, const int &z_value,
                 const std::string &parent_name, std::string display_name = "");
  virtual ~VirtualDisplay();
  bool UpdateDisplay(const std::any &data) { return UpdateData(data); }
  VirtualDisplay *SetRotateEnable(const bool &enable) {
    enable_rotate_ = enable;
    return this;
  }
  double GetScaleValue() { return scale_value_; }
  VirtualDisplay *SetScaleEnable(const bool &enable) {
    enable_scale_ = enable;
    return this;
  }
  VirtualDisplay *SetMoveEnable(const bool &enable) {
    move_enable_ = enable;
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
  bool GetMoveEnable() { return move_enable_; }
  void AddChild(VirtualDisplay *child) { children_.push_back(child); }
  void RemoveChild(VirtualDisplay *child) {
    children_.erase(std::remove(children_.begin(), children_.end(), child),
                    children_.end());
  }
  std::vector<VirtualDisplay *> GetChildren() { return children_; }
  void UpdateMap(OccupancyMap map) { map_data_ = map; }

  double GetRotate() { return rotate_value_; }
  virtual bool UpdateData(const std::any &data) = 0;
  virtual bool SetDisplayConfig(const std::string &config_name,
                                const std::any &config_data);
  bool SetScaled(const double &value);
  bool SetRotate(const double &value);
  void SetBoundingRect(QRectF rect) { bounding_rect_ = rect; }
  QPointF GetOriginPose() { return bounding_rect_.topLeft(); }
  QPointF GetOriginPoseScene() { return mapToScene(GetOriginPose()); }
  QPointF PoseToScene(QPointF pose) {  //将坐标转换为scene(以中心为原点)
    return mapToScene((pose + GetOriginPose()));
  }
  void CenterOnScene(QPointF pose);
  bool IsMoving() { return is_moving_; }
  void UpdatePose(const RobotPose &pose) { SetPoseInParent(pose); }
  void SetPoseInParent(const RobotPose &pose);
  RobotPose GetCurrentScenePose() { return curr_scene_pose_; }
  RobotPose GetPoseInParent() { return pose_in_parent_; }
  std::string GetParentName() { return parent_name_; }
  std::string GetDisplayName() { return display_name_; }
  void SetDisplayName(const std::string &name) { display_name_ = name; }
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