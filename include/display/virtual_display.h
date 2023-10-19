/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-29 14:21:31
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 09:46:05
 * @FilePath:
 * ////include/display/virtual_display.h
 * @Description: 图层的虚拟类
 */
#ifndef VIRTUAL_DISPLAY_H
#define VIRTUAL_DISPLAY_H
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

class VirtualDisplay : public QObject, public QGraphicsItem {
  Q_OBJECT
public:
  class FactoryDisplay {
  public:
    static FactoryDisplay *Instance() {
      static FactoryDisplay *factory = new FactoryDisplay();
      return factory;
    }
    // 获取图层
    VirtualDisplay *GetDisplay(const std::string &display_name) {
      if (display_map_.count(display_name) != 0)
        return display_map_[display_name];
      else
        return nullptr;
    }
    // 设置图层的scene坐标
    bool SetDisplayScenePose(const std::string &display_name,
                             const QPointF &pose) {
      VirtualDisplay *display = GetDisplay(display_name);
      if (display == nullptr)
        return false;
      display->setPos(pose);
      return true;
    }
    // 设置图层放大缩小
    bool SetDisplayScaled(const std::string &display_name,
                          const double &value) {
      VirtualDisplay *display = GetDisplay(display_name);
      if (display == nullptr)
        return false;
      display->SetScaled(value);
      return true;
    }
    void AddDisplay(VirtualDisplay *display) {
      std::string name = display->GetDisplayName();
      if (display_map_.count(name) == 0) {
        display_map_[name] = display;
      } else {
        delete display_map_[name];
        display_map_[name] = display;
      }
    }
    void RemoveDisplay(const std::string &name) {
      auto iter =display_map_.find(name);
      if (iter!= display_map_.end()) {
        delete iter->second;
        display_map_.erase(iter);
      }
    }
    int GetDisplaySize() { return display_map_.size(); }
    std::map<std::string, VirtualDisplay *> GetDisplayMap() {
      return display_map_;
    }
    // 设置响应鼠标事件的图层(同一时刻只有一个图层响应鼠标事件)
    bool SetMainDisplay(const std::string &display_name) {
      if (display_map_.count(display_name) == 0) {
        return false;
      }
      main_display_ = display_name;
      for (auto [name, display] : display_map_) {
        if (name == display_name) {
          display->SetResposeMouseEvent(true);
        } else {
          display->SetResposeMouseEvent(false);
        }
      }
      return true;
    }
    std::string GetMainDisplay() { return main_display_; }

  private:
    std::string main_display_;
    std::map<std::string, VirtualDisplay *> display_map_;
  };

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
  bool is_response_mouse_event_{false};
  bool is_rotate_event_{false};
  bool enable_scale_{true};
  QRectF bounding_rect_;
  QTransform transform_;
  double rotate_value_{0};
  OccupancyMap map_data_;
  QPointF rotate_center_;
  bool align_to_map_{true};
  bool is_rotate_enable_{false};
signals:
  void displayUpdated(std::string name);
  void displaySetScaled(std::string name, double value);
  void updateCursorPose(std::string display_name, QPointF pose);
  void scenePoseChanged(std::string display_name, QPointF pose);
  void displaySetRotate(std::string name, double value);

public:
  VirtualDisplay(const std::string &display_name, const int &z_value);
  ~VirtualDisplay();
  void mouseMoveOnRotate(const QPointF &oldPoint, const QPointF &mousePos);
  bool UpdateDisplay(const std::any &data) {
    emit displayUpdated(display_name_);
    return UpdateData(data);
  }
  VirtualDisplay *SetAlignToMap(const bool &align) {
    align_to_map_ = align;
    return this;
  }
  VirtualDisplay *SetRotateEnable(const bool &enable) {
    is_rotate_enable_ = enable;
    return this;
  }

  bool GetAlignToMap() { return align_to_map_; }
  void UpdateMap(OccupancyMap map) { map_data_ = map; }
  // 设置当前图层是否响应鼠标事件
  void SetResposeMouseEvent(const bool &response) {
    is_response_mouse_event_ = response;
    if (response) {
      // 只有响应的图层才响应鼠标事件
      setAcceptHoverEvents(true);
      setAcceptedMouseButtons(Qt::AllButtons);
      setAcceptDrops(true);
      setFlag(ItemAcceptsInputMethod, true);
    }
  }
  double GetRotate() { return rotate_value_; }
  virtual bool UpdateData(const std::any &data) = 0;
  virtual bool SetDisplayConfig(const std::string &config_name,
                                const std::any &config_data);
  bool SetScaled(const double &value);
  bool SetRotate(const double &value);
  void SetBoundingRect(QRectF rect) { bounding_rect_ = rect; }
  QPointF GetOriginPose() { return bounding_rect_.topLeft(); }
  QPointF GetOriginPoseScene() { return mapToScene(GetOriginPose()); }
  QPointF OccPoseToScene(QPointF pose) { //将坐标转换为scene(以中心为原点)
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

#endif
