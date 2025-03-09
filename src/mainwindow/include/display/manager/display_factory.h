#pragma once
#include <QGraphicsItem>
#include <QGraphicsView>
#include <QThread>
#include <QTimer>
#include "display/manager/scene_manager.h"
#include "virtual_display.h"
namespace Display {
class FactoryDisplay : public QObject {
  Q_OBJECT
 public:
  static FactoryDisplay *Instance() {
    static FactoryDisplay *factory = new FactoryDisplay();
    return factory;
  }
  FactoryDisplay();
  ~FactoryDisplay();
  bool Init(QGraphicsView *viewer, SceneManager *scene_ptr);
  // 获取图层
  VirtualDisplay *GetDisplay(const std::string &display_name);
  // 设置图层的scene坐标
  bool SetDisplayScenePose(const std::string &display_name,
                           const QPointF &pose);
  // 设置图层放大缩小
  bool SetDisplayScaled(const std::string &display_name, const double &value);
  void AddDisplay(VirtualDisplay *display, const std::string &parent_name);
  void SetFocusDisplay(const std::string &display_name);
  void RemoveDisplay(const std::string &name);
  void RemoveDisplay(VirtualDisplay *display);
  int GetDisplaySize();
  std::map<std::string, VirtualDisplay *> GetTotalDisplayMap();
  // 设置响应鼠标事件的图层
  bool SetMoveEnable(const std::string &display_name, bool enable = true);
  bool GetMoveEnable(const std::string &display_name);
  bool SetDisplayPoseInParent(const std::string &display_name,
                              const RobotPose &pose);
  void CenterOnScene(QPointF pose){
    viewer_ptr_->centerOn(pose);
  }
 private slots:
  void Process();

 private:
  QTimer timer_coordinate_system_;
  std::string main_display_;
  std::map<std::string, VirtualDisplay *> total_display_map_;
  SceneManager *scene_manager_ptr_;
  bool initlizated_ = false;
  QGraphicsView *viewer_ptr_;
  std::atomic_bool run_flag_ = false;
  std::string focus_display_name_ = "";
};
}  // namespace Display