#pragma once
#include <QApplication>
#include <QGraphicsView>
#include <QLayout>
#include <QMouseEvent>
#include <QPushButton>
#include <QToolButton>
#include <QLineEdit>
#include <QSlider>
#include <QLabel>
#include "display/manager/scene_manager.h"
namespace Display {
class DisplayManager;

class ViewManager : public QGraphicsView {
  Q_OBJECT
 private:
  QToolButton *focus_robot_btn_;
  QToolButton *add_robot_pos_btn_;
  DisplayManager *display_manager_ptr_;
  QLineEdit *label_pos_map_;
  QLineEdit *label_pos_scene_;
  QLineEdit *label_pos_robot_;
  QSlider *tool_size_slider_;
  QLabel *tool_size_value_label_;

 public:
  ViewManager(QWidget *parent = nullptr);
  void SetDisplayManagerPtr(DisplayManager *display_manager);
  QToolButton* GetAddRobotPosButton() { return add_robot_pos_btn_; }
  void ShowAddRobotPosButton(bool show);
  void UpdateMapPos(const QString &text);
  void UpdateScenePos(const QString &text);
  void UpdateRobotPos(const QString &text);
  void UpdateToolSizeSlider(double range);
  void ShowToolSizeSlider(bool show);

 private slots:
  void OnEditMapModeChanged(MapEditMode mode);

 protected:
  void mouseMoveEvent(QMouseEvent *event) override;

  void enterEvent(QEvent *event) override;

  void leaveEvent(QEvent *event) override;
};
}  // namespace Display