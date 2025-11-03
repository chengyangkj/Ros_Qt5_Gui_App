#pragma once
#include <QApplication>
#include <QGraphicsView>
#include <QLayout>
#include <QMouseEvent>
#include <QPushButton>
#include <QToolButton>
namespace Display {
class DisplayManager;
class ViewManager : public QGraphicsView {
  Q_OBJECT
 private:
  QToolButton *focus_robot_btn_;
  QToolButton *add_robot_pos_btn_;
  DisplayManager *display_manager_ptr_;

 public:
  ViewManager(QWidget *parent = nullptr);
  void SetDisplayManagerPtr(DisplayManager *display_manager);
  QToolButton* GetAddRobotPosButton() { return add_robot_pos_btn_; }
  void ShowAddRobotPosButton(bool show);

 protected:
  void mouseMoveEvent(QMouseEvent *event) override;

  void enterEvent(QEvent *event) override;

  void leaveEvent(QEvent *event) override;
};
}  // namespace Display