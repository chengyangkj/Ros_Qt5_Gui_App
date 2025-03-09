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
  DisplayManager *display_manager_ptr_;

 public:
  ViewManager(QWidget *parent = nullptr);
  void SetDisplayManagerPtr(DisplayManager *display_manager);

 protected:
  void mouseMoveEvent(QMouseEvent *event) override;

  void enterEvent(QEvent *event) override;

  void leaveEvent(QEvent *event) override;
};
}  // namespace Display