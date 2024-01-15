#include "display/manager/view_manager.h"
#include <iostream>
ViewManager::ViewManager(QWidget *parent) : QGraphicsView(parent) {
  setMouseTracking(true); // 开启鼠标追踪，以便捕获鼠标移动事件
}
void ViewManager::mouseMoveEvent(QMouseEvent *event) {
  // 根据需要设置不同的鼠标指针样式
  // if (someCondition)
  //   QApplication::setOverrideCursor(Qt::PointingHandCursor); //
  //   设置为手指指针
  // else
  //   QApplication::restoreOverrideCursor(); // 恢复默认鼠标指针

  QGraphicsView::mouseMoveEvent(event);
}

void ViewManager::enterEvent(QEvent *event) {
  //   QApplication::setOverrideCursor(Qt::ArrowCursor); // 设置为箭头指针
  QGraphicsView::enterEvent(event);
}

void ViewManager::leaveEvent(QEvent *event) {
  QApplication::restoreOverrideCursor(); // 恢复默认鼠标指针
  QGraphicsView::leaveEvent(event);
}