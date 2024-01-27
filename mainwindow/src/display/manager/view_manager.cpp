#include "display/manager/view_manager.h"
#include <QDebug>
#include <iostream>
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
namespace Display {
ViewManager::ViewManager(QWidget *parent) : QGraphicsView(parent) {
  setMouseTracking(true);  // 开启鼠标追踪，以便捕获鼠标移动事件
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Expanding));
  // 创建一个水平布局
  QHBoxLayout *bottom_bar_layout = new QHBoxLayout;
  bottom_bar_layout->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));
  main_layout->addLayout(bottom_bar_layout);
  // 创建工具按钮并添加到布局中

  QToolButton *set_big_btn_ = new QToolButton();
  set_big_btn_->setIcon(QIcon(":/images/big.svg"));
  set_big_btn_->setIconSize(QSize(25, 25));
  set_big_btn_->setToolTip("放大");
  set_big_btn_->setCursor(Qt::PointingHandCursor);
  set_big_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  bottom_bar_layout->addWidget(set_big_btn_);
  QToolButton *set_scale_btn_ = new QToolButton();
  set_scale_btn_->setIcon(QIcon(":/images/scale.svg"));
  set_scale_btn_->setIconSize(QSize(25, 25));
  set_scale_btn_->setToolTip("缩小");
  set_scale_btn_->setCursor(Qt::PointingHandCursor);
  set_scale_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  bottom_bar_layout->addWidget(set_scale_btn_);
  focus_robot_btn_ = new QToolButton();
  focus_robot_btn_->setIcon(QIcon(":/images/unfocus.svg"));
  focus_robot_btn_->setToolTip("聚焦机器人");
  focus_robot_btn_->setCursor(Qt::PointingHandCursor);
  focus_robot_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  focus_robot_btn_->setIconSize(QSize(25, 25));
  bottom_bar_layout->addWidget(focus_robot_btn_);

  setViewportMargins(0, 0, 0, bottom_bar_layout->sizeHint().height());
  // 将布局添加到视口的小部件上

  viewport()->setLayout(main_layout);

  //connect

  connect(focus_robot_btn_, &QToolButton::clicked, [this]() {
    if (focus_robot_btn_->toolTip() == "聚焦机器人") {
      auto display = FactoryDisplay::Instance()->GetDisplay(DISPLAY_ROBOT);
      if (display != nullptr) {
        std::cout << "focus display:" << DISPLAY_ROBOT << std::endl;
        centerOn(display);
      }
      focus_robot_btn_->setToolTip("取消聚焦机器人");
      focus_robot_btn_->setIcon(QIcon(":/images/focus.svg"));
    } else {
    
      focus_robot_btn_->setToolTip("聚焦机器人");
      focus_robot_btn_->setIcon(QIcon(":/images/unfocus.svg"));
    }
  });
}
void ViewManager::SetDisplayManagerPtr(DisplayManager *display_manager) {
  display_manager_ptr_ = display_manager;
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
  QApplication::restoreOverrideCursor();  // 恢复默认鼠标指针
  QGraphicsView::leaveEvent(event);
}
}  // namespace Display