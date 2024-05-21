#include "display/manager/view_manager.h"
#include <QDebug>
#include <iostream>
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
namespace Display {
ViewManager::ViewManager(QWidget *parent) : QGraphicsView(parent) {
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setMouseTracking(true);  // 开启鼠标追踪，以便捕获鼠标移动事件
  QVBoxLayout *main_layout = new QVBoxLayout;

  QHBoxLayout *center_layout = new QHBoxLayout;
  QVBoxLayout *left_bar_layout = new QVBoxLayout;
  center_layout->addLayout(left_bar_layout);
  center_layout->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));
  main_layout->addLayout(center_layout);

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
      "QToolButton:hover {"
      "  cursor: pointer;"
      "}");
  bottom_bar_layout->addWidget(set_big_btn_);
  QToolButton *set_small_btn_ = new QToolButton();
  set_small_btn_->setIcon(QIcon(":/images/scale.svg"));
  set_small_btn_->setIconSize(QSize(25, 25));
  set_small_btn_->setToolTip("缩小");
  set_small_btn_->setCursor(Qt::PointingHandCursor);
  set_small_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:hover {"
      "  cursor: pointer;"
      "}");
  bottom_bar_layout->addWidget(set_small_btn_);
  focus_robot_btn_ = new QToolButton();
  focus_robot_btn_->setIcon(QIcon(":/images/unfocus.svg"));
  focus_robot_btn_->setToolTip("聚焦机器人");
  focus_robot_btn_->setCursor(Qt::PointingHandCursor);
  focus_robot_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:hover {"
      "  cursor: pointer;"
      "}");
  focus_robot_btn_->setIconSize(QSize(25, 25));
  bottom_bar_layout->addWidget(focus_robot_btn_);

  setViewportMargins(0, 0, 0, bottom_bar_layout->sizeHint().height());

  //左侧工具
  QHBoxLayout *display_config_layout = new QHBoxLayout;
  QToolButton *display_list_show_btn = new QToolButton();
  display_list_show_btn->setIcon(QIcon(":/images/display.png"));
  display_list_show_btn->setIconSize(QSize(25, 25));
  display_list_show_btn->setToolTip("放大");
  display_list_show_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:hover {"
      "   cursor: pointer;"
      "}");
  display_config_layout->addWidget(display_list_show_btn);

  //图层列表面板
  QHBoxLayout *display_btn_list_layout = new QHBoxLayout;
  QToolButton *display_laser_btn_ = new QToolButton();
  display_laser_btn_->setIcon(QIcon(":/images/classes/LaserScan.png"));
  display_laser_btn_->setIconSize(QSize(25, 25));
  display_laser_btn_->setToolTip("放大");
  display_laser_btn_->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:hover {"
      "   cursor: pointer;"
      "}");
  display_btn_list_layout->addWidget(display_laser_btn_);

  QWidget *display_btn_list_widget = new QWidget();

  display_btn_list_widget->setLayout(display_btn_list_layout);
  display_btn_list_widget->hide();
  display_btn_list_widget->setStyleSheet("QWidget { margin: 0px; padding: 0px;border: 1px solid red;  }");
  display_btn_list_layout->setSpacing(0);
  display_config_layout->addWidget(display_btn_list_widget);
  display_config_layout->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));
  left_bar_layout->addItem(display_config_layout);
  left_bar_layout->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Expanding));
  setViewportMargins(left_bar_layout->sizeHint().width(), 0, 0, 0);
  connect(display_list_show_btn, &QToolButton::clicked, [this, display_btn_list_widget]() {
    if (display_btn_list_widget->isHidden()) {
      display_btn_list_widget->show();
    } else {
      display_btn_list_widget->hide();
    }
  });
  // 将布局添加到视口的小部件上
  viewport()->setLayout(main_layout);

  //connect

  connect(focus_robot_btn_, &QToolButton::clicked, [this]() {
    if (focus_robot_btn_->toolTip() == "聚焦机器人") {
      FactoryDisplay::Instance()->SetFocusDisplay(DISPLAY_ROBOT);
      focus_robot_btn_->setToolTip("取消聚焦机器人");
      focus_robot_btn_->setIcon(QIcon(":/images/focus.svg"));
    } else {
      FactoryDisplay::Instance()->SetFocusDisplay("");
      focus_robot_btn_->setToolTip("聚焦机器人");
      focus_robot_btn_->setIcon(QIcon(":/images/unfocus.svg"));
    }
  });
  connect(set_big_btn_, &QToolButton::clicked,
          [this]() { display_manager_ptr_->SetScaleBig(); });
  connect(set_small_btn_, &QToolButton::clicked, [this]() {
    display_manager_ptr_->SetScaleSmall();
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