#include "display/scene_display.h"
#include "display/display_manager.h"
#include "display/factory_display.h"
#include "display/point_shape.h"
#include <QDebug>
#include <iostream>
namespace Display {
SceneDisplay::SceneDisplay(QObject *parent) : QGraphicsScene(parent) {}
void SceneDisplay::Init(QGraphicsView *view_ptr, DisplayManager *manager) {
  display_manager_ = manager;
  view_ptr_ = view_ptr;
  set_nav_pose_widget_ = new SetPoseWidget(view_ptr_);
  set_nav_pose_widget_->hide();
  nav_goal_widget_ = new NavGoalWidget(view_ptr_);
  nav_goal_widget_->hide();
}
void SceneDisplay::AddOneNavGoal() {
  (new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL, 8,
                  DISPLAY_MAP))
      ->SetRotateEnable(true)
      ->SetMoveEnable(true)
      ->setVisible(true);
}
void SceneDisplay::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) {

  QPointF position = mouseEvent->scenePos(); // 获取点击位置
  QGraphicsItem *item =
      itemAt(position, views()[0]->transform()); // 获取点击位置下的 item
  if (item != nullptr) { // 判断是否获取到了 item
    Display::VirtualDisplay *display =
        dynamic_cast<Display::VirtualDisplay *>(item);
    std::string display_type = display->GetDisplayType();
    if (display_type == DISPLAY_GOAL) {
      curr_handle_display_ = display;

      //设置初始目标点位置（车头前方1m）
      // display->SetMoveEnable(false);
      // display->UpdateDisplay(display_manager_->wordPose2Map(
      //     absoluteSum(display_manager_->GetRobotPose(), RobotPose(1, 0,
      //     0))));
      // enable move after 300ms
      // QTimer::singleShot(300,
      //                    [this, display]() { display->SetMoveEnable(true);
      //                    });
      //窗体初始化
      QPointF view_pos =
          view_ptr_->mapFromScene(curr_handle_display_->scenePos());
      nav_goal_widget_->move(QPoint(view_pos.x(), view_pos.y()));
      nav_goal_widget_->show();
      nav_goal_widget_->SetPose(
          display_manager_->scenePoseToWord(display->GetCurrentScenePose()));
      nav_goal_widget_->disconnect();

      connect(nav_goal_widget_, &NavGoalWidget::SignalHandleOver,
              [this, display](const NavGoalWidget::HandleResult &flag,
                              const RobotPose &pose) {
                if (flag == NavGoalWidget::HandleResult::kSend) {
                  emit display_manager_->signalPub2DGoal(pose);
                  nav_goal_widget_->hide();
                } else if (flag == NavGoalWidget::HandleResult::kRemove) {
                  curr_handle_display_ = nullptr;
                  FactoryDisplay::Instance()->RemoveDisplay(display);
                  nav_goal_widget_->disconnect();
                  delete display;
                  nav_goal_widget_->hide();
                } else {
                  curr_handle_display_ = nullptr;
                  nav_goal_widget_->hide();
                }
              });
      connect(nav_goal_widget_, &NavGoalWidget::SignalPoseChanged,
              [this, display](const RobotPose &pose) {
                display->UpdateDisplay(display_manager_->wordPose2Map(pose));
              });
    }
  }
  QGraphicsScene::mousePressEvent(mouseEvent);
}
void SceneDisplay::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  QPointF position = mouseEvent->scenePos(); // 获取点击位置
  QGraphicsItem *item =
      itemAt(position, views()[0]->transform()); // 获取点击位置下的 item
  if (item != nullptr) { // 判断是否获取到了 item
    Display::VirtualDisplay *display =
        dynamic_cast<Display::VirtualDisplay *>(item);
    std::string display_type = display->GetDisplayType();
    if (display_type == DISPLAY_GOAL) {
    }
  }
  QGraphicsScene::mouseReleaseEvent(mouseEvent);
}
void SceneDisplay::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  QPointF position = mouseEvent->scenePos(); // 获取点击位置
  if (curr_handle_display_ != nullptr) {
    std::string display_type = curr_handle_display_->GetDisplayType();
    if (display_type == DISPLAY_GOAL) {
      QPointF view_pos =
          view_ptr_->mapFromScene(curr_handle_display_->scenePos());
      nav_goal_widget_->move(QPoint(view_pos.x(), view_pos.y()));
      nav_goal_widget_->show();
      nav_goal_widget_->SetPose(display_manager_->scenePoseToWord(
          curr_handle_display_->GetCurrentScenePose()));
    }
  }

  QGraphicsScene::mouseMoveEvent(mouseEvent);
}
} // namespace Display