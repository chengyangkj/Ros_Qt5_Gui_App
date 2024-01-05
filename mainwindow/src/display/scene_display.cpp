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
    std::string display_name = display->GetDisplayName();
    if (display_name == DISPLAY_GOAL) {
      curr_handle_display_ = display;
      QPointF view_pos =
          view_ptr_->mapFromScene(curr_handle_display_->scenePos());
      set_nav_pose_widget_->move(QPoint(view_pos.x(), view_pos.y()));
      set_nav_pose_widget_->show();
      set_nav_pose_widget_->SetPose(
          display_manager_->scenePoseToWord(display->GetCurrentScenePose()));
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
    std::string display_name = display->GetDisplayName();
    if (display_name == DISPLAY_GOAL) {
    }
  }
  QGraphicsScene::mouseReleaseEvent(mouseEvent);
}
void SceneDisplay::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  QPointF position = mouseEvent->scenePos(); // 获取点击位置
  if (curr_handle_display_ != nullptr) {
    std::string display_name = curr_handle_display_->GetDisplayName();
    std::cout << "move display name: " << display_name
              << " pose:" << position.x() << " " << position.y() << std::endl;
    if (display_name == DISPLAY_GOAL) {
      QPointF view_pos =
          view_ptr_->mapFromScene(curr_handle_display_->scenePos());
      set_nav_pose_widget_->move(QPoint(view_pos.x(), view_pos.y()));
      set_nav_pose_widget_->show();
      std::cout << "get :" << curr_handle_display_->GetCurrentScenePose()
                << std::endl;
      set_nav_pose_widget_->SetPose(display_manager_->scenePoseToWord(
          curr_handle_display_->GetCurrentScenePose()));
    }
  }

  QGraphicsScene::mouseMoveEvent(mouseEvent);
}
} // namespace Display