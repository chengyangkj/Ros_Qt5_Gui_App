#include "display/manager/scene_manager.h"
#include "config/config_manager.h"
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
#include "display/point_shape.h"
#include <QDebug>
#include <iostream>
namespace Display {
SceneDisplay::SceneDisplay(QObject *parent) : QGraphicsScene(parent) {}
void SceneDisplay::Init(QGraphicsView *view_ptr, DisplayManager *manager) {
  Config::ConfigManager::Instacnce()->SetDefaultConfig(
      "TopologyMap/Path", "./default_topology_map.json");

  // 1s自动保存1次 拓扑地图
  QTimer::singleShot(1000, this, [=] { saveTopologyMap(); });

  display_manager_ = manager;
  view_ptr_ = view_ptr;
  set_nav_pose_widget_ = new SetPoseWidget(view_ptr_);
  set_nav_pose_widget_->hide();
  nav_goal_widget_ = new NavGoalWidget(view_ptr_);
  nav_goal_widget_->hide();
  QPixmap goal_image;
  goal_image.load("://images/goal_green.png");
  QMatrix matrix;
  matrix.rotate(90);
  goal_image = goal_image.transformed(matrix, Qt::SmoothTransformation);
  nav_goal_cursor_ = std::make_shared<QCursor>(goal_image, 0, 0);
}
void SceneDisplay::LoadTopologyMap() {
  std::cout << "start load topology map" << std::endl;
  Config::ConfigManager::Instacnce()->ReadTopologyMap(
      Config::ConfigManager::Instacnce()->GetConfig("TopologyMap/Path"),
      topology_map_);
  for (auto &point : topology_map_.points) {
    auto goal_point =
        (new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL,
                        point.name, 8, DISPLAY_MAP));

    goal_point->SetRotateEnable(true)->SetMoveEnable(true)->setVisible(true);
    goal_point->UpdateDisplay(
        display_manager_->wordPose2Map(point.ToRobotPose()));
    std::cout << "update topoMpa:" << point.name
              << "pose:" << point.ToRobotPose() << std::endl;
  }
}
void SceneDisplay::saveTopologyMap() {
  if (topology_map_.points.size() == 0)
    return;
  Config::ConfigManager::Instacnce()->WriteTopologyMap(
      Config::ConfigManager::Instacnce()->GetConfig("TopologyMap/Path"),
      topology_map_);
  // 递归
  QTimer::singleShot(1000, this, [=] { saveTopologyMap(); });
}
void SceneDisplay::AddOneNavPoint() {
  std::string name = "NAV_POINT_" + std::to_string(topology_map_.points.size());
  // this->setCursor(nav_goal_cursor_);
  (new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL, name, 8,
                  DISPLAY_MAP))
      ->SetRotateEnable(true)
      ->SetMoveEnable(true)
      ->setVisible(true);
  topology_map_.AddPoint(TopologyMap::PointInfo(0, 0, 0, name));
  std::cout << "add one nav point size:" << topology_map_.points.size()
            << " name:" << name << std::endl;
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
      //窗体初始化
      QPointF view_pos =
          view_ptr_->mapFromScene(curr_handle_display_->scenePos());
      nav_goal_widget_->move(QPoint(view_pos.x(), view_pos.y()));
      nav_goal_widget_->show();
      nav_goal_widget_->SetPose(NavGoalWidget::PointInfo{
          .pose =
              display_manager_->scenePoseToWord(display->GetCurrentScenePose()),
          .name = QString::fromStdString(display->GetDisplayName())});
      nav_goal_widget_->disconnect();

      connect(nav_goal_widget_, &NavGoalWidget::SignalHandleOver,
              [this, display](const NavGoalWidget::HandleResult &flag,
                              const RobotPose &pose) {
                if (flag == NavGoalWidget::HandleResult::kSend) {
                  emit display_manager_->signalPub2DGoal(pose);
                  nav_goal_widget_->hide();
                } else if (flag == NavGoalWidget::HandleResult::kRemove) {
                  topology_map_.RemovePoint(display->GetDisplayName());
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
      topology_map_.UpdatePoint(
          display->GetDisplayName(),
          TopologyMap::PointInfo(
              display_manager_->scenePoseToWord(display->GetCurrentScenePose()),
              display->GetDisplayName()));
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
      nav_goal_widget_->SetPose(NavGoalWidget::PointInfo{
          .pose = display_manager_->scenePoseToWord(
              curr_handle_display_->GetCurrentScenePose()),
          .name =
              QString::fromStdString(curr_handle_display_->GetDisplayName())});
    }
  }

  QGraphicsScene::mouseMoveEvent(mouseEvent);
}
SceneDisplay::~SceneDisplay() {}
} // namespace Display