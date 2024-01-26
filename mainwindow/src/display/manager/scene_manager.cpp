#include "display/manager/scene_manager.h"
#include <QDebug>
#include <iostream>
#include "config/config_manager.h"
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
#include "display/point_shape.h"
#include "logger/logger.h"
namespace Display {
SceneManager::SceneManager(QObject *parent) : QGraphicsScene(parent) {}
void SceneManager::Init(QGraphicsView *view_ptr, DisplayManager *manager) {
  // 1s自动保存1次 拓扑地图

  display_manager_ = manager;
  view_ptr_ = view_ptr;
  set_nav_pose_widget_ = new SetPoseWidget(view_ptr_);
  set_nav_pose_widget_->hide();
  nav_goal_widget_ = new NavGoalWidget(view_ptr_);
  nav_goal_widget_->hide();
  QPixmap goal_image;
  goal_image.load("://images/add_16.svg");
  QMatrix matrix;
  matrix.rotate(90);
  goal_image =
      goal_image.transformed(QTransform(matrix), Qt::SmoothTransformation);

  nav_goal_cursor_ =
      QCursor(goal_image, goal_image.width() / 2, goal_image.height() / 2);
  LoadTopologyMap();
  saveTopologyMap();
}
void SceneManager::LoadTopologyMap() {
  Config::ConfigManager::Instacnce()->ReadTopologyMap(
      Config::ConfigManager::Instacnce()
          ->GetRootConfig()
          .topology_map_config.map_name,
      topology_map_);
  for (auto &point : topology_map_.points) {
    auto goal_point =
        (new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL,
                        point.name, 8, DISPLAY_MAP));

    goal_point->SetRotateEnable(true)->SetMoveEnable(true)->setVisible(true);
    goal_point->UpdateDisplay(
        display_manager_->wordPose2Map(point.ToRobotPose()));
  }
}
void SceneManager::saveTopologyMap() {
  Config::ConfigManager::Instacnce()->WriteTopologyMap(
      Config::ConfigManager::Instacnce()
          ->GetRootConfig()
          .topology_map_config.map_name,
      topology_map_);
  emit signalTopologyMapUpdate(topology_map_);
  // 递归
  QTimer::singleShot(1000, this, [=] { saveTopologyMap(); });
}
void SceneManager::AddOneNavPoint() {
  view_ptr_->setCursor(nav_goal_cursor_);

  current_mode_ = eMode::kAddNavGoal;
}
void SceneManager::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  QPointF position = mouseEvent->scenePos();  // 获取点击位置
  switch (current_mode_) {
    case eMode::kNone: {
      QGraphicsItem *item =
          itemAt(position, views()[0]->transform());  // 获取点击位置下的 item

      if (item != nullptr) {  // 判断是否获取到了 item
        Display::VirtualDisplay *display =
            dynamic_cast<Display::VirtualDisplay *>(item);
        std::string display_type = display->GetDisplayType();
        if (display_type == DISPLAY_GOAL) {
          curr_handle_display_ = display;
          // 窗体初始化
          blindNavGoalWidget(display);
          emit signalCurrentSelectPointChanged(
              TopologyMap::PointInfo(TopologyMap::PointInfo(
                  display_manager_->scenePoseToWord(
                      basic::RobotPose(position.x(), position.y(), 0)),
                  display->GetDisplayName())));
        } else if (display_type != DISPLAY_GOAL && curr_handle_display_ != nullptr && curr_handle_display_->GetDisplayType() == DISPLAY_GOAL) {
          curr_handle_display_ = nullptr;
          nav_goal_widget_->hide();
        }
      }
    } break;
    case eMode::kAddNavGoal: {
      std::string name = generatePointName("NAV_POINT");
      auto goal_point = (new PointShape(PointShape::ePointType::kNavGoal,
                                        DISPLAY_GOAL, name, 8, DISPLAY_MAP));
      goal_point->SetRotateEnable(true)->SetMoveEnable(true)->setVisible(true);
      goal_point->UpdateDisplay(display_manager_->scenePoseToMap(
          basic::RobotPose(position.x(), position.y(), 0)));
      topology_map_.AddPoint(TopologyMap::PointInfo(
          display_manager_->scenePoseToWord(
              basic::RobotPose(position.x(), position.y(), 0)),
          name));
      LOG_INFO("add one nav point size:" << topology_map_.points.size()
                                         << " name:" << name);
      curr_handle_display_ = goal_point;
      blindNavGoalWidget(goal_point);
      current_mode_ = kNone;
      view_ptr_->unsetCursor();
    } break;
    default:
      break;
  }

  QGraphicsScene::mousePressEvent(mouseEvent);
}
std::string SceneManager::generatePointName(const std::string &prefix) {
  int index = topology_map_.points.size();
  std::string name = prefix + "_" + std::to_string(topology_map_.points.size());
  // 生成不重复的ID
  while (true) {
    auto iter =
        std::find_if(topology_map_.points.begin(), topology_map_.points.end(),
                     [name](const TopologyMap::PointInfo &point) {
                       return point.name == name;
                     });
    if (iter == topology_map_.points.end()) {
      break;
    } else {
      name = prefix + "_" + std::to_string(++index);
    }
  }
  return name;
}
void SceneManager::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  QPointF position = mouseEvent->scenePos();  // 获取点击位置
  QGraphicsScene::mouseReleaseEvent(mouseEvent);
}
void SceneManager::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  switch (current_mode_) {
    case eMode::kNone: {
      QPointF position = mouseEvent->scenePos();  // 获取点击位置
      if (curr_handle_display_ != nullptr) {
        std::string display_type = curr_handle_display_->GetDisplayType();
        if (display_type == DISPLAY_GOAL) {
          updateNavGoalWidgetPose(curr_handle_display_);
        }
      }
    } break;
    case eMode::kAddNavGoal: {
    } break;
    default:
      break;
  }

  QGraphicsScene::mouseMoveEvent(mouseEvent);
}
void SceneManager::blindNavGoalWidget(Display::VirtualDisplay *display) {
  QPointF view_pos = view_ptr_->mapFromScene(display->scenePos());
  nav_goal_widget_->move(QPoint(view_pos.x(), view_pos.y()));
  nav_goal_widget_->show();
  nav_goal_widget_->SetPose(NavGoalWidget::PointInfo{
      .pose = display_manager_->scenePoseToWord(display->GetCurrentScenePose()),
      .name = QString::fromStdString(display->GetDisplayName())});
  nav_goal_widget_->disconnect();

  connect(nav_goal_widget_, &NavGoalWidget::SignalHandleOver,
          [this, display](const NavGoalWidget::HandleResult &flag,
                          const RobotPose &pose) {
            if (flag == NavGoalWidget::HandleResult::kSend) {
              emit display_manager_->signalPub2DGoal(pose);
              nav_goal_widget_->hide();
              curr_handle_display_ = nullptr;
              nav_goal_widget_->hide();
            } else if (flag == NavGoalWidget::HandleResult::kRemove) {
              LOG_INFO("remove:" << display->GetDisplayName());
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
void SceneManager::updateNavGoalWidgetPose(
    const Display::VirtualDisplay *display) {
  QPointF view_pos = view_ptr_->mapFromScene(display->scenePos());
  nav_goal_widget_->move(QPoint(view_pos.x(), view_pos.y()));
  nav_goal_widget_->show();
  nav_goal_widget_->SetPose(NavGoalWidget::PointInfo{
      .pose = display_manager_->scenePoseToWord(
          curr_handle_display_->GetCurrentScenePose()),
      .name = QString::fromStdString(curr_handle_display_->GetDisplayName())});
}
SceneManager::~SceneManager() {}
}  // namespace Display