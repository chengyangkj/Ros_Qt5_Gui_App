#include "display/manager/scene_manager.h"
#include <QDebug>
#include <iostream>
#include "config/config_manager.h"
#include "display/display_occ_map.h"
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
#include "display/point_shape.h"
#include "logger/logger.h"
namespace Display {
SceneManager::SceneManager(QObject *parent) : QGraphicsScene(parent), current_mode_(MapEditMode::kStop) {}
void SceneManager::Init(QGraphicsView *view_ptr, DisplayManager *manager) {
  // 1s自动保存1次 拓扑地图

  display_manager_ = manager;
  view_ptr_ = view_ptr;
  set_nav_pose_widget_ = new SetPoseWidget(view_ptr_);
  set_nav_pose_widget_->hide();
  nav_goal_widget_ = new NavGoalWidget(view_ptr_);
  nav_goal_widget_->hide();
  QPixmap goal_image;
  goal_image.load("://images/add_32.svg");
  QMatrix matrix;
  matrix.rotate(90);
  goal_image =
      goal_image.transformed(QTransform(matrix), Qt::SmoothTransformation);
  nav_goal_cursor_ =
      QCursor(goal_image, goal_image.width() / 2, goal_image.height() / 2);
  QPixmap pen_image;
  pen_image.load("://images/pen_32.svg");
  pen_cursor_ = QCursor(pen_image, 0, pen_image.height());

  QPixmap line_image;
  line_image.load("://images/line_btn_32.svg");
  line_cursor_ = QCursor(line_image, 0, line_image.height());
}
void SceneManager::LoadTopologyMap() {
  OpenTopologyMap(Config::ConfigManager::Instacnce()
                      ->GetRootConfig()
                      .topology_map_config.map_name);
}
void SceneManager::OpenTopologyMap(const std::string &file_path) {
  //删除原有点
  for (auto point : topology_map_.points) {
    auto display = FactoryDisplay::Instance()->GetDisplay(point.name);
    if (display == nullptr) continue;
    topology_map_.RemovePoint(display->GetDisplayName());
    FactoryDisplay::Instance()->RemoveDisplay(display);
    delete display;
  }
  Config::ConfigManager::Instacnce()->ReadTopologyMap(
      file_path,
      topology_map_);
  for (auto &point : topology_map_.points) {
    auto goal_point =
        (new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL,
                        point.name, 8, DISPLAY_MAP));

    goal_point->SetRotateEnable(true)->SetMoveEnable(false)->setVisible(true);
    goal_point->UpdateDisplay(
        display_manager_->wordPose2Map(point.ToRobotPose()));
    LOG_INFO("Load Point:" << point.name)
  }
  LOG_INFO("Load Topology Map Success!")
  SetPointMoveEnable(false);
  emit signalTopologyMapUpdate(topology_map_);
}
void SceneManager::SetEditMapMode(MapEditMode mode) {
  current_mode_ = mode;
  switch (mode) {
    case kStop: {
      SetPointMoveEnable(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_LOCAL_COST_MAP)->setVisible(true);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_GLOBAL_COST_MAP)->setVisible(true);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP)->SetMoveEnable(true);
      saveTopologyMap();
      view_ptr_->setCursor(Qt::ArrowCursor);
    } break;
    case kAddPoint: {
      view_ptr_->setCursor(nav_goal_cursor_);
    } break;
    case kNormal: {
      view_ptr_->setCursor(Qt::OpenHandCursor);
      SetPointMoveEnable(true);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP)->SetMoveEnable(true);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_LOCAL_COST_MAP)->setVisible(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_GLOBAL_COST_MAP)->setVisible(false);
    } break;
    case kErase: {
      SetPointMoveEnable(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP)->SetMoveEnable(false);
      setEraseCursor();
    } break;
    case kDrawWithPen: {
      SetPointMoveEnable(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP)->SetMoveEnable(false);
      view_ptr_->setCursor(pen_cursor_);
    } break;
    case kDrawLine: {
      SetPointMoveEnable(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP)->SetMoveEnable(false);
      view_ptr_->setCursor(line_cursor_);
    } break;
    default:
      break;
  }
  LOG_INFO("set edit mode:" << mode)
}
void SceneManager::SetPointMoveEnable(bool is_enable) {
  nav_goal_widget_->SetEditEnabled(is_enable);
  for (auto point : topology_map_.points) {
    auto display = FactoryDisplay::Instance()->GetDisplay(point.name);
    if (display != nullptr) {
      display->SetMoveEnable(is_enable);
    } else {
      LOG_ERROR("not find display:" << point.name)
    }
  }
}
void SceneManager::saveTopologyMap() {
  Config::ConfigManager::Instacnce()->WriteTopologyMap(
      Config::ConfigManager::Instacnce()
          ->GetRootConfig()
          .topology_map_config.map_name,
      topology_map_);
  LOG_INFO("save topology map");
  emit signalTopologyMapUpdate(topology_map_);
}
void SceneManager::AddOneNavPoint() {
}
void SceneManager::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  if (mouseEvent->button() == Qt::LeftButton) {
    left_pressed_ = true;
  }
  if (mouseEvent->button() == Qt::RightButton) {
    right_pressed_ = true;
  }
  QPointF position = mouseEvent->scenePos();  // 获取点击位置
  switch (current_mode_) {
    case MapEditMode::kStop: {
    } break;
    case MapEditMode::kNormal: {
    } break;
    case MapEditMode::kDrawLine: {
      auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
      map_ptr->StartDrawLine(map_ptr->mapFromScene(position));
    } break;
    case MapEditMode::kAddPoint: {
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
    } break;
    case MapEditMode::kErase: {
      eraseScenePointRange(position, 3);
    } break;
    case MapEditMode::kDrawWithPen: {
      drawPoint(position);
    } break;
    default:
      break;
  }
  QGraphicsItem *item =
      itemAt(position, views()[0]->transform());  // 获取点击位置下的 item
  if (item != nullptr) {                          // 判断是否获取到了 item
    Display::VirtualDisplay *display =
        dynamic_cast<Display::VirtualDisplay *>(item);
    std::string display_type = display->GetDisplayType();

    //点击到目标点弹窗
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
  left_pressed_ = false;
  right_pressed_ = false;
  switch (current_mode_) {
    case MapEditMode::kDrawLine: {
      auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
      map_ptr->EndDrawLine(map_ptr->mapFromScene(position), true);
    } break;
    case MapEditMode::kNormal: {
      if (curr_handle_display_ != nullptr) {
        std::string display_type = curr_handle_display_->GetDisplayType();
        if (display_type == DISPLAY_GOAL) {
          LOG_INFO("release goal:" << curr_handle_display_->GetCurrentScenePose());
          topology_map_.UpdatePoint(
              curr_handle_display_->GetDisplayName(),
              TopologyMap::PointInfo(
                  display_manager_->scenePoseToWord(
                      curr_handle_display_->GetCurrentScenePose()),
                  curr_handle_display_->GetDisplayName()));
        }
      }
    } break;
    default:
      break;
  }

}  // namespace Display
void SceneManager::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  QPointF position = mouseEvent->scenePos();  // 获取点击位置
  //点位属性框跟随移动处理
  switch (current_mode_) {
    case MapEditMode::kStop: {
      if (curr_handle_display_ != nullptr) {
        std::string display_type = curr_handle_display_->GetDisplayType();
        if (display_type == DISPLAY_GOAL) {
          updateNavGoalWidgetPose(curr_handle_display_, false);
        }
      }
    } break;
    case MapEditMode::kNormal: {
      if (curr_handle_display_ != nullptr) {
        std::string display_type = curr_handle_display_->GetDisplayType();
        if (display_type == DISPLAY_GOAL) {
          updateNavGoalWidgetPose(curr_handle_display_);
        }
      }
    } break;
    case MapEditMode::kDrawLine: {
      if (left_pressed_) {
        auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
        map_ptr->EndDrawLine(map_ptr->mapFromScene(position), false);
      }
    } break;
    case MapEditMode::kAddPoint: {
    } break;
    case MapEditMode::kErase: {
      if (left_pressed_) {
        eraseScenePointRange(position, 3);
      }
    } break;
    case MapEditMode::kDrawWithPen: {
      if (left_pressed_) {
        drawPoint(position);
      }
    } break;
    default:
      break;
  }

  QGraphicsScene::mouseMoveEvent(mouseEvent);
}
void SceneManager::wheelEvent(QGraphicsSceneWheelEvent *event) {
  switch (current_mode_) {
    case kErase: {
      setEraseCursor();
    } break;
  }
  QGraphicsScene::wheelEvent(event);
}
void SceneManager::setEraseCursor() {
  auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
  double scale_value = map_ptr->GetScaleValue();
  QPixmap pixmap(eraser_range_ * 2 * scale_value, eraser_range_ * 2 * scale_value);
  // 使用 QPainter 绘制一个红色圆形
  pixmap.fill(Qt::transparent);
  QPainter painter(&pixmap);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(255, 0, 0, 50));
  painter.drawRect(0, 0, pixmap.width(), pixmap.height());

  // 将 QPixmap 设置为鼠标样式
  eraser_cursor_ = QCursor(pixmap, pixmap.width() / 2, pixmap.height() / 2);
  view_ptr_->setCursor(eraser_cursor_);
}

void SceneManager::blindNavGoalWidget(Display::VirtualDisplay *display) {
  QPointF view_pos = view_ptr_->mapFromScene(display->scenePos());
  std::string name = display->GetDisplayName();
  auto point_info = topology_map_.GetPoint(name);
  nav_goal_widget_->move(QPoint(view_pos.x(), view_pos.y()));
  nav_goal_widget_->show();
  nav_goal_widget_->SetPose(NavGoalWidget::PointInfo{
      .pose = point_info.ToRobotPose(),
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
    Display::VirtualDisplay *display, bool is_move) {
  auto pose = display_manager_->scenePoseToWord(
      curr_handle_display_->GetCurrentScenePose());
  //如果点位没有移动 则从拓扑地图中读取
  if (!is_move) {
    std::string name = display->GetDisplayName();
    auto point_info = topology_map_.GetPoint(name);
    pose = point_info.ToRobotPose();
  }
  QPointF view_pos = view_ptr_->mapFromScene(display->scenePos());
  nav_goal_widget_->move(QPoint(view_pos.x(), view_pos.y()));
  nav_goal_widget_->show();
  nav_goal_widget_->SetPose(NavGoalWidget::PointInfo{
      .pose = pose,
      .name = QString::fromStdString(curr_handle_display_->GetDisplayName())});
}
void SceneManager::eraseScenePointRange(const QPointF &pose, double range) {
  auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
  QPointF pose_map = map_ptr->mapFromScene(pose);
  map_ptr->EraseMapRange(pose_map, range);
}
void SceneManager::drawPoint(const QPointF &pose) {
  auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
  QPointF pose_map = map_ptr->mapFromScene(pose);
  map_ptr->DrawPoint(pose_map);
}
void SceneManager::SaveTopologyMap(const std::string &file_path) {
  Config::ConfigManager::Instacnce()->WriteTopologyMap(
      file_path + ".topology",
      topology_map_);
  saveTopologyMap();
}
SceneManager::~SceneManager() {}
}  // namespace Display