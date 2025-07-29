#include "display/manager/scene_manager.h"
#include <QDebug>
#include <QKeyEvent>
#include <QTimer>
#include <iostream>
#include "config/config_manager.h"
#include "display/display_occ_map.h"
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
#include "display/point_shape.h"
#include "display/virtual_display.h"
#include "logger/logger.h"
namespace Display {
SceneManager::SceneManager(QObject *parent) : QGraphicsScene(parent), current_mode_(MapEditMode::kStopEdit) {}
void SceneManager::Init(QGraphicsView *view_ptr, DisplayManager *manager) {

  display_manager_ = manager;
  view_ptr_ = view_ptr;
  set_nav_pose_widget_ = new SetPoseWidget(view_ptr_);
  set_nav_pose_widget_->hide();
  nav_goal_widget_ = new NavGoalWidget(view_ptr_);
  nav_goal_widget_->hide();
  topology_route_widget_ = new TopologyRouteWidget(view_ptr_);
  topology_route_widget_->SetSupportControllers(topology_map_.map_property.support_controllers);
  topology_route_widget_->hide();
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
  
  // 启用场景自动更新，这样TopologyLine的advance方法会被自动调用
  QTimer *timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &SceneManager::advance);
  timer->start(16); // 约60FPS更新
}
void SceneManager::LoadTopologyMap() {
  OpenTopologyMap(Config::ConfigManager::Instacnce()
                      ->GetRootConfig()
                      .topology_map_config.map_name);
}
void SceneManager::OpenTopologyMap(const std::string &file_path) {
  // 先保存当前拓扑地图中的点名称列表，用于后续删除
  std::vector<std::string> old_point_names;
  for (const auto &point : topology_map_.points) {
    old_point_names.push_back(point.name);
  }
  
  // 清理现有的拓扑连线
  for (auto line : topology_lines_) {
    removeItem(line);
    delete line;
  }
  topology_lines_.clear();
  selected_topology_line_ = nullptr;
  
  // 读取新的拓扑地图数据
  TopologyMap new_topology_map;
  if (!Config::ConfigManager::Instacnce()->ReadTopologyMap(file_path, new_topology_map)) {
    LOG_ERROR("Failed to read topology map from: " << file_path);
    return;
  }
  
  // 删除原有的显示对象
  for (const auto &point_name : old_point_names) {
    auto display = FactoryDisplay::Instance()->GetDisplay(point_name);
    if (display != nullptr) {
      FactoryDisplay::Instance()->RemoveDisplay(display);
      delete display;
    }
  }
  
  // 清空当前拓扑地图并更新为新数据
  topology_map_ = new_topology_map;
  topology_route_widget_->SetSupportControllers(topology_map_.map_property.support_controllers);
  
  // 为每个点创建显示对象
  for (auto &point : topology_map_.points) {
    auto goal_point = new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL,
                                   point.name, 8, DISPLAY_MAP);
    
    goal_point->SetRotateEnable(true)->SetMoveEnable(false)->setVisible(true);
    
    // 使用统一的坐标转换：世界坐标 -> 地图坐标
    auto robot_pose = point.ToRobotPose();
    auto map_pose = display_manager_->wordPose2Map(robot_pose);
    goal_point->UpdateDisplay(map_pose);
    
    
    LOG_INFO("Load Point: " << point.name << " at world pose(" 
             << robot_pose.x << ", " << robot_pose.y << ", " << robot_pose.theta 
             << ") -> map pose(" << map_pose.x << ", " << map_pose.y << ", " << map_pose.theta << ")");
  }
  
  // 加载拓扑路径
  loadTopologyRoutes();
  
  LOG_INFO("Load Topology Map Success! Total points: " << topology_map_.points.size() 
           << ", Total routes: " << topology_map_.routes.size());
  emit signalTopologyMapUpdate(topology_map_);
}
void SceneManager::SetEditMapMode(MapEditMode mode) {
  current_mode_ = mode;
  
  // 清理拓扑连接状态
  first_selected_point_.clear();
  is_linking_mode_ = false;
  is_drawing_line_ = false;
  clearTopologyLineSelection();
  
  // 清除预览线段
  if (preview_line_) {
    removeItem(preview_line_);
    delete preview_line_;
    preview_line_ = nullptr;
  }
  
  switch (mode) {
    case kStopEdit: {
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
    case kMoveCursor: {
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
    case kLinkTopology: {
      SetPointMoveEnable(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP)->SetMoveEnable(false);
      view_ptr_->setCursor(Qt::CrossCursor);
      is_linking_mode_ = true;
      LOG_INFO("进入拓扑连接模式，点击两个点位进行连接，拖拽可实时预览");
    } break;
    default:
      break;
  }
  LOG_INFO("set edit mode:" << mode)
}
void SceneManager::SetPointMoveEnable(bool is_enable) {
  nav_goal_widget_->SetEditMode(is_enable);
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
  LOG_INFO("Save topology map with " << topology_map_.points.size() << " points, " 
           << topology_map_.routes.size() << " routes");
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
  QGraphicsItem *click_item = itemAt(position, views()[0]->transform());
  Display::VirtualDisplay *display = dynamic_cast<Display::VirtualDisplay *>(click_item);

  if (display != nullptr) {                          // 判断是否获取到了 item
    curr_handle_display_ = display;
    //点击在地图上，隐藏窗体
    if(display->GetDisplayType() == DISPLAY_MAP || display->GetDisplayType() == DISPLAY_LOCAL_COST_MAP || display->GetDisplayType() == DISPLAY_GLOBAL_COST_MAP) {
      if (topology_route_widget_->isVisible()) {
        topology_route_widget_->hide();
      }
      if(nav_goal_widget_->isVisible()) {
        nav_goal_widget_->hide();
      }
    }
  }

  switch (current_mode_) {
    case MapEditMode::kStopEdit: 
    case MapEditMode::kMoveCursor: {
    } break;
    case MapEditMode::kDrawLine: {
      auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
      map_ptr->StartDrawLine(map_ptr->mapFromScene(position));
    } break;
    case MapEditMode::kAddPoint: {
      std::string name = generatePointName("NAV_POINT");
      auto goal_point = new PointShape(PointShape::ePointType::kNavGoal,
                                      DISPLAY_GOAL, name, 8, DISPLAY_MAP);
      goal_point->SetRotateEnable(true)->SetMoveEnable(true)->setVisible(true);
      
      // 统一的坐标转换：场景坐标 -> 世界坐标 -> 地图坐标
      auto scene_pose = basic::RobotPose(position.x(), position.y(), 0);
      auto world_pose = display_manager_->scenePoseToWord(scene_pose);
      auto map_pose = display_manager_->wordPose2Map(world_pose);
      
      goal_point->UpdateDisplay(map_pose);
      topology_map_.AddPoint(TopologyMap::PointInfo(world_pose, name));

      
      LOG_INFO("Add nav point: " << name << " at scene pose(" 
               << scene_pose.x << ", " << scene_pose.y << ", " << scene_pose.theta 
               << ") -> world pose(" << world_pose.x << ", " << world_pose.y << ", " << world_pose.theta
               << ") -> map pose(" << map_pose.x << ", " << map_pose.y << ", " << map_pose.theta << ")");
      LOG_INFO("Total points: " << topology_map_.points.size());
      curr_handle_display_ = goal_point;
    } break;
    case MapEditMode::kErase: {
      eraseScenePointRange(position, 3);
    } break;
    case MapEditMode::kDrawWithPen: {
      drawPoint(position);
    } break;
    case MapEditMode::kLinkTopology: {
      // 拓扑连接模式特殊处理
      if (click_item != nullptr) {
        Display::VirtualDisplay *display = dynamic_cast<Display::VirtualDisplay *>(click_item);
        // LOG_INFO("点击到item: " << display->GetDisplayName());
        if (display && display->GetDisplayType() == DISPLAY_GOAL) {
          LOG_INFO("点击到点位: " << display->GetDisplayName());
          handleTopologyLinking(QString::fromStdString(display->GetDisplayName()));
          return; // 直接返回，不执行后面的通用处理
        }
      }
      clearTopologyLineSelection(); // 点击空白处清除选择

      return;
    } break;
    default:
      break;
  }

  //控件点击事件
  
  if (display != nullptr && display->GetDisplayType() == DISPLAY_TOPOLINE) {
    // 点击到拓扑连线
    clearTopologyLineSelection();
    TopologyLine* line = dynamic_cast<TopologyLine*>(click_item);
    line->SetSelected(true);
    selected_topology_line_ = line;
    LOG_INFO("选中拓扑连线: " << line->GetDisplayName());
    
    // 弹出拓扑路径属性窗体
    blindTopologyRouteWidget(line,current_mode_ != MapEditMode::kStopEdit);
    return;
  }else if(display != nullptr && display->GetDisplayType() == DISPLAY_GOAL) {
    // 窗体初始化
    blindNavGoalWidget(display,current_mode_ != MapEditMode::kStopEdit);
    emit signalCurrentSelectPointChanged(
        TopologyMap::PointInfo(TopologyMap::PointInfo(
            display_manager_->scenePoseToWord(
                basic::RobotPose(position.x(), position.y(), 0)),
            display->GetDisplayName())));
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
    case MapEditMode::kMoveCursor: {
      if (curr_handle_display_ != nullptr) {
        std::string display_type = curr_handle_display_->GetDisplayType();
        if (display_type == DISPLAY_GOAL) {
          auto scene_pose = curr_handle_display_->GetCurrentScenePose();
          auto world_pose = display_manager_->scenePoseToWord(scene_pose);
          auto point_name = curr_handle_display_->GetDisplayName();
          
          // 更新拓扑地图中的点坐标
          topology_map_.UpdatePoint(point_name, TopologyMap::PointInfo(world_pose, point_name));
          
          // 线段会自动跟随点位移动，不需要手动更新
          
          LOG_INFO("Update point: " << point_name << " to scene pose(" 
                   << scene_pose.x << ", " << scene_pose.y << ", " << scene_pose.theta 
                   << ") -> world pose(" << world_pose.x << ", " << world_pose.y << ", " << world_pose.theta << ")");
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
    case MapEditMode::kStopEdit: {
      if (curr_handle_display_ != nullptr) {
        std::string display_type = curr_handle_display_->GetDisplayType();
        if (display_type == DISPLAY_GOAL) {
          updateNavGoalWidgetPose(curr_handle_display_, false);
        }
      }
    } break;
    case MapEditMode::kMoveCursor: {
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
    case MapEditMode::kLinkTopology: {
      if (is_drawing_line_ && preview_line_) {
        preview_line_->SetPreviewEndPos(position);
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

void SceneManager::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) {
    if (current_mode_ == kLinkTopology && selected_topology_line_ != nullptr) {
      deleteSelectedTopologyLine();
    }
  }
  QGraphicsScene::keyPressEvent(event);
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

void SceneManager::blindNavGoalWidget(Display::VirtualDisplay *display, bool is_edit) {
  QPointF view_pos = view_ptr_->mapFromScene(display->scenePos());
  std::string name = display->GetDisplayName();
  auto point_info = topology_map_.GetPoint(name);
  LOG_INFO("blind nav goal widget display name:" << name <<" world pose:" << point_info.ToRobotPose());
  //先断开上一个点位的信号链接
  nav_goal_widget_->disconnect();
  nav_goal_widget_->move(QPoint(view_pos.x()+10, view_pos.y()+10));
  nav_goal_widget_->SetPose(NavGoalWidget::PointInfo{
    .pose = point_info.ToRobotPose(),
    .name = QString::fromStdString(display->GetDisplayName())});
  nav_goal_widget_->SetEditMode(is_edit);
  nav_goal_widget_->show();

  connect(nav_goal_widget_, &NavGoalWidget::SignalPointNameChanged,
          [this, display](const QString &new_name) {
            std::string old_name = display->GetDisplayName();
            // 先更新拓扑地图中的点名称
            topology_map_.UpdatePointName(old_name, new_name.toStdString());
            // 使用安全的方法更新显示对象名称和映射表
            if (!FactoryDisplay::Instance()->UpdateDisplayName(old_name, new_name.toStdString())) {
              LOG_ERROR("Failed to update display name from " << old_name << " to " << new_name.toStdString());
              return;
            }
            LOG_INFO("Successfully updated point name: " << old_name << " -> " << new_name.toStdString());
          });

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
              
              // 删除相关的拓扑连线
              std::string point_name = display->GetDisplayName();
              auto it = topology_lines_.begin();
              while (it != topology_lines_.end()) {
                TopologyLine* line = *it;
                // 获取起点和终点的显示名称
                VirtualDisplay* from_display = dynamic_cast<VirtualDisplay*>(line->GetFromItem());
                VirtualDisplay* to_display = dynamic_cast<VirtualDisplay*>(line->GetToItem());
                
                bool should_remove = false;
                if (from_display && from_display->GetDisplayName() == point_name) {
                  should_remove = true;
                }
                if (to_display && to_display->GetDisplayName() == point_name) {
                  should_remove = true;
                }
                
                if (should_remove) {
                  removeItem(line);
                  delete line;
                  it = topology_lines_.erase(it);
                } else {
                  ++it;
                }
              }
              
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
            // 更新显示位置：世界坐标 -> 地图坐标
            auto map_pose = display_manager_->wordPose2Map(pose);
            display->UpdateDisplay(map_pose);
            
            // 同步更新拓扑地图中的点坐标
            topology_map_.UpdatePoint(display->GetDisplayName(), TopologyMap::PointInfo(pose, display->GetDisplayName()));
            
            // 线段会自动跟随点位移动，不需要手动更新
            
            LOG_INFO("Widget update point: " << display->GetDisplayName() << " to world pose(" 
                     << pose.x << ", " << pose.y << ", " << pose.theta 
                     << ") -> map pose(" << map_pose.x << ", " << map_pose.y << ", " << map_pose.theta << ")");
          });
}

void SceneManager::blindTopologyRouteWidget(TopologyLine* line, bool is_edit) {
  if (!line) return;
  
  // 获取路径信息
  std::string route_id = line->GetDisplayName();
  auto route_info = topology_map_.GetRouteInfo(route_id);
  
  // 计算窗体位置（在线段中点附近）
  QGraphicsItem* from_item = line->GetFromItem();
  QGraphicsItem* to_item = line->GetToItem();
  if (!from_item || !to_item) return;
  
  QPointF from_pos = from_item->scenePos();
  QPointF to_pos = to_item->scenePos();
  QPointF mid_pos = (from_pos + to_pos) / 2.0;
  QPointF view_pos = view_ptr_->mapFromScene(mid_pos);
  
  // 先断开上一个路径的信号链接
  topology_route_widget_->disconnect();
  topology_route_widget_->move(QPoint(view_pos.x()+10, view_pos.y()+10));
  
  // 设置路径信息
  TopologyRouteWidget::RouteInfo info;
  info.route_name = QString::fromStdString(route_id);
  info.controller = route_info.controller;
  topology_route_widget_->SetRouteInfo(info);
  topology_route_widget_->SetEditMode(is_edit);
  topology_route_widget_->show();
  
  // 连接信号槽
  connect(topology_route_widget_, &TopologyRouteWidget::SignalRouteInfoChanged,
          [this, line](const TopologyRouteWidget::RouteInfo &info) {
            std::string route_id = info.route_name.toStdString();
            
            // 更新拓扑地图中的路径属性
            TopologyMap::RouteInfo route_info;
            route_info.controller = info.controller;
            topology_map_.SetRouteInfo(route_id, route_info);
            
            LOG_INFO("更新路径属性: " << route_id 
                      << " 控制器: " << info.controller);
          });
  
  connect(topology_route_widget_, &TopologyRouteWidget::SignalHandleOver,
          [this, line](const TopologyRouteWidget::HandleResult &flag,
                       const TopologyRouteWidget::RouteInfo &info) {
            if (flag == TopologyRouteWidget::HandleResult::kDelete) {
              LOG_INFO("删除路径: " << info.route_name.toStdString());
              
              // 删除选中的拓扑连线
              deleteSelectedTopologyLine();
              
              topology_route_widget_->hide();
            } else {
              topology_route_widget_->hide();
            }
          });
}

void SceneManager::updateNavGoalWidgetPose(
    Display::VirtualDisplay *display, bool is_move) {
  auto pose = display_manager_->scenePoseToWord(
      display->GetCurrentScenePose());
  //如果点位没有移动 则从拓扑地图中读取
  if (!is_move) {
    std::string name = display->GetDisplayName();
    auto point_info = topology_map_.GetPoint(name);
    pose = point_info.ToRobotPose();
  }
  QPointF view_pos = view_ptr_->mapFromScene(display->scenePos());
  nav_goal_widget_->move(QPoint(view_pos.x()+10, view_pos.y()+10));
  nav_goal_widget_->show();
  nav_goal_widget_->SetPose(NavGoalWidget::PointInfo{
      .pose = pose,
      .name = QString::fromStdString(display->GetDisplayName())});
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

// 拓扑连接相关方法实现
void SceneManager::handleTopologyLinking(const QString &point_name) {
  if (first_selected_point_.isEmpty()) {
    // 选择第一个点
    first_selected_point_ = point_name;
    is_drawing_line_ = true;
    
    // 获取第一个点的位置
    auto first_display = FactoryDisplay::Instance()->GetDisplay(first_selected_point_.toStdString());
    if (first_display) {
      first_point_pos_ = first_display->scenePos();
      
      // 创建预览线段
      preview_line_ = new TopologyLine(first_display, nullptr, "preview");
      preview_line_->SetPreviewMode(true);
      preview_line_->SetPreviewEndPos(first_point_pos_);
      addItem(preview_line_);
    }
    
    LOG_INFO("选择起点: " << point_name.toStdString() << " 开始拉线");
  } else if (first_selected_point_ == point_name) {
    // 点击同一个点，取消选择
    first_selected_point_.clear();
    is_drawing_line_ = false;
    
    // 清除预览线段
    if (preview_line_) {
      removeItem(preview_line_);
      delete preview_line_;
      preview_line_ = nullptr;
    }
    
    LOG_INFO("取消选择");
  } else {
    // 选择第二个点，创建连接
    QString second_point = point_name;
    LOG_INFO("选择终点: " << second_point.toStdString());
    
    // 检查是否已存在连接
    if (topology_map_.HasRoute(first_selected_point_.toStdString(), second_point.toStdString())) {
      LOG_WARN("连接已存在: " << first_selected_point_.toStdString() << " -> " << second_point.toStdString());
    } else {
      // 直接创建单向连接
      createTopologyLine(first_selected_point_, second_point);
      LOG_INFO("创建单向连接: " << first_selected_point_.toStdString() << " -> " << second_point.toStdString());
    }
    
    // 清理状态
    first_selected_point_.clear();
    is_drawing_line_ = false;
    
    // 清除预览线段
    if (preview_line_) {
      removeItem(preview_line_);
      delete preview_line_;
      preview_line_ = nullptr;
    }
  }
}

void SceneManager::createTopologyLine(const QString &from, const QString &to) {
  std::string route_id = generateRouteId(from, to);
  
  // 添加路径到拓扑地图
  topology_map_.AddRoute(from.toStdString(), to.toStdString());
  
  // 获取起点和终点的QGraphicsItem对象
  auto from_display = FactoryDisplay::Instance()->GetDisplay(from.toStdString());
  auto to_display = FactoryDisplay::Instance()->GetDisplay(to.toStdString());
  
  if (from_display && to_display) {
    // 创建新的TopologyLine对象
    TopologyLine *line = new TopologyLine(from_display, to_display, route_id);
    topology_lines_.push_back(line);
    
    // 添加到场景中
    addItem(line);
    
    // 检查并设置是否为双向连接的一部分
    bool is_part_of_bidirectional = topology_map_.IsBidirectional(from.toStdString(), to.toStdString());
    line->SetPartOfBidirectional(is_part_of_bidirectional);
    
    // 如果这个连接使得反向连接也变成双向，需要更新反向连接的显示
    updateAllTopologyLinesBidirectionalStatus();
    
    LOG_INFO("创建拓扑连接: " << from.toStdString() << " -> " << to.toStdString() 
             << (is_part_of_bidirectional ? " (双向)" : " (单向)"));
  } else {
    LOG_ERROR("无法找到连接点位: " << from.toStdString() << " 或 " << to.toStdString());
  }
}

void SceneManager::updateTopologyLinePositions() {
  // 线段现在会自动跟随关联的点位移动，不需要手动更新
  // 这个方法保留用于兼容性，但实际上不执行任何操作
}

void SceneManager::clearTopologyLineSelection() {
  if (selected_topology_line_) {
    selected_topology_line_->SetSelected(false);
    selected_topology_line_ = nullptr;
  }

  if(preview_line_){
    removeItem(preview_line_);
    delete preview_line_;
    preview_line_ = nullptr;
    first_selected_point_.clear();
    is_drawing_line_ = false;
  }
}

void SceneManager::deleteSelectedTopologyLine() {
  if (selected_topology_line_) {
    std::string route_id = selected_topology_line_->GetDisplayName();
    LOG_INFO("delete route_id: " << route_id);
    // 从拓扑地图中删除路径
    topology_map_.RemoveRoute(route_id);
    
    // 从场景中移除
    removeItem(selected_topology_line_);    
    // 从显示列表中删除
    auto it = std::find(topology_lines_.begin(), topology_lines_.end(), selected_topology_line_);
    if (it != topology_lines_.end()) {
      topology_lines_.erase(it);
    }
    
    LOG_INFO("删除拓扑连线: " << selected_topology_line_->GetDisplayName());
    delete selected_topology_line_;
    selected_topology_line_ = nullptr;
    
    // 删除路径后，需要更新所有连线的双向状态
    updateAllTopologyLinesBidirectionalStatus();
  }
}

std::string SceneManager::generateRouteId(const QString &from, const QString &to) {
  return from.toStdString() + "->" + to.toStdString();
}

TopologyLine* SceneManager::findTopologyLine(const QString &route_id) {
  for (auto line : topology_lines_) {
    if (line->GetDisplayName() == route_id.toStdString()) {
      return line;
    }
  }
  return nullptr;
}

void SceneManager::loadTopologyRoutes() {
  // 为现有的路径创建显示对象
  for (const auto &from_routes : topology_map_.routes) {
    const std::string &from = from_routes.first;
    for (const auto &route : from_routes.second) {
      const std::string &to = route.first;
      // 获取起点和终点的QGraphicsItem对象
      auto from_display = FactoryDisplay::Instance()->GetDisplay(from);
      auto to_display = FactoryDisplay::Instance()->GetDisplay(to);
      
      if (from_display && to_display) {
        std::string route_id = from + "->" + to;
        TopologyLine *line = new TopologyLine(from_display, to_display, route_id);
        topology_lines_.push_back(line);
        
        // 添加到场景中
        addItem(line);
        
        // 检查并设置是否为双向连接的一部分
        bool is_part_of_bidirectional = topology_map_.IsBidirectional(from, to);
        line->SetPartOfBidirectional(is_part_of_bidirectional);
        
        LOG_INFO("Load topology route: " << from << " -> " << to 
                 << (is_part_of_bidirectional ? " (双向)" : " (单向)"));
      } else {
        LOG_ERROR("无法找到连接点位: " << from << " 或 " << to);
      }
    }
  }
}

void SceneManager::updateAllTopologyLinesBidirectionalStatus() {
  // 更新所有拓扑连线的双向状态
  for (auto line : topology_lines_) {
    if (line && line->GetFromItem() && line->GetToItem()) {
      // 获取起点和终点的显示名称
      VirtualDisplay* from_display = dynamic_cast<VirtualDisplay*>(line->GetFromItem());
      VirtualDisplay* to_display = dynamic_cast<VirtualDisplay*>(line->GetToItem());
      
      if (from_display && to_display) {
        std::string from_name = from_display->GetDisplayName();
        std::string to_name = to_display->GetDisplayName();
        
        // 检查并更新是否为双向连接的一部分
        bool is_part_of_bidirectional = topology_map_.IsBidirectional(from_name, to_name);
        line->SetPartOfBidirectional(is_part_of_bidirectional);
      }
    }
  }
}

SceneManager::~SceneManager() {
  // 清理拓扑连线
  for (auto line : topology_lines_) {
    removeItem(line);
    delete line;
  }
  topology_lines_.clear();
  // 清除预览线段
  if (preview_line_) {
    removeItem(preview_line_);
    delete preview_line_;
    preview_line_ = nullptr;
  }
  // 隐藏路径属性窗体
  if (topology_route_widget_) {
    topology_route_widget_->hide();
  }
}
}  // namespace Display