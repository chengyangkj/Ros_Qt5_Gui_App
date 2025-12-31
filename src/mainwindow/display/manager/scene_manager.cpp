#include "display/manager/scene_manager.h"
#include "display/manager/map_edit_command.h"
#include "display/manager/view_manager.h"
#include <QDebug>
#include <QKeyEvent>
#include <QTimer>
#include <iostream>
#include <algorithm>
#include <map>
#include "config/config_manager.h"
#include "display/display_occ_map.h"
#include "display/manager/display_factory.h"
#include "display/manager/display_manager.h"
#include "display/point_shape.h"
#include "display/virtual_display.h"
#include "logger/logger.h"
#include <QMessageBox>

namespace Display {

SceneManager::SceneManager(QObject *parent) : QGraphicsScene(parent), current_mode_(MapEditMode::kStopEdit) {}
void SceneManager::Init(QGraphicsView *view_ptr, DisplayManager *manager) {

  display_manager_ = manager;
  view_ptr_ = view_ptr;
  set_nav_pose_widget_ = new SetPoseWidget(view_ptr_);
  set_nav_pose_widget_->hide();
  // nav_goal_widget_ 和 topology_route_widget_ 改为智能指针，每次绑定时创建新实例，此处不再初始化
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

void SceneManager::OpenTopologyMap(const std::string &file_path) {
  // 读取新的拓扑地图数据
  TopologyMap new_topology_map;
  if (!Config::ConfigManager::Instance()->ReadTopologyMap(file_path, new_topology_map)) {
    LOG_ERROR("Failed to read topology map from: " << file_path);
    return;
  }
  
  // 调用UpdateTopologyMap方法更新显示
  UpdateTopologyMap(new_topology_map);
  
  LOG_INFO("Open Topology Map Success! File: " << file_path);
}

void SceneManager::UpdateTopologyMap(const TopologyMap &topology_map) {
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
  
  // 删除原有的显示对象
  for (const auto &point_name : old_point_names) {
    auto display = FactoryDisplay::Instance()->GetDisplay(point_name);
    if (display != nullptr) {
      FactoryDisplay::Instance()->RemoveDisplay(display);
      delete display;
    }
  }
  
  // 更新拓扑地图数据
  topology_map_ = topology_map;

  // 为每个点创建显示对象
  for (auto &point : topology_map_.points) {
    auto goal_point = new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL,
                                   point.name, 8, DISPLAY_MAP);
    
    goal_point->SetRotateEnable(true)->SetMoveEnable(false)->setVisible(true);
    
    // 使用统一的坐标转换：世界坐标 -> 地图坐标
    auto robot_pose = point.ToRobotPose();
    auto map_pose = display_manager_->wordPose2Map(robot_pose);
    goal_point->UpdateData(map_pose);
    
    LOG_INFO("Update Point: " << point.name << " at world pose(" 
             << robot_pose.x << ", " << robot_pose.y << ", " << robot_pose.theta 
             << ") -> map pose(" << map_pose.x << ", " << map_pose.y << ", " << map_pose.theta << ")");
  }

  
  // 加载拓扑路径
  loadTopologyRoutes();
  
  LOG_INFO("Update Topology Map Success! Total points: " << topology_map_.points.size() 
           << ", Total routes: " << topology_map_.routes.size());
  emit signalTopologyMapUpdate(topology_map_);
}
void SceneManager::SetToolRange(double range) {
  double clamped_range = qMax(0.1, qMin(50.0, range));  // 限制范围在 0.1 到 50 米
  pen_range_ = clamped_range;
  pen_range_ = clamped_range;
  if (current_mode_ == MapEditMode::kErase) {
    setEraseCursor();
  } else if (current_mode_ == MapEditMode::kDrawWithPen) {
    setPenCursor();
  }
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
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_ROBOT)->setVisible(true);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_ROBOT_FOOTPRINT)->setVisible(true);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_GLOBAL_PATH)->setVisible(true);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_LOCAL_PATH)->setVisible(true);
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
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_ROBOT)->setVisible(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_ROBOT_FOOTPRINT)->setVisible(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_GLOBAL_PATH)->setVisible(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_LOCAL_PATH)->setVisible(false);
    } break;
    case kErase: {
      SetPointMoveEnable(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP)->SetMoveEnable(false);
      setEraseCursor();
    } break;
    case kDrawWithPen: {
      SetPointMoveEnable(false);
      FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP)->SetMoveEnable(false);
      setPenCursor();
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
  emit signalEditMapModeChanged(mode);
  LOG_INFO("set edit mode:" << mode)
}
void SceneManager::SetPointMoveEnable(bool is_enable) {
  if (nav_goal_widget_) {
    nav_goal_widget_->SetEditMode(is_enable);
  }
  for (auto point : topology_map_.points) {
    auto display = FactoryDisplay::Instance()->GetDisplay(point.name);
    if (display != nullptr) {
      display->SetMoveEnable(is_enable);
    } else {
      LOG_ERROR("not find display:" << point.name)
    }
  }
}

void SceneManager::AddOneNavPoint() {
}

void SceneManager::AddPointAtRobotPosition() {
  // 获取机器人当前位置（世界坐标）
  RobotPose robot_pose = display_manager_->GetRobotPose();
  
  // 生成点位名称
  std::string name = generatePointName("NAV_POINT");
  
  TopologyMap::PointInfo point_info(robot_pose, name);
  auto command = std::make_unique<AddPointCommand>(name, point_info);
  PushCommand(std::move(command));
  
  // 创建点位显示对象
  auto goal_point = new PointShape(PointShape::ePointType::kNavGoal,
                                  DISPLAY_GOAL, name, 8, DISPLAY_MAP);
  goal_point->SetRotateEnable(true)->SetMoveEnable(true)->setVisible(true);
  
  // 世界坐标 -> 地图坐标
  auto map_pose = display_manager_->wordPose2Map(robot_pose);
  goal_point->UpdateData(map_pose);
  
  // 添加到拓扑地图
  topology_map_.AddPoint(point_info);
  
  LOG_INFO("Add nav point at robot position: " << name << " at world pose(" 
           << robot_pose.x << ", " << robot_pose.y << ", " << robot_pose.theta 
           << ") -> map pose(" << map_pose.x << ", " << map_pose.y << ", " << map_pose.theta << ")");
  LOG_INFO("Total points: " << topology_map_.points.size());
  
  curr_handle_display_ = goal_point;
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
    //点击时，先隐藏窗体
    if (topology_route_widget_ && topology_route_widget_->isVisible()) {
      topology_route_widget_->hide();
    }
    if(nav_goal_widget_ && nav_goal_widget_->isVisible()) {
      nav_goal_widget_->hide();
    }
    
    // 如果是点位且处于移动模式，保存初始位置用于撤销
    if (current_mode_ == MapEditMode::kMoveCursor && display->GetDisplayType() == DISPLAY_GOAL) {
      std::string point_name = display->GetDisplayName();
      auto point_info = topology_map_.GetPoint(point_name);
      point_move_start_positions_[point_name] = point_info;
    }
  }

  switch (current_mode_) {
    case MapEditMode::kStopEdit: 
    case MapEditMode::kMoveCursor: {
    } break;
    case MapEditMode::kDrawLine: {
      auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
      // 开始绘制线段操作，保存整个地图的初始状态
      is_draw_line_operation_active_ = true;
      current_map_ptr_ = map_ptr;
      map_ptr->StartDrawLine(map_ptr->mapFromScene(position));
      line_start_pose_ = map_ptr->mapFromScene(position);
      // 保存整个地图的初始状态（在操作前保存），以便后续可以提取任意区域
      draw_line_operation_saved_image_ = map_ptr->GetMapImage();
    } break;
    case MapEditMode::kAddPoint: {
      std::string name = generatePointName("NAV_POINT");
      
      // 统一的坐标转换：场景坐标 -> 世界坐标 -> 地图坐标
      auto scene_pose = basic::RobotPose(position.x(), position.y(), 0);
      auto world_pose = display_manager_->scenePoseToWord(scene_pose);
      auto map_pose = display_manager_->wordPose2Map(world_pose);
      
      TopologyMap::PointInfo point_info(world_pose, name);
      auto command = std::make_unique<AddPointCommand>(name, point_info);
      PushCommand(std::move(command));
      
      auto goal_point = new PointShape(PointShape::ePointType::kNavGoal,
                                      DISPLAY_GOAL, name, 8, DISPLAY_MAP);
      goal_point->SetRotateEnable(true)->SetMoveEnable(true)->setVisible(true);
      goal_point->UpdateData(map_pose);
      topology_map_.AddPoint(point_info);

      
      LOG_INFO("Add nav point: " << name << " at scene pose(" 
               << scene_pose.x << ", " << scene_pose.y << ", " << scene_pose.theta 
               << ") -> world pose(" << world_pose.x << ", " << world_pose.y << ", " << world_pose.theta
               << ") -> map pose(" << map_pose.x << ", " << map_pose.y << ", " << map_pose.theta << ")");
      LOG_INFO("Total points: " << topology_map_.points.size());
      curr_handle_display_ = goal_point;
    } break;
    case MapEditMode::kErase: {
      auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
      QPointF pose_map = map_ptr->mapFromScene(position);
      // 开始擦除操作，记录初始区域并保存整个地图的初始状态
      is_erase_operation_active_ = true;
      current_map_ptr_ = map_ptr;
      double range = pen_range_;
      float x = pose_map.x();
      float y = pose_map.y();
      int left = qMax(0, static_cast<int>(x - range));
      int top = qMax(0, static_cast<int>(y - range));
      int right = qMin(map_ptr->GetMapImage().width() - 1, static_cast<int>(x + range));
      int bottom = qMin(map_ptr->GetMapImage().height() - 1, static_cast<int>(y + range));
      erase_operation_region_ = QRectF(left, top, right - left + 1, bottom - top + 1);
      // 保存整个地图的初始状态（在操作前保存），以便后续可以提取任意区域
      erase_operation_saved_image_ = map_ptr->GetMapImage();
      // 执行第一次擦除
      eraseScenePointRange(position, range);
    } break;
    case MapEditMode::kDrawWithPen: {
      auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
      QPointF pose_map = map_ptr->mapFromScene(position);
      // 开始绘制点操作，记录初始区域并保存整个地图的初始状态
      is_draw_point_operation_active_ = true;
      current_map_ptr_ = map_ptr;
      double range = pen_range_;
      float x = pose_map.x();
      float y = pose_map.y();
      int left = qMax(0, static_cast<int>(x - range));
      int top = qMax(0, static_cast<int>(y - range));
      int right = qMin(map_ptr->GetMapImage().width() - 1, static_cast<int>(x + range));
      int bottom = qMin(map_ptr->GetMapImage().height() - 1, static_cast<int>(y + range));
      draw_point_operation_region_ = QRectF(left, top, right - left + 1, bottom - top + 1);
      // 保存整个地图的初始状态（在操作前保存），以便后续可以提取任意区域
      draw_point_operation_saved_image_ = map_ptr->GetMapImage();
      // 执行第一次绘制
      drawScenePointRange(position, range);
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
  
  // 处理擦除操作结束
  if (is_erase_operation_active_ && current_map_ptr_) {
    // 确保区域有效
    if (!erase_operation_region_.isEmpty() && !erase_operation_saved_image_.isNull()) {
      // 从保存的整个地图初始状态中提取操作区域的初始状态
      QImage region_image = erase_operation_saved_image_.copy(erase_operation_region_.toRect());
      // 创建擦除命令，使用整个操作区域和提取的初始状态
      auto command = std::make_unique<EraseCommand>(current_map_ptr_, erase_operation_region_, region_image);
      PushCommand(std::move(command));
    }
    is_erase_operation_active_ = false;
    current_map_ptr_ = nullptr;
  }
  
  // 处理绘制点操作结束
  if (is_draw_point_operation_active_ && current_map_ptr_) {
    // 确保区域有效
    if (!draw_point_operation_region_.isEmpty() && !draw_point_operation_saved_image_.isNull()) {
      // 从保存的整个地图初始状态中提取操作区域的初始状态
      QImage region_image = draw_point_operation_saved_image_.copy(draw_point_operation_region_.toRect());
      // 创建绘制点命令，使用整个操作区域和提取的初始状态
      auto command = std::make_unique<DrawPointCommand>(current_map_ptr_, draw_point_operation_region_, region_image);
      PushCommand(std::move(command));
    }
    is_draw_point_operation_active_ = false;
    current_map_ptr_ = nullptr;
  }
  
  // 检查点位移动，创建撤销命令
  if (current_mode_ == MapEditMode::kMoveCursor && curr_handle_display_ != nullptr) {
    std::string display_type = curr_handle_display_->GetDisplayType();
    if (display_type == DISPLAY_GOAL) {
      std::string point_name = curr_handle_display_->GetDisplayName();
      if (point_move_start_positions_.find(point_name) != point_move_start_positions_.end()) {
        auto old_info = point_move_start_positions_[point_name];
        auto new_info = topology_map_.GetPoint(point_name);
        // 检查位置是否真的改变了
        auto old_pose = old_info.ToRobotPose();
        auto new_pose = new_info.ToRobotPose();
        if (old_pose.x != new_pose.x || old_pose.y != new_pose.y || old_pose.theta != new_pose.theta) {
          auto command = std::make_unique<UpdatePointCommand>(point_name, old_info, new_info);
          PushCommand(std::move(command));
        }
        point_move_start_positions_.erase(point_name);
      }
    }
  }
  
  // 处理绘制线段操作结束
  if (is_draw_line_operation_active_ && current_map_ptr_) {
    auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
    QPointF end_pose = map_ptr->mapFromScene(position);
    // 计算操作区域（根据起点和终点）
    QRectF line_rect = QRectF(line_start_pose_, end_pose).normalized();
    int padding = 2;
    QRectF operation_region = QRectF(line_rect.x() - padding, line_rect.y() - padding,
                                     line_rect.width() + 2 * padding, line_rect.height() + 2 * padding);
    // 确保区域有效
    if (!operation_region.isEmpty() && !draw_line_operation_saved_image_.isNull()) {
      // 从保存的整个地图初始状态中提取操作区域的初始状态
      QRect rect = operation_region.toRect();
      rect = rect.intersected(QRect(0, 0, draw_line_operation_saved_image_.width(), draw_line_operation_saved_image_.height()));
      if (!rect.isEmpty()) {
        QImage region_image = draw_line_operation_saved_image_.copy(rect);
        QRectF region_rect(rect);
        // 创建绘制线段命令，使用操作区域和提取的初始状态
        auto command = std::make_unique<DrawLineCommand>(map_ptr, line_start_pose_, end_pose, region_rect, region_image);
        PushCommand(std::move(command));
        // 完成绘制操作（此时图像已经是最终状态，因为 EndDrawLine 在 mouseMoveEvent 中已经执行）
        map_ptr->EndDrawLine(end_pose, true);
      } else {
        // 如果区域无效，直接完成绘制
        map_ptr->EndDrawLine(end_pose, true);
      }
    } else {
      // 如果区域无效，直接完成绘制
      map_ptr->EndDrawLine(end_pose, true);
    }
    is_draw_line_operation_active_ = false;
    current_map_ptr_ = nullptr;
  }
  
  switch (current_mode_) {
    default:
      break;
  }

}  // namespace Display
void SceneManager::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) {
  QPointF position = mouseEvent->scenePos();  // 获取点击位置
  //点位属性框跟随移动处理
  switch (current_mode_) {
    case MapEditMode::kStopEdit: {
    } break;
    case MapEditMode::kMoveCursor: {
      if ((left_pressed_ || right_pressed_) && curr_handle_display_ != nullptr) {
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
      if (left_pressed_ && is_erase_operation_active_ && current_map_ptr_) {
        auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
        QPointF pose_map = map_ptr->mapFromScene(position);
        double range = pen_range_;
        float x = pose_map.x();
        float y = pose_map.y();
        int left = qMax(0, static_cast<int>(x - range));
        int top = qMax(0, static_cast<int>(y - range));
        int right = qMin(map_ptr->GetMapImage().width() - 1, static_cast<int>(x + range));
        int bottom = qMin(map_ptr->GetMapImage().height() - 1, static_cast<int>(y + range));
        QRectF new_region(left, top, right - left + 1, bottom - top + 1);
        // 扩展操作区域（不重新保存图像，因为图像已经被修改了）
        erase_operation_region_ = erase_operation_region_.united(new_region);
        eraseScenePointRange(position, range);
      }
    } break;
    case MapEditMode::kDrawWithPen: {
      if (left_pressed_ && is_draw_point_operation_active_ && current_map_ptr_) {
        auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
        QPointF pose_map = map_ptr->mapFromScene(position);
        double range = pen_range_;
        float x = pose_map.x();
        float y = pose_map.y();
        int left = qMax(0, static_cast<int>(x - range));
        int top = qMax(0, static_cast<int>(y - range));
        int right = qMin(map_ptr->GetMapImage().width() - 1, static_cast<int>(x + range));
        int bottom = qMin(map_ptr->GetMapImage().height() - 1, static_cast<int>(y + range));
        QRectF new_region(left, top, right - left + 1, bottom - top + 1);
        // 扩展操作区域（不重新保存图像，因为图像已经被修改了）
        draw_point_operation_region_ = draw_point_operation_region_.united(new_region);
        drawScenePointRange(position, range);
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
    case kDrawWithPen: {
      setPenCursor();
    } break;
  }
  QGraphicsScene::wheelEvent(event);
}

void SceneManager::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) {
    if (current_mode_ == kLinkTopology && selected_topology_line_ != nullptr) {
      deleteSelectedTopologyLine();
    } else if (selected_topology_line_ != nullptr) {
      deleteSelectedTopologyLine();
    }
  } else if (event->key() == Qt::Key_Z && event->modifiers() & Qt::ControlModifier) {
    Undo();
    event->accept();
    return;
  }
  QGraphicsScene::keyPressEvent(event);
}

void SceneManager::setEraseCursor() {
  auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
  double scale_value = map_ptr->GetScaleValue();
  QPixmap pixmap(pen_range_ * 2 * scale_value, pen_range_ * 2 * scale_value);
  // 使用 QPainter 绘制一个红色正方形
  pixmap.fill(Qt::transparent);
  QPainter painter(&pixmap);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(255, 0, 0, 50));
  painter.drawRect(0, 0, pixmap.width(), pixmap.height());

  // 将 QPixmap 设置为鼠标样式
  eraser_cursor_ = QCursor(pixmap, pixmap.width() / 2, pixmap.height() / 2);
  view_ptr_->setCursor(eraser_cursor_);
}

void SceneManager::setPenCursor() {
  auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
  double scale_value = map_ptr->GetScaleValue();
  QPixmap pixmap(pen_range_ * 2 * scale_value, pen_range_ * 2 * scale_value);
  // 使用 QPainter 绘制一个蓝色正方形
  pixmap.fill(Qt::transparent);
  QPainter painter(&pixmap);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(0, 0, 255, 50));
  painter.drawRect(0, 0, pixmap.width(), pixmap.height());

  // 将 QPixmap 设置为鼠标样式
  pen_cursor_ = QCursor(pixmap, pixmap.width() / 2, pixmap.height() / 2);
  view_ptr_->setCursor(pen_cursor_);
}

void SceneManager::blindNavGoalWidget(Display::VirtualDisplay *display, bool is_edit) {
  if (!display) {
    LOG_ERROR("blindNavGoalWidget: display is nullptr");
    return;
  }
  
  // 清理旧的 widget：先隐藏并断开所有连接，然后销毁
  if (nav_goal_widget_) {
    nav_goal_widget_->hide();
    nav_goal_widget_->disconnect();  // 断开所有信号槽连接
    nav_goal_widget_.reset();  // 显式销毁旧实例
  }
  
  QPointF view_pos = view_ptr_->mapFromScene(display->scenePos());
  std::string name = display->GetDisplayName();
  auto point_info = topology_map_.GetPoint(name);
  LOG_INFO("blind nav goal widget display name:" << name <<" world pose:" << point_info.ToRobotPose());
  
  // 创建新的 widget 实例（每次都是全新的，确保没有历史数据残留）
  nav_goal_widget_ = std::make_unique<NavGoalWidget>(view_ptr_);
  
  // 保存初始位置用于撤销（如果处于编辑模式）
  if (is_edit) {
    point_move_start_positions_[name] = point_info;
  }
  
  // 设置新的点位数据
  nav_goal_widget_->SetPose(NavGoalWidget::PointInfo{
    .pose = point_info.ToRobotPose(),
    .name = QString::fromStdString(name)});
  nav_goal_widget_->SetEditMode(is_edit);
  nav_goal_widget_->move(QPoint(view_pos.x()+10, view_pos.y()+10));
  nav_goal_widget_->show();


  // 使用原始指针进行连接，避免 lambda 中访问已销毁的智能指针
  NavGoalWidget* widget_ptr = nav_goal_widget_.get();
  
  connect(widget_ptr, &NavGoalWidget::SignalHandleOver,
          [this, display, widget_ptr](const NavGoalWidget::HandleResult &flag,
                          const RobotPose &pose,const QString &new_name) {
            // 安全检查：确保 widget 仍然有效且是当前活跃的 widget
            if (!nav_goal_widget_ || nav_goal_widget_.get() != widget_ptr) {
              LOG_WARN("NavGoalWidget was replaced or destroyed, ignoring signal");
              return;
            }
            // 检查 display 是否仍然有效（防止点位被删除后访问）
            if (!display ) {
              LOG_WARN("Display was deleted or changed, ignoring signal");
              return;
            }

            std::string point_name = display->GetDisplayName();  

            
            if (flag == NavGoalWidget::HandleResult::kSend) {
              // 检查点位坐标是否被修改，如果是则创建撤销命令
              std::string point_name = display->GetDisplayName();
              if (point_move_start_positions_.find(point_name) != point_move_start_positions_.end()) {
                auto old_info = point_move_start_positions_[point_name];
                auto new_info = topology_map_.GetPoint(point_name);
                auto old_pose = old_info.ToRobotPose();
                auto new_pose = new_info.ToRobotPose();
                if (old_pose.x != new_pose.x || old_pose.y != new_pose.y || old_pose.theta != new_pose.theta) {
                  auto command = std::make_unique<UpdatePointCommand>(point_name, old_info, new_info);
                  PushCommand(std::move(command));
                }
                point_move_start_positions_.erase(point_name);
              }
              
              emit display_manager_->signalPub2DGoal(pose);
              nav_goal_widget_->hide();
              curr_handle_display_ = nullptr;
            } else if (flag == NavGoalWidget::HandleResult::kRemove) {
              LOG_INFO("remove:" << point_name);
              
              // 收集相关的拓扑连线
              std::vector<std::string> related_routes;
              auto it = topology_lines_.begin();
              while (it != topology_lines_.end()) {
                TopologyLine* line = *it;
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
                  related_routes.push_back(line->GetDisplayName());
                  removeItem(line);
                  delete line;
                  it = topology_lines_.erase(it);
                } else {
                  ++it;
                }
              }
              
              // 创建删除命令
              auto point_info = topology_map_.GetPoint(point_name);
              auto command = std::make_unique<RemovePointCommand>(point_name, point_info, related_routes);
              PushCommand(std::move(command));
              
              topology_map_.RemovePoint(point_name);
              curr_handle_display_ = nullptr;
              FactoryDisplay::Instance()->RemoveDisplay(display);
              nav_goal_widget_->disconnect();
              delete display;
              nav_goal_widget_->hide();

            } else if (flag == NavGoalWidget::HandleResult::kChangeName) {
              std::string old_name = point_name;  // 使用保存的点位名称
              auto iter=std::find_if(topology_map_.points.begin(), topology_map_.points.end(), [new_name](const TopologyMap::PointInfo &point) {
                return point.name == new_name.toStdString();
              });
  
              if (iter != topology_map_.points.end()) {
                LOG_ERROR("Point already exists: " << new_name.toStdString());
                QMessageBox::warning(nullptr, "失败", "点位名称已存在");
                return;
              }
  
              // 先更新拓扑地图中的点名称
              topology_map_.UpdatePointName(old_name, new_name.toStdString());

              // 使用安全的方法更新显示对象名称和映射表
              if (!FactoryDisplay::Instance()->UpdateDisplayName(old_name, new_name.toStdString())) {
                LOG_ERROR("Failed to update display name from " << old_name << " to " << new_name.toStdString());
                return;
              }

              //更新拓扑路径
              for(auto line : topology_lines_) {
                if(line->GetFromItem() == display || line->GetToItem() == display) {
                  std::string from_name = static_cast<VirtualDisplay*>(line->GetFromItem())->GetDisplayName();
                  std::string to_name = static_cast<VirtualDisplay*>(line->GetToItem())->GetDisplayName();
                  std::string new_route_name = generateRouteId(QString::fromStdString(from_name), QString::fromStdString(to_name));
                  FactoryDisplay::Instance()->UpdateDisplayName(line->GetDisplayName(), new_route_name);
                }
              }
       
              nav_goal_widget_->hide();
              LOG_INFO("Successfully updated point name: " << old_name << " -> " << new_name.toStdString());
            } else if (flag == NavGoalWidget::HandleResult::kCancel) {
              // 取消时，如果点位被修改过，恢复原位置
              std::string point_name = display->GetDisplayName();
              if (point_move_start_positions_.find(point_name) != point_move_start_positions_.end()) {
                auto old_info = point_move_start_positions_[point_name];
                auto old_pose = old_info.ToRobotPose();
                auto map_pose = display_manager_->wordPose2Map(old_pose);
                auto* point_shape = dynamic_cast<PointShape*>(display);
                if (point_shape) {
                  point_shape->UpdateData(map_pose);
                }
                topology_map_.UpdatePoint(point_name, old_info);
                point_move_start_positions_.erase(point_name);
              }
              
              curr_handle_display_ = nullptr;
              nav_goal_widget_->hide();
            } else {
              curr_handle_display_ = nullptr;
              nav_goal_widget_->hide();
            }
          });
          
  connect(widget_ptr, &NavGoalWidget::SignalPoseChanged,
          [this, display, widget_ptr](const RobotPose &pose) {
            // 安全检查：确保 widget 和 display 仍然有效
            if (!nav_goal_widget_ || nav_goal_widget_.get() != widget_ptr) {
              LOG_WARN("NavGoalWidget was replaced or destroyed, ignoring pose changed signal");
              return;
            }
            if (!display) {
              LOG_WARN("Display was deleted or changed, ignoring pose changed signal");
              return;
            }
            std::string point_name = display->GetDisplayName();
            
            // 如果是第一次修改，保存旧位置用于撤销
            if (point_move_start_positions_.find(point_name) == point_move_start_positions_.end()) {
              auto old_info = topology_map_.GetPoint(point_name);
              point_move_start_positions_[point_name] = old_info;
            }

            // 更新显示位置：世界坐标 -> 地图坐标
            auto map_pose = display_manager_->wordPose2Map(pose);
            auto* point_shape = dynamic_cast<PointShape*>(display);
            if (point_shape) {
              point_shape->UpdateData(map_pose);
            }
            
            // 同步更新拓扑地图中的点坐标
            topology_map_.UpdatePoint(point_name, TopologyMap::PointInfo(pose, point_name));
            
            // 线段会自动跟随点位移动，不需要手动更新
            
            LOG_INFO("Widget update point: " << point_name << " to world pose(" 
                     << pose.x << ", " << pose.y << ", " << pose.theta 
                     << ") -> map pose(" << map_pose.x << ", " << map_pose.y << ", " << map_pose.theta << ")");
            LOG_INFO("After update point: " << point_name << " topology map world pose: " << nlohmann::json(topology_map_.GetPoint(point_name)).dump());
          });
}

void SceneManager::blindTopologyRouteWidget(TopologyLine* line, bool is_edit) {
  if (!line) {
    LOG_ERROR("blindTopologyRouteWidget: line is nullptr");
    return;
  }
  
  // 清理旧的 widget：先隐藏并断开所有连接，然后销毁
  if (topology_route_widget_) {
    topology_route_widget_->hide();
    topology_route_widget_->disconnect();  // 断开所有信号槽连接
    topology_route_widget_.reset();  // 显式销毁旧实例
  }
  
  // 获取路径信息
  std::string route_id = line->GetDisplayName();
  auto route_info = topology_map_.GetRouteInfo(route_id);
  
  // 计算窗体位置（在线段中点附近）
  QGraphicsItem* from_item = line->GetFromItem();
  QGraphicsItem* to_item = line->GetToItem();
  if (!from_item || !to_item) {
    LOG_ERROR("blindTopologyRouteWidget: from_item or to_item is nullptr");
    return;
  }
  
  QPointF from_pos = from_item->scenePos();
  QPointF to_pos = to_item->scenePos();
  QPointF mid_pos = (from_pos + to_pos) / 2.0;
  QPointF view_pos = view_ptr_->mapFromScene(mid_pos);
  
  // 创建新的 widget 实例（每次都是全新的，确保没有历史数据残留）
  topology_route_widget_ = std::make_unique<TopologyRouteWidget>(view_ptr_);
  
  // 设置支持列表
  topology_route_widget_->SetSupportControllers(topology_map_.map_property.support_controllers);
  topology_route_widget_->SetSupportGoalCheckers(topology_map_.map_property.support_goal_checkers);
  
  LOG_INFO("blindTopologyRouteWidget: route_id:" << route_id << " route_info:" << nlohmann::json(route_info).dump());
  // 设置路径信息
  TopologyRouteWidget::RouteInfo info;
  info.route_name = QString::fromStdString(route_id);
  info.controller = route_info.controller;
  info.speed_limit = route_info.speed_limit;
  info.goal_checker = route_info.goal_checker;
  topology_route_widget_->SetRouteInfo(info);
  topology_route_widget_->SetEditMode(is_edit);
  topology_route_widget_->move(QPoint(view_pos.x()+10, view_pos.y()+10));
  topology_route_widget_->show();
  
  // 使用原始指针进行连接，避免 lambda 中访问已销毁的智能指针
  TopologyRouteWidget* widget_ptr = topology_route_widget_.get();
  std::string saved_route_id = route_id;  // 保存路径ID，避免 line 被删除后访问无效
  
  connect(widget_ptr, &TopologyRouteWidget::SignalRouteInfoChanged,
          [this, widget_ptr, saved_route_id](const TopologyRouteWidget::RouteInfo &info) {
            // 安全检查：确保 widget 仍然有效且是当前活跃的 widget
            if (!topology_route_widget_ || topology_route_widget_.get() != widget_ptr) {
              LOG_WARN("TopologyRouteWidget was replaced or destroyed, ignoring route info changed signal");
              return;
            }
            
            std::string route_id = info.route_name.toStdString();
            
            // 验证路径ID是否匹配（防止切换到其他路径后处理旧数据）
            if (route_id != saved_route_id) {
              LOG_WARN("Route ID mismatch: expected " << saved_route_id << ", got " << route_id);
              return;
            }
            
            // 更新拓扑地图中的路径属性
            TopologyMap::RouteInfo route_info;
            route_info.controller = info.controller;
            route_info.speed_limit = info.speed_limit;
            route_info.goal_checker = info.goal_checker;
            topology_map_.SetRouteInfo(saved_route_id, route_info);
            
            LOG_INFO("更新路径属性: " << saved_route_id 
                      << " 控制器: " << info.controller
                      << " 目标检查器: " << info.goal_checker
                      << " 速度限制: " << info.speed_limit);
          });
  
  connect(widget_ptr, &TopologyRouteWidget::SignalHandleOver,
          [this, widget_ptr, saved_route_id](const TopologyRouteWidget::HandleResult &flag,
                       const TopologyRouteWidget::RouteInfo &info) {
            // 安全检查：确保 widget 仍然有效且是当前活跃的 widget
            if (!topology_route_widget_ || topology_route_widget_.get() != widget_ptr) {
              LOG_WARN("TopologyRouteWidget was replaced or destroyed, ignoring handle over signal");
              return;
            }
            
            std::string route_id = info.route_name.toStdString();
            
            // 验证路径ID是否匹配
            if (route_id != saved_route_id) {
              LOG_WARN("Route ID mismatch: expected " << saved_route_id << ", got " << route_id);
              return;
            }
            
            if (flag == TopologyRouteWidget::HandleResult::kDelete) {
              LOG_INFO("删除路径: " << saved_route_id);
              
              auto find_line = std::find_if(topology_lines_.begin(), topology_lines_.end(), [saved_route_id]( TopologyLine *line) {
                return line->GetDisplayName() == saved_route_id;
              });
              
              if(find_line != topology_lines_.end()) {
                TopologyLine* line = *find_line;
                
                // 获取起点和终点名称
                VirtualDisplay* from_display = dynamic_cast<VirtualDisplay*>(line->GetFromItem());
                VirtualDisplay* to_display = dynamic_cast<VirtualDisplay*>(line->GetToItem());
                QString from = from_display ? QString::fromStdString(from_display->GetDisplayName()) : QString();
                QString to = to_display ? QString::fromStdString(to_display->GetDisplayName()) : QString();
                
                // 创建删除命令
                auto command = std::make_unique<RemoveTopologyLineCommand>(from, to);
                PushCommand(std::move(command));
                
                // 如果这个 line 是当前选中的，先清除选中状态
                if (selected_topology_line_ == line) {
                  selected_topology_line_ = nullptr;
                }
                
                // 从场景中移除
                removeItem(line);
                
                // 从拓扑地图中删除路径
                topology_map_.RemoveRoute(saved_route_id);
                
                // 从显示列表中删除
                topology_lines_.erase(find_line);
                
                // 从 FactoryDisplay 映射表中删除（不删除对象）
                FactoryDisplay::Instance()->RemoveDisplay(line);
                
                // 手动删除对象
                delete line;
                
                // 更新所有连线的双向状态
                updateAllTopologyLinesStatus();
              }
              
              topology_route_widget_->hide();
            } else {
              topology_route_widget_->hide();
            }
          });
}

void SceneManager::updateNavGoalWidgetPose(
    Display::VirtualDisplay *display, bool is_move) {
  if (!nav_goal_widget_) {
    return;  // widget 不存在，直接返回
  }
  
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
  
  //更新移动后的点位坐标
  std::string display_type = display->GetDisplayType();
  if (display_type == DISPLAY_GOAL) {
    auto scene_pose = display->GetCurrentScenePose();
    auto world_pose = display_manager_->scenePoseToWord(scene_pose);
    auto point_name = display->GetDisplayName();
    
    // 更新拓扑地图中的点坐标
    topology_map_.UpdatePoint(point_name, TopologyMap::PointInfo(world_pose, point_name));

    LOG_INFO("Update point: " << point_name << " to scene pose(" 
              << scene_pose.x << ", " << scene_pose.y << ", " << scene_pose.theta 
              << ") -> world pose(" << world_pose.x << ", " << world_pose.y << ", " << world_pose.theta << ")");
  }
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

void SceneManager::drawScenePointRange(const QPointF &pose, double range) {
  auto map_ptr = static_cast<DisplayOccMap *>(FactoryDisplay::Instance()->GetDisplay(DISPLAY_MAP));
  QPointF pose_map = map_ptr->mapFromScene(pose);
  map_ptr->DrawMapRange(pose_map, range);
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
      auto command = std::make_unique<AddTopologyLineCommand>(first_selected_point_, second_point);
      PushCommand(std::move(command));
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
    updateAllTopologyLinesStatus();
    
    LOG_INFO("创建拓扑连接: " << from.toStdString() << " -> " << to.toStdString() 
             << (is_part_of_bidirectional ? " (双向)" : " (单向)"));
  } else {
    LOG_ERROR("无法找到连接点位: " << from.toStdString() << " 或 " << to.toStdString());
  }
}


void SceneManager::clearTopologyLineSelection() {
  if (selected_topology_line_) {
    // 检查 line 是否还在 scene 中（防止访问已删除的对象）
    if (selected_topology_line_->scene()) {
      selected_topology_line_->SetSelected(false);
    }
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
    
    // 获取起点和终点名称
    VirtualDisplay* from_display = dynamic_cast<VirtualDisplay*>(selected_topology_line_->GetFromItem());
    VirtualDisplay* to_display = dynamic_cast<VirtualDisplay*>(selected_topology_line_->GetToItem());
    QString from = from_display ? QString::fromStdString(from_display->GetDisplayName()) : QString();
    QString to = to_display ? QString::fromStdString(to_display->GetDisplayName()) : QString();
    
    // 创建删除命令
    auto command = std::make_unique<RemoveTopologyLineCommand>(from, to);
    PushCommand(std::move(command));
    
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
    updateAllTopologyLinesStatus();
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

void SceneManager::updateAllTopologyLinesStatus() {
  // 更新所有拓扑连线的双向状态
  for (auto line : topology_lines_) {
    if (!line) continue;
    if (!line->GetFromItem() || !line->GetToItem()) continue;
    
    // 获取起点和终点的显示名称
    VirtualDisplay* from_display = dynamic_cast<VirtualDisplay*>(line->GetFromItem());
    VirtualDisplay* to_display = dynamic_cast<VirtualDisplay*>(line->GetToItem());
    
    if (!from_display || !to_display) continue;
    
    try {

      //根据开始与结束点位 更新名字
      std::string from_name = from_display->GetDisplayName();
      std::string to_name = to_display->GetDisplayName();
      
      // 检查并更新是否为双向连接的一部分
      bool is_part_of_bidirectional = topology_map_.IsBidirectional(from_name, to_name);
      line->SetPartOfBidirectional(is_part_of_bidirectional);
      
      std::string current_name = line->GetDisplayName();
      std::string new_name = generateRouteId(QString::fromStdString(from_name), QString::fromStdString(to_name));
      
      LOG_INFO("update topology line status: " << current_name << " -> " << new_name);
      if (current_name != new_name && !current_name.empty() && !new_name.empty()) {
        // 检查 line 是否还在 topology_lines_ 中（防止在更新过程中被删除）
        bool line_exists = std::find(topology_lines_.begin(), topology_lines_.end(), line) != topology_lines_.end();
        if (line_exists) {
          FactoryDisplay::Instance()->UpdateDisplayName(current_name, new_name);
        }
      }


    } catch (...) {
      LOG_ERROR("Error updating topology line status for line");
      continue;
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
  // 清理命令历史
  ClearCommandHistory();
}

void SceneManager::PushCommand(std::unique_ptr<MapEditCommand> command) {
  if (!command) {
    return;
  }
  
  // 删除当前位置之后的所有命令（当有新操作时，丢弃重做历史）
  if (command_history_index_ < command_history_.size()) {
    command_history_.erase(command_history_.begin() + command_history_index_, command_history_.end());
  }
  
  // 添加新命令
  command_history_.push_back(std::move(command));
  command_history_index_ = command_history_.size();
  
  // 限制历史记录大小
  if (command_history_.size() > kMaxHistorySize) {
    command_history_.erase(command_history_.begin());
    command_history_index_--;
  }
}

void SceneManager::Undo() {
  if (command_history_index_ > 0) {
    command_history_index_--;
    command_history_[command_history_index_]->Undo(this);
    LOG_INFO("Undo command, remaining: " << command_history_index_);
  }
}

void SceneManager::Redo() {
  if (command_history_index_ < command_history_.size()) {
    command_history_[command_history_index_]->Redo(this);
    command_history_index_++;
    LOG_INFO("Redo command, index: " << command_history_index_);
  }
}

void SceneManager::ClearCommandHistory() {
  command_history_.clear();
  command_history_index_ = 0;
}
}  // namespace Display