#include "display/manager/map_edit_command.h"
#include "display/manager/scene_manager.h"
#include "display/manager/display_manager.h"
#include "display/display_occ_map.h"
#include "display/manager/display_factory.h"
#include "display/point_shape.h"
#include "display/virtual_display.h"
#include "display/topology_line.h"
#include <algorithm>
#include <QtCore/qmath.h>
#include <QtCore/qglobal.h>

namespace Display {

EraseCommand::EraseCommand(DisplayOccMap* map_ptr, const QPointF& pose, double range)
    : map_ptr_(map_ptr) {
  float x = pose.x();
  float y = pose.y();
  int left = qMax(0, static_cast<int>(x - range));
  int top = qMax(0, static_cast<int>(y - range));
  int right = qMin(map_ptr_->GetMapImage().width() - 1, static_cast<int>(x + range));
  int bottom = qMin(map_ptr_->GetMapImage().height() - 1, static_cast<int>(y + range));
  region_ = QRectF(left, top, right - left + 1, bottom - top + 1);
  saved_image_ = map_ptr_->GetMapImageRegion(region_);
}

EraseCommand::EraseCommand(DisplayOccMap* map_ptr, const QRectF& region, const QImage& saved_image)
    : map_ptr_(map_ptr), region_(region), saved_image_(saved_image) {
}

void EraseCommand::Undo(SceneManager* manager) {
  if (map_ptr_ && !saved_image_.isNull()) {
    map_ptr_->RestoreMapImageRegion(region_, saved_image_);
  }
}

void EraseCommand::Redo(SceneManager* manager) {
  // Redo 时，当前图像应该已经恢复到操作后的状态
  // 这里不需要重新执行擦除，因为操作已经在执行时完成了
  // 但为了完整性，我们可以从保存的图像中恢复，然后重新应用操作
  // 实际上，Redo 应该恢复操作后的状态，而操作后的状态就是当前状态
  // 所以这里不需要做任何事情，因为操作已经在执行时完成了
  // 但为了正确性，我们需要确保当前状态是操作后的状态
  // 由于操作已经在执行时完成，Redo 时不需要额外操作
}

DrawPointCommand::DrawPointCommand(DisplayOccMap* map_ptr, const QPointF& point)
    : map_ptr_(map_ptr), point_(point), is_continuous_operation_(false) {
  int x = static_cast<int>(point.x());
  int y = static_cast<int>(point.y());
  region_ = QRectF(x - 1, y - 1, 3, 3);
  saved_image_ = map_ptr_->GetMapImageRegion(region_);
}

DrawPointCommand::DrawPointCommand(DisplayOccMap* map_ptr, const QRectF& region, const QImage& saved_image)
    : map_ptr_(map_ptr), region_(region), saved_image_(saved_image), is_continuous_operation_(true) {
}

void DrawPointCommand::Undo(SceneManager* manager) {
  if (map_ptr_ && !saved_image_.isNull()) {
    map_ptr_->RestoreMapImageRegion(region_, saved_image_);
  }
}

void DrawPointCommand::Redo(SceneManager* manager) {
  // 对于连续操作，Redo 时不需要重新绘制，因为操作已经在执行时完成了
  // 对于单点操作，需要重新绘制
  if (!is_continuous_operation_ && map_ptr_) {
    map_ptr_->DrawPoint(point_);
  }
}

DrawLineCommand::DrawLineCommand(DisplayOccMap* map_ptr, const QPointF& start, const QPointF& end)
    : map_ptr_(map_ptr), start_(start), end_(end) {
  QRectF line_rect = QRectF(start, end).normalized();
  int padding = 2;
  region_ = QRectF(line_rect.x() - padding, line_rect.y() - padding,
                   line_rect.width() + 2 * padding, line_rect.height() + 2 * padding);
  saved_image_ = map_ptr_->GetMapImageRegion(region_);
}

DrawLineCommand::DrawLineCommand(DisplayOccMap* map_ptr, const QPointF& start, const QPointF& end,
                                 const QRectF& region, const QImage& saved_image)
    : map_ptr_(map_ptr), start_(start), end_(end), region_(region), saved_image_(saved_image) {
}

void DrawLineCommand::Undo(SceneManager* manager) {
  if (map_ptr_ && !saved_image_.isNull()) {
    map_ptr_->RestoreMapImageRegion(region_, saved_image_);
  }
}

void DrawLineCommand::Redo(SceneManager* manager) {
  if (map_ptr_) {
    map_ptr_->StartDrawLine(start_);
    map_ptr_->EndDrawLine(end_, true);
  }
}

AddPointCommand::AddPointCommand(const std::string& name, const TopologyMap::PointInfo& info)
    : point_name_(name), point_info_(info) {}

void AddPointCommand::Undo(SceneManager* manager) {
  auto display = FactoryDisplay::Instance()->GetDisplay(point_name_);
  if (display) {
    manager->topology_map_.RemovePoint(point_name_);
    FactoryDisplay::Instance()->RemoveDisplay(display);
    manager->removeItem(display);
    delete display;
  }
}

void AddPointCommand::Redo(SceneManager* manager) {
  manager->topology_map_.AddPoint(point_info_);
  auto goal_point = new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL,
                                   point_name_, 8, DISPLAY_MAP);
  goal_point->SetRotateEnable(true)->SetMoveEnable(true)->setVisible(true);
  auto map_pose = manager->display_manager_->wordPose2Map(point_info_.ToRobotPose());
  goal_point->UpdateData(map_pose);
  manager->addItem(goal_point);
}

RemovePointCommand::RemovePointCommand(const std::string& name, const TopologyMap::PointInfo& info,
                                       const std::vector<std::string>& routes)
    : point_name_(name), point_info_(info), related_routes_(routes) {}

void RemovePointCommand::Undo(SceneManager* manager) {
  manager->topology_map_.AddPoint(point_info_);
  auto goal_point = new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL,
                                   point_name_, 8, DISPLAY_MAP);
  goal_point->SetRotateEnable(true)->SetMoveEnable(true)->setVisible(true);
  auto map_pose = manager->display_manager_->wordPose2Map(point_info_.ToRobotPose());
  goal_point->UpdateData(map_pose);
  manager->addItem(goal_point);
  
  for (const auto& route_id : related_routes_) {
    size_t arrow_pos = route_id.find("->");
    if (arrow_pos != std::string::npos) {
      std::string from = route_id.substr(0, arrow_pos);
      std::string to = route_id.substr(arrow_pos + 2);
      manager->createTopologyLine(QString::fromStdString(from), QString::fromStdString(to));
    }
  }
}

void RemovePointCommand::Redo(SceneManager* manager) {
  auto display = FactoryDisplay::Instance()->GetDisplay(point_name_);
  if (display) {
    for (const auto& route_id : related_routes_) {
      auto line = manager->findTopologyLine(QString::fromStdString(route_id));
      if (line) {
        manager->removeItem(line);
        manager->topology_map_.RemoveRoute(route_id);
        auto it = std::find(manager->topology_lines_.begin(), manager->topology_lines_.end(), line);
        if (it != manager->topology_lines_.end()) {
          manager->topology_lines_.erase(it);
        }
        FactoryDisplay::Instance()->RemoveDisplay(line);
        delete line;
      }
    }
    manager->topology_map_.RemovePoint(point_name_);
    FactoryDisplay::Instance()->RemoveDisplay(display);
    manager->removeItem(display);
    delete display;
  }
}

AddTopologyLineCommand::AddTopologyLineCommand(const QString& from, const QString& to)
    : from_(from), to_(to) {
  route_id_ = from.toStdString() + "->" + to.toStdString();
}

void AddTopologyLineCommand::Undo(SceneManager* manager) {
  auto line = manager->findTopologyLine(QString::fromStdString(route_id_));
  if (line) {
    manager->removeItem(line);
    manager->topology_map_.RemoveRoute(route_id_);
    auto it = std::find(manager->topology_lines_.begin(), manager->topology_lines_.end(), line);
    if (it != manager->topology_lines_.end()) {
      manager->topology_lines_.erase(it);
    }
    FactoryDisplay::Instance()->RemoveDisplay(line);
    delete line;
    manager->updateAllTopologyLinesStatus();
  }
}

void AddTopologyLineCommand::Redo(SceneManager* manager) {
  manager->createTopologyLine(from_, to_);
}

RemoveTopologyLineCommand::RemoveTopologyLineCommand(const QString& from, const QString& to)
    : from_(from), to_(to) {
  route_id_ = from.toStdString() + "->" + to.toStdString();
}

void RemoveTopologyLineCommand::Undo(SceneManager* manager) {
  manager->createTopologyLine(from_, to_);
}

void RemoveTopologyLineCommand::Redo(SceneManager* manager) {
  auto line = manager->findTopologyLine(QString::fromStdString(route_id_));
  if (line) {
    manager->removeItem(line);
    manager->topology_map_.RemoveRoute(route_id_);
    auto it = std::find(manager->topology_lines_.begin(), manager->topology_lines_.end(), line);
    if (it != manager->topology_lines_.end()) {
      manager->topology_lines_.erase(it);
    }
    FactoryDisplay::Instance()->RemoveDisplay(line);
    delete line;
    manager->updateAllTopologyLinesStatus();
  }
}

UpdatePointCommand::UpdatePointCommand(const std::string& name, const TopologyMap::PointInfo& old_info, const TopologyMap::PointInfo& new_info)
    : point_name_(name), old_point_info_(old_info), new_point_info_(new_info) {}

void UpdatePointCommand::Undo(SceneManager* manager) {
  manager->topology_map_.UpdatePoint(point_name_, old_point_info_);
  auto display = FactoryDisplay::Instance()->GetDisplay(point_name_);
  if (display) {
    auto map_pose = manager->display_manager_->wordPose2Map(old_point_info_.ToRobotPose());
    auto* point_shape = dynamic_cast<PointShape*>(display);
    if (point_shape) {
      point_shape->UpdateData(map_pose);
    }
  }
}

void UpdatePointCommand::Redo(SceneManager* manager) {
  manager->topology_map_.UpdatePoint(point_name_, new_point_info_);
  auto display = FactoryDisplay::Instance()->GetDisplay(point_name_);
  if (display) {
    auto map_pose = manager->display_manager_->wordPose2Map(new_point_info_.ToRobotPose());
    auto* point_shape = dynamic_cast<PointShape*>(display);
    if (point_shape) {
      point_shape->UpdateData(map_pose);
    }
  }
}

UpdatePointNameCommand::UpdatePointNameCommand(const std::string& old_name, const std::string& new_name)
    : old_name_(old_name), new_name_(new_name) {}

void UpdatePointNameCommand::Undo(SceneManager* manager) {
  if (!FactoryDisplay::Instance()->UpdateDisplayName(new_name_, old_name_)) {
    return;
  }
  manager->topology_map_.UpdatePointName(new_name_, old_name_);
  
  for (auto line : manager->topology_lines_) {
    if (!line) continue;
    VirtualDisplay* from_display = dynamic_cast<VirtualDisplay*>(line->GetFromItem());
    VirtualDisplay* to_display = dynamic_cast<VirtualDisplay*>(line->GetToItem());
    if (!from_display || !to_display) continue;
    
    std::string from_name = from_display->GetDisplayName();
    std::string to_name = to_display->GetDisplayName();
    
    if (from_name == new_name_ || to_name == new_name_) {
      std::string from = (from_name == new_name_) ? old_name_ : from_name;
      std::string to = (to_name == new_name_) ? old_name_ : to_name;
      std::string new_route_name = from + "->" + to;
      FactoryDisplay::Instance()->UpdateDisplayName(line->GetDisplayName(), new_route_name);
    }
  }
}

void UpdatePointNameCommand::Redo(SceneManager* manager) {
  if (!FactoryDisplay::Instance()->UpdateDisplayName(old_name_, new_name_)) {
    return;
  }
  manager->topology_map_.UpdatePointName(old_name_, new_name_);
  
  for (auto line : manager->topology_lines_) {
    if (!line) continue;
    VirtualDisplay* from_display = dynamic_cast<VirtualDisplay*>(line->GetFromItem());
    VirtualDisplay* to_display = dynamic_cast<VirtualDisplay*>(line->GetToItem());
    if (!from_display || !to_display) continue;
    
    std::string from_name = from_display->GetDisplayName();
    std::string to_name = to_display->GetDisplayName();
    
    if (from_name == old_name_ || to_name == old_name_) {
      std::string from = (from_name == old_name_) ? new_name_ : from_name;
      std::string to = (to_name == old_name_) ? new_name_ : to_name;
      std::string new_route_name = from + "->" + to;
      FactoryDisplay::Instance()->UpdateDisplayName(line->GetDisplayName(), new_route_name);
    }
  }
}

}  // namespace Display
