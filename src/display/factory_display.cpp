#pragma once
#include "display/factory_display.h"
#include "common/logger/logger.h"
namespace Display {

bool FactoryDisplay::Init(QGraphicsView *viewer) {
  if (!initlizated_) {
    initlizated_ = true;
    scene_ptr_ = new QGraphicsScene();
    viewer_ptr_ = viewer;
    viewer_ptr_->setScene(scene_ptr_);
    viewer_ptr_->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    run_flag_ = true;
    connect(&timer_coordinate_system_, SIGNAL(timeout()), this,
            SLOT(updateCoordinateSystem()));
    timer_coordinate_system_.setInterval(50);
    timer_coordinate_system_.start();
    return true;
  }
  return false;
}
FactoryDisplay::FactoryDisplay() {}
FactoryDisplay::~FactoryDisplay() { run_flag_ = false; }
// 获取图层
VirtualDisplay *FactoryDisplay::GetDisplay(const std::string &display_name) {
  if (free_display_map_.count(display_name) != 0) {
    return free_display_map_[display_name];
  } else {
    for (auto one_group : group_display_map_) {
      auto iter =
          std::find_if(one_group.second.child_list.begin(),
                       one_group.second.child_list.end(),
                       [&display_name](VirtualDisplay *display) -> bool {
                         return display->GetDisplayName() == display_name;
                       });
      if (iter != one_group.second.child_list.end()) {
        return *iter;
      }
    }
  }
  return nullptr;
}
// 设置图层的scene坐标
bool FactoryDisplay::SetDisplayScenePose(const std::string &display_name,
                                         const QPointF &pose) {
  VirtualDisplay *display = GetDisplay(display_name);
  if (display == nullptr)
    return false;
  display->setPos(pose);
  return true;
}
// 设置图层的scene坐标
bool FactoryDisplay::SetDisplayPoseInParent(const std::string &display_name,
                                            const std::string &parent_name,
                                            const Eigen::Vector3f &pose) {
  VirtualDisplay *display = GetDisplay(display_name);
  if (display == nullptr)
    return false;
  display->SetPoseInParent(parent_name, pose);
  return true;
}
// 设置图层放大缩小
bool FactoryDisplay::SetDisplayScaled(const std::string &display_name,
                                      const double &value) {
  VirtualDisplay *display = GetDisplay(display_name);
  if (display == nullptr)
    return false;
  display->SetScaled(value);
  return true;
}
void FactoryDisplay::AddDisplay(VirtualDisplay *display,
                                const std::string &group_name) {
  if (total_display_map_.count(display->GetDisplayName()) != 0)
    return;
  // add group
  if (!group_name.empty()) {
    if (group_display_map_.count(group_name) == 0) {
      group_display_map_[group_name] = GroupInfo{
          .group_ptr = new DisplayGroup(group_name),
      };
      group_display_map_[group_name].group_ptr->addToGroup(display);
      group_display_map_[group_name].child_list.push_back(display);

      scene_ptr_->addItem(group_display_map_[group_name].group_ptr);
    } else {
      group_display_map_[group_name].group_ptr->addToGroup(display);
      group_display_map_[group_name].child_list.push_back(display);
    }
  } else {
    std::string name = display->GetDisplayName();
    if (free_display_map_.count(name) == 0) {
      free_display_map_[name] = display;
    } else {
      delete free_display_map_[name];
      free_display_map_[name] = display;
    }
    scene_ptr_->addItem(display);
  }
  total_display_map_[display->GetDisplayName()] = display;
}
void FactoryDisplay::FocusDisplay(const std::string &display_name) {
  auto display = GetDisplay(display_name);
  if (display != nullptr) {
    viewer_ptr_->centerOn(display);
  }
}
void FactoryDisplay::RemoveDisplay(const std::string &name) {
  auto iter = free_display_map_.find(name);
  if (iter != free_display_map_.end()) {
    delete iter->second;
    free_display_map_.erase(iter);
  }
  iter = total_display_map_.find(name);
  if (iter != total_display_map_.end()) {
    delete iter->second;
    total_display_map_.erase(iter);
  }
}
int FactoryDisplay::GetDisplaySize() { return free_display_map_.size(); }
std::map<std::string, VirtualDisplay *> FactoryDisplay::GetTotalDisplayMap() {
  return total_display_map_;
}
// 设置响应鼠标事件的图层
bool FactoryDisplay::SetEnableMosuleEvent(const std::string &display_name,
                                          bool enable) {
  if (free_display_map_.count(display_name) != 0) {
    free_display_map_[display_name]->SetEnableMosuleEvent(enable);
    return true;
  } else {
    for (auto one_group : group_display_map_) {
      auto iter =
          std::find_if(one_group.second.child_list.begin(),
                       one_group.second.child_list.end(),
                       [&display_name](VirtualDisplay *display) -> bool {
                         return display->GetDisplayName() == display_name;
                       });
      if (iter != one_group.second.child_list.end()) {
        one_group.second.group_ptr->SetEnableMosuleEvent(enable);
        return true;
      }
    }
  }
  return false;
}
/**
 *
 * @description: 更新图层间的坐标系关系
 * @return {*}
 */
void FactoryDisplay::updateCoordinateSystem() {
  for (auto &[name, display] : total_display_map_) {
    auto trans_pair = display->GetPoseInParent();
    std::string &parent_name = trans_pair.first;
    Eigen::Vector3f &pose_in_parent = trans_pair.second;
    if (!trans_pair.first.empty()) {
      auto parent_ptr = GetDisplay(parent_name);
      if (parent_ptr != nullptr) {
        QPointF pose_in_scene = parent_ptr->PoseToScene(
            QPointF(pose_in_parent[0], pose_in_parent[1]));
        display->setPos(pose_in_scene);
      } else {
        LOGGER_ERROR("display" << name << "transform to parent:" << parent_name
                               << " failed! not found the parrent");
      }
    }
  }
}
} // namespace Display