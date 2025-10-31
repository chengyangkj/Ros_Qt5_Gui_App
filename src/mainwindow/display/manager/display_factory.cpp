
#include "display//manager/display_factory.h"
#include "display/manager/display_manager.h"
#include "logger/logger.h"
namespace Display {

bool FactoryDisplay::Init(QGraphicsView *viewer, SceneManager *scene_ptr) {
  if (!initlizated_) {
    initlizated_ = true;
    viewer_ptr_ = viewer;
    scene_manager_ptr_ = scene_ptr;
    viewer_ptr_->setScene(scene_manager_ptr_);
    viewer_ptr_->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    run_flag_ = true;
    connect(&timer_coordinate_system_, SIGNAL(timeout()), this,
            SLOT(Process()));
    timer_coordinate_system_.setInterval(10);
    timer_coordinate_system_.start();
    return true;
  }
  return false;
}
FactoryDisplay::FactoryDisplay() {}
FactoryDisplay::~FactoryDisplay() { run_flag_ = false; }
// 获取图层
VirtualDisplay *FactoryDisplay::GetDisplay(const std::string &display_name) {
  std::lock_guard<std::mutex> lock(display_map_mutex_);
  auto it = total_display_map_.find(display_name);
  if (it != total_display_map_.end()) {
    return it->second;
  }
  return nullptr;
}
void FactoryDisplay::RemoveDisplay(VirtualDisplay *display) {
  std::string parent_name;
  {
    std::lock_guard<std::mutex> lock(display_map_mutex_);
    auto find_iter = std::find_if(
        total_display_map_.begin(), total_display_map_.end(),
        [display](std::pair<std::string, VirtualDisplay *> pair) -> bool {
          return pair.second == display;
        });
    if (find_iter != total_display_map_.end()) {
      total_display_map_.erase(find_iter);
    }
    if (display) {
      parent_name = display->GetParentName();
    }
  }
  
  if (!parent_name.empty()) {
    auto parent_ptr_ = GetDisplay(parent_name);
    if (parent_ptr_ != nullptr) {
      parent_ptr_->RemoveChild(display);
    }
  }
}  // namespace Display
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
                                            const RobotPose &pose) {
  VirtualDisplay *display = GetDisplay(display_name);
  if (display == nullptr)
    return false;
  display->SetPoseInParent(pose);
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
                                const std::string &parent_name) {
  std::string display_name;
  {
    std::lock_guard<std::mutex> lock(display_map_mutex_);
    display_name = display->GetDisplayName();
    if (total_display_map_.count(display_name) != 0) {
      std::cout << "display name:" << display_name
                << " already exist" << std::endl;
      return;
    }
    total_display_map_[display_name] = display;
  }

  if (!parent_name.empty()) {
    auto parent_ptr = GetDisplay(parent_name);
    if (parent_ptr == nullptr) {
      LOG_ERROR("not find parent display:" << parent_name << " current display:" << display_name);
      return;
    }
    parent_ptr->AddChild(display);
  }
  scene_manager_ptr_->addItem(display);
}
void FactoryDisplay::SetFocusDisplay(const std::string &display_name) {
  focus_display_name_ = display_name;
}
void FactoryDisplay::RemoveDisplay(const std::string &name) {
  std::lock_guard<std::mutex> lock(display_map_mutex_);
  auto iter = total_display_map_.find(name);
  if (iter != total_display_map_.end()) {
    delete iter->second;
    total_display_map_.erase(iter);
  }
}
int FactoryDisplay::GetDisplaySize() { 
  std::lock_guard<std::mutex> lock(display_map_mutex_);
  return total_display_map_.size(); 
}
std::map<std::string, VirtualDisplay *> FactoryDisplay::GetTotalDisplayMap() {
  std::lock_guard<std::mutex> lock(display_map_mutex_);
  return total_display_map_;
}
// 设置响应鼠标事件的图层
bool FactoryDisplay::SetMoveEnable(const std::string &display_name,
                                   bool enable) {
  std::lock_guard<std::mutex> lock(display_map_mutex_);
  auto it = total_display_map_.find(display_name);
  if (it != total_display_map_.end()) {
    it->second->SetMoveEnable(enable);
    return true;
  }
  return false;
}
bool FactoryDisplay::GetMoveEnable(const std::string &display_name) {
  std::lock_guard<std::mutex> lock(display_map_mutex_);
  auto it = total_display_map_.find(display_name);
  if (it != total_display_map_.end()) {
    return it->second->GetMoveEnable();
  }
  return false;
}
/**
 *
 * @description: 更新图层间的坐标系关系
 * @return {*}
 */
void FactoryDisplay::Process() {
  // // focus on
  auto display = GetDisplay(focus_display_name_);
  if (display != nullptr) {
    viewer_ptr_->centerOn(display);
  }
}
// 安全地更新显示对象名称
bool FactoryDisplay::UpdateDisplayName(const std::string &old_name, const std::string &new_name) {
  std::lock_guard<std::mutex> lock(display_map_mutex_);
  
  // 检查旧名称是否存在
  auto iter = total_display_map_.find(old_name);
  if (iter == total_display_map_.end()) {
    LOG_ERROR("Display with old name not found: " << old_name);
    for(auto &item : total_display_map_) {
      LOG_INFO("Display name: " << item.first);
    }
    return false;
  }

  LOG_INFO("Update display name: " << old_name << " -> " << new_name);
  
  // 检查新名称是否已经存在
  if (total_display_map_.find(new_name) != total_display_map_.end()) {
    LOG_ERROR("Display with new name already exists: " << new_name);
    return false;
  }
  
  // 获取显示对象指针
  VirtualDisplay* display = iter->second;
  if (!display) {
    LOG_ERROR("Display pointer is null for name: " << old_name);
    return false;
  }

  // 更新显示对象的内部名称
  display->SetDisplayName(new_name);
  
  // 更新映射表：先添加新的映射，再删除旧的映射
  total_display_map_[new_name] = display;
  total_display_map_.erase(iter);
  
  LOG_INFO("Successfully updated display name:[" << old_name << "] to [" << new_name<<"] get new name:"<<display->GetDisplayName());
  return true;
}
}  // namespace Display