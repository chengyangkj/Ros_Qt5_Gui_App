
#include "display/factory_display.h"
#include "display/display_manager.h";
#include "logger/logger.h"
namespace Display {

bool FactoryDisplay::Init(QGraphicsView *viewer, SceneDisplay *scene_ptr) {
  if (!initlizated_) {
    initlizated_ = true;
    viewer_ptr_ = viewer;
    scene_display_ptr_ = scene_ptr;
    viewer_ptr_->setScene(scene_display_ptr_);
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
VirtualDisplay *FactoryDisplay::GetDisplay(const std::string &display_type) {
  if (total_display_map_.count(display_type) != 0) {
    return total_display_map_[display_type];
  }
  return nullptr;
}
void FactoryDisplay::RemoveDisplay(VirtualDisplay *display) {
  auto find_iter = std::find_if(
      total_display_map_.begin(), total_display_map_.end(),
      [display](std::pair<std::string, VirtualDisplay *> pair) -> bool {
        return pair.second == display;
      });
  if (find_iter != total_display_map_.end()) {
    total_display_map_.erase(find_iter);
  }
  auto parent_ptr_ =
      FactoryDisplay::Instance()->GetDisplay(display->GetParentName());
  if (parent_ptr_ != nullptr) {
    parent_ptr_->RemoveChild(display);
  }
} // namespace Display
// 设置图层的scene坐标
bool FactoryDisplay::SetDisplayScenePose(const std::string &display_type,
                                         const QPointF &pose) {
  VirtualDisplay *display = GetDisplay(display_type);
  if (display == nullptr)
    return false;
  display->setPos(pose);
  return true;
}
// 设置图层的scene坐标
bool FactoryDisplay::SetDisplayPoseInParent(const std::string &display_type,
                                            const RobotPose &pose) {
  VirtualDisplay *display = GetDisplay(display_type);
  if (display == nullptr)
    return false;
  display->SetPoseInParent(pose);
  return true;
}
// 设置图层放大缩小
bool FactoryDisplay::SetDisplayScaled(const std::string &display_type,
                                      const double &value) {
  VirtualDisplay *display = GetDisplay(display_type);
  if (display == nullptr)
    return false;
  display->SetScaled(value);
  return true;
}
void FactoryDisplay::AddDisplay(VirtualDisplay *display,
                                const std::string &parent_name) {
  if (total_display_map_.count(display->GetDisplayType()) != 0) {
    total_display_map_
        [display->GetDisplayType() + ":" +
         std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count())] = display;
  } else {
    total_display_map_[display->GetDisplayType()] = display;
  }
  if (!parent_name.empty()) {
    auto parent_ptr = GetDisplay(parent_name);
    parent_ptr->AddChild(display);
    // display->setParent(parent_ptr);
  }

  scene_display_ptr_->addItem(display);
}
void FactoryDisplay::FocusDisplay(const std::string &display_type) {

  focus_display_type_ = display_type;
}
void FactoryDisplay::RemoveDisplay(const std::string &name) {
  auto iter = total_display_map_.find(name);
  if (iter != total_display_map_.end()) {
    delete iter->second;
    total_display_map_.erase(iter);
  }
  iter = total_display_map_.find(name);
  if (iter != total_display_map_.end()) {
    delete iter->second;
    total_display_map_.erase(iter);
  }
}
int FactoryDisplay::GetDisplaySize() { return total_display_map_.size(); }
std::map<std::string, VirtualDisplay *> FactoryDisplay::GetTotalDisplayMap() {
  return total_display_map_;
}
// 设置响应鼠标事件的图层
bool FactoryDisplay::SetMoveEnable(const std::string &display_type,
                                   bool enable) {
  if (total_display_map_.count(display_type) != 0) {
    total_display_map_[display_type]->SetMoveEnable(enable);
    return true;
  }
  return false;
}
bool FactoryDisplay::GetMoveEnable(const std::string &display_type) {
  if (total_display_map_.count(display_type) != 0) {
    return total_display_map_[display_type]->GetMoveEnable();
  }
  return false;
}
/**
 *
 * @description: 更新图层间的坐标系关系
 * @return {*}
 */
void FactoryDisplay::Process() {
  // focus on
  auto display = GetDisplay(focus_display_type_);
  if (display != nullptr) {
    viewer_ptr_->centerOn(display);
  }
}
} // namespace Display