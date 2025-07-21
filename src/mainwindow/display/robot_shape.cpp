/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-14 09:47:41
 * @FilePath: ////src/display/laser_points.cpp
 * @Description:
 */
#include "display/robot_shape.h"
#include "algorithm.h"
#include "msg/msg_info.h"
namespace Display {
RobotShape::RobotShape(const std::string &display_type, const int &z_value,
                       std::string parent_name)
    : VirtualDisplay(display_type, z_value, parent_name) {
  // 使用默认颜色和透明度
  color_ = QColor(0x1E90FF);  // 默认蓝色
  opacity_ = 0.5;
  
  // 初始化空的路径
  path_ = QPainterPath();
  SetBoundingRect(QRectF(0, 0, 0, 0));
  setZValue(10);
}

void RobotShape::paint(QPainter *painter,
                       const QStyleOptionGraphicsItem *option,
                       QWidget *widget) {
  drawFrame(painter);
}

RobotShape::~RobotShape() {}

bool RobotShape::UpdateData(const std::any &data) {
  try {
    // 检查是否是RobotFootprint数据
    GetAnyData(RobotPath, data, robot_footprint_);
    updateFootprintPath();
  
    rotate_value_ = 0;
    SetPoseInParent(robot_pose_);
    update();
  } catch (const std::bad_any_cast &e) {
    LOG_ERROR("RobotShape UpdateData error: " << e.what());
  }
  return true;
}

void RobotShape::updateFootprintPath() {
  path_.clear();
  
  if (robot_footprint_.empty()) {
    SetBoundingRect(QRectF(0, 0, 0, 0));
    return;
  }
  
  // 计算边界框
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  
  bool first_point = true;
  for (const auto& point : robot_footprint_) {
    double scene_x, scene_y;
    map_data_.xy2ScenePose(point.x, point.y, scene_x, scene_y);
    
    if (first_point) {
      path_.moveTo(scene_x, scene_y);
      first_point = false;
    } else {
      path_.lineTo(scene_x, scene_y);
    }
    
    // 更新边界框
    if (scene_x > max_x) max_x = scene_x;
    if (scene_y > max_y) max_y = scene_y;
    if (scene_x < min_x) min_x = scene_x;
    if (scene_y < min_y) min_y = scene_y;
  }
  
  // 闭合路径
  if (!robot_footprint_.empty()) {
    path_.closeSubpath();
  }
  
  SetBoundingRect(QRectF(min_x, min_y, max_x - min_x, max_y - min_y));
}

void RobotShape::drawFrame(QPainter *painter) {
  painter->setRenderHint(QPainter::Antialiasing, true);  // 设置反锯齿 反走样

  painter->save();
  painter->rotate(-basic::rad2deg(robot_pose_.theta) - basic::rad2deg(rotate_value_));
  painter->setPen(QPen(Qt::transparent, 1));
  painter->setOpacity(opacity_);
  painter->setBrush(color_);
  painter->drawPath(path_);
  painter->restore();
}
}  // namespace Display