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
namespace Display {
RobotShape::RobotShape(const std::string &display_type, const int &z_value,
                       std::string parent_name)
    : VirtualDisplay(display_type, z_value, parent_name) {
  if (
      Config::ConfigManager::Instacnce()->GetRootConfig().robot_shape_config.shaped_points.empty()) {
    Config::ConfigManager::Instacnce()
        ->GetRootConfig()
        .robot_shape_config.shaped_points = {Config::Point{0.5, 0.5}, Config::Point{0.5, -0.5}, Config::Point{-0.5, -0.5}, Config::Point{-0.5, 0.5}};
  }
  color_ = QColor(QString::fromStdString(Config::ConfigManager::Instacnce()->GetRootConfig().robot_shape_config.color).toUInt(nullptr, 16));
  opacity_ = Config::ConfigManager::Instacnce()->GetRootConfig().robot_shape_config.opacity;
  int index = 0;

  int max_x = 0;
  int max_y = 0;
  int min_x = 9999;
  int min_y = 9999;
  if (Config::ConfigManager::Instacnce()->GetRootConfig().robot_shape_config.is_ellipse) {
    for (auto point :
         Config::ConfigManager::Instacnce()->GetRootConfig().robot_shape_config.shaped_points) {
      double scene_x, scene_y;
      map_data_.xy2ScenePose(point.x, point.y, scene_x, scene_y);
      if (scene_x > max_x)
        max_x = scene_x;
      if (scene_y > max_y)
        max_y = scene_y;
      if (scene_x < min_x)
        min_x = scene_x;
      if (scene_y < min_y)
        min_y = scene_y;
    }
    path_.addEllipse(QRectF(min_x, min_y, max_x - min_x, max_y - min_y));  // 参数分别为圆心坐标和宽高
  } else {
    for (auto point :
         Config::ConfigManager::Instacnce()->GetRootConfig().robot_shape_config.shaped_points) {
      double scene_x, scene_y;
      map_data_.xy2ScenePose(point.x, point.y, scene_x, scene_y);
      if (index == 0) {
        path_.moveTo(scene_x, scene_y);
      } else {
        path_.lineTo(scene_x, scene_y);
      }
      if (scene_x > max_x)
        max_x = scene_x;
      if (scene_y > max_y)
        max_y = scene_y;
      if (scene_x < min_x)
        min_x = scene_x;
      if (scene_y < min_y)
        min_y = scene_y;
      index++;
    }
    path_.closeSubpath();  // 闭合路径
  }

  SetBoundingRect(QRectF(min_x, min_y, max_x - min_x, max_y - min_y));
}
void RobotShape::paint(QPainter *painter,
                       const QStyleOptionGraphicsItem *option,
                       QWidget *widget) {
  drawFrame(painter);
}

RobotShape::~RobotShape() {}
bool RobotShape::UpdateData(const std::any &data) {
  try {
    GetAnyData(RobotPose, data, robot_pose_);
    rotate_value_ = 0;
    SetPoseInParent(robot_pose_);
    update();
  } catch (const std::bad_any_cast &e) {
    std::cout << e.what() << '\n';
  }
  return true;
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