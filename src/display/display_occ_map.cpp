
/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-28 10:21:04
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 02:49:39
 * @FilePath: ////src/display/robot_map.cpp
 */
#include <algorithm>
#include <iostream>

#include "display/display_occ_map.h"
namespace Display {
DisplayOccMap::DisplayOccMap(const std::string &display_name,
                             const int &z_value, std::string parent_name)
    : VirtualDisplay(display_name, z_value, parent_name) {
  this->setCursor(*curr_cursor_);
  SetEnableMosuleEvent(true);
}
bool DisplayOccMap::UpdateData(const std::any &data) {
  try {
    if (data.type() == typeid(OccupancyMap)) {
      map_data_ = std::any_cast<OccupancyMap>(data);
      std::cout << "recv occ map" << std::endl;
    }

  } catch (const std::bad_any_cast &e) {
    std::cout << e.what() << '\n';
    return false;
  }

  ParseOccupyMap();
  //绘制原点在地图中心
  SetBoundingRect(QRectF(0, 0, map_image_.width(), map_image_.height()));
  update();
  return true;
}
bool DisplayOccMap::SetDisplayConfig(const std::string &config_name,
                                     const std::any &config_data) {
  if (config_name == "SubMapValue") {
    GetAnyData(double, config_data, sub_map_value_);
  } else if (config_name == "RobotPose") {
    // using type = Eigen::Vector3f;
    GetAnyData(Eigen::Vector3f, config_data, sub_map_center_pose_);
  } else {
    return false;
  }
  update();
  return true;
}
void DisplayOccMap::paint(QPainter *painter,
                          const QStyleOptionGraphicsItem *option,
                          QWidget *widget) {
  // //按照比例裁减地图
  // if (sub_map_value_ != 1) {
  //   //以机器人为中心按比例进行裁剪
  //   int width = std::min(map_data_.Cols(), map_data_.Rows());
  //   width *= sub_map_value_;
  //   int top_left_x = sub_map_center_pose_[0] - width / 2.0;
  //   int top_left_y = sub_map_center_pose_[1] - width / 2.0;
  //   top_left_x = top_left_x < 0 ? 0 : top_left_x;
  //   top_left_y = top_left_y < 0 ? 0 : top_left_y;
  //   int bottom_right_x = sub_map_center_pose_[0] + width / 2.0;
  //   int bottom_right_y = sub_map_center_pose_[1] + width / 2.0;
  //   bottom_right_x =
  //       bottom_right_x > map_data_.Rows() ? map_data_.Rows() :
  //       bottom_right_x;
  //   bottom_right_y =
  //       bottom_right_y > map_data_.Cols() ? map_data_.Cols() :
  //       bottom_right_y;
  //   sub_image = map_image_.copy(QRect(QPoint(top_left_x, top_left_y),
  //                                     QPoint(bottom_right_x,
  //                                     bottom_right_y)));
  //   draw_x = top_left_x;
  //   draw_y = top_left_y;
  // }
  //以图片中心做原点进行绘制(方便旋转)
  painter->drawImage(GetOriginPose().x(), GetOriginPose().y(), map_image_);
  // std::cout << "map painter event" << std::endl;
}
void DisplayOccMap::ParseOccupyMap() {
  // Eigen::matrix 坐标系与QImage坐标系不同,这里行列反着遍历
  map_image_ = QImage(map_data_.Cols(), map_data_.Rows(), QImage::Format_RGB32);
  QVector<QPointF> points;

  for (int i = 0; i < map_data_.Cols(); i++)
    for (int j = 0; j < map_data_.Rows(); j++) {
      double map_value = map_data_(j, i);
      QColor color;
      if (map_value > 0) {
        color = Qt::black; // black
      } else if (map_value < 0) {
        color = Qt::gray; // gray
      } else {
        color = Qt::white; // white
      }
      map_image_.setPixel(i, j, qRgb(color.red(), color.green(), color.blue()));
    }
}

} // namespace Display