
/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-28 10:21:04
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 02:49:39
 * @FilePath: ////src/display/robot_map.cpp
 */
#include <algorithm>
#include <iostream>

#include "display/robot_map.h"
namespace Display {
RobotMap::RobotMap(const MapType &type, const std::string &display_name,
                   const int &z_value, std::string group_name)
    : VirtualDisplay(display_name, z_value, group_name), map_type_(type) {
  this->setCursor(*curr_cursor_);
}
bool RobotMap::UpdateData(const std::any &data) {
  try {
    if (data.type() == typeid(OccupancyMap))
      map_data_ = std::any_cast<OccupancyMap>(data);
    if (data.type() == typeid(CostMap))
      cost_map_data_ = std::any_cast<CostMap>(data);
  } catch (const std::bad_any_cast &e) {
    std::cout << e.what() << '\n';
    return false;
  }

  switch (map_type_) {
  case kGridMap: {
    ParseGridMap();
  } break;
  case kOccupyMap: {
    ParseOccupyMap();
  } break;
  case kTrustMap: {
    ParseTrustMap();
  } break;
  case kCostMap: {
    ParseCostMap();
  } break;
  }
  //绘制原点在地图中心
  SetBoundingRect(QRectF(0, 0, map_image_.width(), map_image_.height()));
  update();
  return true;
}
bool RobotMap::SetDisplayConfig(const std::string &config_name,
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
void RobotMap::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
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
void RobotMap::ParseCostMap() {
  Eigen::Matrix<Eigen::Vector4i, Eigen::Dynamic, Eigen::Dynamic> cost_map =
      cost_map_data_.GetColorMapData();
  map_image_ = QImage(cost_map_data_.Cols(), cost_map_data_.Rows(),
                      QImage::Format_ARGB32);
  for (int i = 0; i < cost_map.cols(); i++)
    for (int j = 0; j < cost_map.rows(); j++) {

      Eigen::Vector4i color_data = cost_map(j, i);
      QColor color;
      color.setRgb(color_data[0], color_data[1], color_data[2]);
      color.setAlpha(color_data[3]);
      map_image_.setPixelColor(i, j, color);
    }
}
void RobotMap::ParseOccupyMap() {
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
void RobotMap::ParseGridMap() {
  // QImage map_image(map_data_.rows(), map_data_.cols(), QImage::Format_RGB32);
  // QVector<QPointF> points;
  // for (int i = 0; i < map_data_.rows(); i++)
  //   for (int j = 0; j < map_data_.cols(); j++) {
  //     double map_value = map_data_(i, j);

  //     QColor color;
  //     if (map_value > 100) {
  //       color = Qt::black;  // black
  //       points.push_back(QPointF(i, j));
  //       // std::cout << "map value:" << map_value << std::endl;
  //     } else if (map_value < 0) {
  //       color = Qt::gray;  // gray
  //     } else {
  //       color = Qt::white;  // white
  //     }
  //     // map_image.setPixel(i, j, qRgb(color.red(), color.green(),
  //     // color.blue()));
  //   }
  // std::cout << " map_data_.rows():" << map_data_.rows()
  //           << " cols:" << map_data_.cols() << std::endl;
  // painter->setPen(QPen(QColor(0, 255, 0), 1));
  // painter->drawPoints(QPolygonF(points));
}
void RobotMap::ParseTrustMap() {}
} // namespace Display