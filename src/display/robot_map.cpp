
/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-03-28 10:21:04
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-05-09 15:02:05
 * @FilePath: /hontai/src/tools/localizationViewer/src/display/robot_map.cpp
 */
#include <algorithm>
#include <iostream>

#include "display/robot_map.h"
RobotMap::RobotMap(const MapType &type, const std::string &display_name,
                   const int &z_value)
    : VirtualDisplay(display_name, z_value), map_type_(type) {
  this->setCursor(*curr_cursor_);
}
bool RobotMap::UpdateData(const std::any &data) {
  try {
    map_data_ = std::any_cast<OccupancyMap>(data);
    map_image_ =
        QImage(map_data_.Cols(), map_data_.Rows(), QImage::Format_RGB32);
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
  bounding_rect_ = QRectF(0, 0, map_image_.width(), map_image_.height());
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

    // //保持机器人坐标在地图正中心
    // QPointF point =
    //     (QPointF(sub_map_center_pose_[0], sub_map_center_pose_[1]) -
    //      QPointF(map_image_.width() / 2.0, map_image_.height() / 2.0)) *
    //     scale_value_;
    // std::cout << "x:" << point.x() << " y:" << point.y()
    //           << " robot x:" << sub_map_center_pose_[0]
    //           << " robot y:" << sub_map_center_pose_[1]
    //           << " center x:" << map_image_.width() / 2.0
    //           << " center y:" << map_image_.height() / 2.0 << std::endl;
    // moveBy(point.x(), point.y());
  } else {
    return false;
  }
  update();
  return true;
}
QRectF RobotMap::boundingRect() const { return bounding_rect_; }
void RobotMap::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                     QWidget *widget) {
  QImage sub_image = map_image_;
  int draw_x = 0;
  int draw_y = 0;
  //按照比例裁减地图
  if (sub_map_value_ != 1) {
    //以机器人为中心按比例进行裁剪
    int width = std::min(map_data_.Cols(), map_data_.Rows());
    width *= sub_map_value_;
    int top_left_x = sub_map_center_pose_[0] - width / 2.0;
    int top_left_y = sub_map_center_pose_[1] - width / 2.0;
    top_left_x = top_left_x < 0 ? 0 : top_left_x;
    top_left_y = top_left_y < 0 ? 0 : top_left_y;
    int bottom_right_x = sub_map_center_pose_[0] + width / 2.0;
    int bottom_right_y = sub_map_center_pose_[1] + width / 2.0;
    bottom_right_x =
        bottom_right_x > map_data_.Rows() ? map_data_.Rows() : bottom_right_x;
    bottom_right_y =
        bottom_right_y > map_data_.Cols() ? map_data_.Cols() : bottom_right_y;
    sub_image = map_image_.copy(QRect(QPoint(top_left_x, top_left_y),
                                      QPoint(bottom_right_x, bottom_right_y)));
    draw_x = top_left_x;
    draw_y = top_left_y;
  }
  painter->drawImage(draw_x, draw_y, sub_image);
}
void RobotMap::ParseCostMap() {}
void RobotMap::ParseOccupyMap() {
  // Eigen::matrix 坐标系与QImage坐标系不同,这里行列反着遍历
  map_image_ = QImage(map_data_.Cols(), map_data_.Rows(), QImage::Format_RGB32);
  QVector<QPointF> points;

  for (int i = 0; i < map_data_.Cols(); i++)
    for (int j = 0; j < map_data_.Rows(); j++) {
      double map_value = map_data_(j, i);
      QColor color;
      if (map_value > 0) {
        color = Qt::black;  // black
      } else if (map_value < 0) {
        color = Qt::gray;  // gray
      } else {
        color = Qt::white;  // white
      }
      map_image_.setPixel(i, j, qRgb(color.red(), color.green(), color.blue()));
    }
  //栅格地图需要翻转成原始样式
  map_image_ = rotateMapWithY(map_image_);
}
QImage RobotMap::rotateMapWithY(QImage map) {
  QImage res = map;
  for (int x = 0; x < map.width(); x++) {
    for (int y = 0; y < map.height(); y++) {
      res.setPixelColor(x, map.height() - y - 1, map.pixelColor(x, y));
    }
  }
  return res;
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
