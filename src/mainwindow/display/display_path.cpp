/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-14 09:55:52
 * @FilePath: /src/display/laser_points.cpp
 * @Description:
 */
#include "display/display_path.h"
namespace Display {
DisplayPath::DisplayPath(const std::string &display_type, const int &z_value,
                         std::string parent_name)
    : VirtualDisplay(display_type, z_value, parent_name) {
  // enable_scale_ = false;
}
void DisplayPath::paint(QPainter *painter,
                        const QStyleOptionGraphicsItem *option,
                        QWidget *widget) {
  drawPath(painter);
}

DisplayPath::~DisplayPath() {}
bool DisplayPath::SetDisplayConfig(const std::string &config_name,
                                   const std::any &config_data) {
  if (config_name == "Color") {
    Color color;
    GetAnyData(Color, config_data, color);
    color_ = QColor(color[0], color[1], color[2]);
  } else {
    return false;
  }
  return true;
}
bool DisplayPath::UpdateData(const std::any &data) {
  try {
    auto path_data = std::any_cast<RobotPath>(data);
    computeBoundRect(path_data);
    path_points_.clear();
    for (auto one_path : path_data) {
      path_points_.push_back(QPointF(one_path.x, one_path.y));
    }
    update();
  } catch (const std::bad_any_cast &e) {
    std::cout << e.what() << '\n';
  }
  return true;
}
void DisplayPath::drawPath(QPainter *painter) {

  painter->setRenderHints(QPainter::Antialiasing |
                          QPainter::SmoothPixmapTransform);
  painter->setPen(QPen(color_, 1));
  // for (int i = 1; i < path_points_.size(); i++) {
  //   painter->drawLine(path_points_[i - 1], path_points_[i]);
  // }
  painter->drawPoints(path_points_);
}
void DisplayPath::computeBoundRect(const RobotPath &path) {
  if (path.empty())
    return;
  float xmax, xmin, ymax, ymin;

  xmax = xmin = path[0].x;
  ymax = ymin = path[0].y;
  for (auto p : path) {
    xmax = xmax > p.x ? xmax : p.x;
    xmin = xmin < p.x ? xmin : p.x;
    ymax = ymax > p.y ? ymax : p.y;
    ymin = ymin < p.y ? ymin : p.y;
  }
  // std::cout << "xmax:" << xmax << "xmin:" << xmin << "ymax:" << ymax
  //           << "ymin:" << ymin << std::endl;
  SetBoundingRect(QRectF(xmin, ymin, xmax, ymax));
}
} // namespace Display