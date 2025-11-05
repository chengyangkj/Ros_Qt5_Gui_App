/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-14 09:55:52
 * @FilePath: /src/display/laser_points.cpp
 * @Description:
 */
#include "display/display_path.h"
#include "core/framework/framework.h"
#include "msg/msg_info.h"
namespace Display {
DisplayPath::DisplayPath(const std::string &display_type, const int &z_value,
                         std::string parent_name)
    : VirtualDisplay(display_type, z_value, parent_name) {
  setZValue(9);
  
  SUBSCRIBE(MSG_ID_OCCUPANCY_MAP, [this](const OccupancyMap& data) {
    map_data_ = data;
    if (!path_points_.empty()) {
      update();
    }
  });
  
  if (display_type == DISPLAY_GLOBAL_PATH) {
    SUBSCRIBE(MSG_ID_GLOBAL_PATH, [this](const RobotPath& data) {
      updatePathPoints(data);
    });
  } else if (display_type == DISPLAY_LOCAL_PATH) {
    SUBSCRIBE(MSG_ID_LOCAL_PATH, [this](const RobotPath& data) {
      updatePathPoints(data);
    });
  }
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
void DisplayPath::drawPath(QPainter *painter) {

  painter->setRenderHints(QPainter::Antialiasing |
                          QPainter::SmoothPixmapTransform);
  painter->setPen(QPen(color_, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter->setBrush(QBrush(color_));
  // for (int i = 1; i < path_points_.size(); i++) {
  //   painter->drawLine(path_points_[i - 1], path_points_[i]);
  // }
  painter->drawPoints(path_points_);
}
void DisplayPath::updatePathPoints(const RobotPath& path) {
  path_points_.clear();
  
  if (path.empty() || map_data_.Cols() == 0 || map_data_.Rows() == 0) {
    SetBoundingRect(QRectF(0, 0, 0, 0));
    update();
    return;
  }
  
  RobotPath path_data_trans;
  for (auto one_point : path) {
    double x, y;
    map_data_.xy2ScenePose(one_point.x, one_point.y, x, y);
    path_points_.push_back(QPointF(x, y));
    path_data_trans.push_back(Point(x, y));
  }
  
  computeBoundRect(path_data_trans);
  update();
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
  SetBoundingRect(QRectF(xmin, ymin, xmax - xmin, ymax - ymin));
}
} // namespace Display