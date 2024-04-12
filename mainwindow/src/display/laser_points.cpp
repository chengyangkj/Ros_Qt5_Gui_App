/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-14 09:56:25
 * @FilePath: ////src/display/laser_points.cpp
 * @Description:
 */
#include "display/laser_points.h"
namespace Display {
LaserPoints::LaserPoints(const std::string &display_type, const int &z_value,
                         std::string parent_name)
    : VirtualDisplay(display_type, z_value, parent_name) {}
void LaserPoints::paint(QPainter *painter,
                        const QStyleOptionGraphicsItem *option,
                        QWidget *widget) {
  for (auto [id, data] : laser_data_scene_) {
    drawLaser(painter, id, data);
  }
}

LaserPoints::~LaserPoints() {}

bool LaserPoints::UpdateData(const std::any &data) {
  if (data.type() == typeid(LaserScan)) {
    auto laser_scan = std::any_cast<LaserScan>(data);
    laser_data_scene_[laser_scan.id] = laser_scan.data;
    computeBoundRect(laser_data_scene_);
  }
  update();
  return true;
}
void LaserPoints::computeBoundRect(
    const std::map<int, std::vector<Point>> &laser_scan) {
  float xmax, xmin, ymax, ymin;
  for (auto [id, points] : laser_scan) {
    if (points.empty())
      continue;
    xmax = xmin = points[0].x;
    ymax = ymin = points[0].y;
    for (int i = 1; i < points.size(); ++i) {
      Point p = points[i];
      xmax = xmax > p.x ? xmax : p.x;
      xmin = xmin < p.x ? xmin : p.x;
      ymax = ymax > p.y ? ymax : p.y;
      ymin = ymin < p.y ? ymin : p.y;
    }
  }
  // std::cout << "xmax:" << xmax << "xmin:" << xmin << "ymax:" << ymax
  //           << "ymin:" << ymin << std::endl;
  SetBoundingRect(QRectF(0, 0, xmax, ymax));
}
bool LaserPoints::SetDisplayConfig(const std::string &config_name,
                                   const std::any &config_data) {
  return true;
}
void LaserPoints::drawLaser(QPainter *painter, int id,
                            std::vector<Point> data) {
  QColor color;
  if (!location_to_color_.count(id)) {
    int r, g, b;
    Id2Color(id, r, g, b);
    color = QColor(r, g, b);
  } else {
    color = location_to_color_[id];
  }
  painter->setPen(QPen(color));
  for (auto one_point : data) {
    QPointF point = QPointF(one_point.x, one_point.y);
    // std::cout<<"point:"<<point.x() <<" "<<point.y()<<std::endl;
    painter->drawPoint(point);
  }
  // std::cout << "paint laser" << std::endl;
  // qDebug() << "boundRet:" << bounding_rect_ << std::endl;
}
void LaserPoints::Id2Color(int id, int &R, int &G, int &B) {
#define LocationColorJudge(JudegeId, color) \
  if (id == JudegeId) {                     \
    R = color & 0xFF0000;                   \
    R >>= 16;                               \
    G = color & 0x00FF00;                   \
    G >>= 8;                                \
    B = color & 0x0000FF;                   \
  }
  LocationColorJudge(0, 0xFF6347);
  LocationColorJudge(1, 0xff6600);
  LocationColorJudge(2, 0x228B22);
  LocationColorJudge(3, 0x800000);
  LocationColorJudge(4, 0x8A2BE2);
  LocationColorJudge(5, 0xF4A460);
  LocationColorJudge(6, 0xD2B48C);
  LocationColorJudge(7, 0xADFF2F);
  LocationColorJudge(8, 0xFF00FF);
  LocationColorJudge(9, 0x00FF00);
  LocationColorJudge(-1, 0x40E0D0);
  LocationColorJudge(-2, 0x2F4F4F);
  LocationColorJudge(-3, 0x00BFFF);
  LocationColorJudge(-4, 0x708090);
  LocationColorJudge(-5, 0x00008B);
  LocationColorJudge(-6, 0x006633);
  LocationColorJudge(-7, 0x003300);
  LocationColorJudge(-8, 0xDA70D6);
  LocationColorJudge(-9, 0x9900cc);
  LocationColorJudge(-20, 0x551A8B);
  LocationColorJudge(10, 0x00FF33);
}
}  // namespace Display