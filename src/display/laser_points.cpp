/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-04-26 16:04:37
 * @FilePath: /hontai/src/tools/localizationViewer/src/display/laser_points.cpp
 * @Description:
 */
#include "display/laser_points.h"
LaserPoints::LaserPoints(const std::string &display_name, const int &z_value)
    : VirtualDisplay(display_name, z_value) {}
QRectF LaserPoints::boundingRect() const { return boundRect_; }
void LaserPoints::paint(QPainter *painter,
                        const QStyleOptionGraphicsItem *option,
                        QWidget *widget) {
  for (auto [id, data] : laser_data_scene_) {
    drawLaser(painter, id, data);
  }
}

LaserPoints::~LaserPoints() {}
/**
 * @description:激光车身坐标系转换为图元坐标系
 * @return {*}
 */
std::vector<Eigen::Vector2f> LaserPoints::transLaserPoint(
    const std::vector<Eigen::Vector2f> &point) {
  std::vector<Eigen::Vector2f> res;
  for (auto one_point : point) {
    Eigen::Vector3d point_map = Display::absoluteSum(
        Eigen::Vector3d(robot_pose_[0], robot_pose_[1], robot_pose_[2]),
        Eigen::Vector3d(one_point[0], one_point[1], 0));
    // 转换为图元坐标系
    int x, y;
    map_data_.xy2scene(point_map[0], point_map[1], x, y);
    res.push_back(Eigen::Vector2f(x, y));
  }
  return res;
}
bool LaserPoints::UpdateData(const std::any &data) {
  if (data.type() == typeid(OccupancyMap)) {
    map_data_ = std::any_cast<OccupancyMap>(data);
  } else if (data.type() == typeid(Display::LaserDataMap)) {
    laser_data_scene_.clear();
    laser_data_map_ = std::any_cast<Display::LaserDataMap>(data);
    // 点坐标转换为地图坐标系下
    for (auto one_laser : laser_data_map_) {
      laser_data_scene_[one_laser.first] = transLaserPoint(one_laser.second);
    }
    computeBoundRect(laser_data_scene_);
  } else if (data.type() == typeid(Eigen::Vector3f)) {
    robot_pose_ = std::any_cast<Eigen::Vector3f>(data);
  }

  update();
}
void LaserPoints::computeBoundRect(const Display::LaserDataMap &laser_scan) {
  float xmax, xmin, ymax, ymin;
  for (auto [id, points] : laser_scan) {
    if (points.empty()) continue;
    xmax = xmin = points[0][0];
    ymax = ymin = points[0][1];
    for (int i = 1; i < points.size(); ++i) {
      Eigen::Vector2f p = points[i];
      xmax = xmax > p[0] ? xmax : p[0];
      xmin = xmin < p[0] ? xmin : p[0];
      ymax = ymax > p[1] ? ymax : p[1];
      ymin = ymin < p[1] ? ymin : p[1];
    }
  }
  // std::cout << "xmax:" << xmax << "xmin:" << xmin << "ymax:" << ymax
  //           << "ymin:" << ymin << std::endl;
  boundRect_ = QRectF(xmin, ymin, xmax - xmin, ymax - ymin);
}
bool LaserPoints::SetDisplayConfig(const std::string &config_name,
                                   const std::any &config_data) {
  if (config_name == "Color") {
    Display::LaserColorMap new_map;
    GetAnyData(Display::LaserColorMap, config_data, new_map);
    for (auto item : new_map) {
      location_to_color_[item.first] = item.second;
    }
  } else {
    return false;
  }
  return true;
}
void LaserPoints::drawLaser(QPainter *painter, int id,
                            std::vector<Eigen::Vector2f> data) {
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
    QPointF point = QPointF(one_point[0], one_point[1]);
    painter->drawPoint(point);
  }
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
