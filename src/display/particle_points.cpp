/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-04-24 13:46:43
 * @FilePath: /hontai/src/tools/localizationViewer/src/display/laser_points.cpp
 * @Description:
 */
#include "display/particle_points.h"
ParticlePoints::ParticlePoints(const std::string &display_name,
                               const int &z_value)
    : VirtualDisplay(display_name, z_value) {}
QRectF ParticlePoints::boundingRect() const { return boundRect_; }
void ParticlePoints::paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget) {
  drawParticle(painter, particle_data_);
}

ParticlePoints::~ParticlePoints() {}
bool ParticlePoints::UpdateData(const std::any &data) {
  try {
    // particle_data_ 是地图图元坐标系下的坐标
    particle_data_ = std::any_cast<Display::ParticlePointsType>(data);
    computeBoundRect(particle_data_);
    update();
  } catch (const std::bad_any_cast &e) {
    std::cout << e.what() << '\n';
  }
}
void ParticlePoints::drawParticle(QPainter *painter,
                                  Display::ParticlePointsType particle) {
  for (auto one_point : particle) {
    QPen pen(QColor(0x0e, 0xbe, 0xff, 255), 1, Qt::SolidLine, Qt::RoundCap,
             Qt::RoundJoin);
    painter->setPen(pen);

    double theta = -one_point[2];
    int length = 25;  // 粒子的箭头长1m
    int x1 = one_point[0];
    int y1 = one_point[1];
    int x2 = x1 + cos(theta) * length;
    int y2 = y1 + sin(theta) * length;
    QPointF startPoint = QPointF(x1, y1);
    QPointF endPoint = QPointF(x2, y2);
    QLineF line(startPoint, endPoint);
    painter->drawLine(line);
    float angle =
        atan2(endPoint.y() - startPoint.y(), endPoint.x() - startPoint.x()) +
        3.1415926;  //
    //绘制三角形
    QPolygonF points;
    points.push_back(endPoint);
    QPointF point1, point2;
    point1.setX(endPoint.x() + 10 * cos(angle - 0.5));  //求得箭头点1坐标
    point1.setY(endPoint.y() + 10 * sin(angle - 0.5));
    point2.setX(endPoint.x() + 10 * cos(angle + 0.5));  //求得箭头点2坐标
    point2.setY(endPoint.y() + 10 * sin(angle + 0.5));
    points.push_back(point1);
    points.push_back(point2);
    painter->drawPolygon(points);
  }
}
void ParticlePoints::computeBoundRect(
    const Display::ParticlePointsType &particle) {
  if (particle.empty()) return;
  float xmax, xmin, ymax, ymin;

  xmax = xmin = particle[0][0];
  ymax = ymin = particle[0][1];
  for (auto p : particle) {
    xmax = xmax > p[0] ? xmax : p[0];
    xmin = xmin < p[0] ? xmin : p[0];
    ymax = ymax > p[1] ? ymax : p[1];
    ymin = ymin < p[1] ? ymin : p[1];
  }
  // std::cout << "xmax:" << xmax << "xmin:" << xmin << "ymax:" << ymax
  //           << "ymin:" << ymin << std::endl;
  boundRect_ = QRectF(xmin, ymin, xmax - xmin, ymax - ymin);
}
