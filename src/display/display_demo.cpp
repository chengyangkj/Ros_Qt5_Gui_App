/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-04-20 16:44:15
 * @FilePath: /hontai/src/tools/localizationViewer/src/display/laser_points.cpp
 * @Description:
 */
#include "display/display_demo.h"
DisplayDemo::DisplayDemo(const std::string &display_name,
                               const int &z_value)
    : VirtualDisplay(display_name, z_value) {}
QRectF DisplayDemo::boundingRect() const {}
void DisplayDemo::paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget) {
  drawParticle(painter, particle_data_);
}

DisplayDemo::~DisplayDemo() {}
bool DisplayDemo::UpdateData(const std::any &data) {
  try {
    particle_data_ = std::any_cast<Display::ParticlePointsType>(data);
    update();
  } catch (const std::bad_any_cast &e) {
    std::cout << e.what() << '\n';
  }
}
void DisplayDemo::drawParticle(QPainter *painter,
                                  Display::ParticlePointsType particle) {}
