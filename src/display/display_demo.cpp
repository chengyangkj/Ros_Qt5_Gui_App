/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-14 09:47:41
 * @FilePath: ////src/display/laser_points.cpp
 * @Description:
 */
#include "display/display_demo.h"
DisplayDemo::DisplayDemo(const std::string &display_name, const int &z_value)
    : VirtualDisplay(display_name, z_value) {}
void DisplayDemo::paint(QPainter *painter,
                        const QStyleOptionGraphicsItem *option,
                        QWidget *widget) {
  drawFrame(painter);
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
void DisplayDemo::drawFrame(QPainter *painter) {}
