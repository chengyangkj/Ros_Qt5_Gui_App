/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-04-20 16:44:15
 * @FilePath: ////src/display/laser_points.cpp
 * @Description:
 */
#include "display/display_cost_map.h"
DisplayCostMap::DisplayCostMap(const std::string &display_name,
                               const int &z_value)
    : VirtualDisplay(display_name, z_value) {}
QRectF DisplayCostMap::boundingRect() const {}
void DisplayCostMap::paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget) {
  drawFrame(painter);
}

DisplayCostMap::~DisplayCostMap() {}
bool DisplayCostMap::UpdateData(const std::any &data) {
  try {
    particle_data_ = std::any_cast<Display::ParticlePointsType>(data);
    update();
  } catch (const std::bad_any_cast &e) {
    std::cout << e.what() << '\n';
  }
}
void DisplayCostMap::drawFrame(QPainter *painter) {}
