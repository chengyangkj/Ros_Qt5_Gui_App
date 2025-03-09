/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-14 09:47:41
 * @FilePath: ////src/display/laser_points.cpp
 * @Description:
 */
#include "display/display_demo.h"
namespace Display {
DisplayDemo::DisplayDemo(const std::string &display_type, const int &z_value,
                         std::string parent_name)
    : VirtualDisplay(display_type, z_value, parent_name) {}
void DisplayDemo::paint(QPainter *painter,
                        const QStyleOptionGraphicsItem *option,
                        QWidget *widget) {
  drawFrame(painter);
}

DisplayDemo::~DisplayDemo() {}
bool DisplayDemo::UpdateData(const std::any &data) {
  try {

    update();
  } catch (const std::bad_any_cast &e) {
    std::cout << e.what() << '\n';
  }
  return true;
}
void DisplayDemo::drawFrame(QPainter *painter) {}
} // namespace Display