
/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-28 10:21:04
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 02:49:39
 * @FilePath: ////src/display/robot_map.cpp
 */
#include <algorithm>
#include <iostream>

#include "display/display_cost_map.h"
namespace Display {
DisplayCostMap::DisplayCostMap(const std::string &display_type,
                               const int &z_value, std::string parent_name)
    : VirtualDisplay(display_type, z_value, parent_name) {
}
bool DisplayCostMap::UpdateData(const std::any &data) {
  try {
    if (data.type() == typeid(OccupancyMap))
      cost_map_data_ = std::any_cast<OccupancyMap>(data);
  } catch (const std::bad_any_cast &e) {
    std::cout << e.what() << '\n';
    return false;
  }
  ParseCostMap();
  //绘制原点在地图中心
  SetBoundingRect(QRectF(0, 0, map_image_.width(), map_image_.height()));
  update();
  return true;
}
bool DisplayCostMap::SetDisplayConfig(const std::string &config_name,
                                      const std::any &config_data) {
  update();
  return true;
}
void DisplayCostMap::paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget) {
  //以图片中心做原点进行绘制(方便旋转)
  painter->drawImage(0, 0, map_image_);
}
void DisplayCostMap::ParseCostMap() {
  Eigen::Matrix<Eigen::Vector4i, Eigen::Dynamic, Eigen::Dynamic> cost_map =
      cost_map_data_.GetCostMapData();
  map_image_ = QImage(cost_map_data_.Cols(), cost_map_data_.Rows(),
                      QImage::Format_ARGB32);
  // map_image_.save("./test.png");
  for (int i = 0; i < cost_map.cols(); i++)
    for (int j = 0; j < cost_map.rows(); j++) {
      Eigen::Vector4i color_data = cost_map(j, i);
      QColor color;
      color.setRgb(color_data[0], color_data[1], color_data[2]);
      color.setAlpha(color_data[3]);
      map_image_.setPixelColor(i, j, color);
    }
}

} // namespace Display