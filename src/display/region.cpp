/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-05-11 16:45:43
 * @FilePath: /hontai/src/tools/localizationViewer/src/display/laser_points.cpp
 * @Description:
 */
#include "display/region.h"
Region::Region(const std::string &display_name, const int &z_value)
    : VirtualDisplay(display_name, z_value) {}
QRectF Region::boundingRect() const { return bounding_rect_; }
void Region::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                   QWidget *widget) {
  for (auto [region_name, region] : region_data_) {
    // std::cout << "region name:" << region_name << std::endl;
    for (auto one_region : region) {
      // std::cout << "region:" << one_region[0] << " " << one_region[1] << " "
      //           << one_region[2] << " " << one_region[3] << std::endl;
      QPen pen;
      pen.setColor(QColor(0, 255, 0, 70));
      QBrush brush;
      brush.setColor(QColor(0, 255, 0, 70));
      brush.setStyle(Qt::SolidPattern);
      painter->setPen(pen);
      painter->setBrush(brush);

      painter->drawRect(QRectF(QPointF(one_region[0], one_region[1]),
                               QPointF(one_region[2], one_region[3])));
      painter->setPen(Qt::blue);
      QFont font;

      font.setCapitalization(QFont::SmallCaps);
      // 大小
      font.setPixelSize(bounding_rect_.height() / 50);
      painter->setFont(font);
      painter->drawText(QRectF(QPointF(one_region[0], one_region[1]),
                               QPointF(one_region[2], one_region[3])),
                        Qt::AlignCenter, QString::fromStdString(region_name));
    }
  }
}
void Region::computeBoundRect(Display::RegionDataMap &data_map) {
  float xmax = -99999, xmin = 99999, ymax = -99999, ymin = 99999;
  for (auto [region_name, region] : region_data_) {
    for (auto one_region : region) {
      if (xmin > one_region[0]) xmin = one_region[0];
      if (xmax < one_region[1]) xmax = one_region[1];

      if (ymin > one_region[2]) ymin = one_region[2];
      if (ymax < one_region[3]) ymax = one_region[3];
    }
    // std::cout << "xmax:" << xmax << "xmin:" << xmin << "ymax:" << ymax
    //           << "ymin:" << ymin << std::endl;
    bounding_rect_ = QRectF(xmin, ymin, xmax - xmin, ymax - ymin);
  }
}
Region::~Region() {}
bool Region::UpdateData(const std::any &data) {
  if (data.type() == typeid(Display::RegionDataMap)) {
    region_data_ = std::any_cast<Display::RegionDataMap>(data);
    computeBoundRect(region_data_);
    update();
    return true;
  }
}
