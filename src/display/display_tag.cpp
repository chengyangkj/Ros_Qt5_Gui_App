/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-04-10 15:38:40
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-05-12 14:37:07
 * @FilePath: /hontai/src/tools/localizationViewer/src/display/laser_points.cpp
 * @Description:
 */
#include "display/display_tag.h"
DisplayTag::DisplayTag(const std::string &display_name, const int &z_value)
    : VirtualDisplay(display_name, z_value) {
  tag_image_.load("://images/qr.png");
  // enable_scale_ = false;
}
QRectF DisplayTag::boundingRect() const { return bounding_rect_; }
void DisplayTag::paint(QPainter *painter,
                       const QStyleOptionGraphicsItem *option,
                       QWidget *widget) {
  painter->setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
  for (auto [qr_name, pose] : tag_data_) {
    // std::cout << "paint qr_name:" << qr_name << std::endl;
    // 坐标转换为图元坐标系
    int x, y;
    map_data_.xy2scene(pose[0], pose[1], x, y);
    painter->drawPixmap(x - tag_image_.width() / 2, y - tag_image_.height() / 2,
                        tag_image_);
  }
}

DisplayTag::~DisplayTag() {}
bool DisplayTag::UpdateData(const std::any &data) {
  if (data.type() == typeid(Display::TagDataMap)) {
    tag_data_ = std::any_cast<Display::TagDataMap>(data);
    // std::cout << "update tag data size: " << tag_data_.size() << std::endl;
    computeBoundRect(tag_data_);
    update();
    return true;
  }
}
void DisplayTag::computeBoundRect(Display::TagDataMap &data_map) {
  float xmax = -99999, xmin = 99999, ymax = -99999, ymin = 99999;
  // std::cout << "data map size:" << data_map.size() << std::endl;
  for (auto [region_name, pose] : data_map) {
    int x, y;
    map_data_.xy2scene(pose[0], pose[1], x, y);
    if (xmin > x) xmin = x;
    if (xmax < x) xmax = x;

    if (ymin > y) ymin = y;
    if (ymax < y) ymax = y;

    // std::cout << "xmax:" << xmax << "xmin:" << xmin << "ymax:" << ymax
    //           << "ymin:" << ymin <<" x:"<<x<<" y:"<<pose[1]<< std::endl;
  }
  bounding_rect_ = QRectF(xmin, ymin, xmax - xmin, ymax - ymin);
}
