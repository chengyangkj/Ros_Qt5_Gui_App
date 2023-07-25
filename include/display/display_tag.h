/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-04-11 10:13:22
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-05-11 15:17:57
 * @FilePath:
 * /hontai/src/tools/localizationViewer/include/display/display_demo.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef DISPLAY_TAG
#define DISPLAY_TAG
#include <Eigen/Dense>
#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>

#include "virtual_display.h"

class DisplayTag : public VirtualDisplay {

 public:
  Display::TagDataMap tag_data_;
    QPixmap tag_image_;
  DisplayTag(const std::string &display_name, const int &z_value);
  ~DisplayTag();
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  bool UpdateData(const std::any &data) override;
  void computeBoundRect(Display::TagDataMap &data_map);
};

#endif
