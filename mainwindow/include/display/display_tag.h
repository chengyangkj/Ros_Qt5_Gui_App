#ifndef DISPLAY_TAG
#define DISPLAY_TAG
#include <Eigen/Dense>
#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>

#include "virtual_display.h"

namespace Display {
class DisplayTag : public VirtualDisplay {

public:
  Display::TagDataMap tag_data_;
  QPixmap tag_image_;
  DisplayTag(const std::string &display_name, const int &z_value,
             std::string parent_name = "");
  ~DisplayTag();
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  bool UpdateData(const std::any &data) override;
  void computeBoundRect(Display::TagDataMap &data_map);
};
} // namespace Display
#endif
