#ifndef LASER_SCAN_H
#define LASER_SCAN_H
#include <Eigen/Dense>
#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>

#include "virtual_display.h"
using namespace basic;
namespace Display {
class LaserPoints : public VirtualDisplay {
 public:
  QColor laser_color_;
  std::map<int, std::vector<Point>> laser_data_scene_;
  LaserPoints(const std::string &display_type, const int &z_value,
              std::string parent_name = "");
  ~LaserPoints();
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  bool UpdateData(const std::any &data) override;
  bool SetDisplayConfig(const std::string &config_name,
                        const std::any &config_data) override;

 private:
  void Id2Color(int id, int &R, int &G, int &B);
  void drawLaser(QPainter *painter, int id, std::vector<Point>);
  void computeBoundRect(const std::map<int, std::vector<Point>> &laser_scan);

 private:
  std::map<int, QColor> location_to_color_;
};
}  // namespace Display
#endif
