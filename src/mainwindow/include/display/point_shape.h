/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2022-12-15 09:59:43
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-07-25 16:29:16
 * @FilePath: ////include/PointShape.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef PointShape_H
#define PointShape_H
// NOLINTBEGIN
#include <QCursor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QMenu>
#include <QObject>
#include <QPainter>
#include <QSvgRenderer>
#include "virtual_display.h"
namespace Display {
class PointShape : public VirtualDisplay {
  Q_OBJECT
 public:
  enum ePointType { kRobot,
                    kParticle,
                    kNavGoal };
  PointShape(const ePointType &type, const std::string &display_type,
             const std::string &display_name, const int &z_value,
             std::string parent_name = "");
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  bool UpdateData(const std::any &data) override;
  bool SetDisplayConfig(const std::string &config_name,
                        const std::any &config_data) override;
  // void contextMenuEvent(QGraphicsSceneContextMenuEvent *event) override;

 private:
  QSvgRenderer robot_svg_renderer_;
  ePointType type_;
  RobotPose robot_pose_;
  double deg_offset_{0};
  bool enable_{true};

 private:
  void drawRobot(QPainter *painter);
  void drawNavGoal(QPainter *painter);
  void drawParticle(QPainter *painter);
  void setEnable(const bool &enable);
  // 角度转弧度
  inline double deg2rad(double x) { return M_PI * x / 180.0; }
  // 弧度转角度
  inline double rad2deg(double x) { return 180.0 * x / M_PI; }
  QVariant itemChange(GraphicsItemChange change,
                      const QVariant &value) override;
};
// NOLINTEND
}  // namespace Display
#endif  // PointShape_H
