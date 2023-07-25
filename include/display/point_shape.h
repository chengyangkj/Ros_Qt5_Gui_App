/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2022-12-15 09:59:43
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-07-25 15:20:26
 * @FilePath: /hontai/src/tools/localizationViewer/include/PointShape.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef PointShape_H
#define PointShape_H
// NOLINTBEGIN
#include <QCursor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QObject>
#include <QPainter>

#include "virtual_display.h"

class PointShape : public VirtualDisplay {
 public:
  enum ePointType { kRobot, kParticle };
  PointShape(const ePointType &type, const std::string &display_name,
             const int &z_value);
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  QRectF boundingRect() const override;
  bool UpdateData(const std::any &data) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;

 private:
  QPixmap robot_image_;
  ePointType type_;
  Eigen::Vector3f robot_pose_;

 private:
  void drawRobot(QPainter *painter);
  void drawParticle(QPainter *painter);

  // 角度转弧度
  inline double deg2rad(double x) { return M_PI * x / 180.0; }
  // 弧度转角度
  inline double rad2deg(double x) { return 180.0 * x / M_PI; }
};
// NOLINTEND
#endif  // PointShape_H
