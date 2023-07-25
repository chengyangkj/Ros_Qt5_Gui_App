/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-04-11 10:13:22
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-04-20 17:02:58
 * @FilePath:
 * /hontai/src/tools/localizationViewer/include/display/display_demo.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef PARTICLE_POINTS_H
#define PARTICLE_POINTS_H
#include <Eigen/Dense>
#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>

#include "virtual_display.h"
class ParticlePoints : public VirtualDisplay {
 private:
  void drawParticle(QPainter *painter, Display::ParticlePointsType particle);

 public:
  Display::ParticlePointsType particle_data_;
  ParticlePoints(const std::string &display_name, const int &z_value);
  ~ParticlePoints();
  void computeBoundRect(const Display::ParticlePointsType &particle);
  QRectF boundingRect() const override;
  QRectF boundRect_;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  bool UpdateData(const std::any &data) override;
};

#endif
