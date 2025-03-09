/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-11 10:13:22
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-09-17 08:34:39
 * @FilePath:
 * ////include/display/display_demo.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <Eigen/Dense>
#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include "config/config_manager.h"
#include "virtual_display.h"
namespace Display {

class RobotShape : public VirtualDisplay {
 private:
  QPainterPath path_;
  RobotPose robot_pose_{0, 0, 0};
  QColor color_{0x1E90FF};
  float opacity_{0.5};

 private:
  void drawFrame(QPainter *painter);

 public:
  RobotShape(const std::string &display_type, const int &z_value,
             std::string parent_name = "");
  ~RobotShape();
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget = nullptr) override;
  bool UpdateData(const std::any &data) override;
};

}  // namespace Display