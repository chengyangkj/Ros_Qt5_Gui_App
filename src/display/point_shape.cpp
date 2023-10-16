/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2022-12-15 09:59:43
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-14 09:48:15
 * @FilePath: ////src/PointShape.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// NOLINTBEGIN
#include "display/point_shape.h"

#include "QDebug"

PointShape::PointShape(const ePointType &type, const std::string &display_name,
                       const int &z_value)
    : VirtualDisplay(display_name, z_value), type_(type) {
  enable_scale_ = false;
  moveBy(0, 0);
  robot_image_.load("://images/dir.png");
  QMatrix matrix;
  matrix.rotate(45);
  robot_image_ = robot_image_.transformed(matrix, Qt::SmoothTransformation);
  SetBoundingRect(QRectF(0 - robot_image_.width() / 2,
                         0 - robot_image_.height() / 2, robot_image_.width(),
                         robot_image_.height()));
}
bool PointShape::UpdateData(const std::any &data) {
  GetAnyData(Eigen::Vector3f, data, robot_pose_);
  update();
}
void PointShape::paint(QPainter *painter,
                       const QStyleOptionGraphicsItem *option,
                       QWidget *widget) {
  switch (type_) {
  case kRobot:
    drawRobot(painter);
    break;
  case kParticle:
    drawParticle(painter);
    break;
  }
}

void PointShape::drawRobot(QPainter *painter) {
  painter->setRenderHint(QPainter::Antialiasing, true); // 设置反锯齿 反走样
  painter->save();
  double deg = robot_pose_[2];
  double real_angle = rad2deg(-deg) + rotate_value_;
  painter->rotate(real_angle);
  painter->drawPixmap(-robot_image_.width() / 2, -robot_image_.height() / 2,
                      robot_image_);

  painter->restore();
}

void PointShape::drawParticle(QPainter *painter) {}
// NOLINTEND
