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
namespace Display {
PointShape::PointShape(const ePointType &type, const std::string &display_name,
                       const int &z_value, std::string parent_name)
    : VirtualDisplay(display_name, z_value, parent_name), type_(type) {
  SetRotateEnable(true);
  SetScaleEnable(false);
  moveBy(0, 0);
  switch (type_) {
  case kRobot: {
    robot_image_.load("://images/dir.png");
    QMatrix matrix;
    matrix.rotate(45);
    robot_image_ = robot_image_.transformed(matrix, Qt::SmoothTransformation);
    SetBoundingRect(QRectF(0 - robot_image_.width() / 2,
                           0 - robot_image_.height() / 2, robot_image_.width(),
                           robot_image_.height()));
  } break;
  case kParticle: {

  } break;
  case kNavGoal: {
    robot_image_.load("://images/goal_green.png");
    QMatrix matrix;
    matrix.rotate(90);
    robot_image_ = robot_image_.transformed(matrix, Qt::SmoothTransformation);
    SetBoundingRect(QRectF(0 - robot_image_.width() / 2,
                           0 - robot_image_.height() / 2, robot_image_.width(),
                           robot_image_.height()));
  } break;
  }
}
bool PointShape::UpdateData(const std::any &data) {
  GetAnyData(Eigen::Vector3f, data, robot_pose_);
  update();
}
bool PointShape::SetDisplayConfig(const std::string &config_name,
                                  const std::any &config_data) {
  if (config_name == "Enable") {
    GetAnyData(bool, config_data, enable_);
  } else {
    return false;
  }
  return true;
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
  case kNavGoal:
    drawNavGoal(painter);
    break;
  }
  static QPointF last_scene_pose = scenePos();
  QPointF diff = last_scene_pose - scenePos();
  static double last_rotate_value = rotate_value_;
  if (fabs(diff.x()) >= 1 || fabs(diff.y()) >= 1 ||
      fabs(last_rotate_value - rotate_value_) >= 1) {
    last_scene_pose = scenePos();
    last_rotate_value = rotate_value_;
    emit signalPointScenePoseUpdate(
        Eigen::Vector3f(scenePos().x(), scenePos().y(), rotate_value_));
  }
}
void PointShape::setEnable(const bool &enable) {}
void PointShape::drawRobot(QPainter *painter) {
  painter->setRenderHint(QPainter::Antialiasing, true); // 设置反锯齿 反走样
  painter->save();
  double deg = robot_pose_[2];
  painter->rotate(rad2deg(-deg) + rotate_value_);
  painter->drawPixmap(-robot_image_.width() / 2, -robot_image_.height() / 2,
                      robot_image_);

  painter->restore();
}
void PointShape::drawNavGoal(QPainter *painter) {
  painter->setRenderHint(QPainter::Antialiasing, true); // 设置反锯齿 反走样
  painter->save();
  painter->rotate(0 + rotate_value_);
  painter->drawPixmap(-robot_image_.width() / 2, -robot_image_.height() / 2,
                      robot_image_);
  painter->restore();
}

void PointShape::drawParticle(QPainter *painter) {}
// NOLINTEND
} // namespace Display