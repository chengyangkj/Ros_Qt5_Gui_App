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
#include "algorithm.h"
using namespace basic;
#define circle_radius 20
namespace Display {
PointShape::PointShape(const ePointType &type, const std::string &display_type,
                       const std::string &display_name, const int &z_value,
                       std::string parent_name)
    : VirtualDisplay(display_type, z_value, parent_name, display_name),
      type_(type) {
  SetRotateEnable(true);
  SetScaleEnable(false);
  moveBy(0, 0);
  switch (type_) {
    case kRobot: {
      robot_svg_renderer_.load(QString("://images/robot.svg"));
      deg_offset_ = 45;
      SetBoundingRect(QRectF(0 - robot_svg_renderer_.defaultSize().width() / 2,
                             0 - robot_svg_renderer_.defaultSize().height() / 2, robot_svg_renderer_.defaultSize().width(),
                             robot_svg_renderer_.defaultSize().height()));
    } break;
    case kParticle: {
    } break;
    case kNavGoal: {
      robot_svg_renderer_.load(QString("://images/target.svg"));
      deg_offset_ = -135;
      SetBoundingRect(QRectF(0 - robot_svg_renderer_.defaultSize().width() / 2,
                             0 - robot_svg_renderer_.defaultSize().height() / 2, robot_svg_renderer_.defaultSize().width(),
                             robot_svg_renderer_.defaultSize().height()));
    } break;
  }
  // this->setCursor(Qt::);
}
QVariant PointShape::itemChange(GraphicsItemChange change,
                                const QVariant &value) {
  switch (change) {
    case ItemPositionHasChanged:
      curr_scene_pose_ = RobotPose(scenePos().x(), scenePos().y(),
                                   normalize(robot_pose_.theta + rotate_value_));
      emit signalPoseUpdate(curr_scene_pose_);
      break;
    // case ItemTransformHasChanged:
    //   emit signalPoseUpdate(
    //       Eigen::Vector3f(scenePos().x(), scenePos().y(), rotate_value_));
    //   break;
    // case ItemRotationHasChanged:
    //   emit signalPoseUpdate(
    //       Eigen::Vector3f(scenePos().x(), scenePos().y(), rotate_value_));
    //   break;
    default:
      break;
  };
  return QGraphicsItem::itemChange(change, value);
}
bool PointShape::UpdateData(const std::any &data) {
  GetAnyData(RobotPose, data, robot_pose_);
  rotate_value_ = 0;
  SetPoseInParent(robot_pose_);
  update();
  return true;
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
  painter->setRenderHints(QPainter::Antialiasing |
                          QPainter::SmoothPixmapTransform);
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
  static double last_rotate_value = rotate_value_;

  if (fabs(rotate_value_ - last_rotate_value) > deg2rad(0.1)) {
    curr_scene_pose_ = RobotPose(scenePos().x(), scenePos().y(),
                                 normalize(robot_pose_.theta + rotate_value_));
    emit signalPoseUpdate(
        RobotPose(scenePos().x(), scenePos().y(),
                  normalize(robot_pose_.theta + rotate_value_)));
  }
}
void PointShape::setEnable(const bool &enable) {}
void PointShape::drawRobot(QPainter *painter) {
  painter->setRenderHint(QPainter::Antialiasing, true);  // 设置反锯齿 反走样
  painter->save();
  painter->rotate(-rad2deg(robot_pose_.theta) - rad2deg(rotate_value_) + deg_offset_);
  QRectF targetRect(-robot_svg_renderer_.defaultSize().width() / 2, -robot_svg_renderer_.defaultSize().height() / 2, robot_svg_renderer_.defaultSize().width(), robot_svg_renderer_.defaultSize().height());
  // 将SVG图形渲染到QPainter
  robot_svg_renderer_.render(painter, targetRect);
  painter->restore();
}
void PointShape::drawNavGoal(QPainter *painter) {
  painter->setRenderHint(QPainter::Antialiasing, true);  // 设置反锯齿 反走样
  painter->save();
  painter->rotate(-rad2deg(robot_pose_.theta) - rad2deg(rotate_value_) + deg_offset_);
  QRectF targetRect(-robot_svg_renderer_.defaultSize().width() / 2, -robot_svg_renderer_.defaultSize().height() / 2, robot_svg_renderer_.defaultSize().width(), robot_svg_renderer_.defaultSize().height());
  // 将SVG图形渲染到QPainter
  robot_svg_renderer_.render(painter, targetRect);
  painter->restore();
}
// void PointShape::contextMenuEvent(QGraphicsSceneContextMenuEvent *event) {
//   QMenu menu;
//   QAction *removeAction = menu.addAction("Navigation");
//   QAction *selectedAction = menu.addAction("Delete");
//   menu.exec(event->screenPos());
//   connect(removeAction, SIGNAL(triggered()), this, SLOT(slotRemoveItem()));
// }
void PointShape::drawParticle(QPainter *painter) {}
// NOLINTEND
}  // namespace Display