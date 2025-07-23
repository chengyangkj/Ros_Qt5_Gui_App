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
#include <cmath>
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
      setZValue(10);
      robot_svg_renderer_.load(QString("://images/robot.svg"));
      deg_offset_ = 45;
      SetBoundingRect(QRectF(0 - robot_svg_renderer_.defaultSize().width() / 2,
                             0 - robot_svg_renderer_.defaultSize().height() / 2, robot_svg_renderer_.defaultSize().width(),
                             robot_svg_renderer_.defaultSize().height()));
    } break;
    case kParticle: {
    } break;
    case kNavGoal: {
      setZValue(9);
      robot_svg_renderer_.load(QString("://images/target.svg"));
      deg_offset_ = 0;
      SetBoundingRect(QRectF(0 - robot_svg_renderer_.defaultSize().width() / 2,
                             0 - robot_svg_renderer_.defaultSize().height() / 2, robot_svg_renderer_.defaultSize().width(),
                             robot_svg_renderer_.defaultSize().height()));
    } break;
  }
 
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
  
  // 设置目标图标透明度
  painter->save();
  
  // 绘制不旋转的目标SVG图标
  QRectF targetRect(-robot_svg_renderer_.defaultSize().width() / 2, 
                    -robot_svg_renderer_.defaultSize().height() / 2, 
                    robot_svg_renderer_.defaultSize().width(), 
                    robot_svg_renderer_.defaultSize().height());
  robot_svg_renderer_.render(painter, targetRect);
  
  painter->restore();
  
  // 在正右侧添加小箭头，只让箭头绕中心旋转
  painter->save();
  
  // 计算总旋转角度
  double total_rotation = (-rad2deg(robot_pose_.theta) - rad2deg(rotate_value_) + deg_offset_);
  
  // 基于targetRect计算箭头距离
  double arrow_distance = targetRect.width() / 2.8 ;
  
  // 根据旋转角度计算箭头的位置
  double arrow_x = arrow_distance * cos(deg2rad(total_rotation));
  double arrow_y = arrow_distance * sin(deg2rad(total_rotation));
  
  // 移动到箭头位置并旋转箭头
  painter->translate(arrow_x, arrow_y);
  painter->rotate(total_rotation);
  
  // 计算箭头尺寸：底边为图片宽度的2/3
  double arrow_base_width = targetRect.width() / 3.0;
  double arrow_height = arrow_base_width / 3.0;  // 高度为底边的一半，保持美观比例
  
  // 绘制箭头（三角形）- 使用与绿色图标搭配的半透明深绿色
  painter->setPen(QPen(QColor(34, 139, 34, 200), 2));   // 森林绿边框，透明度200/255
  painter->setBrush(QBrush(QColor(50, 205, 50, 180)));  // 酸橙绿填充，透明度180/255
  
  QPolygonF arrow;
  arrow << QPointF(arrow_height, 0)                    // 箭头尖端
        << QPointF(-arrow_height/2, -arrow_base_width/2)  // 上方点
        << QPointF(-arrow_height/2, arrow_base_width/2);  // 下方点
  
  painter->drawPolygon(arrow);
  
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