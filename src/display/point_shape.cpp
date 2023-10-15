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
  SetBoundingRect(QRectF(0, 0, robot_image_.width(), robot_image_.height()));
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
  painter->rotate(rad2deg(-deg));
  painter->drawPixmap(-robot_image_.width() / 2, -robot_image_.height() / 2,
                      robot_image_);
  painter->restore();
}
void PointShape::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  if (!is_response_mouse_event_) {
    // 如果当前图层不响应鼠标时间,则事件向下传递
    QGraphicsItem::mouseMoveEvent(event);
    return;
  }
  // 左健移动
  if (pressed_button_ == Qt::LeftButton) {
    if (curr_cursor_ == move_cursor_) {
      QPointF point = (event->pos() - pressed_pose_) * scale_value_;
      moveBy(point.x(), point.y());
    }
    end_pose_ = event->pos();
  }
  ////////////////////////// 右健旋转
  if (pressed_button_ == Qt::RightButton) {
    QPointF loacalPos = event->pos();

    // 获取并设置为单位向量
    QVector2D startVec(pressed_pose_.x() - 0, pressed_pose_.y() - 0);
    startVec.normalize();
    QVector2D endVec(loacalPos.x() - 0, loacalPos.y() - 0);
    endVec.normalize();

    // 单位向量点乘，计算角度
    qreal dotValue = QVector2D::dotProduct(startVec, endVec);
    if (dotValue > 1.0)
      dotValue = 1.0;
    else if (dotValue < -1.0)
      dotValue = -1.0;

    dotValue = qAcos(dotValue);
    if (isnan(dotValue))
      dotValue = 0.0;

    // 获取角度
    qreal angle = dotValue * 1.0 / (PI / 180);

    // 向量叉乘获取方向
    QVector3D crossValue = QVector3D::crossProduct(QVector3D(startVec, 1.0),
                                                   QVector3D(endVec, 1.0));

    if (crossValue.z() < 0)
      angle = -angle;
    rotate_value_ += angle;

    // 设置变化矩阵
    transform_.rotate(rotate_value_);
    this->setTransform(transform_);
  }

  update();
  emit scenePoseChanged(display_name_, scenePos());
  emit displayUpdated(display_name_);
}
void PointShape::drawParticle(QPainter *painter) {}
// NOLINTEND
