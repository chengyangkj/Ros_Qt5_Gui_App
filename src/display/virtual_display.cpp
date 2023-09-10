#include "display/virtual_display.h"
VirtualDisplay::VirtualDisplay(const std::string& display_name,
                               const int& z_value)
    : display_name_(display_name) {
  FactoryDisplay::Instance()->AddDisplay(this);
  this->setZValue(z_value);
  pose_cursor_ = new QCursor(QPixmap("://images/cursor_pos.png"), 0, 0);
  goal_cursor_ = new QCursor(QPixmap("://images/cursor_pos.png"), 0, 0);
  move_cursor_ = new QCursor(QPixmap("://images/MoveCamera.png"), 0, 0);
}
VirtualDisplay::~VirtualDisplay() {
  if (pose_cursor_ != nullptr) {
    delete pose_cursor_;
    pose_cursor_ = nullptr;
  }
  if (goal_cursor_ != nullptr) {
    delete goal_cursor_;
    goal_cursor_ = nullptr;
  }
  if (move_cursor_ != nullptr) {
    delete move_cursor_;
    move_cursor_ = nullptr;
  }
  if (curr_cursor_ != nullptr) {
    delete curr_cursor_;
    curr_cursor_ = nullptr;
  }
}
bool VirtualDisplay::SetDisplayConfig(const std::string& config_name,
                                      const std::any& config_data) {}
bool VirtualDisplay::SetScaled(const double& value) {
  if (!enable_scale_) return false;
  scale_value_ = value;
  setScale(value);
  update();
  emit displaySetScaled(display_name_, value);
  return true;
}
void VirtualDisplay::SetBoundingRect(QRectF rect) {
  bounding_rect_ = rect;

  std::cout << "set  bounding_rect_:" << bounding_rect_.width() << " "
            << bounding_rect_.height() << std::endl;
}
void VirtualDisplay::Update() { update(); }
std::string VirtualDisplay::GetDisplayName() { return display_name_; }
void VirtualDisplay::SetDisplayName(const std::string& display_name) {
  display_name_ = display_name;
}
void VirtualDisplay::wheelEvent(QGraphicsSceneWheelEvent* event) {
  if (!is_response_mouse_event_) {
    // 如果当前图层不响应鼠标时间,则事件向下传递
    QGraphicsItem::wheelEvent(event);
    return;
  }

  this->setCursor(Qt::CrossCursor);
  double beforeScaleValue = scale_value_;
  if (event->delta() > 0) {
    //  qDebug()<<"放大";
    scale_value_ *= 1.1;  // 每次放大10% 
  } else {
    //  qDebug()<<"缩小";
    scale_value_ *= 0.9;  // 每次缩小10%
  }
  // qDebug()<<"scale:"<<scale_value_;
  SetScaled(scale_value_);

  // 使放大缩小的效果看起来像是以鼠标中心点进行放大缩小
  if (event->delta() > 0) {
    moveBy(-event->pos().x() * beforeScaleValue * 0.1,
           -event->pos().y() * beforeScaleValue * 0.1);
  } else {
    moveBy(event->pos().x() * beforeScaleValue * 0.1,
           event->pos().y() * beforeScaleValue * 0.1);
  }
  update();
  emit scenePoseChanged(display_name_, scenePos());
  emit displayUpdated(display_name_);
}
void VirtualDisplay::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  if (!is_response_mouse_event_) {
    // 如果当前图层不响应鼠标时间,则事件向下传递
    QGraphicsItem::mouseMoveEvent(event);
    return;
  }
  if (is_mouse_press_) {
    if (curr_cursor_ == move_cursor_) {
      QPointF point = (event->pos() - pressed_pose_) * scale_value_;
      moveBy(point.x(), point.y());
      // std::cout << "mouse move:" << GetDisplayName() << "(" << event->pos().x()
      //           << "," << event->pos().y() << ")" << std::endl;
    }
    end_pose_ = event->pos();
  }

  ////////////////////////// 旋转
  // QPointF loacalPos = event->pos();

  // // 获取并设置为单位向量
  // QVector2D startVec(pressed_pose_.x() - 0, pressed_pose_.y() - 0);
  // startVec.normalize();
  // QVector2D endVec(loacalPos.x() - 0, loacalPos.y() - 0);
  // endVec.normalize();

  // // 单位向量点乘，计算角度
  // qreal dotValue = QVector2D::dotProduct(startVec, endVec);
  // if (dotValue > 1.0)
  //   dotValue = 1.0;
  // else if (dotValue < -1.0)
  //   dotValue = -1.0;

  // dotValue = qAcos(dotValue);
  // if (isnan(dotValue)) dotValue = 0.0;

  // // 获取角度
  // qreal angle = dotValue * 1.0 / (PI / 180);

  // // 向量叉乘获取方向
  // QVector3D crossValue =
  //     QVector3D::crossProduct(QVector3D(startVec, 1.0),
  //     QVector3D(endVec, 1.0));

  // if (crossValue.z() < 0) angle = -angle;
  // rotate_value_ += angle;

  // // 设置变化矩阵
  // transform_.rotate(rotate_value_);
  // this->setTransform(transform_);

  update();
  emit scenePoseChanged(display_name_, scenePos());
  emit displayUpdated(display_name_);
}
void VirtualDisplay::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  if (!is_response_mouse_event_) {
    // 如果当前图层不响应鼠标时间,则事件向下传递
    QGraphicsItem::mousePressEvent(event);
    return;
  }
  pressed_button_ = event->button();
  if (event->button() == Qt::LeftButton) {
    pressed_pose_ = event->pos();
    is_mouse_press_ = true;
    start_pose_ = event->pos();
    curr_cursor_ = move_cursor_;
    this->setCursor(*curr_cursor_);
  }
  update();
  emit scenePoseChanged(display_name_, scenePos());
  emit displayUpdated(display_name_);
}
void VirtualDisplay::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  if (!is_response_mouse_event_) {
    // 如果当前图层不响应鼠标时间,则事件向下传递
    QGraphicsItem::mouseReleaseEvent(event);
    return;
  }
  if (pressed_button_ == event->button()) pressed_button_ == Qt::NoButton;
  if (event->button() == Qt::LeftButton) {
    pressed_pose_ = QPointF();
    is_mouse_press_ = false;
    if (curr_cursor_ == pose_cursor_) {
      curr_cursor_ = move_cursor_;
      this->setCursor(*curr_cursor_);
    } else if (curr_cursor_ == goal_cursor_) {
      curr_cursor_ = move_cursor_;
      this->setCursor(*curr_cursor_);
    } else if (curr_cursor_ == move_cursor_) {
      curr_cursor_ = move_cursor_;
      this->setCursor(*curr_cursor_);
    }
    start_pose_ = QPointF();
    end_pose_ = QPointF();
  }
  update();
  emit scenePoseChanged(display_name_, scenePos());
  emit displayUpdated(display_name_);
}
void VirtualDisplay::hoverMoveEvent(QGraphicsSceneHoverEvent* event) {
  if (!is_response_mouse_event_) {
    // 如果当前图层不响应鼠标时间,则事件向下传递
    QGraphicsItem::hoverMoveEvent(event);
    return;
  }
  update();
  emit scenePoseChanged(display_name_, scenePos());
  emit updateCursorPose(display_name_, event->pos());
}
