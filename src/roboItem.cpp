#include "roboItem.h"

#include "QDebug"

roboItem::roboItem() {
  setAcceptHoverEvents(true);
  setAcceptedMouseButtons(Qt::AllButtons);
  setAcceptDrops(true);
  setFlag(ItemAcceptsInputMethod, true);
  moveBy(0, 0);
  m_robotImg.load("://images/robot_blue.png");
  QMatrix matrix;
  matrix.rotate(90);
  m_robotImg = m_robotImg.transformed(matrix, Qt::SmoothTransformation);
  m_set2DPoseCursor = new QCursor(QPixmap("://images/cursor_pos.png"), 0, 0);
  m_set2DGoalCursor = new QCursor(QPixmap("://images/cursor_pos.png"), 0, 0);
  m_moveCursor = new QCursor(QPixmap("://images/cursor_move.png"), 0, 0);
  m_currCursor = m_moveCursor;
  this->setCursor(*m_currCursor);
}
QRectF roboItem::boundingRect() const {
  return QRectF(0, 0, m_map.width() + 100, m_map.height() + 100);
}

void roboItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                     QWidget *widget) {
  //    qDebug()<<"我被调用啦";
  painter->setRenderHint(QPainter::Antialiasing, true);  //设置反锯齿 反走样
  drawImage(painter);
  drawPoints(painter);
  drawLine(painter);
  drawMap(painter);
  drawGlobalCostMap(painter);
  drawLocalCostMap(painter);
  // drawRobotPose(painter);
  drawLaserScan(painter);
  drawPath(painter);
  drawTools(painter);
}
void roboItem::drawPath(QPainter *painter) {
  painter->setPen(QPen(QColor(0, 0, 255), 1));
  painter->drawPoints(m_pathPoints);
}
void roboItem::drawLaserScan(QPainter *painter) {
  painter->setPen(QPen(QColor(255, 0, 0), 1));
  painter->drawPoints(m_laserPoints);
}
void roboItem::drawRobotPose(QPainter *painter) {
  //  qDebug()<<" m_currRobotPose.x:"<<m_currRobotPose.x<<"
  //  m_currRobotPose.y:"<<m_currRobotPose.y;
  //  painter->setPen(QPen(QColor(255,0,0),10));
  //    qDebug()<<"m_currRobotPose:"<<m_currRobotPose.theta;
  painter->save();
  painter->translate(QPointF(m_currRobotPose.x, m_currRobotPose.y));
  painter->setPen(QPen(QColor(255, 0, 0), 1));
  painter->drawPoint(0, 0);
  painter->rotate(rad2deg(-m_currRobotPose.theta));
  painter->drawPixmap(-m_robotImg.width() / 2, -m_robotImg.height() / 2,
                      m_robotImg);
  painter->restore();
}
void roboItem::drawTools(QPainter *painter) {
  if (m_currCursor == m_set2DPoseCursor || m_currCursor == m_set2DGoalCursor) {
    //绘制箭头
    if (m_startPose.x() != 0 && m_startPose.y() != 0 && m_endPose.x() != 0 &&
        m_endPose.y() != 0) {
      QPen pen(QColor(50, 205, 50, 255), 4, Qt::SolidLine, Qt::RoundCap,
               Qt::RoundJoin);
      QBrush brush(QColor(50, 205, 50, 255), Qt::SolidPattern);

      painter->setPen(pen);
      painter->setBrush(brush);
      //计算线弧度
      double theta = atan((m_endPose.y() - m_startPose.y()) /
                          (m_endPose.x() - m_startPose.x()));
      //绘制直线
      QPointF startPoint, endPoint;
      startPoint = m_startPose;
      endPoint = m_endPose;
      QLineF line(startPoint, endPoint);
      painter->drawLine(line);
      float angle =
          atan2(endPoint.y() - startPoint.y(), endPoint.x() - startPoint.x()) +
          3.1415926;  //
      //绘制三角形
      QPolygonF points;
      points.push_back(endPoint);
      QPointF point1, point2;
      point1.setX(endPoint.x() + 10 * cos(angle - 0.5));  //求得箭头点1坐标
      point1.setY(endPoint.y() + 10 * sin(angle - 0.5));
      point2.setX(endPoint.x() + 10 * cos(angle + 0.5));  //求得箭头点2坐标
      point2.setY(endPoint.y() + 10 * sin(angle + 0.5));
      points.push_back(point1);
      points.push_back(point2);
      painter->drawPolygon(points);
    }
  }
}
void roboItem::drawMap(QPainter *painter) { painter->drawImage(0, 0, m_map); }
void roboItem::drawGlobalCostMap(QPainter *painter) {
  //显示方法1 直接绘制图片
  painter->save();
  painter->drawImage(0, 0, m_GlobalCostMap);
  painter->restore();
}
void roboItem::drawLocalCostMap(QPainter *painter) {
  //显示方法1 直接绘制图片
  painter->save();
  painter->translate(m_lastLocalCostMapRobotPose.x,
                     m_lastLocalCostMapRobotPose.y);
  painter->drawImage(
      QPoint(-m_LocalCostMap.width() / 2, -m_LocalCostMap.height() / 2),
      m_LocalCostMap);
  painter->restore();
  //显示方法2 绘制点
  //        painter->save();
  //        painter->translate(m_currRobotPose.x, m_currRobotPose.y);
  //        for(int x=0;x<m_LocalCostMap.width();x++){
  //            for(int y=0;y<m_LocalCostMap.height();y++){
  //              QColor color =  m_LocalCostMap.pixelColor(x,y);
  //              int r,g,b,a;
  //              color.getRgb(&r,&g,&b,&a);
  //              if(color==Qt::transparent) continue;
  //              painter->setPen(QPen(color,1));
  //              painter->drawPoint(-m_LocalCostMap.width() /
  //              2+x,-m_LocalCostMap.height() / 2+y);
  //            }
  //        }
  //        painter->restore();
}
void roboItem::drawImage(QPainter *painter) {
  painter->drawImage(0, 0, m_images);
}
void roboItem::drawPoints(QPainter *painter) {
  painter->setPen(QPen(QColor(0, 0, 255), 2));
  painter->drawPoints(m_points);
}

void roboItem::drawLine(QPainter *painter) {
  painter->setPen(QPen(QColor(255, 0, 0), 1));
  painter->drawLine(m_lines);
}
void roboItem::updateRobotPose(RobotPose pose) {
  m_currRobotPose = pose;
  update();
}
void roboItem::updatePoints(QPolygonF points) {
  m_points = points;
  update();
}
void roboItem::updateImage(QImage img) {
  m_images = img;
  update();
}
void roboItem::updateLocalCostMap(QImage img) {
  m_LocalCostMap = img;
  m_lastLocalCostMapRobotPose = m_currRobotPose;
  update();
}
void roboItem::updateGlobalCostMap(QImage img) {
  m_GlobalCostMap = img;
  update();
}
void roboItem::updateMap(QImage img) {
  m_map = img;
  update();
}
void roboItem::updateLine(QLine line) {
  m_lines = line;
  update();
}
void roboItem::updateLaserPoints(QPolygonF points) {
  //    qDebug()<<"我收到激光点云信息";
  m_laserPoints = points;
  update();
}
void roboItem::updatePath(QPolygonF points) {
  m_pathPoints = points;
  update();
}
void roboItem::wheelEvent(QGraphicsSceneWheelEvent *event) {
  this->setCursor(Qt::CrossCursor);
  double beforeScaleValue = m_scaleValue;
  if (event->delta() > 0) {
    //        qDebug()<<"放大";
    m_scaleValue *= 1.1;  //每次放大10%
  } else {
    //        qDebug()<<"缩小";
    m_scaleValue *= 0.9;  //每次缩小10%
  }
  setScale(m_scaleValue);

  //使放大缩小的效果看起来像是以鼠标中心点进行放大缩小
  if (event->delta() > 0) {
    moveBy(-event->pos().x() * beforeScaleValue * 0.1,
           -event->pos().y() * beforeScaleValue * 0.1);
  } else {
    moveBy(event->pos().x() * beforeScaleValue * 0.1,
           event->pos().y() * beforeScaleValue * 0.1);
  }
  update();
  //    qDebug()<<"m_scaleValue:"<<m_scaleValue;
}
void roboItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event) {
  emit cursorPos(event->pos());
}
void roboItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  if (m_isMousePress) {
    if (m_currCursor == m_moveCursor) {
      QPointF point = (event->pos() - m_pressedPose) * m_scaleValue;
      moveBy(point.x(), point.y());
    }
    m_endPose = event->pos();
  }
  update();
}
void roboItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    m_pressedPose = event->pos();
    m_isMousePress = true;
    m_startPose = event->pos();
  }
  update();
}
void roboItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    m_pressedPose = QPointF();
    m_isMousePress = false;
    if (m_currCursor == m_set2DPoseCursor) {
      emit signalPub2DPose(m_startPose, m_endPose);
      m_currCursor = m_moveCursor;
      this->setCursor(*m_currCursor);
    } else if (m_currCursor == m_set2DGoalCursor) {
      emit signalPub2DGoal(m_startPose, m_endPose);
      m_currCursor = m_moveCursor;
      this->setCursor(*m_currCursor);
    }
    m_startPose = QPointF();
    m_endPose = QPointF();
  }
  update();
}
void roboItem::start2DPose() {
  this->setCursor(*m_set2DPoseCursor);
  m_currCursor = m_set2DPoseCursor;
}
void roboItem::start2DGoal() {
  this->setCursor(*m_set2DGoalCursor);
  m_currCursor = m_set2DGoalCursor;
}
