/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-07-25 14:28:09
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-07-25 15:23:47
 * @FilePath: /Ros_Qt5_Gui_App/include/roboItem.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef roboItem_H
#define roboItem_H

#include <QColor>
#include <QCursor>
#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QObject>
#include <QPainter>
#include <QPolygon>
#include <QTimer>
#include <QtMath>
#include <opencv2/highgui/highgui.hpp>

#include "RobotAlgorithm.h"
namespace ros_qt5_gui_app {

enum eRobotColor { blue, red, yellow };

class roboItem : public QObject, public QGraphicsItem {
  Q_OBJECT

 public:
  roboItem();
  QRectF boundingRect() const;
  void wheelEvent(QGraphicsSceneWheelEvent *event);
  void mousePressEvent(QGraphicsSceneMouseEvent *event);
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
  void hoverMoveEvent(QGraphicsSceneHoverEvent *event);
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
             QWidget *widget);
  int QColorToInt(const QColor &color);
  void setRobotColor(eRobotColor color);
  void setRobotSize(QSize size);
  QPolygon MapPoints;
  QPolygonF plannerPath;
  QPolygonF laserPoints;
  QPointF RoboPostion;
  QSizeF mapSize;
  QImage m_image;
  QImage m_imageMap;
  QTimer timer_update;
  int m_sizeCar = 4;
  double m_roboYaw;
  double m_roboR = 5;
  double map_size = 1;
  double defaultScale = 2;
  void get_version() { qDebug() << "1.0.0"; }
  void setMax();
  void setMin();
  void setDefault();
  void move(double x, double y);
  QCursor *m_moveCursor = NULL;
  QCursor *set2DPoseCursor = NULL;
  QCursor *set2DGoalCursor = NULL;
  QCursor *m_currCursor = NULL;
 signals:
  void cursorPos(QPointF);
  void signalPub2DPose(QPointF, QPointF);
  void signalPub2DGoal(QPointF, QPointF);
 public slots:
  void paintMaps(QImage map);
  void paintRoboPos(RobotPose pos);
  void paintImage(int, QImage);
  void paintPlannerPath(QPolygonF);
  void paintLaserScan(QPolygonF);
  void slot_set2DPos();
  void slot_set2DGoal();
  void slot_setMoveCamera();

 private:
  void drawMap(QPainter *painter);
  void drawRoboPos(QPainter *painter);
  void drawLaserScan(QPainter *painter);
  void drawPlannerPath(QPainter *painter);
  void drawTools(QPainter *painter);

 private:
  int m_zoomState;
  bool m_isMousePress{false};
  QPixmap robotImg;
  QPointF m_startPose;
  QPointF m_endPose;
  QPointF m_pressedPoint = QPointF(0, 0);
  QPointF m_pressingPoint = QPointF(0, 0);
  qreal m_scaleValue = 1;
  qreal m_scaleDafault = 1;
};
}  // namespace ros_qt5_gui_app
#endif  // roboItem_H
