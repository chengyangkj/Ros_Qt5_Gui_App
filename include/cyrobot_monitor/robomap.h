#ifndef ROBOMAP_H
#define ROBOMAP_H

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
namespace cyrobot_monitor {

enum eRobotColor{
    blue,
    red,
    yellow
};

class roboMap : public QObject, public QGraphicsItem {
  Q_OBJECT

 public:
  roboMap();
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
  double PI = 3.1415926;
  void get_version() { qDebug() << "1.0.0"; }
  void setMax();
  void setMin();
  void setDefault();
  void move(double x, double y);
  QCursor *moveCursor = NULL;
  QCursor *set2DPoseCursor = NULL;
  QCursor *set2DGoalCursor = NULL;
  QCursor *currCursor = NULL;
 signals:
  void cursorPos(QPointF);
  void signalPub2DPos(algo::RobotPose pose);
  void signalPub2DGoal(algo::RobotPose pose);
 public slots:
  void paintMaps(QImage map);
  void paintRoboPos(algo::RobotPose pos);
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
  bool m_isPress;
  bool m_isOtherCursor{false};
  QPixmap robotImg;
  QPointF m_startPos;
  QPointF m_pressedPoint = QPointF(0, 0);
  QPointF m_pressingPoint = QPointF(0, 0);
  qreal m_scaleValue = 1;
  qreal m_scaleDafault = 1;
};
}  // namespace cyrobot_monitor
#endif  // ROBOMAP_H
