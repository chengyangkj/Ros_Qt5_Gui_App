#ifndef ROBOMAP_H
#define ROBOMAP_H

#include <QObject>
#include <QDebug>
#include <QGraphicsItem>
#include <QPainter>
#include <QPolygon>
#include <QTimer>
#include <QGraphicsSceneWheelEvent>
#include <QColor>
#include <opencv2/highgui/highgui.hpp>
#include <QCursor>
#define PI 3.1415926
class roboMap  : public QObject, public QGraphicsItem
{
    Q_OBJECT

public:
  roboMap();
  QRectF  boundingRect() const;
  void    wheelEvent(QGraphicsSceneWheelEvent *event);
  void    mousePressEvent(QGraphicsSceneMouseEvent *event);
  void    mouseMoveEvent(QGraphicsSceneMouseEvent *event);
  void    mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
  void    paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  int QColorToInt(const QColor& color);
  QPolygon MapPoints;
  QPolygonF plannerPath;
  QPolygonF laserPoints;
  QPointF RoboPostion;
  QSizeF mapSize;
  QImage m_image;
  QImage m_imageMap;
  QTimer timer_update;
  int m_sizeCar=4;
  double m_roboYaw;
  double m_roboR=5;
  double map_size=1;
  void get_version(){
      qDebug()<<"1.0.0";
  }
  void setMax();
  void setMin();
  void move(double x,double y);
public slots:
    void paintMaps(QImage map);
    void paintRoboPos(QPointF pos,float yaw);
    void paintImage(int,QImage);
    void paintPlannerPath(QPolygonF);
    void paintLaserScan(QPolygonF);
    void slot_set2DPos();
    void slot_set2DGoal();
private:
    int         m_zoomState;
    bool        m_isPress;
    bool        m_isOtherCursor{false};
    QPointF     m_startPos;
    qreal       m_scaleValue=1;
    qreal       m_scaleDafault=1;
};

#endif // ROBOMAP_H
