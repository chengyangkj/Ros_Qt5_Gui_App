#ifndef ROBOMAP_H
#define ROBOMAP_H

#include <QObject>
#include <QDebug>
#include <QQuickPaintedItem>
#include <QGraphicsItem>
#include <QPainter>
#include <QPolygon>
#include <QTimer>
#include <QColor>
#include <opencv2/highgui/highgui.hpp>
#include <QCursor>
#define PI 3.1415926
class roboMap  : public QQuickPaintedItem
{
    Q_OBJECT

public:
  roboMap(QQuickItem* parent=nullptr);
  void paint(QPainter* painter) override;
  int QColorToInt(const QColor& color);
  QPolygon MapPoints;
  QPointF RoboPostion;
  QSizeF mapSize;
  QImage m_image;
  QImage m_imageMap;
  QTimer timer_update;
  int m_sizeCar=4;
  double m_roboYaw;
  double m_roboR=5;
  double map_size=1;
  Q_INVOKABLE void get_version(){
      qDebug()<<"1.0.0";
  }
  Q_INVOKABLE void setMax();
  Q_INVOKABLE void setMin();
  Q_INVOKABLE void move(double x,double y);
public slots:
    void paintMaps(QImage map,QSizeF size);
    void paintRoboPos(QPointF pos,float yaw);
    void paintImage(int,QImage);
};

#endif // ROBOMAP_H
