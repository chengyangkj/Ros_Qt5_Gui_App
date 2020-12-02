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
  QTimer timer_update;
  double map_size=1;
  Q_INVOKABLE void get_version(){
      qDebug()<<"1.0.0";
  }
  Q_INVOKABLE void setMax();
  Q_INVOKABLE void setMin();
  Q_INVOKABLE void move(double x,double y);
public slots:
    void paintMaps(QPolygon poinsts,QSizeF size);
    void paintRoboPos(QPointF pos);
};

#endif // ROBOMAP_H
