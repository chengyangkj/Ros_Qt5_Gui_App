#ifndef ROBOMAP_H
#define ROBOMAP_H

#include <QObject>
#include <QDebug>
#include <QQuickPaintedItem>
#include <QPainter>
#include <QPolygon>
#include <QTimer>
#include <QColor>
#include <opencv2/highgui/highgui.hpp>
class roboMap  : public QQuickPaintedItem
{
    Q_OBJECT

public:
  roboMap(QQuickItem* parent=nullptr);
  void paint(QPainter* painter) override;

  int QColorToInt(const QColor& color);
  QPolygon MapPoints;
  QTimer timer_update;
  double map_size=1;
  Q_INVOKABLE void get_version(){
      qDebug()<<"1.0.0";
  }
  Q_INVOKABLE void setMax();
  Q_INVOKABLE void setMin();
  Q_INVOKABLE void move(double x,double y);
};

#endif // ROBOMAP_H
