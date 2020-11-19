#include "../include/cyrobot_monitor/robomap.h"
#include <QDebug>
roboMap::roboMap(QQuickItem *parent) : QQuickPaintedItem (parent)
{
  //初始绘制
      QPixmap* bmp    = new QPixmap("/home/chengyangkj/lxind_ws/src/lxind_cfgs/lxind_nav/maps/good_original.pgm");  // //打开pgm图片，提取纯黑的像素，存入pointList
      uint     height = bmp->height();
      uint     width  = bmp->width();
      for(int y = 0; y < height; ++y)
          for(int x = 0; x < width; ++x) {
              QColor color = bmp->toImage().pixel(QPoint(x, y));
              if(0 == QColorToInt(color)) {
                  m_laserPoints.push_back(QPoint(x, y));
              }
          }

      this->setHeight(height);;
      setAcceptHoverEvents(true);
      setAcceptedMouseButtons(Qt::AllButtons);
      setFlag(ItemAcceptsInputMethod, true);
      setAntialiasing(true);
      setMipmap(true);
      update();
}
int roboMap::QColorToInt(const QColor& color) {
  //将Color 从QColor 转换成 int
  return ( int )((( unsigned int )color.blue() << 16) | ( unsigned short )((( unsigned short )color.green() << 8) | color.red()));
}
void roboMap::paint(QPainter *painter){
   painter->setPen(QPen(QColor(0, 0, 0, 255), 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
   painter->drawPoints(m_laserPoints);
}
void roboMap::setMax(){
     map_size+=0.1;
     this->setScale(map_size);
}
void roboMap::setMin(){
  if(map_size-0.1>0){
     map_size-=0.1;
  }
   this->setScale(map_size);
}
void roboMap::move(double x,double y){
   //this->setTransformOrigin(50,50);
//   this->setRotation(180);
  this->setTransformOrigin(TransformOrigin::Top);
   this->setX(-100);
   this->setY(-100);
   qDebug()<<"ok!";
}
