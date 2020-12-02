#include "../include/cyrobot_monitor/robomap.h"
#include <QDebug>
roboMap::roboMap(QQuickItem *parent) : QQuickPaintedItem (parent)
{
  setAcceptHoverEvents(true);
  setAcceptedMouseButtons(Qt::AllButtons);
  setFlag(ItemAcceptsInputMethod, true);
  setAntialiasing(true);
  setMipmap(true);
}
int roboMap::QColorToInt(const QColor& color) {
  //将Color 从QColor 转换成 int
  return ( int )((( unsigned int )color.blue() << 16) | ( unsigned short )((( unsigned short )color.green() << 8) | color.red()));
}
void roboMap::paintMaps(QPolygon poinsts,QSizeF size){
   this->setSize(size);
   mapSize=size;
   MapPoints=poinsts;
   update();
}
void roboMap::paintRoboPos(QPointF pos){
   RoboPostion=pos;
   update();
}
void roboMap::paint(QPainter *painter){
   painter->setPen(QPen(QColor(0, 0, 0, 255), 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
   painter->drawPoints(MapPoints);
   painter->drawRect(0,0,mapSize.width(),mapSize.height());
   painter->save();
   //设置地图中心为坐标原点
//   painter->translate(mapSize.width()/2,mapSize.height()/2);
   painter->setPen(QPen(QColor(255, 0, 0, 255), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
   painter->drawPoint(RoboPostion);
   painter->restore();
  // painter->drawPoint(QPoint(-mapSize.width()/2,mapSize.height()/2));
//   painter->drawPoint(QPoint(mapSize.width(),-mapSize.height()));
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
//  this->setTransformOrigin(TransformOrigin::Top);
//   this->setX(-100);
//   this->setY(-100);
 // this->setPosition(QPointF(x,y));

  //this->setCursor(QCursor(Qt::OpenHandCursor));
   qDebug()<<"ok!";
}
