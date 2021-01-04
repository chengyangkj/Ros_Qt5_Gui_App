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
void roboMap::paintImage(int id, QImage image){
  m_image=image;
}
//palnner规划path绘制
void roboMap::paintPlannerPath(QPolygonF path){
  plannerPath=path;
  update();
}
void roboMap::paintMaps(QImage map,QSizeF size){
   this->setSize(size);
   mapSize=size;
   m_imageMap=map;
   update();
}
void roboMap::paintRoboPos(QPointF pos,float yaw){
   RoboPostion=pos;
   //yaw弧度全转换为正值
   m_roboYaw=abs(yaw);
   if(yaw>=0){
     m_roboYaw=PI*2-yaw;
   }

   update();
}
void roboMap::paint(QPainter *painter){
  //map
  painter->drawImage(0,0,m_imageMap);
  /* painter->save();
   painter->translate(0,mapSize.height())*/;
   //robot pos
   painter->setPen(QPen(QColor(255, 0, 0, 255), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
   painter->drawPoint(RoboPostion);
   //画等边三角形 以小车位置为中心 m_roboR为半径的外接圆
   //x+r*cos(yaw)
   //y-r*sin(yaw)
   QPointF head;
   head.setX(RoboPostion.x()+m_roboR*cos(m_roboYaw));
   head.setY(RoboPostion.y()-m_roboR*sin(m_roboYaw));
   //qDebug()<<"m_roboYaw:"<<m_roboYaw<<"robo pos:"<<RoboPostion<<"robo head:"<<head<<"cos: "<<cos(m_roboYaw)<<"sin:"<<sin(m_roboYaw);
   QPointF backR;
   backR.setX(RoboPostion.x()+m_roboR*cos(m_roboYaw+2.094));
   backR.setY(RoboPostion.y()-m_roboR*sin(m_roboYaw+2.094));
   QPointF backL;
   backL.setX(RoboPostion.x()+m_roboR*cos(m_roboYaw+4.188));
   backL.setY(RoboPostion.y()-m_roboR*sin(m_roboYaw+4.188));

   painter->setPen(QPen(QColor(0, 0, 255, 255), 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
//   painter->drawLine(QLineF(RoboPostion,head));
   //三角形
   QPainterPath path;
   path.moveTo(head);
   path.lineTo(backR);
   path.lineTo(backL);
   path.lineTo(head);

   painter->fillPath(path, QBrush(QColor ("blue")));
   //绘制planner Path
   painter->setPen(QPen(QColor(0, 0, 0, 255), 1));
   painter->drawPoints(plannerPath);
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
