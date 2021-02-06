#include "../include/cyrobot_monitor/robomap.h"
#include <QDebug>
roboMap::roboMap()
{
  setAcceptHoverEvents(true);
  setAcceptedMouseButtons(Qt::AllButtons);
  setAcceptDrops(true);
  setFlag(ItemAcceptsInputMethod, true);
//  m_scaleValue =10;
//  setScale(m_scaleValue);
  moveBy(0,0);
  //setCursor(Qt::CrossCursor);   //改变光标形状
}
int roboMap::QColorToInt(const QColor& color) {
  //将Color 从QColor 转换成 int
  return ( int )((( unsigned int )color.blue() << 16) | ( unsigned short )((( unsigned short )color.green() << 8) | color.red()));
}
void roboMap::paintImage(int id, QImage image){
  m_image=image;
}
void roboMap::paintLaserScan(QPolygonF points){
  laserPoints=points;
  update();
}
//palnner规划path绘制
void roboMap::paintPlannerPath(QPolygonF path){
  plannerPath=path;
  update();
}
void roboMap::paintMaps(QImage map){
   m_imageMap=map;
   update();
}
void roboMap::paintRoboPos(QPointF pos,float yaw){
  //qDebug()<<"pos:"<<pos;
   RoboPostion=pos;
   //yaw弧度全转换为正值
   m_roboYaw=abs(yaw);
   if(yaw>=0){
     m_roboYaw=PI*2-yaw;
   }

   update();
}
void roboMap::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
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

   //绘制laser
   painter->setPen(QPen(QColor(255, 0, 0, 255), 1));
   painter->drawPoints(laserPoints);
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
QRectF roboMap::boundingRect() const{
  //设置当前item绘制区域 (x,y,width,height)
  return QRectF(0,0,m_imageMap.width(),m_imageMap.height());
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
//mouse event
void roboMap::wheelEvent(QGraphicsSceneWheelEvent *event){
  if((event->delta() > 0)&&(m_scaleValue >= 50))//最大放大到原始图像的50倍
  {
      return;
  }
  else if((event->delta() < 0)&&(m_scaleValue <= m_scaleDafault))//图像缩小到自适应大小之后就不继续缩小
  {
      //ResetItemPos();//重置图片大小和位置，使之自适应控件窗口大小
  }
  else
  {
      qreal qrealOriginScale = m_scaleValue;
      if(event->delta() > 0)//鼠标滚轮向前滚动
      {
          m_scaleValue*=1.1;//每次放大10%
      }
      else
      {
          m_scaleValue*=0.9;//每次缩小10%
      }
      setScale(m_scaleValue);
      if(event->delta() > 0)
      {
          moveBy(-event->pos().x()*qrealOriginScale*0.1, -event->pos().y()*qrealOriginScale*0.1);//使图片缩放的效果看起来像是以鼠标所在点为中心进行缩放的
      }
      else
      {
          moveBy(event->pos().x()*qrealOriginScale*0.1, event->pos().y()*qrealOriginScale*0.1);//使图片缩放的效果看起来像是以鼠标所在点为中心进行缩放的
      }
  }
}
void roboMap::slot_set2DPos(){
  m_isOtherCursor=true;
  QCursor myCursor(QPixmap("://images/cursor_pos.png"),0,0);
  this->setCursor(myCursor); //设置自定义的鼠标样式
}
void roboMap::slot_set2DGoal(){
  m_isOtherCursor=true;
  QCursor myCursor(QPixmap("://images/cursor_pos.png"),0,0);
  this->setCursor(myCursor); //设置自定义的鼠标样式
}
void roboMap::mousePressEvent(QGraphicsSceneMouseEvent *event){
      if(event->button()== Qt::LeftButton)
      {
          m_startPos = event->pos();//鼠标左击时，获取当前鼠标在图片中的坐标，
          m_isPress = true;//标记鼠标左键被按下
      }
      else if(event->button() == Qt::RightButton)
      {
          //ResetItemPos();//右击鼠标重置大小
      }
}

void roboMap::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
      if(!m_isOtherCursor){
        QCursor myCursor(QPixmap("://images/cursor_move.png"),0,0);
        this->setCursor(myCursor); //设置自定义的鼠标样式
      }
      if(m_isPress)
      {
          QPointF point = (event->pos() - m_startPos)*m_scaleValue;
          moveBy(point.x(), point.y());
      }
}

void roboMap::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
     m_isPress = false;//标记鼠标左键已经抬起
}
