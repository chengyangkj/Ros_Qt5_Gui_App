#include "../include/cyrobot_monitor/robomap.h"
#include <QDebug>
roboMap::roboMap()
{
  setAcceptHoverEvents(true);
  setAcceptedMouseButtons(Qt::AllButtons);
  setAcceptDrops(true);
  setFlag(ItemAcceptsInputMethod, true);
  moveBy(0,0);
  moveCursor=new QCursor(QPixmap("://images/cursor_move"),0,0);
  set2DPoseCursor=new QCursor(QPixmap("://images/cursor_pos.png"),0,0);
  setDefault();
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
  drawMap(painter);
  drawRoboPos(painter);
  drawPlannerPath(painter);
  drawLaserScan(painter);
  drawTools(painter);

}
void roboMap::drawTools(QPainter *painter){
  if(currCursor == set2DPoseCursor){
    //绘制箭头
    if(m_pressedPoint.x()!=0 && m_pressedPoint.y()!=0 &&m_pressingPoint.x()!=0 && m_pressingPoint.y()!=0){
      painter->setPen(QPen(QColor(0, 255, 0, 255), 2));
      //计算线弧度
      double theta=qAtan((m_pressingPoint.y()-m_pressedPoint.y())/(m_pressingPoint.x()-m_pressedPoint.x()));
      //根据固定长度计算箭头主体结束点坐标
      double dy=sin(theta)*20;
      double dx=cos(theta)*20;
      QPointF startPoint,endPoint;
      startPoint = m_pressedPoint;
      if(m_pressingPoint.x()-m_pressedPoint.x()>0){
         endPoint = QPointF(m_pressedPoint.x()+dx,m_pressedPoint.y()+dy);
      }
      else {
         endPoint = QPointF(m_pressedPoint.x()-dx,m_pressedPoint.y()-dy);
      }
      QLineF line(startPoint,endPoint);
      painter->drawLine(line);
      QLineF v = line.unitVector();
          if (!v.isNull()) {
            v.setLength(5);
            v.translate(QPointF(line.dx(), line.dy()));
            QLineF n = v.normalVector();
            n.setLength(n.length() * 0.5);
            QLineF n2 = n.normalVector().normalVector();
            QPointF p1 = v.p2();
            QPointF p2 = n.p2();
            QPointF p3 = n2.p2();
            //painter->setBrush(QBrush(color));
            if (p1.isNull() == false && p2.isNull() == false) {
              QLineF lineA(p1, p2);
              if (lineA.length() > 4) {
                painter->drawLine(lineA);
              }
            }
            if (p2.isNull() == false && p3.isNull() == false) {
              QLineF lineB(p2, p3);
              if (lineB.length() > 4) {
                painter->drawLine(lineB);
              }
            }
            if (p3.isNull() == false && p1.isNull() == false) {
              QLineF lineC(p3, p1);
              if (lineC.length() > 4) {
                painter->drawLine(lineC);
              }
            }
          }
       }
  }


}
void roboMap::drawMap(QPainter* painter){
     painter->drawImage(0,0,m_imageMap);
}
void roboMap::drawRoboPos(QPainter* painter){
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
}
void roboMap::drawLaserScan(QPainter* painter){
  //绘制laser
  painter->setPen(QPen(QColor(255, 0, 0, 255), 1));
  painter->drawPoints(laserPoints);
}
void roboMap::drawPlannerPath(QPainter* painter){
  //绘制planner Path
  painter->setPen(QPen(QColor(0, 0, 0, 255), 1));
  painter->drawPoints(plannerPath);
}
void roboMap::setMax(){
  m_scaleValue*=1.1;//每次放大10%
  setScale(m_scaleValue);
}
void roboMap::setMin(){
  m_scaleValue*=0.9;//每次缩小10%
  setScale(m_scaleValue);
}
void roboMap::setDefault(){
  this->setScale(defaultScale);
  this->moveBy(0,0);
  m_scaleValue=defaultScale;
}
QRectF roboMap::boundingRect() const{
  //设置当前item绘制区域 (x,y,width,height)
  return QRectF(0,0,m_imageMap.width(),m_imageMap.height());
}

void roboMap::move(double x,double y){
  this->moveBy(x,y);
}
//mouse event
void roboMap::wheelEvent(QGraphicsSceneWheelEvent *event){
  this->setCursor(Qt::CrossCursor);
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
  this->setCursor(*set2DPoseCursor); //设置自定义的鼠标样式
  currCursor=set2DPoseCursor;
}
void roboMap::slot_set2DGoal(){
  this->setCursor(*set2DPoseCursor); //设置自定义的鼠标样式
  currCursor=set2DPoseCursor;
}
void roboMap::slot_setMoveCamera(){
  this->setCursor(*moveCursor); //设置自定义的鼠标样式
  currCursor=moveCursor;
}
void roboMap::mousePressEvent(QGraphicsSceneMouseEvent *event){
      if(event->button()== Qt::LeftButton)
      {
          m_startPos = event->pos();//鼠标左击时，获取当前鼠标在图片中的坐标，
          m_isPress = true;//标记鼠标左键被按下
          m_pressedPoint=event->pos();
      }
      else if(event->button() == Qt::RightButton)
      {
          //ResetItemPos();//右击鼠标重置大小
      }
      update();
}

void roboMap::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
       m_pressingPoint=event->pos();
      //设置鼠标样式为移动
      if(currCursor==NULL){
        this->setCursor(*moveCursor); //设置自定义的鼠标样式
        currCursor=moveCursor;
      }
      //移动图层
      if(m_isPress && currCursor==moveCursor)
      {
          QPointF point = (event->pos() - m_startPos)*m_scaleValue;
          moveBy(point.x(), point.y());
      }
      update();
}
void roboMap::hoverMoveEvent(QGraphicsSceneHoverEvent *event){
     emit cursorPos(event->pos());
}
void roboMap::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
     m_isPress = false;//标记鼠标左键已经抬起
     m_pressedPoint=QPointF(0,0);
     //如果是选择点位模式 重置
     if(currCursor == set2DPoseCursor){
       m_isOtherCursor=false;
       m_pressedPoint=QPointF(0,0);
       m_pressingPoint=QPointF(0,0);
       this->setCursor(*moveCursor); //设置自定义的鼠标样式
       currCursor=moveCursor;
     }
}
