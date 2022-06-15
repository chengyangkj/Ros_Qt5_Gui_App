#include "roboItem.h"
#include "QDebug"
roboItem::roboItem()
{
   m_robotImg.load("://images/robot.png");
   QMatrix matrix;
   matrix.rotate(90);
   m_robotImg = m_robotImg.transformed(matrix,Qt::SmoothTransformation);
}
QRectF roboItem::boundingRect() const {
   return QRectF(0,0,400,400);
}

void roboItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
//    qDebug()<<"我被调用啦";

   drawImage(painter);
   drawPoints(painter);
   drawLine(painter);
   drawMap(painter);
   drawRobotPose(painter);
   drawLaserScan(painter);
   drawPath(painter);
}
void roboItem::drawPath(QPainter* painter){
     painter->setPen(QPen(QColor(0,0,255),1));
     painter->drawPoints(m_pathPoints);
}
void roboItem::drawLaserScan(QPainter* painter){
     painter->setPen(QPen(QColor(255,0,0),1));
     painter->drawPoints(m_laserPoints);
}
void roboItem::drawRobotPose(QPainter* painter){

//  qDebug()<<" m_currRobotPose.x:"<<m_currRobotPose.x<<" m_currRobotPose.y:"<<m_currRobotPose.y;
//  painter->setPen(QPen(QColor(255,0,0),10));
//    qDebug()<<"m_currRobotPose:"<<m_currRobotPose.theta;
  painter->save();
  painter->translate(QPointF(m_currRobotPose.x,m_currRobotPose.y));
  painter->setPen(QPen(QColor(255,0,0),1));
  painter->drawPoint(0,0);
  painter->rotate(rad2deg(-m_currRobotPose.theta));
  painter->drawPixmap(-m_robotImg.width()/2,-m_robotImg.height()/2,m_robotImg);
  painter->restore();
}
void roboItem::drawMap(QPainter* painter){
    painter->drawImage(0,0,m_map);
}
void roboItem::drawImage(QPainter* painter){
    painter->drawImage(0,0,m_images);
}
void roboItem::drawPoints(QPainter* painter){
    painter->setPen(QPen(QColor(0,0,255),2));
    painter->drawPoints(m_points);
}

void roboItem::drawLine(QPainter* painter){
    painter->setPen(QPen(QColor(255,0,0),1));
   painter->drawLine(m_lines);
}
void roboItem::updateRobotPose(RobotPose pose){
    m_currRobotPose=pose;
    update();
}
void roboItem::updatePoints(QPolygonF points){
    m_points=points;
    update();
}
void roboItem::updateImage(QImage img){
    m_images=img;
    update();
}
void roboItem::updateMap(QImage img){
    m_map=img;
    update();
}
void roboItem::updateLine(QLine line){
    m_lines=line;
    update();
}
void roboItem::updateLaserPoints(QPolygonF points){
//    qDebug()<<"我收到激光点云信息";
    m_laserPoints=points;
    update();
}
void roboItem::updatePath(QPolygonF points){
    m_pathPoints=points;
    update();
}
void roboItem::wheelEvent(QGraphicsSceneWheelEvent *event) {
    double beforeScaleValue=m_scaleValue;
    if(event->delta()>0){
//        qDebug()<<"放大";
        m_scaleValue*=1.1; //每次放大10%
    }else{
//        qDebug()<<"缩小";
        m_scaleValue*=0.9; //每次缩小10%
    }
    setScale(m_scaleValue);

    //使放大缩小的效果看起来像是以鼠标中心点进行放大缩小
    if(event->delta()>0){
        moveBy(-event->pos().x()*beforeScaleValue*0.1,-event->pos().y()*beforeScaleValue*0.1);
    }else{
        moveBy(event->pos().x()*beforeScaleValue*0.1,event->pos().y()*beforeScaleValue*0.1);
    }
    update();
//    qDebug()<<"m_scaleValue:"<<m_scaleValue;
}
void roboItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
    if(m_isMousePress){
        QPointF point=(event->pos()-m_pressedPose)*m_scaleValue;
        moveBy(point.x(),point.y());
    }
}
void roboItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {

    if(event->button()==Qt::LeftButton){
        m_pressedPose = event->pos();
        m_isMousePress=true;
    }
}
void roboItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    if(event->button()==Qt::LeftButton){
        m_pressedPose = QPointF();
        m_isMousePress=false;
    }
}
