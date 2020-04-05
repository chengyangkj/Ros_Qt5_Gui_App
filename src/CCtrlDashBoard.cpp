#include "../include/cyrobot_monitor/CCtrlDashBoard.h"
#include <QPainter>
#include <QDebug>
#include <qmath.h>

CCtrlDashBoard::CCtrlDashBoard(QWidget *parent, StyleType type) :
    QWidget(parent),
    m_StyleType(type)
{
    m_BorderColor = QColor(60,60,60);
    m_BgColor = QColor(160,160,160);
    m_FrontColor = Qt::white;

    m_DashValue= 0;
    m_MinValue = 0;
    m_MaxValue = 100;
    m_DashNum = 1;
    m_LineLength=0;

    m_StartAngle=90;  //510
    m_EndAngle=0;   //120
    update();
}

void CCtrlDashBoard::drawBackGround(QPainter *painter, qreal hlafWidth)
{
    m_MaxBorderRadius = ((hlafWidth*5)/6);
    qreal startX=hlafWidth-m_MaxBorderRadius;
    painter->save();
    painter->setPen(m_BorderColor);
    painter->setBrush(m_BorderColor);

    QPainterPath bigCircle;
    bigCircle.addEllipse(startX, startX, (m_MaxBorderRadius*2), (m_MaxBorderRadius*2));

    m_MinBorderRadius = m_MaxBorderRadius-20;
    startX=hlafWidth-m_MinBorderRadius;
    QPainterPath smallCircle;
    smallCircle.addEllipse(startX, startX, (m_MinBorderRadius*2), (m_MinBorderRadius*2));

    QPainterPath path = bigCircle - smallCircle;
    painter->drawPath(path);
    painter->restore();

    painter->save();
    painter->setBrush(m_BgColor);
    painter->drawEllipse(startX,startX,(m_MinBorderRadius*2), (m_MinBorderRadius*2));
    painter->drawText(startX+90,startX+170,"CM/S");
    painter->restore();
    m_DialsRadius = m_MinBorderRadius-10;
    if(m_DialsRadius < 0){
        m_DialsRadius=2;
    }
}
void CCtrlDashBoard::drawScaleDials(QPainter *painter, qreal hlafWidth)
{
    qreal tSteps = (m_MaxValue - m_MinValue)/m_DashNum; //相乘后的值是分的份数
    qreal angleStep = 1.0*(360.0-m_StartAngle - m_EndAngle) / tSteps; //每一个份数的角度

    painter->save();
    QPen pen ;
    pen.setColor(m_FrontColor); //推荐使用第二种方式
    painter->translate(hlafWidth,hlafWidth);
    painter->rotate(m_StartAngle);
    m_LineLength = (hlafWidth/16);
    qreal lineStart = m_DialsRadius-4*m_LineLength-m_LineLength;
    qreal lineSmStart = m_DialsRadius-4*m_LineLength-m_LineLength/2;
    qreal lineEnd = m_DialsRadius-4*m_LineLength;
    for (int i = 0; i <= tSteps; i++)
    {
        if (i % 10 == 0)//整数刻度显示加粗
        {
            pen.setWidth(2); //设置线宽
            painter->setPen(pen); //使用面向对象的思想，把画笔关联上画家。通过画家画出来
            painter->drawLine(lineStart,lineStart, lineEnd, lineEnd); //两个参数应该是两个坐标值
            //painter->drawText(lineEnd+6,lineEnd+6, tr("%1").arg(m_MinValue+i));
        }
        else
        {
            pen.setWidth(1);
            painter->setPen(pen);
            painter->drawLine(lineSmStart, lineSmStart, lineEnd, lineEnd); //两个参数应该是两个坐标值
        }
        painter->rotate(angleStep);
    }
    painter->restore();

    painter->save();
    painter->setPen(pen);
    //m_startAngle是起始角度，m_endAngle是结束角度，m_scaleMajor在一个量程中分成的刻度数
    qreal startRad = ( 315-m_StartAngle) * (3.14 / 180);
    qreal deltaRad = (360-m_StartAngle - m_EndAngle) * (3.14 / 180) / tSteps;
    qreal sina,cosa;
    qreal x, y;
    QFontMetricsF fm(this->font());
    double w, h, tmpVal;
    QString str;
    painter->translate(hlafWidth,hlafWidth);
    lineEnd = m_MinBorderRadius-8;
    for (int i = 0; i <= tSteps; i++)
    {
        if (i % 10 == 0)//整数刻度显示加粗
        {
            sina = qSin(startRad - i * deltaRad);
            cosa = cos(startRad - i * deltaRad);

            tmpVal = 1.0 * i *((m_MaxValue - m_MinValue) / tSteps) + m_MinValue;
            str = QString( "%1" ).arg(tmpVal);  //%1作为占位符   arg()函数比起 sprintf()来是类型安全的
            w = fm.size(Qt::TextSingleLine,str).width();
            h = fm.size(Qt::TextSingleLine,str).height();
            x = lineEnd * cosa - w / 2;
            y = -lineEnd * sina + h / 4;
            painter->drawText(x, y, str); //函数的前两个参数是显示的坐标位置，后一个是显示的内容，是字符类型""
        }
    }
    painter->restore();
}

void CCtrlDashBoard::drawIndicator(QPainter *painter, qreal hlafWidth)
{
    QPolygon pts;
    pts.setPoints(3, -8,0, 8,0, 0,(int)m_DialsRadius-20); /* (-2,0)/(2,0)/(0,60) *///第一个参数是 ，坐标的个数。后边的是坐标
    painter->save();
    painter->translate(hlafWidth, hlafWidth);
    painter->rotate(m_StartAngle-45);

    if(m_DashValue>0){
        qreal tSteps = (m_MaxValue - m_MinValue)/m_DashNum; //相乘后的值是分的份数
        qreal angleStep = 1.0*(360.0-m_StartAngle - m_EndAngle) / tSteps; //每一个份数的角度
        double degRotate = angleStep*tSteps*(m_DashValue/100.0)+angleStep;

        //画指针
        //qDebug() <<"degRotate =="<<degRotate<<"m_DashValue =="<<m_DashValue<<m_StartAngle<<m_DialsRadius<<hlafWidth;
        if(m_DashValue == 99){
            painter->rotate(m_EndAngle-m_StartAngle);  //顺时针旋转坐标系统
        }else
        {
            painter->rotate(degRotate);  //顺时针旋转坐标系统
        }

    }
    QRadialGradient haloGradient(0, 0, hlafWidth/2, 0, 0);  //辐射渐变
    haloGradient.setColorAt(0, QColor(255,69,0));
    haloGradient.setColorAt(1, QColor(255,0,0));
    painter->setPen(Qt::white); //定义线条文本颜色  设置线条的颜色
    painter->setBrush(haloGradient);//刷子定义形状如何填满 填充后的颜色
    painter->drawConvexPolygon(pts); //这是个重载函数，绘制多边形。
    painter->restore();

    //画中心点
    QColor niceBlue(150, 150, 200);
    QConicalGradient coneGradient(0, 0, -90.0);  //角度渐变
    coneGradient.setColorAt(0.0, Qt::darkGray);
    coneGradient.setColorAt(0.2, niceBlue);
    coneGradient.setColorAt(0.5, Qt::white);
    coneGradient.setColorAt(1.0, Qt::darkGray);
    painter->save();
    painter->translate(hlafWidth,hlafWidth);
    painter->setPen(Qt::NoPen);  //没有线，填满没有边界
    painter->setBrush(coneGradient);
    painter->drawEllipse(-10, -10, 20, 20);
    painter->restore();
}

void CCtrlDashBoard::drawIndicatorBar(QPainter *painter, qreal hlafWidth)
{
    // 渐变色
    painter->save();
    painter->translate(hlafWidth,hlafWidth);
    qreal lineEnd = m_DialsRadius-3*m_LineLength;
    QRadialGradient gradient(0, 0, lineEnd);
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1.0, Qt::blue);
    painter->setBrush(gradient);

    // << 1（左移1位）相当于radius*2 即：150*2=300
    //QRectF(-150, -150, 300, 300)
    QRectF rect(-lineEnd, -lineEnd, lineEnd*2, lineEnd*2);
    QPainterPath path;
    path.arcTo(rect, m_StartAngle, 270);

    // QRectF(-120, -120, 240, 240)
    QPainterPath subPath;
    subPath.addEllipse(rect.adjusted(10, 10, -10, -10));

    // path为扇形 subPath为椭圆
    path -= subPath;
    painter->setPen(Qt::NoPen);
    painter->rotate(m_StartAngle+45);
    painter->drawPath(path);
    painter->restore();


    qreal degRotate=0.1;
    if(m_DashValue>0){
        qreal tSteps = (m_MaxValue - m_MinValue)/m_DashNum; //相乘后的值是分的份数
        qreal angleStep = 1.0*(360.0-m_StartAngle - m_EndAngle) / tSteps; //每一个份数的角度
        degRotate = angleStep*tSteps*(m_DashValue/100.0)+angleStep;

        //画指针
        //qDebug() <<"degRotate =="<<degRotate<<"m_DashValue =="<<m_DashValue<<m_StartAngle<<m_DialsRadius<<hlafWidth;
        /*if(m_DashValue == 99){
            painter->rotate(m_EndAngle-m_StartAngle);  //顺时针旋转坐标系统
        }else
        {
            painter->rotate(degRotate);  //顺时针旋转坐标系统
        }*/
    }

    painter->save();
    painter->translate(hlafWidth, hlafWidth);

    QRadialGradient ftGradient(0, 0, lineEnd);
    ftGradient.setColorAt(0, Qt::white);
    ftGradient.setColorAt(1.0, Qt::darkYellow);
    painter->setBrush(ftGradient);

    // << 1（左移1位）相当于radius*2 即：150*2=300
    //QRectF(-150, -150, 300, 300)
    QRectF ftRect(-lineEnd, -lineEnd, lineEnd*2, lineEnd*2);
    QPainterPath ftPath;
    ftPath.arcTo(ftRect, m_EndAngle-m_StartAngle, -degRotate);
    // path为扇形 subPath为椭圆
    ftPath -= subPath;
    painter->rotate(m_StartAngle-45);
    painter->drawPath(ftPath);
    painter->restore();

    QPolygon pts;
    int pointLength=lineEnd-12;
    pts.setPoints(3, -6,pointLength-10, 6,pointLength-10, 0,pointLength);
    painter->save();
    painter->translate(hlafWidth, hlafWidth);
    painter->rotate(m_StartAngle-45);

    if(m_DashValue == 99){
        painter->rotate(m_EndAngle-m_StartAngle);  //顺时针旋转坐标系统
    }else
    {
        painter->rotate(degRotate);  //顺时针旋转坐标系统
    }
    painter->setPen(Qt::white); //定义线条文本颜色  设置线条的颜色
    painter->setBrush(Qt::blue);//刷子定义形状如何填满 填充后的颜色
    painter->drawConvexPolygon(pts); //这是个重载函数，绘制多边形。
    painter->restore();
}

void CCtrlDashBoard::paintEvent(QPaintEvent * event)
{
    QPainter p(this);
    qreal width = qMin((this->width()>>1), (this->height()>>1));

    p.setRenderHints(QPainter::Antialiasing|QPainter::TextAntialiasing);

    drawBackGround(&p, width);

    drawScaleDials(&p, width);
    switch(m_StyleType)
    {
    case ST_DEFAULT:
        drawIndicator(&p, width);
        break;
    case ST_ARCBAR:
        drawIndicatorBar(&p, width);
        break;
    }
}
