#include "../include/cyrobot_monitor/rocker.h"
#include <QDebug>
rocker::rocker(QWidget *parent)
    : QWidget(parent)
{
    qDebug()<<parent->width()<<" "<<parent->height();
    setPalette(QPalette(Qt::white));
    resize(parent->width(),parent->height());
    setMinimumSize(100,100);
    mouseX=width()/2;
    mouseY=height()/2;
    tim=new QTimer(this);
    connect(tim,&QTimer::timeout,this,[=]{
       emit keyNumchanged(getKeyNum());
   });
//    connect(this,&rocker::keyNumchanged,this,[=](int num){
//        qDebug()<<num<<endl;
//    });
}

rocker::~rocker()
{

}
void rocker::paintEvent(QPaintEvent *){

    QPainter painter(this);

    int side = qMin(width(), height());

    padR=side/2; //底盘半径
    padX=padR;//底盘圆心
    padY=padR;//底盘圆心
    rockerR=padR/4;//摇杆圆半径
    int rockerMaxR=padR-rockerR;
    QColor rockerColor(Qt::gray);
    //加载底盘图像
//    painter.save();

//    painter.scale(side / 400.0, side / 400.0);//坐标会随窗口缩放
//    painter.drawPixmap(0, 0, QPixmap(":/image/pad.png"));
//    painter.restore();
    //自绘底盘
    painter.save();
    QRadialGradient RadialGradient(padR,padR,padR*3,padR,padR);//圆心2，半径1，焦点2
    RadialGradient.setColorAt(0,QColor(90,90,90,127));//渐变
    RadialGradient.setColorAt(1,QColor(255,255,255,190));//渐变
    painter.setBrush(RadialGradient);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPoint(padR,padR),side/2,side/2);//大圆盘
    painter.restore();

    //painter.drawText(20,20,tr("%1,%2,%3").arg(mouseX).arg(mouseY).arg(handPadDis));

    if(!mousePressed){//鼠标没按下则摇杆恢复到底盘中心
        mouseX=padX;
        mouseY=padY;
    }
    handPadDis=Pointdis(padR,padR,mouseX,mouseY);
    if(handPadDis<=rockerMaxR){
        rockerX=mouseX;
        rockerY=mouseY;
    }
    else {
        rockerX=(int)(rockerMaxR*(mouseX-padX)/handPadDis+padX);
        rockerY=(int)(rockerMaxR*(mouseY-padY)/handPadDis+padY);
    }
   // painter.drawText(200,200,tr("%1,%2,%3").arg(rockerX).arg(rockerY).arg(handPaddis));
    painter.setPen(Qt::NoPen);
    painter.setBrush(rockerColor);
    painter.drawEllipse(QPoint(rockerX,rockerY),rockerR,rockerR);//摇杆
}
void rocker::mouseMoveEvent(QMouseEvent* event){
    static bool r=false;
    mouseX=event->pos().x();
    mouseY=event->pos().y();
    if(r==true){
       update();
       r=false;
    }
    else{
        r=true;
    }
}
void rocker::mouseReleaseEvent(QMouseEvent* event){
    mouseX=width()/2;
    mouseY=height()/2;
    tim->stop();
    mousePressed=false;
    emit keyNumchanged(rocker::stop);
    update();
}
void rocker::mousePressEvent(QMouseEvent* event){
    mouseX=event->pos().x();
    mouseY=event->pos().y();
    tim->start(100);
    mousePressed=true;
    update();
}

double rocker::Pointdis(int a,int b,int x,int y){
    return sqrt((double)((x-a)*(x-a)+(y-b)*(y-b)));
}
int rocker::getKeyNum(){
    int x,y;
    int keynum;
    x=(int)(rockerX*3.0/(padR*2));
    y=(int)(rockerY*3.0/(padR*2));
    keynum=3*y+x;
    return keynum;
}
