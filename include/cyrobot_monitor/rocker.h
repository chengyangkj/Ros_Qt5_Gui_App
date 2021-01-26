#ifndef rocker_H
#define rocker_H

#include <QWidget>
#include <QPainter>
#include <QDrag>
#include <QMouseEvent>
#include <QtMath>
#include <QTimer>
#include <QDebug>
class rocker : public QWidget
{
    Q_OBJECT

public:
    rocker(QWidget *parent = 0);
    ~rocker();
    enum {upleft=0,up,upright,left,stop,right,downleft,down,downright};
signals:
    void keyNumchanged(int num);
protected:
    void paintEvent(QPaintEvent *event)override;
    void mouseMoveEvent(QMouseEvent *event)override;
    void mouseReleaseEvent(QMouseEvent *event)override;
    void mousePressEvent(QMouseEvent *event)override;
  //  void resizeEvent(QResizeEvent *event)override;
private:
    int mouseX;
    int mouseY;
  int rockerX;//摇杆
  int rockerY;
  int rockerR;
  int padX;//底盘
  int padY;
  int padR;
  double handPadDis;//两圆圆心距离
  bool mousePressed;
  QTimer *tim;
private:
   double Pointdis(int a,int b,int x,int y);//两点距离
   int getKeyNum();

};

#endif // rocker_H
