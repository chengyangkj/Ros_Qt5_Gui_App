#include <QApplication>
#include <iostream>
#include <QSplashScreen>
#include <QMovie>
#include <QPixmap>
#include "mainwindow.h"
int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  //启动动画
   QPixmap pixmap("://background/loding5.gif");
  QSplashScreen splash(pixmap);
  splash.setWindowOpacity(1); // 设置窗口透明度
  QLabel label(&splash);
  QMovie mv("://background/loding5.gif");
  label.setMovie(&mv);
  mv.start();
  splash.show();
  splash.setCursor(Qt::BlankCursor);
  for (int i = 0; i < 2000; i += mv.speed()) {
    a.processEvents(); //使程序在显示启动画面的同时仍能响应鼠标等其他事件
    QThread::msleep(50); // 延时
  }
  qRegisterMetaType<RobotPose>("RobotPose");
  MainWindow w;
  w.show();
  splash.finish(&w); //在主体对象初始化完成后结束启动动画
  return a.exec();
}
