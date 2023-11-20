/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-09-28 14:56:04
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-05 11:39:01
 * @FilePath: /ROS2_Qt5_Gui_App/src/main.cpp
 */
#include "logger/easylogging++.h"
#include "mainwindow.h"
#include <QApplication>
#include <QLabel>
#include <QMovie>
#include <QPixmap>
#include <QSplashScreen>
#include <QThread>
#include <iostream>
INITIALIZE_EASYLOGGINGPP

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

  // logger
  el::Configurations defaultConf;
  defaultConf.setToDefault();
  //设置最大文件大小
  defaultConf.setGlobally(el::ConfigurationType::MaxLogFileSize, "1000000");
  //是否写入文件
  defaultConf.setGlobally(el::ConfigurationType::ToFile, "true");
  //是否输出控制台
  defaultConf.setGlobally(el::ConfigurationType::ToStandardOutput, "false");
  // filename
  defaultConf.setGlobally(el::ConfigurationType::Filename,
                          "ros_qt_gui_app.log");
  //设置配置文件
  el::Loggers::reconfigureLogger("default", defaultConf);

  /// 防止Fatal级别日志中断程序
  el::Loggers::addFlag(el::LoggingFlag::DisableApplicationAbortOnFatalLog);

  /// 选择划分级别的日志
  el::Loggers::addFlag(el::LoggingFlag::HierarchicalLogging);

  /// 设置级别门阀值，修改参数可以控制日志输出
  el::Loggers::setLoggingLevel(el::Level::Global);
  LOG(INFO) << "ros qt5 gui app init";

  CMainWindow w;
  w.show();
  splash.finish(&w); //在主体对象初始化完成后结束启动动画
  return a.exec();
}
