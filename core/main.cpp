/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-09-28 14:56:04
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-05 11:39:01
 * @FilePath: /ROS2_Qt5_Gui_App/src/main.cpp
 */
#include <QApplication>
#include <QLabel>
#include <QMovie>
#include <QPixmap>
#include <QSplashScreen>
#include <QThread>
#include <csignal>
#include <iostream>
#include "logger/logger.h"
#include "runtime/application_manager.h"
void signalHandler(int signal) {
  if (signal == SIGINT) {
    QCoreApplication::quit();
  }
}
int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  // 注册信号处理函数
  std::signal(SIGINT, signalHandler);

  ApplicationManager manager_;
  LOG_INFO("ros_qt5_gui_app init!")
  return a.exec();
}
