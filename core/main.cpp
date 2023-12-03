/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-09-28 14:56:04
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-05 11:39:01
 * @FilePath: /ROS2_Qt5_Gui_App/src/main.cpp
 */
#include "logger/logger.h"
#include "runtime/application_manager.h"
#include <QApplication>
#include <QLabel>
#include <QMovie>
#include <QPixmap>
#include <QSplashScreen>
#include <QThread>
#include <iostream>

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  ApplicationManager manager_;
  LOGGER_INFO("ros_qt5_gui_app init!")
  return a.exec();
}
