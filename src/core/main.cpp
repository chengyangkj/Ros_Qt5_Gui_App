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

static QApplication* g_app = nullptr;

void signalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    if (g_app) {
      g_app->exit(0);
  }
}
}

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  g_app = &a;
  
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  ApplicationManager manager;
  if (!manager.Initialize()) {
    LOG_ERROR("Failed to initialize application");
    return -1;
  }
  
  LOG_INFO("ros_qt5_gui_app initialized successfully");
  return a.exec();
}
