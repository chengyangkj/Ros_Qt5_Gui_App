#include <QApplication>
#include <iostream>

#include "mainwindow.h"
int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  qRegisterMetaType<RobotPose>("RobotPose");
  MainWindow w;
  w.show();
  return a.exec();
}
