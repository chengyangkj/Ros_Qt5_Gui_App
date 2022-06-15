#include "mainwindow.h"

#include <QApplication>
#include <iostream>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType<RobotPose>("RobotPose");
    MainWindow w;
    w.show();
    return a.exec();
}
