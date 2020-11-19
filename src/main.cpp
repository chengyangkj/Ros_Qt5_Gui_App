/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QGuiApplication>
#include <QtGui>
#include <QApplication>
#include <QObject>
#include <QtQml/QQmlApplicationEngine>
#include <QtQuick/QQuickItem>
#include <QQmlContext>
#include <QDebug>
#include "../include/cyrobot_monitor/robomap.h"
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
   //注册c++类为qml类型 第一个参数* uri指的是QML中import后的内容，相当于头文件名；第二个第三个参数分别是主次版本号；第四个指的是QML中类的名字 第四个参数开头需大写
   qmlRegisterType<roboMap>("QRoboMap",1,0,"RoboMap");
   QQmlApplicationEngine egin;
   //将c++类的对象绑定到qml
   //roboMap *map=new roboMap();
   //egin.rootContext()->setContextProperty("roboMap", map);
   egin.load(QUrl(QStringLiteral("qrc:/qml/main.qml")));
   int result = app.exec();
   return result;
}
