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
#include "../include/cyrobot_monitor/qnode.hpp"
#include <iostream>
#include <QtQml/QQmlApplicationEngine>
#include <QtQuick/QQuickItem>
#include <QQmlContext>
/*****************************************************************************
** Main
*****************************************************************************/
using namespace cyrobot_monitor;
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
   QList<QObject*> objList = egin.rootObjects();
   roboMap *roboMap_ =(roboMap*) objList.at(0)->findChild<QObject*>("roboMap_");
   QNode node(argc,argv);
   node.init();
   QObject::connect(&node,SIGNAL(updateMap(QImage,QSizeF)),roboMap_,SLOT(paintMaps(QImage,QSizeF)));
   QObject::connect(&node,SIGNAL(updateRoboPose(QPointF,float)),roboMap_,SLOT(paintRoboPos(QPointF,float)));
   QObject::connect(&node,SIGNAL(Show_image(int,QImage)),roboMap_,SLOT(paintImage(int,QImage)));
   QObject::connect(&node,SIGNAL(plannerPath(QPolygonF)),roboMap_,SLOT(paintPlannerPath(QPolygonF)));
   QObject::connect(&node,SIGNAL(updateLaserScan(QPolygonF)),roboMap_,SLOT(paintLaserScan(QPolygonF)));
   int result = app.exec();
   return result;
}
