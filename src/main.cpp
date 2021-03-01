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

#include <QtGui>
#include <QApplication>
#include <QDesktopWidget>
#include "../include/cyrobot_monitor/main_window.hpp"


/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    //共享opengl上下文
//    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts)
//    QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);
   //记住上一次的窗体大小和位置
    QSettings windows_setting("cyrobot_monitor","windows");
   int x = windows_setting.value("WindowGeometry/x").toInt();
   int y = windows_setting.value("WindowGeometry/y").toInt();
   int width = windows_setting.value("WindowGeometry/width").toInt();
   int height = windows_setting.value("WindowGeometry/height").toInt();
   QDesktopWidget* desktopWidget = QApplication::desktop();
   QRect clientRect = desktopWidget->availableGeometry();
   QRect targRect0 = QRect(clientRect.width()/4,clientRect.height()/4,clientRect.width()/2,clientRect.height()/2);
   QRect targRect = QRect(x,y,width,height);
   if(width == 0|| height == 0 || x<0 || x>clientRect.width() || y<0 || y>clientRect.height())//如果上一次关闭软件的时候，窗口位置不正常，则本次显示在显示器的正中央
   {
       targRect = targRect0;
   }
    cyrobot_monitor::MainWindow w(argc,argv);
    w.setGeometry(targRect);//设置主窗口的大小
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
