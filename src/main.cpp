
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

#include <QApplication>
#include <QDesktopWidget>
#include <QtGui>

#include "../include/cyrobot_monitor/loginwidget.h"

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
  LoginWidget w;
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
