/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-04-20 15:46:29
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-04-20 16:03:40
 * @FilePath: /ROS2_Qt5_Gui_App/include/mainwindow.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QDateTime>
#include <QMainWindow>

#include "QTimer"
#include "dashboard.h"
#include "rclcomm.h"
#include "roboGLWidget.h"
#include "roboImg.h"
#include "roboItem.h"
#include "ui_mainwindow.h"
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  rclcomm *commNode;
 public slots:
  void onRecvData(QString);
  void updateRobotPose(RobotPose pose);
  void updateOdomInfo(RobotState);
 private:
  void initUi();
  void closeEvent(QCloseEvent *event);  // Overloaded function
  void setCurrentMenu(QPushButton *cur_btn);
 private:
  Ui::MainWindowDesign *ui;
    QPoint pLast;
  bool m_bPressed;
  QPoint clickPos;
  bool m_bresize = false;
  QGraphicsScene *m_qGraphicScene = nullptr;
  roboItem *m_roboItem = nullptr;
  roboImg *m_roboImg = nullptr;
  DashBoard *m_speedDashBoard;
  QTimer *m_timerCurrentTime;
  roboGLWidget *m_roboGLWidget;
};
#endif  // MAINWINDOW_H
