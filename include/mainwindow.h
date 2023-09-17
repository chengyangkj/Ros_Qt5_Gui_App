/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-04-20 15:46:29
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-07-27 13:56:13
 * @FilePath: /ros_qt5_gui_app/include/mainwindow.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QDateTime>
#include <QMainWindow>
#include <QMouseEvent>

#include "QTimer"
#include "basic/point_type.h"
#include "channel/base/comm_instance.h"
#include "dashboard.h"
#include "display/display_manager.h"
#include "roboGLWidget.h"
#include "roboImg.h"
#include "roboItem.h"
#include "ui_mainwindow.h"
QT_BEGIN_NAMESPACE
#define MARGIN 10
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
public slots:
  void onRecvData(QString);
  void updateOdomInfo(RobotState);
  void slotUpdateRobotPose(basic::RobotPose pose);
  void slotUpdateLaserPoint(LaserScan scan);
  void updateGlobalPath(RobotPath path);
  void updateLocalPath(RobotPath path);
  void updateLocalCostMap(CostMap map, RobotPose);
  void updateGlobalCostMap(CostMap map);
  // ui 相关函数
private:
  void mouseReleaseEvent(QMouseEvent *event);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mouseHoverMoveEvent(QMouseEvent *event);
  int countRow(QPoint p); // 获取光标在窗口所在区域的 行   返回行数
  int countFlag(QPoint p, int row); // 获取光标在窗口所在区域的 列 返回行列坐标
  void setCursorType(int flag); // 根据传入的坐标，设置光标样式
private:
  void initUi();
  void closeEvent(QCloseEvent *event); // Overloaded function
  void setCurrentMenu(QPushButton *cur_btn);

  // ui相关变量
private:
  Ui::MainWindowDesign *ui;
  QPoint pLast;
  bool left_pressed_{false};
  QPoint left_pressed_point_; // 鼠标按下后的pose
  QPoint last_pressed_point_; // 鼠标按下后的pose
  int curpos_ = 0;
  bool m_bresize = false;
  QGraphicsScene *m_qGraphicScene = nullptr;
  DashBoard *m_speedDashBoard;

private:
  QTimer *m_timerCurrentTime;
  roboGLWidget *m_roboGLWidget;
  Display::DisplayManager *display_manager_;
};
#endif // MAINWINDOW_H
