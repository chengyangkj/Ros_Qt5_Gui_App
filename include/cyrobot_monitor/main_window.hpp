/**
 * @file /include/cyrobot_monitor/main_window.hpp
 *
 * @brief Qt based gui for cyrobot_monitor.
 *
 * @date November 2010
 **/
#ifndef cyrobot_monitor_MAIN_WINDOW_H
#define cyrobot_monitor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui/QMainWindow>
#include <sensor_msgs/BatteryState.h>

#include <QComboBox>
#include <QDesktopWidget>
#include <QHBoxLayout>
#include <QQueue>
#include <QSoundEffect>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QTimer>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QVariant>
#include <map>

#include "QProcess"
#include "RobotAlgorithm.h"
#include "dashboard.h"
#include "joystick.h"
#include "qnode.hpp"
#include "qrviz.hpp"
#include "robomap.h"
#include "ui_main_window.h"
/*****************************************************************************
** Namespace
*****************************************************************************/
namespace cyrobot_monitor {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(int argc, char **argv, QWidget *parent = 0);
  ~MainWindow();
  enum {
    upleft = 0,
    up,
    upright,
    left,
    stop,
    right,
    downleft,
    down,
    downright
  };
  void ReadSettings();   // Load up qt program settings at startup
  void WriteSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage();
  void initUis();
  void initVideos();
  void initTopicList();
  void initOthers();
  bool connectMaster(QString master_ip, QString ros_ip, bool use_envirment);
  enum SHOWMODE {
    robot,
    control,
  };

 public slots:
  /******************************************
   ** Auto-connections (connectSlotsByName())
   *******************************************/
  void on_actionAbout_triggered();
  void slot_speed_x(double x);
  void slot_speed_yaw(double yaw);
  void slot_batteryState(sensor_msgs::BatteryState);
  void slot_rosShutdown();
  void quick_cmds_check_change(int);
  void cmd_output();
  void cmd_error_output();
  void refreashTopicList();
  /******************************************
  ** Manual connections
  *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically
  void Slider_raw_valueChanged(int v);
  void Slider_linear_valueChanged(int value);
  void slot_cmd_control();
  void slot_tab_manage_currentChanged(int);
  void slot_tab_Widget_currentChanged(int);
  void slot_set_2D_Goal();
  void slot_set_2D_Pos();
  void slot_set_select();
  void slot_move_camera_btn();
  //设置界面
  void slot_setting_frame();
  void slot_set_mutil_goal_btn();
  //返航
  void slot_return_point();
  //机器人位置
  void slot_position_change(QString, double, double, double, double);
  void quick_cmd_add();
  void quick_cmd_remove();
  //显示图像
  //显示图像
  void slot_show_image(int, QImage);
  void slot_dis_connect();
  void slot_hide_table_widget();
  void slot_rockKeyChange(int);
  void slot_closeWindows();
  void slot_minWindows();
  void slot_maxWindows();
  void slot_chartTimerTimeout();
  void slot_pubImageMapTimeOut();
  void slot_updateCursorPos(QPointF pos);
  void slot_changeMapType(int);
  void slot_updateRobotStatus(algo::RobotStatus);
  //    void on_horizontalSlider_raw_valueChanged(int value);
 private slots:

 signals:
  void signalSet2DPose();
  void signalSet2DGoal();
  void signalSetMoveCamera();
  void signalDisconnect();

 private:
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mouseReleaseEvent(QMouseEvent *event);
  void connections();
  void add_quick_cmd(QString name, QString shell);
  void display_rviz();
  void setCurrentMenu(QPushButton *btn);

 private:
  Ui::MainWindowDesign ui;
  bool isPressedWidget;
  QPoint m_lastPos;
  QNode qnode;
  QProcess *quick_cmd = NULL;
  QProcess *close_remote_cmd = NULL;
  QProcess *base_cmd = NULL;
  QStandardItemModel *treeView_rviz_model = NULL;
  //存放rviz treewidget当前显示的控件及控件的父亲的地址
  QMap<QWidget *, QTreeWidgetItem *> widget_to_parentItem_map;
  //存放状态栏的对应关系 display名 状态item
  QMap<QString, QTreeWidgetItem *> tree_rviz_stues;
  //存放display的当前值 item名，参数名称和值
  QMap<QTreeWidgetItem *, QMap<QString, QString>> tree_rviz_values;
  QSoundEffect *media_player = NULL;
  bool m_useEnviorment = false;
  bool m_autoConnect = false;
  SHOWMODE m_showMode;
  QString m_masterUrl;
  QString m_hostUrl;
  double m_turnLightThre = 0.1;
  JoyStick *rock_widget;
  QGraphicsScene *m_qgraphicsScene = NULL;
  roboMap *m_roboMap = NULL;
  QRviz *map_rviz = NULL;
  QVariantList m_sendVelList, m_recvVelList, m_timeList;
  //曲线
  //    QSplineSeries* line;
  //曲线点的最大数量
  int line_max = 10;
  //绘图变量和坐标
  //    QChart* chart;
  //    QValueAxis *axisX;
  //    QValueAxis *axisY;
  //    QQueue<QPointF> data1;
  //    QQueue<QPointF> data2;
  QTimer *m_timerChart;
  QTimer *m_timerPubImageMap;
  QTimer *m_timerCurrentTime;
  //    QChartView *chartView;
  DashBoard *speedDashBoard;
};
}  // namespace cyrobot_monitor

#endif  // cyrobot_monitor_MAIN_WINDOW_H
