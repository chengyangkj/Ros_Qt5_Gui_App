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
#include "ui_main_window.h"
#include "qnode.hpp"
#include "settings.h"
#include "joystick.h"
#include "robomap.h"
#include "QProcess"
#include <QStandardItemModel>
#include <QTreeWidgetItem>
#include <QSoundEffect>
#include <QComboBox>
#include <QSpinBox>
#include <QVariant>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QtCharts>
#include <QTimer>
#include <QQueue>
#include <map>
#include "dashboard.h"

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
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
  enum {upleft=0,up,upright,left,stop,right,downleft,down,downright};
	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void initUis();
    void initVideos();
    void initTopicList();
    void initCharts();
public slots:
    /******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check );
    void slot_speed_x(double x);
    void slot_speed_yaw(double yaw);
    void slot_power(float p);
    void slot_rosShutdown();
    void quick_cmds_check_change(int);
    void cmd_output();
    void cmd_error_output();
    void refreashTopicList();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
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
    //设置返航点
    void slot_set_return_point();
    //返航
    void slot_return_point();
    //机器人位置
    void slot_position_change(QString,double,double,double,double);
    void quick_cmd_add();
    void quick_cmd_remove();
    //显示图像
    //显示图像
    void slot_show_image(int,QImage);
    void slot_dis_connect();
    void slot_hide_table_widget();
    void slot_rockKeyChange(int);
    void slot_closeWindows();
    void slot_minWindows();
    void slot_maxWindows();
    void slot_chartTimerTimeout();
//    void on_horizontalSlider_raw_valueChanged(int value);
private slots:


signals:
    void signalSet2DPose();
    void signalSet2DGoal();
private:
  Ui::MainWindowDesign ui;
    bool isPressedWidget;
    QPoint m_lastPos;
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void connections();
    void add_quick_cmd(QString name,QString shell);
    QNode qnode;
    QProcess *quick_cmd=NULL;
    QProcess *close_remote_cmd=NULL;
    QProcess *base_cmd=NULL;
    QStandardItemModel* treeView_rviz_model=NULL;
    //存放rviz treewidget当前显示的控件及控件的父亲的地址
    QMap <QWidget*,QTreeWidgetItem *> widget_to_parentItem_map;
    //存放状态栏的对应关系 display名 状态item
    QMap <QString,QTreeWidgetItem *> tree_rviz_stues;
    //存放display的当前值 item名，参数名称和值
    QMap <QTreeWidgetItem*,QMap<QString,QString>> tree_rviz_values;
    Settings *set=NULL;
    QSoundEffect *media_player=NULL;
    bool m_useEnviorment=false;
    bool m_autoConnect=false;
    QString m_masterUrl;
    QString m_hostUrl;
    double m_turnLightThre=0.1;
    JoyStick *rock_widget;
    QGraphicsScene  *m_qgraphicsScene=NULL;
    roboMap *m_roboMap=NULL;
    QVariantList m_sendVelList,m_recvVelList,m_timeList;
    //曲线
    QSplineSeries* line;
    //曲线点的最大数量
    int line_max = 10;
    //绘图变量和坐标
    QChart* chart;
    QValueAxis *axisX;
    QValueAxis *axisY;
    QQueue<QPointF> data1;
    QQueue<QPointF> data2;
    QTimer *m_timerChart;
    QChartView *chartView;
    DashBoard *speedDashBoard;
};
}// namespace cyrobot_monitor

#endif // cyrobot_monitor_MAIN_WINDOW_H
