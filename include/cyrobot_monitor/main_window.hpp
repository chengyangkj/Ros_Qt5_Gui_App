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
#include "addtopics.h"
#include "settings.h"
#include "qrviz.h"
#include "qrviz_widget.h"
//仪表盘头文件
#include "CCtrlDashBoard.h"
#include "QProcess"
#include <QStandardItemModel>
#include <QTreeWidgetItem>
#include <QComboBox>
#include <QSpinBox>
#include <QVariant>
#include <map>
//rviz
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include<rviz/tool_manager.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
/*****************************************************************************
** Namespace
*****************************************************************************/
namespace rvt = rviz_visual_tools;
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

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void initRviz();
    void initUis();


public slots:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check );
    void on_checkbox_use_environment_stateChanged(int state);
    void slot_speed_x(double x);
    void slot_speed_y(double y);
    void slot_power(float p);
    void slot_rosShutdown();
    void quick_cmds();
    void cmd_output();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void on_Slider_raw_valueChanged(int value);
    void on_Slider_linear_valueChanged(int value);
    void slot_cmd_control();
    void slot_tab_manage_currentChanged(int);
    void slot_tab_Widget_currentChanged(int);
    void slot_add_topic_btn();
    void slot_choose_topic(QTreeWidgetItem *choose);
    void slot_treewidget_item_value_change(QString);
    void slot_treewidget_item_check_change(int);
//    void on_horizontalSlider_raw_valueChanged(int value);
private slots:


private:
	Ui::MainWindowDesign ui;
    void connections();
    QNode qnode;
    CCtrlDashBoard *m_DashBoard_x;
    CCtrlDashBoard *m_DashBoard_y;
    QProcess *laser_cmd=NULL;
    QProcess *close_remote_cmd=NULL;
    QProcess *base_cmd=NULL;
    QRviz *map_rviz=NULL;
    QRviz_widget *qrviz=NULL;
    QStandardItemModel* treeView_rviz_model=NULL;
    AddTopics *addtopic_form=NULL;
    QMap <QWidget*,QTreeWidgetItem *> tree_rviz_keys;
    Settings *set;
};
}// namespace cyrobot_monitor

#endif // cyrobot_monitor_MAIN_WINDOW_H
