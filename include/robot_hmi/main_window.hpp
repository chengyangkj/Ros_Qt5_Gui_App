/**
 * @file /include/robot_hmi/main_window.hpp
 *
 * @brief Qt based gui for robot_hmi.
 *
 * @date November 2010
 **/
#ifndef robot_hmi_MAIN_WINDOW_H
#define robot_hmi_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "CCtrlDashBoard.h"
#include <QImage>
#include <QProcess>
#include <QComboBox>
#include <QSpinBox>
#include "qrviz.hpp"
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QHttpPart>
#include <QHttpMultiPart>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robot_hmi {

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

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_linera_value_change(int);
    void slot_raw_value_change(int);
    void slot_pushbtn_click();
    void slot_update_dashboard(float,float);
    void slot_update_power(float);
    void slot_update_image(QImage);
    void slot_sub_image();
    void slot_quick_cmd_clicked();
    void slot_quick_output();
    void slot_treewidget_value_change(QString);
    void slot_display_grid(int);
    void slot_display_tf(int);
    void slot_display_laser(int);
    void slot_display_RobotModel(int);
    void slot_display_Map(int);
    void slot_display_Path(int);
    void slot_set_start_pose();
    void slot_set_goal_pose();
    void slot_display_local_map(int state);
    void slot_display_global_map(int state);
    void slot_update_pos(double,double,double);
    void slot_set_return_pos();
    void slot_return();
    void slot_add_point();
    void slot_get_pose();
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    CCtrlDashBoard* speed_x_dashBoard;
    CCtrlDashBoard* speed_y_dashBoard;
    QProcess *laser_cmd;
    qrviz* myrviz;
    QComboBox* fixed_box;
    QSpinBox* Cell_Count_Box;
    QComboBox* Grid_Color_Box;
    QComboBox* Laser_Topic_box;
    QComboBox* Map_Topic_box;
    QComboBox* Map_Color_Scheme_box;
    QComboBox* Path_Topic_box;
    QComboBox* Path_Color_box;
    //Navigate
    QComboBox* Global_CostMap_Topic_box;
    QComboBox* GlobalMapColorScheme_box;
    QComboBox* Local_CostMap_Topic_box;
    QComboBox* LocalMapColorScheme_box;
    QComboBox* Global_Planner_Topic_box;
    QComboBox* Global_Planner_Color_box;
    QComboBox* Local_Planner_Topic_box;
    QComboBox* Local_Planner_Color_box;

    QNetworkAccessManager *networkManager;
    QNetworkRequest request;

    double return_x;
    double return_y;
    double return_z;
};

}  // namespace robot_hmi

#endif // robot_hmi_MAIN_WINDOW_H
