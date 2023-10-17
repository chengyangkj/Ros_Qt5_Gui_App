/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-10-01 06:31:04
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-06 12:09:12
 * @FilePath: /examples/centralwidget/mainwindow.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "DockAreaWidget.h"
#include "DockManager.h"
#include "DockWidget.h"
#include "basic/point_type.h"
#include "channel/base/comm_instance.h"
#include "dashboard.h"
#include "display/display_manager.h"
#include "joystick.h"
#include <QCalendarWidget>
#include <QComboBox>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QGraphicsItem>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QTableWidget>
#include <QToolBar>
#include <QTreeView>
#include <QWidgetAction>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListView>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
QT_BEGIN_NAMESPACE
namespace Ui {
class CMainWindow;
}
QT_END_NAMESPACE

class CMainWindow : public QMainWindow {
  Q_OBJECT

public:
  CMainWindow(QWidget *parent = nullptr);
  ~CMainWindow();
public slots:
  void onRecvData(QString);
  void updateOdomInfo(RobotState);
  void slotUpdateRobotPose(basic::RobotPose pose);
  void slotUpdateLaserPoint(LaserScan scan);
  void updateGlobalPath(RobotPath path);
  void updateLocalPath(RobotPath path);
  void updateLocalCostMap(CostMap, basic::RobotPose);
  void updateGlobalCostMap(CostMap map);
  void updateCurpose(QPointF pos);


protected:
  virtual void closeEvent(QCloseEvent *event) override;

private:
  QAction *SavePerspectiveAction = nullptr;
  QWidgetAction *PerspectiveListAction = nullptr;
  QComboBox *PerspectiveComboBox = nullptr;

  Ui::CMainWindow *ui;
  DashBoard *speed_dash_board_;
  ads::CDockManager *dock_manager_;
  ads::CDockAreaWidget *StatusDockArea;
  ads::CDockWidget *TimelineDockWidget;
  Display::DisplayManager *display_manager_;
  QPushButton *pushButton_status_;
  QProgressBar *battery_bar_;

  QLabel *label_power_;
  QLabel *label_pos_map_;
  QLabel *label_pos_scene_;
  JoyStick *joyStick_widget_;

private:
  void setupUi();
private slots:
  void savePerspective();
};
#endif // MAINWINDOW_H
