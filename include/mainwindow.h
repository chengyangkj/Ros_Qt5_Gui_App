#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "DockAreaWidget.h"
#include "DockManager.h"
#include "DockWidget.h"
#include "basic/point_type.h"
#include "channel/base/comm_instance.h"
#include "display/display_manager.h"
#include "widgets/dashboard.h"

#include "widgets/speed_ctrl.h"
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
  void slotUpdateRobotPose(RobotPose pose);
  void slotUpdateLaserPoint(LaserScan scan);
  void updateGlobalPath(RobotPath path);
  void updateLocalPath(RobotPath path);
  void updateLocalCostMap(CostMap, RobotPose);
  void updateGlobalCostMap(CostMap map);
  void signalCursorPose(QPointF pos);



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
  QCheckBox *checkBox_use_all_;
  QProgressBar *battery_bar_;
  QSlider *horizontalSlider_linear_;
  QSlider *horizontalSlider_raw_;
  QLabel *label_power_;
  QLabel *label_pos_map_;
  QLabel *label_pos_scene_;

  SpeedCtrlWidget *speed_ctrl_widget_;

private:
  void setupUi();
private slots:
  void savePerspective();
};
#endif // MAINWINDOW_H
