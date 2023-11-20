#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "point_type.h"
#include "DockAreaWidget.h"
#include "DockManager.h"
#include "DockWidget.h"
#include "channel_instance.h"
#include "display/display_manager.h"
#include "widgets/dashboard.h"
#include "widgets/set_pose_widget.h"
#include "widgets/speed_ctrl.h"
#include "widgets/status_bar.h"
#include "widgets/tools_bar.h"
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

  QLabel *label_pos_map_;
  QLabel *label_pos_scene_;

  SpeedCtrlWidget *speed_ctrl_widget_;
  StatusBarWidget *status_bar_widget_;
  ToolsBarWidget *tools_bar_widget_;

private:
  void setupUi();
private slots:
  void savePerspective();
};
#endif // MAINWINDOW_H
