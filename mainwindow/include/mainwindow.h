#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "DockAreaWidget.h"
#include "DockManager.h"
#include "DockWidget.h"
#include "channel_manager.h"
#include "display/display_manager.h"
#include "point_type.h"
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
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
public slots:
  void signalCursorPose(QPointF pos);
  void SendChannelMsg(const MsgId &id, const std::any &data);
  void updateOdomInfo(RobotState state);

protected:
  virtual void closeEvent(QCloseEvent *event) override;

private:
  QAction *SavePerspectiveAction = nullptr;
  QWidgetAction *PerspectiveListAction = nullptr;
  QComboBox *PerspectiveComboBox = nullptr;
  ChannelManager channel_manager_;
  Ui::MainWindow *ui;
  DashBoard *speed_dash_board_;
  ads::CDockManager *dock_manager_;
  ads::CDockAreaWidget *StatusDockArea;
  ads::CDockWidget *TimelineDockWidget;
  Display::DisplayManager *display_manager_;

  QLabel *label_pos_map_;
  QLabel *label_pos_scene_;
  QThread message_thread_;
  SpeedCtrlWidget *speed_ctrl_widget_;
  StatusBarWidget *status_bar_widget_;
  ToolsBarWidget *tools_bar_widget_;
signals:
  void OnRecvChannelData(const MsgId &id, const std::any &data);

private:
  void setupUi();
  bool openChannel();
  bool openChannel(const std::string &channel_name);
  void closeChannel();
  void registerChannel();
private slots:
  void savePerspective();
};
#endif // MAINWINDOW_H
