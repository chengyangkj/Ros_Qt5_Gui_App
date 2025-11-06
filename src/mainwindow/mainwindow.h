#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QCalendarWidget>
#include <QComboBox>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QGraphicsItem>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
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
#include <opencv2/imgproc/imgproc.hpp>
#include "DockAreaWidget.h"
#include "DockManager.h"
#include "DockWidget.h"
#include "channel_manager.h"
#include "config/config_manager.h"
#include "display/manager/display_manager.h"
#include "point_type.h"
#include "widgets/dashboard.h"
#include "widgets/nav_goal_table_view.h"
#include "widgets/set_pose_widget.h"
#include "widgets/speed_ctrl.h"
#include "widgets/ratio_layouted_frame.h"
#include "core/framework/framework.h"
#include <memory>
#include <vector>
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
  void RecvChannelMsg(const MsgId &id, const std::any &data);
  void updateOdomInfo(RobotState state);
  void RestoreState();
  void SlotSetBatteryStatus(double percent, double voltage);
  void SlotRecvImage(const std::string &location, std::shared_ptr<cv::Mat> data);

 protected:
  virtual void closeEvent(QCloseEvent *event) override;

 private:
  QAction *SavePerspectiveAction = nullptr;
  QWidgetAction *PerspectiveListAction = nullptr;
  ChannelManager channel_manager_;
  Ui::MainWindow *ui;
  DashBoard *speed_dash_board_;
  ads::CDockManager *dock_manager_;
  ads::CDockAreaWidget *StatusDockArea;
  ads::CDockWidget *TimelineDockWidget;
  Display::DisplayManager *display_manager_;

  QLineEdit *label_pos_map_;
  QLineEdit *label_pos_scene_;
  QLineEdit *label_pos_robot_;
  QThread message_thread_;
  SpeedCtrlWidget *speed_ctrl_widget_;
  NavGoalTableView *nav_goal_table_view_;
  QProgressBar *battery_bar_;
  QLabel *label_power_;
  ads::CDockAreaWidget *center_docker_area_;
  std::map<std::string, RatioLayoutedFrame *> image_frame_map_;
  std::string map_path_{"./map"};
  
 signals:
  void OnRecvChannelData(const MsgId &id, const std::any &data);
  
 private:
  void setupUi();
  bool openChannel();
  bool openChannel(const std::string &channel_name);
  void closeChannel();
  void registerChannel();
  void SaveState();
};
#endif  // MAINWINDOW_H
