/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-10-06 07:12:50
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-06 14:02:27
 * @FilePath: /ROS2_Qt5_Gui_App/src/ MainWindow.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "mainwindow.h"
#include "AutoHideDockContainer.h"
#include "DockAreaTabBar.h"
#include "DockAreaTitleBar.h"
#include "DockAreaWidget.h"
#include "DockComponentsFactory.h"
#include "Eigen/Dense"
#include "FloatingDockContainer.h"
#include "algorithm.h"
#include "logger/logger.h"
#include "ui_mainwindow.h"
#include "widgets/speed_ctrl.h"
#include <QDebug>
#include <iostream>
using namespace ads;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  Q_INIT_RESOURCE(images);
  Q_INIT_RESOURCE(media);
  LOG_INFO(" MainWindow init thread id" << QThread::currentThreadId());
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<RobotPose>("RobotPose");
  qRegisterMetaType<RobotSpeed>("RobotSpeed");
  qRegisterMetaType<RobotState>("RobotState");
  qRegisterMetaType<OccupancyMap>("OccupancyMap");
  qRegisterMetaType<OccupancyMap>("OccupancyMap");
  qRegisterMetaType<LaserScan>("LaserScan");
  qRegisterMetaType<RobotPath>("RobotPath");
  qRegisterMetaType<MsgId>("MsgId");
  qRegisterMetaType<std::any>("std::any");
  qRegisterMetaType<TopologyMap>("TopologyMap");
  qRegisterMetaType<TopologyMap::PointInfo>("TopologyMap::PointInfo");
  setupUi();
  openChannel();
  QTimer::singleShot(50, [=]() { RestoreState(); });
}
bool MainWindow::openChannel() {
  if (channel_manager_.OpenChannelAuto()) {
    registerChannel();
    return true;
  }
  return false;
}
bool MainWindow::openChannel(const std::string &channel_name) {
  if (channel_manager_.OpenChannel(channel_name)) {
    registerChannel();
    return true;
  }
  return false;
}
void MainWindow::registerChannel() {
  channel_manager_.RegisterOnDataCallback(
      std::move([this](const MsgId &id, const std::any &data) {
        emit OnRecvChannelData(id, data);
      }));
}
void MainWindow::RecvChannelMsg(const MsgId &id, const std::any &data) {
  switch (id) {
  case MsgId::kOdomPose:
    updateOdomInfo(std::any_cast<RobotState>(data));
    break;
  case MsgId::kRobotPose: {
    nav_goal_table_view_->UpdateRobotPose(std::any_cast<RobotPose>(data));
  } break;
  case MsgId::kBatteryState: {
    std::map<std::string, std::string> map =
        std::any_cast<std::map<std::string, std::string>>(data);
    status_bar_widget_->SlotSetBatteryStatus(std::stod(map["percent"]),
                                             std::stod(map["voltage"]));
  } break;
  default:
    break;
  }
  display_manager_->UpdateTopicData(id, data);
}
void MainWindow::SendChannelMsg(const MsgId &id, const std::any &data) {
  channel_manager_.SendMessage(id, data);
}
void MainWindow::closeChannel() { channel_manager_.CloseChannel(); }
MainWindow::~MainWindow() { delete ui; }
void MainWindow::setupUi() {
  ui->setupUi(this);
  CDockManager::setConfigFlag(CDockManager::OpaqueSplitterResize, true);
  CDockManager::setConfigFlag(CDockManager::XmlCompressionEnabled, false);
  CDockManager::setConfigFlag(CDockManager::FocusHighlighting, true);
  CDockManager::setConfigFlag(CDockManager::DockAreaHasUndockButton, false);
  CDockManager::setConfigFlag(CDockManager::DockAreaHasTabsMenuButton, false);
  CDockManager::setConfigFlag(CDockManager::MiddleMouseButtonClosesTab, true);
  CDockManager::setConfigFlag(CDockManager::EqualSplitOnInsertion, true);
  CDockManager::setConfigFlag(CDockManager::ShowTabTextOnlyForActiveTab, true);
  CDockManager::setAutoHideConfigFlags(CDockManager::DefaultAutoHideConfig);
  dock_manager_ = new CDockManager(this);
  QVBoxLayout *central_layout = new QVBoxLayout();
  /////////////////////////////////////////////////////////////////状态栏
  status_bar_widget_ = new StatusBarWidget();
  central_layout->addWidget(status_bar_widget_);

  ///////////////////////////////////////////////////////////////地图工具栏
  tools_bar_widget_ = new ToolsBarWidget();
  central_layout->addWidget(tools_bar_widget_);
  /////////////////////////////////////////////////////////////////////////地图显示

  display_manager_ = new Display::DisplayManager();
  central_layout->addWidget(display_manager_->GetViewPtr());

  //////////////////////////////////////////////////////////////////////////坐标显示
  QHBoxLayout *horizontalLayout_12 = new QHBoxLayout();
  horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
  QLabel *label = new QLabel();
  label->setText("map:");
  label->setObjectName(QString::fromUtf8("label"));
  label->setMinimumSize(QSize(80, 40));

  horizontalLayout_12->addWidget(label);

  label_pos_map_ = new QLabel();
  label_pos_map_->setObjectName(QString::fromUtf8("label_pos_map_"));
  label_pos_map_->setMinimumSize(QSize(200, 20));
  label_pos_map_->setMaximumSize(QSize(200, 20));
  label_pos_map_->setStyleSheet(QString::fromUtf8(""));

  horizontalLayout_12->addWidget(label_pos_map_);

  QLabel *label_5 = new QLabel();
  label_5->setText("scene:");

  label_5->setObjectName(QString::fromUtf8("label_5"));
  label_5->setMinimumSize(QSize(80, 40));

  horizontalLayout_12->addWidget(label_5);

  label_pos_scene_ = new QLabel();
  label_pos_scene_->setObjectName(QString::fromUtf8("label_pos_scene_"));
  label_pos_scene_->setMinimumSize(QSize(200, 20));
  label_pos_scene_->setMaximumSize(QSize(200, 20));

  horizontalLayout_12->addWidget(label_pos_scene_);

  QSpacerItem *horizontalSpacer_3 =
      new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

  horizontalLayout_12->addItem(horizontalSpacer_3);

  central_layout->addLayout(horizontalLayout_12);
  QWidget *center_widget = new QWidget();

  center_widget->setLayout(central_layout);
  CDockWidget *CentralDockWidget = new CDockWidget("CentralWidget");
  CentralDockWidget->setWidget(center_widget);
  auto *CentralDockArea = dock_manager_->setCentralWidget(CentralDockWidget);
  CentralDockArea->setAllowedAreas(DockWidgetArea::OuterDockAreas);

  //////////////////////////////////////////////////////////速度仪表盘
  ads::CDockWidget *DashBoardDockWidget = new ads::CDockWidget("DashBoard");
  QWidget *speed_dashboard_widget = new QWidget();
  // speed_dashboard_widget->setMinimumSize(QSize(300, 300));
  DashBoardDockWidget->setWidget(speed_dashboard_widget);
  speed_dash_board_ = new DashBoard(speed_dashboard_widget);
  auto dashboard_area =
      dock_manager_->addDockWidget(ads::DockWidgetArea::LeftDockWidgetArea,
                                   DashBoardDockWidget, CentralDockArea);
  ui->menuView->addAction(DashBoardDockWidget->toggleViewAction());

  ////////////////////////////////////////////////////////速度控制
  speed_ctrl_widget_ = new SpeedCtrlWidget();
  connect(speed_ctrl_widget_, &SpeedCtrlWidget::signalControlSpeed,
          [this](const RobotSpeed &speed) {
            SendChannelMsg(MsgId::kSetRobotSpeed, speed);
          });
  ads::CDockWidget *SpeedCtrlDockWidget = new ads::CDockWidget("SpeedCtrl");
  SpeedCtrlDockWidget->setWidget(speed_ctrl_widget_);
  auto speed_ctrl_area =
      dock_manager_->addDockWidget(ads::DockWidgetArea::BottomDockWidgetArea,
                                   SpeedCtrlDockWidget, dashboard_area);
  ui->menuView->addAction(SpeedCtrlDockWidget->toggleViewAction());

  /////////////////////////////////////////////////////////导航任务列表

  QWidget *task_list_widget = new QWidget();
  nav_goal_table_view_ = new NavGoalTableView();
  QVBoxLayout *horizontalLayout_13 = new QVBoxLayout();
  horizontalLayout_13->addWidget(nav_goal_table_view_);
  task_list_widget->setLayout(horizontalLayout_13);
  ads::CDockWidget *nav_goal_list_dock_widget = new ads::CDockWidget("Task");
  QPushButton *btn_add_one_goal = new QPushButton("Add Point");
  QHBoxLayout *horizontalLayout_15 = new QHBoxLayout();
  QPushButton *btn_start_task_chain = new QPushButton("Start Task Chain");
  QHBoxLayout *horizontalLayout_14 = new QHBoxLayout();
  horizontalLayout_15->addWidget(btn_add_one_goal);
  horizontalLayout_14->addWidget(btn_start_task_chain);

  QPushButton *btn_load_task_chain = new QPushButton("Load Task Chain");
  QPushButton *btn_save_task_chain = new QPushButton("Save Task Chain");
  QHBoxLayout *horizontalLayout_16 = new QHBoxLayout();
  horizontalLayout_16->addWidget(btn_load_task_chain);
  horizontalLayout_16->addWidget(btn_save_task_chain);

  horizontalLayout_13->addLayout(horizontalLayout_15);
  horizontalLayout_13->addLayout(horizontalLayout_14);
  horizontalLayout_13->addLayout(horizontalLayout_16);
  nav_goal_list_dock_widget->setWidget(task_list_widget);
  nav_goal_list_dock_widget->setMinimumSizeHintMode(
      CDockWidget::MinimumSizeHintFromDockWidget);
  nav_goal_list_dock_widget->setMinimumSize(200, 150);
  nav_goal_list_dock_widget->setMaximumSize(480, 9999);
  dock_manager_->addDockWidget(ads::DockWidgetArea::RightDockWidgetArea,
                               nav_goal_list_dock_widget, CentralDockArea);
  nav_goal_list_dock_widget->toggleView(true);
  connect(nav_goal_table_view_, &NavGoalTableView::signalSendNavGoal,
          [this](const RobotPose &pose) {
            SendChannelMsg(MsgId::kSetNavGoalPose, pose);
          });
  // nav_goal_list_dock_widget->toggleView(false);
  ui->menuView->addAction(nav_goal_list_dock_widget->toggleViewAction());
  connect(
      btn_add_one_goal, &QPushButton::clicked,
      [this, nav_goal_list_dock_widget]() { nav_goal_table_view_->AddItem(); });
  connect(btn_start_task_chain, &QPushButton::clicked,
          [this, btn_start_task_chain]() {
            if (btn_start_task_chain->text() == "Start Task Chain") {
              btn_start_task_chain->setText("Stop Task Chain");
              nav_goal_table_view_->StartTaskChain();
            } else {
              btn_start_task_chain->setText("Start Task Chain");
              nav_goal_table_view_->StopTaskChain();
            }
          });
  connect(nav_goal_table_view_, &NavGoalTableView::signalTaskFinish,
          [this, btn_start_task_chain]() {
            LOG_INFO("task finish!");
            btn_start_task_chain->setText("Start Task Chain");
          });
  connect(display_manager_,
          SIGNAL(signalTopologyMapUpdate(const TopologyMap &)),
          nav_goal_table_view_, SLOT(UpdateTopologyMap(const TopologyMap &)));
  connect(
      display_manager_,
      SIGNAL(signalCurrentSelectPointChanged(const TopologyMap::PointInfo &)),
      nav_goal_table_view_,
      SLOT(UpdateSelectPoint(const TopologyMap::PointInfo &)));
  //////////////////////////////////////////////////////槽链接

  connect(this, SIGNAL(OnRecvChannelData(const MsgId &, const std::any &)),
          this, SLOT(RecvChannelMsg(const MsgId &, const std::any &)));
  connect(display_manager_, &Display::DisplayManager::signalPub2DPose,
          [this](const RobotPose &pose) {
            SendChannelMsg(MsgId::kSetRelocPose, pose);
          });
  connect(display_manager_, &Display::DisplayManager::signalPub2DGoal,
          [this](const RobotPose &pose) {
            SendChannelMsg(MsgId::kSetNavGoalPose, pose);
          });
  // ui相关
  connect(tools_bar_widget_, &ToolsBarWidget::SignalSetRelocPose,
          [=]() { display_manager_->SetRelocPose(); });
  connect(tools_bar_widget_, &ToolsBarWidget::SignalSetNavPose,
          [=]() { display_manager_->AddOneNavPoint(); });
  connect(tools_bar_widget_, &ToolsBarWidget::SignalFocusRobot,
          [=](bool is_focus) {
            display_manager_->FocusDisplay(is_focus ? DISPLAY_ROBOT : "");
          });
  connect(display_manager_->GetDisplay(DISPLAY_MAP),
          SIGNAL(signalCursorPose(QPointF)), this,
          SLOT(signalCursorPose(QPointF)));
}

void MainWindow::signalCursorPose(QPointF pos) {
  basic::Point mapPos =
      display_manager_->mapPose2Word(basic::Point(pos.x(), pos.y()));
  label_pos_map_->setText("x: " + QString::number(mapPos.x).mid(0, 4) +
                          "  y: " + QString::number(mapPos.y).mid(0, 4));
  label_pos_scene_->setText("x: " + QString::number(pos.x()).mid(0, 4) +
                            "  y: " + QString::number(pos.y()).mid(0, 4));
}

//============================================================================
void MainWindow::closeEvent(QCloseEvent *event) {
  // Delete dock manager here to delete all floating widgets. This ensures
  // that all top level windows of the dock manager are properly closed
  // write state
  SaveState();
  dock_manager_->deleteLater();
  QMainWindow::closeEvent(event);
}
void MainWindow::SaveState() {
  QSettings settings("state.ini", QSettings::IniFormat);
  settings.setValue("mainWindow/Geometry", this->saveGeometry());
  settings.setValue("mainWindow/State", this->saveState());
  dock_manager_->addPerspective("history");
  dock_manager_->savePerspectives(settings);
}

//============================================================================
void MainWindow::RestoreState() {
  QSettings settings("state.ini", QSettings::IniFormat);
  this->restoreGeometry(settings.value("mainWindow/Geometry").toByteArray());
  this->restoreState(settings.value("mainWindow/State").toByteArray());
  dock_manager_->loadPerspectives(settings);
  dock_manager_->openPerspective("history");
}
void MainWindow::updateOdomInfo(RobotState state) {
  // 转向灯
  //   if (state.w > 0.1) {
  //     ui->label_turnLeft->setPixmap(
  //         QPixmap::fromImage(QImage("://images/turnLeft_hl.png")));
  //   } else if (state.w < -0.1) {
  //     ui->label_turnRight->setPixmap(
  //         QPixmap::fromImage(QImage("://images/turnRight_hl.png")));
  //   } else {
  //     ui->label_turnLeft->setPixmap(
  //         QPixmap::fromImage(QImage("://images/turnLeft_l.png")));
  //     ui->label_turnRight->setPixmap(
  //         QPixmap::fromImage(QImage("://images/turnRight_l.png")));
  //   }
  //   // 仪表盘
  speed_dash_board_->set_speed(abs(state.vx * 100));
  if (state.vx > 0.001) {
    speed_dash_board_->set_gear(DashBoard::kGear_D);
  } else if (state.vx < -0.001) {
    speed_dash_board_->set_gear(DashBoard::kGear_R);
  } else {
    speed_dash_board_->set_gear(DashBoard::kGear_N);
  }
  //   QString number = QString::number(abs(state.vx * 100)).mid(0, 2);
  //   if (number[1] == ".") {
  //     number = number.mid(0, 1);
  //   }
  //  ui->label_speed->setText(number);
  //  ui->mapViz->grab().save("/home/chengyangkj/test.jpg");
  //  QImage image(mysize,QImage::Format_RGB32);
  //           QPainter painter(&image);
  //           myscene->render(&painter);   //关键函数
}