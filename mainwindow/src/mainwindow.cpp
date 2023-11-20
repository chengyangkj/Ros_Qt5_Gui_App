/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-10-06 07:12:50
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-06 14:02:27
 * @FilePath: /ROS2_Qt5_Gui_App/src/ CMainWindow.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "widgets/speed_ctrl.h"

#include "algorithm.h"
#include "DockAreaTabBar.h"
#include "DockAreaTitleBar.h"
#include "DockAreaWidget.h"
#include "DockComponentsFactory.h"
#include "Eigen/Dense"
#include "FloatingDockContainer.h"
#include "logger/easylogging++.h"

using namespace ads;

CMainWindow::CMainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::CMainWindow) {
  LOG(INFO) << " CMainWindow init";
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<RobotPose>("RobotPose");
  qRegisterMetaType<RobotSpeed>("RobotSpeed");
  qRegisterMetaType<RobotState>("RobotState");
  qRegisterMetaType<OccupancyMap>("OccupancyMap");
  qRegisterMetaType<CostMap>("CostMap");
  qRegisterMetaType<LaserScan>("LaserScan");
  qRegisterMetaType<RobotPath>("RobotPath");
  setupUi();
  CommInstance::Instance()->start();
}

CMainWindow::~CMainWindow() { delete ui; }
void CMainWindow::setupUi() {
  ui->setupUi(this);
  CDockManager::setConfigFlag(CDockManager::OpaqueSplitterResize, true);
  CDockManager::setConfigFlag(CDockManager::XmlCompressionEnabled, false);
  CDockManager::setConfigFlag(CDockManager::FocusHighlighting, true);
  CDockManager::setConfigFlag(CDockManager::DockAreaHasUndockButton, false);
  CDockManager::setConfigFlag(CDockManager::DockAreaHasTabsMenuButton, false);
  CDockManager::setConfigFlag(CDockManager::MiddleMouseButtonClosesTab, true);
  CDockManager::setConfigFlag(CDockManager::MiddleMouseButtonClosesTab, true);
  dock_manager_ = new CDockManager(this);

  QVBoxLayout *central_layout = new QVBoxLayout();
  /////////////////////////////////////////////////////////////////状态栏
  status_bar_widget_ = new StatusBarWidget();
  central_layout->addWidget(status_bar_widget_);

  ///////////////////////////////////////////////////////////////地图工具栏
  tools_bar_widget_ = new ToolsBarWidget();
  central_layout->addWidget(tools_bar_widget_);
  /////////////////////////////////////////////////////////////////////////地图显示
  QGraphicsView *graphicsView = new QGraphicsView();
  display_manager_ = new Display::DisplayManager(graphicsView);
  central_layout->addWidget(graphicsView);

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
            CommInstance::Instance()->pubSpeed(speed);
          });
  ads::CDockWidget *SpeedCtrlDockWidget = new ads::CDockWidget("SpeedCtrl");
  SpeedCtrlDockWidget->setWidget(speed_ctrl_widget_);
  auto speed_ctrl_area =
      dock_manager_->addDockWidget(ads::DockWidgetArea::BottomDockWidgetArea,
                                   SpeedCtrlDockWidget, dashboard_area);
  ui->menuView->addAction(SpeedCtrlDockWidget->toggleViewAction());

  //////////////////////////////////////////////////////槽链接

  connect(CommInstance::Instance(), SIGNAL(emitTopicData(QString)), this,
          SLOT(onRecvData(QString)));
  connect(CommInstance::Instance(), &VirtualCommNode::emitUpdateMap,
          [this](OccupancyMap map) {
            display_manager_->UpdateDisplay(DISPLAY_MAP, map);
          });
  connect(CommInstance::Instance(),
          SIGNAL(emitUpdateLocalCostMap(CostMap, RobotPose)), this,
          SLOT(updateLocalCostMap(CostMap, RobotPose)));
  connect(CommInstance::Instance(), SIGNAL(emitUpdateGlobalCostMap(CostMap)),
          this, SLOT(updateGlobalCostMap(CostMap)));
  connect(CommInstance::Instance(), SIGNAL(emitUpdateRobotPose(RobotPose)),
          this, SLOT(slotUpdateRobotPose(RobotPose)));

  connect(CommInstance::Instance(), SIGNAL(emitUpdateLaserPoint(LaserScan)),
          this, SLOT(slotUpdateLaserPoint(LaserScan)));
  connect(CommInstance::Instance(), SIGNAL(emitUpdatePath(RobotPath)), this,
          SLOT(updateGlobalPath(RobotPath)));
  connect(CommInstance::Instance(), SIGNAL(emitUpdateLocalPath(RobotPath)),
          this, SLOT(updateLocalPath(RobotPath)));
  connect(CommInstance::Instance(), SIGNAL(emitOdomInfo(RobotState)), this,
          SLOT(updateOdomInfo(RobotState)));
  // connect(m_roboItem, SIGNAL(signalRunMap(OccupancyMap)), m_roboGLWidget,
  //         SLOT(updateRunMap(QPixmap)));
  //    connect(CommInstance::Instance(),&rclcomm::emitUpdateMap,[this](QImage
  //    img){
  //        m_roboItem->updateMap(img);
  //    });
  connect(display_manager_, SIGNAL(signalPub2DPose(const RobotPose &)),
          CommInstance::Instance(), SLOT(pub2DPose(const RobotPose &)));
  connect(display_manager_, SIGNAL(signalPub2DGoal(const RobotPose &)),
          CommInstance::Instance(), SLOT(pub2DGoal(const RobotPose &)));
  // ui相关
  connect(tools_bar_widget_, &ToolsBarWidget::SignalSetRelocPose,
          [=]() { display_manager_->SetRelocPose(); });
  connect(tools_bar_widget_, &ToolsBarWidget::SignalSetNavPose,
          [=]() { display_manager_->SetNavPose(); });
  connect(display_manager_->GetDisplay(DISPLAY_MAP),
          SIGNAL(signalCursorPose(QPointF)), this,
          SLOT(signalCursorPose(QPointF)));
}

void CMainWindow::signalCursorPose(QPointF pos) {
  basic::Point mapPos = CommInstance::Instance()->transScenePoint2Word(
      basic::Point(pos.x(), pos.y()));
  label_pos_map_->setText("x: " + QString::number(mapPos.x).mid(0, 4) +
                          "  y: " + QString::number(mapPos.y).mid(0, 4));
  label_pos_scene_->setText("x: " + QString::number(pos.x()).mid(0, 4) +
                            "  y: " + QString::number(pos.y()).mid(0, 4));
}
void CMainWindow::savePerspective() {
  QString PerspectiveName =
      QInputDialog::getText(this, "Save Perspective", "Enter unique name:");
  if (PerspectiveName.isEmpty()) {
    return;
  }

  dock_manager_->addPerspective(PerspectiveName);
  QSignalBlocker Blocker(PerspectiveComboBox);
  PerspectiveComboBox->clear();
  PerspectiveComboBox->addItems(dock_manager_->perspectiveNames());
  PerspectiveComboBox->setCurrentText(PerspectiveName);
}

//============================================================================
void CMainWindow::closeEvent(QCloseEvent *event) {
  // Delete dock manager here to delete all floating widgets. This ensures
  // that all top level windows of the dock manager are properly closed
  dock_manager_->deleteLater();
  QMainWindow::closeEvent(event);
}
void CMainWindow::onRecvData(QString msg) {}
void CMainWindow::updateLocalCostMap(CostMap map, basic::RobotPose pose) {
  display_manager_->UpdateDisplay(DISPLAY_LOCAL_COST_MAP, map);
  display_manager_->UpdateDisplay(DISPLAY_LOCAL_COST_MAP, pose);
}
void CMainWindow::updateGlobalCostMap(CostMap map) {
  display_manager_->UpdateDisplay(DISPLAY_GLOBAL_COST_MAP, map);
}
void CMainWindow::updateGlobalPath(RobotPath path) {
  Display::PathData data;
  for (auto one_point : path) {
    data.push_back(Display::Point2f(one_point.x, one_point.y));
  }
  display_manager_->UpdateDisplay(DISPLAY_GLOBAL_PATH, data);
}
void CMainWindow::updateLocalPath(RobotPath path) {
  Display::PathData data;
  for (auto one_point : path) {
    data.push_back(Display::Point2f(one_point.x, one_point.y));
  }
  display_manager_->UpdateDisplay(DISPLAY_LOCAL_PATH, data);
}
void CMainWindow::slotUpdateLaserPoint(LaserScan scan) {
  // 数据转换
  Display::LaserDataMap laser_data;

  Display::LaserData vector_scan;
  for (auto one_point : scan.data) {
    Eigen::Vector2f point;
    point[0] = one_point.x;
    point[1] = one_point.y;
    vector_scan.push_back(point);
  }
  laser_data[scan.id] = vector_scan;

  display_manager_->UpdateDisplay(DISPLAY_LASER, laser_data);
}
void CMainWindow::slotUpdateRobotPose(basic::RobotPose pose) {
  display_manager_->UpdateDisplay(DISPLAY_ROBOT,
                                  Display::Pose3f(pose.x, pose.y, pose.theta));
}

void CMainWindow::updateOdomInfo(RobotState state) {
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