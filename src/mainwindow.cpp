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

#include "DockAreaTabBar.h"
#include "DockAreaTitleBar.h"
#include "DockAreaWidget.h"
#include "DockComponentsFactory.h"
#include "Eigen/Dense"
#include "FloatingDockContainer.h"
#include "basic/algorithm.h"
#include "common/logger/easylogging++.h"

using namespace ads;

CMainWindow::CMainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::CMainWindow) {
  LOG(INFO) << " CMainWindow init";
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<RobotPose>("RobotPose");
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
  QHBoxLayout *horizontalLayout_status = new QHBoxLayout();
  horizontalLayout_status->setObjectName(
      QString::fromUtf8(" horizontalLayout_status"));
  QLabel *label_time = new QLabel();
  label_time->setObjectName(QString::fromUtf8("label_time"));
  label_time->setMinimumSize(QSize(60, 0));
  QFont font1;
  font1.setPointSize(13);
  label_time->setFont(font1);

  horizontalLayout_status->addWidget(label_time);

  QLabel *label_19 = new QLabel();
  label_19->setObjectName(QString::fromUtf8("label_19"));
  label_19->setMinimumSize(QSize(32, 32));
  label_19->setMaximumSize(QSize(32, 32));
  label_19->setPixmap(QPixmap(QString::fromUtf8(":/images/robot2.png")));

  horizontalLayout_status->addWidget(label_19);

  QLabel *label_18 = new QLabel();
  label_18->setObjectName(QString::fromUtf8("label_18"));
  label_18->setStyleSheet(QString::fromUtf8(""));
  label_18->setText("状态:");
  horizontalLayout_status->addWidget(label_18);

  QPushButton *btn_status = new QPushButton();
  btn_status->setObjectName(QString::fromUtf8("btn_status"));
  btn_status->setMinimumSize(QSize(20, 20));
  btn_status->setMaximumSize(QSize(20, 20));
  btn_status->setCursor(QCursor(Qt::PointingHandCursor));
  btn_status->setStyleSheet(QString::fromUtf8("QPushButton{\n"
                                              "border:none; \n"
                                              "padding:0px 0px 0px 0px;\n"
                                              "margin:0px 0px 0px 0px;\n"
                                              "}"));

  horizontalLayout_status->addWidget(btn_status);

  QSpacerItem *horizontalSpacer =
      new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

  horizontalLayout_status->addItem(horizontalSpacer);

  QLabel *label_10 = new QLabel();
  label_10->setObjectName(QString::fromUtf8("label_10"));
  label_10->setStyleSheet(QString::fromUtf8(""));

  horizontalLayout_status->addWidget(label_10);

  QLabel *label_6 = new QLabel();
  label_6->setObjectName(QString::fromUtf8("label_6"));
  label_6->setMaximumSize(QSize(30, 30));
  label_6->setPixmap(QPixmap(QString::fromUtf8(":/images/power.png")));

  horizontalLayout_status->addWidget(label_6);

  battery_bar_ = new QProgressBar();
  battery_bar_->setObjectName(QString::fromUtf8("battery_bar_"));
  battery_bar_->setMaximumSize(QSize(90, 16777215));
  battery_bar_->setAutoFillBackground(true);
  battery_bar_->setStyleSheet(QString::fromUtf8(
      "QProgressBar#battery_bar_\n"
      "{\n"
      "      border:none;   /*\346\227\240\350\276\271\346\241\206*/\n"
      "      background:rgb(211, 215, 207);\n"
      "      border-radius:5px;\n"
      "      text-align:center;   "
      "/*\346\226\207\346\234\254\347\232\204\344\275\215\347\275\256*/\n"
      "      color: rgb(229, 229, 229);  "
      "/*\346\226\207\346\234\254\351\242\234\350\211\262*/\n"
      "}\n"
      " \n"
      "QProgressBar::chunk \n"
      "{\n"
      "      background-color:rgb(115, 210, 22);\n"
      "      border-radius:4px;\n"
      "}\n"
      ""));
  battery_bar_->setValue(70);
  battery_bar_->setAlignment(Qt::AlignBottom | Qt::AlignHCenter);

  horizontalLayout_status->addWidget(battery_bar_);

  QLabel *label_11 = new QLabel();
  label_11->setObjectName(QString::fromUtf8("label_11"));
  label_11->setMinimumSize(QSize(32, 32));
  label_11->setMaximumSize(QSize(32, 32));
  label_11->setPixmap(QPixmap(QString::fromUtf8(":/images/power-v.png")));

  horizontalLayout_status->addWidget(label_11);

  label_power_ = new QLabel();
  label_power_->setObjectName(QString::fromUtf8("label_power_"));
  label_power_->setMinimumSize(QSize(50, 32));
  label_power_->setMaximumSize(QSize(50, 32));
  label_power_->setStyleSheet(QString::fromUtf8(""));
  label_power_->setText("0.00V");
  horizontalLayout_status->addWidget(label_power_);
  central_layout->addLayout(horizontalLayout_status);
  ///////////////////////////////////////////////////////////////地图工具栏
  QHBoxLayout *horizontalLayout_tools = new QHBoxLayout();
  horizontalLayout_tools->setSpacing(0);
  horizontalLayout_tools->setObjectName(
      QString::fromUtf8(" horizontalLayout_tools"));

  QPushButton *set_pos_btn = new QPushButton();
  set_pos_btn->setObjectName(QString::fromUtf8(" set_pos_btn"));
  set_pos_btn->setMinimumSize(QSize(0, 25));
  set_pos_btn->setMaximumSize(QSize(16777215, 16777215));
  set_pos_btn->setStyleSheet(
      QString::fromUtf8("QPushButton:hover{\n"
                        "background-color:rgb(186, 189, 182);\n"
                        "border-bottom:2px solid rgb(67, 154, 246);\n"
                        "}\n"
                        "QPushButton:checked{\n"
                        "background-color:cyan;\n"
                        "border-bottom:2px solid white \n"
                        "}\n"
                        "QPushButton:pressed{\n"
                        "background-color:rgb(67, 154, 246)\n"
                        "}\n"
                        "QPushButton{\n"
                        "background-color:rgb(238, 238, 236);\n"
                        "border:none; \n"
                        "padding:0px 0px 0px 0px;\n"
                        "margin:0px 0px 0px 0px;\n"
                        "}"));
  QIcon icon4;
  icon4.addFile(QString::fromUtf8(":/images/classes/SetInitialPose.png"),
                QSize(), QIcon::Normal, QIcon::Off);
  set_pos_btn->setIcon(icon4);
  set_pos_btn->setText("重定位");
  horizontalLayout_tools->addWidget(set_pos_btn);

  QPushButton *set_goal_btn = new QPushButton();
  set_goal_btn->setText("set goal");
  set_goal_btn->setObjectName(QString::fromUtf8(" set_goal_btn"));
  set_goal_btn->setMinimumSize(QSize(0, 25));
  set_goal_btn->setStyleSheet(
      QString::fromUtf8("QPushButton:hover{\n"
                        "background-color:rgb(186, 189, 182);\n"
                        "border-bottom:2px solid rgb(67, 154, 246);\n"
                        "}\n"
                        "QPushButton:checked{\n"
                        "background-color:cyan;\n"
                        "border-bottom:2px solid white \n"
                        "}\n"
                        "QPushButton:pressed{\n"
                        "background-color:rgb(67, 154, 246)\n"
                        "}\n"
                        "QPushButton{\n"
                        "background-color:rgb(238, 238, 236);\n"
                        "border:none; \n"
                        "padding:0px 0px 0px 0px;\n"
                        "margin:0px 0px 0px 0px;\n"
                        "}"));
  QIcon icon5;
  icon5.addFile(QString::fromUtf8("://images/mutil_pose.png"), QSize(),
                QIcon::Normal, QIcon::Off);
  set_goal_btn->setIcon(icon5);

  horizontalLayout_tools->addWidget(set_goal_btn);

  QPushButton *set_mutil_goal_btn = new QPushButton();
  set_mutil_goal_btn->setObjectName(QString::fromUtf8(" set_mutil_goal_btn"));
  set_mutil_goal_btn->setText("Focus Robot");
  set_mutil_goal_btn->setStyleSheet(
      QString::fromUtf8("QPushButton:hover{\n"
                        "background-color:rgb(186, 189, 182);\n"
                        "border-bottom:2px solid rgb(67, 154, 246);\n"
                        "}\n"
                        "QPushButton:checked{\n"
                        "background-color:cyan;\n"
                        "border-bottom:2px solid white \n"
                        "}\n"
                        "QPushButton:pressed{\n"
                        "background-color:rgb(67, 154, 246)\n"
                        "}\n"
                        "QPushButton{\n"
                        "background-color:rgb(238, 238, 236);\n"
                        "border:none; \n"
                        "padding:0px 0px 0px 0px;\n"
                        "margin:0px 0px 0px 0px;\n"
                        "}"));
  QIcon icon6;
  icon6.addFile(QString::fromUtf8("://images/mutil_pose.png"), QSize(),
                QIcon::Normal, QIcon::Off);
  set_mutil_goal_btn->setIcon(icon6);

  horizontalLayout_tools->addWidget(set_mutil_goal_btn);
  central_layout->addLayout(horizontalLayout_tools);
  /////////////////////////////////////////////////////////////////////////地图显示
  QGraphicsView *graphicsView = new QGraphicsView();
  display_manager_ = new Display::DisplayManager(graphicsView);
  central_layout->addWidget(graphicsView);
  // Set central widget

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
  QVBoxLayout *verticalLayout_speed_ctrl = new QVBoxLayout();
  verticalLayout_speed_ctrl->setObjectName(
      QString::fromUtf8("verticalLayout_speed_ctrl"));
  QVBoxLayout *verticalLayout_cmd_btn = new QVBoxLayout();
  QHBoxLayout *horizontalLayout_2 = new QHBoxLayout();
  horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
  QPushButton *pushButton_u = new QPushButton();
  pushButton_u->setObjectName(QString::fromUtf8("pushButton_u"));
  pushButton_u->setText("u");
  pushButton_u->setShortcut(QApplication::translate("Widget", "u", nullptr));
  pushButton_u->setMinimumSize(QSize(64, 64));
  pushButton_u->setMaximumSize(QSize(64, 64));
  pushButton_u->setStyleSheet(QString::fromUtf8(
      "QPushButton{border-image: url(://images/up_left.png);}\n"
      "QPushButton{border:none;}\n"
      "QPushButton:pressed{border-image: url(://images/up_left_2.png);}"));

  horizontalLayout_2->addWidget(pushButton_u);

  QPushButton *pushButton_i = new QPushButton();
  pushButton_i->setObjectName(QString::fromUtf8("pushButton_i"));
  pushButton_i->setText("i");
  pushButton_i->setShortcut(QApplication::translate("Widget", "i", nullptr));
  pushButton_i->setMinimumSize(QSize(64, 64));
  pushButton_i->setMaximumSize(QSize(64, 64));
  pushButton_i->setStyleSheet(QString::fromUtf8(
      "QPushButton{border-image: url(://images/up.png);}\n"
      "QPushButton{border:none;}\n"
      "QPushButton:pressed{border-image: url(://images/up_2.png);}"));

  horizontalLayout_2->addWidget(pushButton_i);

  QPushButton *pushButton_o = new QPushButton();
  pushButton_o->setObjectName(QString::fromUtf8("pushButton_o"));
  pushButton_o->setText("o");
  pushButton_o->setShortcut(QApplication::translate("Widget", "o", nullptr));
  pushButton_o->setMinimumSize(QSize(64, 64));
  pushButton_o->setMaximumSize(QSize(64, 64));
  pushButton_o->setStyleSheet(QString::fromUtf8(
      "QPushButton{border-image: url(://images/up_right.png);}\n"
      "QPushButton{border:none;}\n"
      "QPushButton:pressed{border-image: url(://images/up_right_2.png);}"));

  horizontalLayout_2->addWidget(pushButton_o);

  verticalLayout_cmd_btn->addLayout(horizontalLayout_2);

  QHBoxLayout *horizontalLayout_18 = new QHBoxLayout();
  horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
  QPushButton *pushButton_j = new QPushButton();
  pushButton_j->setText("j");
  pushButton_j->setShortcut(QApplication::translate("Widget", "j", nullptr));
  pushButton_j->setObjectName(QString::fromUtf8("pushButton_j"));
  pushButton_j->setMinimumSize(QSize(64, 64));
  pushButton_j->setMaximumSize(QSize(64, 64));
  pushButton_j->setStyleSheet(QString::fromUtf8(
      "QPushButton{border-image: url(://images/left.png);}\n"
      "QPushButton{border:none;}\n"
      "QPushButton:pressed{border-image: url(://images/left_2.png);}"));

  horizontalLayout_18->addWidget(pushButton_j);

  checkBox_use_all_ = new QCheckBox();
  checkBox_use_all_->setObjectName(QString::fromUtf8("checkBox_use_all_"));
  checkBox_use_all_->setMaximumSize(QSize(90, 16777215));
  checkBox_use_all_->setText("全向模式:");
  horizontalLayout_18->addWidget(checkBox_use_all_);

  QPushButton *pushButton_l = new QPushButton();
  pushButton_l->setObjectName(QString::fromUtf8("pushButton_l"));
  pushButton_l->setText("l");
  pushButton_l->setShortcut(QApplication::translate("Widget", "l", nullptr));
  pushButton_l->setMinimumSize(QSize(64, 64));
  pushButton_l->setMaximumSize(QSize(64, 64));
  pushButton_l->setStyleSheet(QString::fromUtf8(
      "QPushButton{border-image: url(://images/right.png);}\n"
      "QPushButton{border:none;}\n"
      "QPushButton:pressed{border-image: url(://images/right_2.png);}"));

  horizontalLayout_18->addWidget(pushButton_l);

  verticalLayout_cmd_btn->addLayout(horizontalLayout_18);

  QHBoxLayout *horizontalLayout_19 = new QHBoxLayout();
  horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
  QPushButton *pushButton_m = new QPushButton();
  pushButton_m->setObjectName(QString::fromUtf8("pushButton_m"));
  pushButton_m->setText("m");
  pushButton_m->setShortcut(QApplication::translate("Widget", "m", nullptr));
  pushButton_m->setMinimumSize(QSize(64, 64));
  pushButton_m->setMaximumSize(QSize(64, 64));
  pushButton_m->setStyleSheet(QString::fromUtf8(
      "QPushButton{border-image: url(://images/down_left.png);}\n"
      "QPushButton{border:none;}\n"
      "QPushButton:pressed{border-image: url(://images/down_left_2.png);}"));

  horizontalLayout_19->addWidget(pushButton_m);

  QPushButton *pushButton_back = new QPushButton();
  pushButton_back->setObjectName(QString::fromUtf8("pushButton_,"));
  pushButton_back->setText(",");
  pushButton_back->setShortcut(QApplication::translate("Widget", ",", nullptr));
  pushButton_back->setMinimumSize(QSize(64, 64));
  pushButton_back->setMaximumSize(QSize(64, 64));
  pushButton_back->setStyleSheet(QString::fromUtf8(
      "QPushButton{border-image: url(://images/down.png);}\n"
      "QPushButton{border:none;}\n"
      "QPushButton:pressed{border-image: url(://images/down_2.png);}"));

  horizontalLayout_19->addWidget(pushButton_back);

  QPushButton *pushButton_backr = new QPushButton();
  pushButton_backr->setObjectName(QString::fromUtf8("pushButton_."));
  pushButton_backr->setText(".");
  pushButton_backr->setShortcut(
      QApplication::translate("Widget", ".", nullptr));
  pushButton_backr->setMinimumSize(QSize(64, 64));
  pushButton_backr->setMaximumSize(QSize(64, 64));
  pushButton_backr->setStyleSheet(QString::fromUtf8(
      "QPushButton{border-image: url(://images/down_right.png);}\n"
      "QPushButton{border:none;}\n"
      "QPushButton:pressed{border-image: url(://images/down_right_2.png);}"));

  connect(pushButton_i, SIGNAL(clicked()), this, SLOT(slotSpeedControl()));
  connect(pushButton_u, SIGNAL(clicked()), this, SLOT(slotSpeedControl()));
  connect(pushButton_o, SIGNAL(clicked()), this, SLOT(slotSpeedControl()));
  connect(pushButton_j, SIGNAL(clicked()), this, SLOT(slotSpeedControl()));
  connect(pushButton_l, SIGNAL(clicked()), this, SLOT(slotSpeedControl()));
  connect(pushButton_m, SIGNAL(clicked()), this, SLOT(slotSpeedControl()));
  connect(pushButton_back, SIGNAL(clicked()), this, SLOT(slotSpeedControl()));
  connect(pushButton_backr, SIGNAL(clicked()), this, SLOT(slotSpeedControl()));

  horizontalLayout_19->addWidget(pushButton_backr);

  verticalLayout_cmd_btn->addLayout(horizontalLayout_19);

  QWidget *cmdCtrlWidget = new QWidget();
  cmdCtrlWidget->setLayout(verticalLayout_cmd_btn);

  QTabWidget *tabWidget = new QTabWidget;

  tabWidget->addTab(cmdCtrlWidget, "CmdCtrl");
  verticalLayout_speed_ctrl->addWidget(tabWidget);
  //////////////////////////////////////////////////////遥杆控制
  QWidget *widget_joyStick = new QWidget();
  QHBoxLayout *horizontalLayout_joyStick = new QHBoxLayout();
  joyStick_widget_ = new JoyStick();
  joyStick_widget_->setMinimumSize(QSize(200, 200));

  connect(joyStick_widget_, SIGNAL(keyNumchanged(int)), this,
          SLOT(slotJoyStickKeyChange(int)));

  horizontalLayout_joyStick->addStretch();
  horizontalLayout_joyStick->addWidget(joyStick_widget_);
  horizontalLayout_joyStick->addStretch();
  widget_joyStick->setLayout(horizontalLayout_joyStick);

  tabWidget->addTab(widget_joyStick, "JoyStickCtrl");

  QHBoxLayout *horizontalLayout_20 = new QHBoxLayout();
  horizontalLayout_20->setObjectName(QString::fromUtf8("horizontalLayout_20"));
  QLabel *label_14 = new QLabel();
  label_14->setObjectName(QString::fromUtf8("label_14"));
  label_14->setText("角速度:");
  horizontalLayout_20->addWidget(label_14);

  horizontalSlider_raw_ = new QSlider();
  horizontalSlider_raw_->setObjectName(
      QString::fromUtf8("horizontalSlider_raw_"));
  horizontalSlider_raw_->setMaximum(100);
  horizontalSlider_raw_->setValue(10);
  horizontalSlider_raw_->setOrientation(Qt::Horizontal);

  horizontalLayout_20->addWidget(horizontalSlider_raw_);

  QLabel *label_raw = new QLabel();
  label_raw->setObjectName(QString::fromUtf8("label_raw"));
  label_raw->setText(QString::number(horizontalSlider_raw_->value() * 0.01) +
                     " deg/s");
  connect(
      horizontalSlider_raw_, &QSlider::valueChanged, [label_raw](qreal value) {
        label_raw->setText(QString::number(rad2deg(value * 0.01)) + " deg/s");
      });
  horizontalLayout_20->addWidget(label_raw);

  verticalLayout_speed_ctrl->addLayout(horizontalLayout_20);

  // linear anglur
  QHBoxLayout *horizontalLayout_21 = new QHBoxLayout();
  horizontalLayout_21->setObjectName(QString::fromUtf8("horizontalLayout_21"));
  QLabel *label_9 = new QLabel();
  label_9->setObjectName(QString::fromUtf8("label_9"));
  label_9->setText("线速度:");
  horizontalLayout_21->addWidget(label_9);

  horizontalSlider_linear_ = new QSlider();
  horizontalSlider_linear_->setObjectName(
      QString::fromUtf8("horizontalSlider_linear_"));
  horizontalSlider_linear_->setMaximum(100);
  horizontalSlider_linear_->setSingleStep(1);
  horizontalSlider_linear_->setValue(10);
  horizontalSlider_linear_->setOrientation(Qt::Horizontal);

  horizontalLayout_21->addWidget(horizontalSlider_linear_);

  QLabel *label_linear = new QLabel();
  label_linear->setObjectName(QString::fromUtf8("label_linear"));
  label_linear->setText(
      QString::number(horizontalSlider_linear_->value() * 0.01) + " m/s");
  connect(horizontalSlider_linear_, &QSlider::valueChanged,
          [label_linear](qreal value) {
            label_linear->setText(QString::number(value * 0.01) + " m/s");
          });
  horizontalLayout_21->addWidget(label_linear);
  verticalLayout_speed_ctrl->addLayout(horizontalLayout_21);
  QHBoxLayout *horizontalLayout_stop_button = new QHBoxLayout();
  QPushButton *btn_stop = new QPushButton();
  btn_stop->setObjectName(QString::fromUtf8("btn_stop"));
  btn_stop->setText("停止(s)");
  btn_stop->setShortcut(QApplication::translate("Widget", "s", nullptr));
  connect(btn_stop, &QPushButton::clicked, [this]() {
    CommInstance::Instance()->pubSpeed(Eigen::Vector3f(0, 0, 0));
  });
  horizontalLayout_stop_button->addStretch();
  horizontalLayout_stop_button->addWidget(btn_stop);
  horizontalLayout_stop_button->addStretch();
  verticalLayout_speed_ctrl->addLayout(horizontalLayout_stop_button);

  QHBoxLayout *horizontalLayout_23 = new QHBoxLayout();
  horizontalLayout_23->setObjectName(QString::fromUtf8("horizontalLayout_23"));
  QSpacerItem *horizontalSpacer_5 =
      new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

  horizontalLayout_23->addItem(horizontalSpacer_5);

  QSpacerItem *horizontalSpacer_6 =
      new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

  horizontalLayout_23->addItem(horizontalSpacer_6);

  verticalLayout_speed_ctrl->addLayout(horizontalLayout_23);

  QSpacerItem *verticalSpacer_4 =
      new QSpacerItem(385, 21, QSizePolicy::Minimum, QSizePolicy::Expanding);

  verticalLayout_speed_ctrl->addItem(verticalSpacer_4);

  ads::CDockWidget *SpeedCtrlDockWidget = new ads::CDockWidget("SpeedCtrl");
  QWidget *speed_ctrl_widget = new QWidget();
  speed_ctrl_widget->setLayout(verticalLayout_speed_ctrl);
  SpeedCtrlDockWidget->setWidget(speed_ctrl_widget);
  auto speed_ctrl_area =
      dock_manager_->addDockWidget(ads::DockWidgetArea::BottomDockWidgetArea,
                                   SpeedCtrlDockWidget, dashboard_area);
  ui->menuView->addAction(SpeedCtrlDockWidget->toggleViewAction());

  //////////////////////////////////////////////////////槽链接
  QTimer *timer_current_time = new QTimer;
  timer_current_time->setInterval(100);
  timer_current_time->start();
  QObject::connect(timer_current_time, &QTimer::timeout, [=]() {
    label_time->setText(QDateTime::currentDateTime().toString("  hh:mm:ss  "));
  });

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
  connect(display_manager_, SIGNAL(signalPub2DPose(Eigen::Vector3f)),
          CommInstance::Instance(), SLOT(pub2DPose(Eigen::Vector3f)));
  connect(display_manager_, SIGNAL(signalPub2DGoal(Eigen::Vector3f)),
          CommInstance::Instance(), SLOT(pub2DGoal(Eigen::Vector3f)));
  // ui相关
  connect(set_pos_btn, &QPushButton::clicked, [=]() {
    if (set_pos_btn->text() == "重定位") {
      display_manager_->start2DPose(true);
      set_pos_btn->setText("发送定位坐标");
    } else {
      display_manager_->start2DPose(false);
      set_pos_btn->setText("重定位");
    }
  });
  connect(set_goal_btn, &QPushButton::clicked, [=]() {
    if (set_goal_btn->text() == "set goal") {
      display_manager_->start2DGoal(true);
      set_goal_btn->setText("send goal");
    } else {
      display_manager_->start2DGoal(false);
      set_goal_btn->setText("set goal");
    }
  });
  connect(display_manager_->GetDisplay(DISPLAY_MAP),
          SIGNAL(signalCursorPose(QPointF)), this,
          SLOT(signalCursorPose(QPointF)));
}
void CMainWindow::slotJoyStickKeyChange(int value) {
  //速度
  float liner = horizontalSlider_linear_->value() * 0.01;
  float turn = horizontalSlider_raw_->value() * 0.01;
  bool is_all = checkBox_use_all_->isChecked();
  char key;
  std::cout << "joy stic value:" << value << std::endl;
  switch (value) {
  case JoyStick::Direction::upleft:
    key = is_all ? 'U' : 'u';
    break;
  case JoyStick::Direction::up:
    key = is_all ? 'I' : 'i';
    break;
  case JoyStick::Direction::upright:
    key = is_all ? 'O' : 'o';
    break;
  case JoyStick::Direction::left:
    key = is_all ? 'J' : 'j';
    break;
  case JoyStick::Direction::right:
    key = is_all ? 'L' : 'l';
    break;
  case JoyStick::Direction::down:
    key = is_all ? 'M' : 'm';
    break;
  case JoyStick::Direction::downleft:
    key = is_all ? '<' : ',';
    break;
  case JoyStick::Direction::downright:
    key = is_all ? '>' : '.';
    break;
  default:
    return;
  }
  std::map<char, std::vector<float>> moveBindings{
      {'i', {1, 0, 0, 0}},  {'o', {1, 0, 0, -1}},  {'j', {0, 0, 0, 1}},
      {'l', {0, 0, 0, -1}}, {'u', {1, 0, 0, 1}},   {',', {-1, 0, 0, 0}},
      {'.', {-1, 0, 0, 1}}, {'m', {-1, 0, 0, -1}}, {'O', {1, -1, 0, 0}},
      {'I', {1, 0, 0, 0}},  {'J', {0, 1, 0, 0}},   {'L', {0, -1, 0, 0}},
      {'U', {1, 1, 0, 0}},  {'<', {-1, 0, 0, 0}},  {'>', {-1, -1, 0, 0}},
      {'M', {-1, 1, 0, 0}}, {'t', {0, 0, 1, 0}},   {'b', {0, 0, -1, 0}},
      {'k', {0, 0, 0, 0}},  {'K', {0, 0, 0, 0}}};
  //计算是往哪个方向
  float x = moveBindings[key][0];
  float y = moveBindings[key][1];
  float z = moveBindings[key][2];
  float th = moveBindings[key][3];
  CommInstance::Instance()->pubSpeed(
      Eigen::Vector3f(x * liner, y * liner, th * turn));
}
void CMainWindow::slotSpeedControl() {
  QPushButton *btn = qobject_cast<QPushButton *>(sender());
  char button_key = btn->text().toStdString()[0];
  //速度
  float liner = horizontalSlider_linear_->value() * 0.01;
  float turn = horizontalSlider_raw_->value() * 0.01;
  bool is_all = checkBox_use_all_->isChecked();
  char key;

  switch (button_key) {
  case 'u':
    key = is_all ? 'U' : 'u';
    break;
  case 'i':
    key = is_all ? 'I' : 'i';
    break;
  case 'o':
    key = is_all ? 'O' : 'o';
    break;
  case 'j':
    key = is_all ? 'J' : 'j';
    break;
  case 'l':
    key = is_all ? 'L' : 'l';
    break;
  case 'm':
    key = is_all ? 'M' : 'm';
    break;
  case ',':
    key = is_all ? '<' : ',';
    break;
  case '.':
    key = is_all ? '>' : '.';
    break;
  default:
    return;
  }

  std::map<char, std::vector<float>> moveBindings{
      {'i', {1, 0, 0, 0}},  {'o', {1, 0, 0, -1}},  {'j', {0, 0, 0, 1}},
      {'l', {0, 0, 0, -1}}, {'u', {1, 0, 0, 1}},   {',', {-1, 0, 0, 0}},
      {'.', {-1, 0, 0, 1}}, {'m', {-1, 0, 0, -1}}, {'O', {1, -1, 0, 0}},
      {'I', {1, 0, 0, 0}},  {'J', {0, 1, 0, 0}},   {'L', {0, -1, 0, 0}},
      {'U', {1, 1, 0, 0}},  {'<', {-1, 0, 0, 0}},  {'>', {-1, -1, 0, 0}},
      {'M', {-1, 1, 0, 0}}, {'t', {0, 0, 1, 0}},   {'b', {0, 0, -1, 0}},
      {'k', {0, 0, 0, 0}},  {'K', {0, 0, 0, 0}}};
  //计算是往哪个方向
  std::cout << "key:" << key << std::endl;
  float x = moveBindings[key][0];
  float y = moveBindings[key][1];
  float z = moveBindings[key][2];
  float th = moveBindings[key][3];
  CommInstance::Instance()->pubSpeed(
      Eigen::Vector3f(x * liner, y * liner, th * turn));
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