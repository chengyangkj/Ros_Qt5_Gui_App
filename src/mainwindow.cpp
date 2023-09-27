#include "mainwindow.h"

#include "Eigen/Dense"
#include "basic/algorithm.h"
#include "common/logger/easylogging++.h"
#include <fstream>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  LOG(INFO) << "mainwindow init";
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<basic::RobotPose>("basic::RobotPose");
  qRegisterMetaType<OccupancyMap>("OccupancyMap");
  qRegisterMetaType<CostMap>("CostMap");
  qRegisterMetaType<LaserScan>("LaserScan");
  qRegisterMetaType<RobotPath>("RobotPath");
  ui->setupUi(this);
  connect(CommInstance::Instance(), SIGNAL(emitTopicData(QString)), this,
          SLOT(onRecvData(QString)));
  // 初始化场景类

  display_manager_ = new Display::DisplayManager(ui->mapViz);
  m_roboGLWidget = new roboGLWidget();
  ui->verticalLayout_status->addWidget(m_roboGLWidget);
  connect(CommInstance::Instance(), &VirtualCommNode::emitUpdateMap,
          [this](OccupancyMap map) {
            display_manager_->UpdateDisplay(DISPLAY_MAP, map);
          });
  connect(CommInstance::Instance(),
          SIGNAL(emitUpdateLocalCostMap(CostMap, basic::RobotPose)), this,
          SLOT(updateLocalCostMap(CostMap, basic::RobotPose)));
  connect(CommInstance::Instance(), SIGNAL(emitUpdateGlobalCostMap(CostMap)),
          this, SLOT(updateGlobalCostMap(CostMap)));
  connect(CommInstance::Instance(),
          SIGNAL(emitUpdateRobotPose(basic::RobotPose)), this,
          SLOT(slotUpdateRobotPose(basic::RobotPose)));

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
  connect(display_manager_, SIGNAL(signalPub2DPose(QPointF, QPointF)),
          CommInstance::Instance(), SLOT(pub2DPose(QPointF, QPointF)));
  connect(display_manager_, SIGNAL(signalPub2DGoal(QPointF, QPointF)),
          CommInstance::Instance(), SLOT(pub2DGoal(QPointF, QPointF)));
  // ui相关
  connect(ui->set_pos_btn, &QPushButton::clicked,
          [=]() { display_manager_->start2DPose(); });
  connect(ui->set_goal_btn, &QPushButton::clicked,
          [=]() { display_manager_->start2DGoal(); });
  connect(ui->close_btn, &QPushButton::clicked, [this]() { this->close(); });
  connect(ui->min_btn, &QPushButton::clicked,
          [this]() { this->showMinimized(); });
  connect(ui->max_btn, &QPushButton::clicked, [this]() {
    if (this->isFullScreen()) {
      this->showNormal();
    } else {
      this->showFullScreen();
    }
  });
  connect(ui->btn_dash, &QPushButton::clicked, [=]() {
    ui->stackedWidget_main->setCurrentIndex(0);
    setCurrentMenu(ui->btn_dash);
  });
  connect(ui->btn_control, &QPushButton::clicked,
          [=]() { ui->stackedWidget_left->setCurrentIndex(1); });
  connect(ui->btn_status, &QPushButton::clicked,
          [=]() { ui->stackedWidget_left->setCurrentIndex(0); });
  connect(ui->btn_map, &QPushButton::clicked, [=]() {
    ui->stackedWidget_main->setCurrentIndex(1);
    setCurrentMenu(ui->btn_map);
  });
  connect(ui->btn_other, &QPushButton::clicked, [=]() {
    ui->stackedWidget_main->setCurrentIndex(2);
    setCurrentMenu(ui->btn_other);
  });
  connect(ui->pushButton_status, &QPushButton::clicked,
          [=]() { ui->btn_other->click(); });
  connect(display_manager_, &Display::DisplayManager::cursorPosScene,
          [=](QPointF pos) {
            basic::Point mapPos =
                CommInstance::Instance()->transScenePoint2Word(
                    basic::Point(pos.x(), pos.y()));
            ui->label_pos_map->setText(
                "x: " + QString::number(mapPos.x).mid(0, 4) +
                "  y: " + QString::number(mapPos.y).mid(0, 4));
            ui->label_pos_scene->setText(
                "x: " + QString::number(pos.x()).mid(0, 4) +
                "  y: " + QString::number(pos.y()).mid(0, 4));
          });
  m_timerCurrentTime = new QTimer;
  m_timerCurrentTime->setInterval(100);
  m_timerCurrentTime->start();
  QObject::connect(m_timerCurrentTime, &QTimer::timeout, [=]() {
    ui->label_time->setText(
        QDateTime::currentDateTime().toString("  hh:mm:ss  "));
  });

  CommInstance::Instance()->start();
  initUi();
}
void MainWindow::updateLocalCostMap(CostMap map, basic::RobotPose pose) {
  display_manager_->UpdateDisplay(DISPLAY_LOCAL_COST_MAP, map);
  display_manager_->UpdateDisplay(DISPLAY_LOCAL_COST_MAP, pose);
}
void MainWindow::updateGlobalCostMap(CostMap map) {
  display_manager_->UpdateDisplay(DISPLAY_GLOBAL_COST_MAP, map);
}
void MainWindow::updateGlobalPath(RobotPath path) {
  Display::PathData data;
  for (auto one_point : path) {
    data.push_back(Display::Point2f(one_point.x, one_point.y));
  }
  display_manager_->UpdateDisplay(DISPLAY_GLOBAL_PATH, data);
}
void MainWindow::updateLocalPath(RobotPath path) {
  Display::PathData data;
  for (auto one_point : path) {
    data.push_back(Display::Point2f(one_point.x, one_point.y));
  }
  display_manager_->UpdateDisplay(DISPLAY_LOCAL_PATH, data);
}
void MainWindow::slotUpdateLaserPoint(LaserScan scan) {
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
void MainWindow::slotUpdateRobotPose(basic::RobotPose pose) {
  display_manager_->UpdateDisplay(DISPLAY_ROBOT,
                                  Display::Pose3f(pose.x, pose.y, pose.theta));
}
void MainWindow::updateOdomInfo(RobotState state) {
  // 转向灯
  if (state.w > 0.1) {
    ui->label_turnLeft->setPixmap(
        QPixmap::fromImage(QImage("://images/turnLeft_hl.png")));
  } else if (state.w < -0.1) {
    ui->label_turnRight->setPixmap(
        QPixmap::fromImage(QImage("://images/turnRight_hl.png")));
  } else {
    ui->label_turnLeft->setPixmap(
        QPixmap::fromImage(QImage("://images/turnLeft_l.png")));
    ui->label_turnRight->setPixmap(
        QPixmap::fromImage(QImage("://images/turnRight_l.png")));
  }
  // 仪表盘
  m_speedDashBoard->set_speed(abs(state.vx * 100));
  if (state.vx > 0.001) {
    m_speedDashBoard->set_gear(DashBoard::kGear_D);
  } else if (state.vx < -0.001) {
    m_speedDashBoard->set_gear(DashBoard::kGear_R);
  } else {
    m_speedDashBoard->set_gear(DashBoard::kGear_N);
  }
  QString number = QString::number(abs(state.vx * 100)).mid(0, 2);
  if (number[1] == ".") {
    number = number.mid(0, 1);
  }
  //  ui->label_speed->setText(number);
  //  ui->mapViz->grab().save("/home/chengyangkj/test.jpg");
  //  QImage image(mysize,QImage::Format_RGB32);
  //           QPainter painter(&image);
  //           myscene->render(&painter);   //关键函数
}

void MainWindow::setCurrentMenu(QPushButton *cur_btn) {
  for (int i = 0; i < ui->horizontalLayout_menu->layout()->count(); i++) {
    QPushButton *btn = qobject_cast<QPushButton *>(
        ui->horizontalLayout_menu->itemAt(i)->widget());
    if (btn == cur_btn) {
      cur_btn->setStyleSheet(
          " QPushButton{ background-color:rgb(67, 154, 246); border:none;  "
          "padding:0px 0px 0px 0px; margin:0px 0px 0px 0px;}");
    } else {
      btn->setStyleSheet(
          "QPushButton:hover{background-color:rgb(186, 189, 182); "
          "border-bottom:2px solid rgb(67, 154, 246);}"
          "QPushButton:checked{ background-color:cyan;border-bottom:2px solid "
          "white }"
          "QPushButton:pressed{background-color:rgb(67, 154, 246)}"
          " QPushButton{ background-color:rgb(238, 238, 236); border:none;  "
          "padding:0px 0px 0px 0px; margin:0px 0px 0px 0px; }");
    }
  }
}
void MainWindow::initUi() {
  setWindowFlags(Qt::CustomizeWindowHint); // 去掉标题栏
  ui->label_turnLeft->setPixmap(
      QPixmap::fromImage(QImage("://images/turnLeft_l.png")));
  ui->label_turnRight->setPixmap(
      QPixmap::fromImage(QImage("://images/turnRight_l.png")));
  ui->pushButton_status->setIcon(QIcon("://images/status/status_none.png"));
  ui->settings_btn->setIcon(QIcon("://images/toolbar_settings.png"));
  ui->min_btn->setIcon(QIcon("://images/min.png"));
  ui->max_btn->setIcon(QIcon("://images/max.png"));
  ui->close_btn->setIcon(QIcon("://images/close.png"));
  ui->btn_dash->setIcon(QIcon("://images/toolbar_dash.png"));
  ui->btn_map->setIcon(QIcon("://images/toolbar_map.png"));
  ui->btn_control->setIcon(QIcon("://images/control.png"));
  ui->btn_status->setIcon(QIcon("://images/status.png"));
  ui->btn_other->setIcon(QIcon("://images/toolbar_other.png"));
  m_speedDashBoard = new DashBoard(ui->widget_dashboard);
}
void MainWindow::closeEvent(QCloseEvent *event) {
  //  WriteSettings();
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::onRecvData(QString msg) {}
void MainWindow::mousePressEvent(QMouseEvent *event) {
  Q_UNUSED(event);
  if (event->button() == Qt::LeftButton) // 判断左键是否按下
  {
    left_pressed_ = true;
    left_pressed_point_ = event->pos();
    last_pressed_point_ = event->pos();
    curpos_ = countFlag(event->pos(), countRow(event->pos()));
  }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event) {
  Q_UNUSED(event);
  if (left_pressed_)
    left_pressed_ = false;
  setCursor(Qt::ArrowCursor);
  curpos_ = 0;
}

void MainWindow::mouseMoveEvent(QMouseEvent *event) {
  Q_UNUSED(event);
  if (this->isFullScreen())
    return; // 窗口铺满全屏，直接返回，不做任何操作
  // if (left_pressed_)
  //   move(event->pos() - left_pressed_point_ + pos());  // 移动当前窗口

  int poss = countFlag(event->pos(), countRow(event->pos()));
  setCursorType(poss);
  if (left_pressed_) {
    QPoint ptemp = event->pos() - left_pressed_point_;
    if (curpos_ == 22) // 按下窗体中心 移动窗口
    {
      ptemp = ptemp + pos();
      move(ptemp);
    } else {
      QRect wid = geometry();
      switch (curpos_) // 窗体四周 改变窗口的大小
      {
      case 11:
        wid.setTopLeft(wid.topLeft() + ptemp);
        break; // 左上角
      case 13:
        wid.setTopRight(wid.topRight() + ptemp);
        break; // 右上角
      case 31:
        wid.setBottomLeft(wid.bottomLeft() + ptemp);
        break; // 左下角
      case 33:
        wid.setBottomRight(wid.bottomRight() + ptemp);
        break; // 右下角
      case 12:
        wid.setTop(wid.top() + ptemp.y());
        break; // 中上角
      case 21:
        wid.setLeft(wid.left() + ptemp.x());
        break; // 中左角
      case 23:
        wid.setRight(wid.right() + ptemp.x());
        break; // 中右角
      case 32:
        wid.setBottom(wid.bottom() + ptemp.y());
        break; // 中下角
      }
      setGeometry(wid);
    }
    last_pressed_point_ = event->globalPos(); // 更新位置
  }
}
// 获取光标在窗口所在区域的 行   返回行数
int MainWindow::countRow(QPoint p) {
  return (p.x() < MARGIN) ? 1 : (p.x() > (this->width() - MARGIN) ? 3 : 2);
}
// 获取光标在窗口所在区域的 列  返回行列坐标
int MainWindow::countFlag(QPoint p, int row) {
  if (p.y() < MARGIN)
    return 10 + row;
  else if (p.y() > this->height() - MARGIN)
    return 30 + row;
  else
    return 20 + row;
}
void MainWindow::setCursorType(int flag) {
  switch (flag) {
  case 11:
  case 33:
    setCursor(Qt::SizeFDiagCursor);
    break;
  case 13:
  case 31:
    setCursor(Qt::SizeBDiagCursor);
    break;
  case 21:
  case 23:
    setCursor(Qt::SizeHorCursor);
    break;
  case 12:
  case 32:
    setCursor(Qt::SizeVerCursor);
    break;
  case 22:
    setCursor(Qt::ArrowCursor);
    QApplication::restoreOverrideCursor(); // 恢复鼠标指针性状
    break;
  }
}