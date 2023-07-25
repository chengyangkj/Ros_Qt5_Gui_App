#include "mainwindow.h"

#include "RobotAlgorithm.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  ui->setupUi(this);
  commNode = new rclcomm();
  connect(commNode, SIGNAL(emitTopicData(QString)), this,
          SLOT(onRecvData(QString)));
  // 初始化场景类
  m_qGraphicScene = new QGraphicsScene();
  m_qGraphicScene->clear();
  // 初始化item
  m_roboItem = new roboItem();
  m_roboImg = new roboImg();

  display_manager_ = new DisplayManager(ui->mapViz);
  m_roboGLWidget = new roboGLWidget();
  ui->verticalLayout_status->addWidget(m_roboGLWidget);
  connect(commNode, SIGNAL(emitUpdateMap(QImage)), m_roboItem,
          SLOT(updateMap(QImage)));
  connect(commNode, SIGNAL(emitUpdateLocalCostMap(QImage, RobotPose)),
          m_roboItem, SLOT(updateLocalCostMap(QImage, RobotPose)));
  connect(commNode, SIGNAL(emitUpdateGlobalCostMap(QImage)), m_roboItem,
          SLOT(updateGlobalCostMap(QImage)));
  connect(commNode, SIGNAL(emitUpdateRobotPose(RobotPose)), this,
          SLOT(updateRobotPose(RobotPose)));
  connect(commNode, SIGNAL(emitUpdateLaserPoint(QPolygonF)), m_roboItem,
          SLOT(updateLaserPoints(QPolygonF)));
  connect(commNode, SIGNAL(emitUpdatePath(QPolygonF)), m_roboItem,
          SLOT(updatePath(QPolygonF)));
  connect(commNode, SIGNAL(emitUpdateLocalPath(QPolygonF)), m_roboItem,
          SLOT(updateLocalPath(QPolygonF)));
  connect(commNode, SIGNAL(emitOdomInfo(RobotState)), this,
          SLOT(updateOdomInfo(RobotState)));
  connect(m_roboItem, SIGNAL(signalRunMap(QPixmap)), m_roboGLWidget,
          SLOT(updateRunMap(QPixmap)));
  //    connect(commNode,&rclcomm::emitUpdateMap,[this](QImage img){
  //        m_roboItem->updateMap(img);
  //    });
  connect(m_roboItem, SIGNAL(signalPub2DPose(QPointF, QPointF)), commNode,
          SLOT(pub2DPose(QPointF, QPointF)));
  connect(m_roboItem, SIGNAL(signalPub2DGoal(QPointF, QPointF)), commNode,
          SLOT(pub2DGoal(QPointF, QPointF)));
  // ui相关
  connect(ui->set_pos_btn, &QPushButton::clicked,
          [=]() { m_roboItem->start2DPose(); });
  connect(ui->set_goal_btn, &QPushButton::clicked,
          [=]() { m_roboItem->start2DGoal(); });
  connect(ui->close_btn, &QPushButton::clicked, [=]() { this->close(); });
  connect(ui->min_btn, &QPushButton::clicked, [=]() { this->showMinimized(); });
  connect(ui->max_btn, &QPushButton::clicked, [=]() {
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
  connect(m_roboItem, &roboItem::cursorPos, [=](QPointF pos) {
    QPointF mapPos = commNode->transScenePoint2Word(pos);
    ui->label_pos_map->setText("x: " + QString::number(mapPos.x()).mid(0, 4) +
                               "  y: " + QString::number(mapPos.y()).mid(0, 4));
    ui->label_pos_scene->setText("x: " + QString::number(pos.x()).mid(0, 4) +
                                 "  y: " + QString::number(pos.y()).mid(0, 4));
  });
  m_timerCurrentTime = new QTimer;
  m_timerCurrentTime->setInterval(100);
  m_timerCurrentTime->start();
  QObject::connect(m_timerCurrentTime, &QTimer::timeout, [=]() {
    ui->label_time->setText(
        QDateTime::currentDateTime().toString("  hh:mm:ss  "));
  });
  //   QImage image(600, 600, QImage::Format_RGB888);
  //   QPainter painter(&image);
  //   painter.setRenderHint(QPainter::Antialiasing);
  //   m_qGraphicScene->render(&painter);
  //   image.save("/home/chengyangkj/test.jpg");

  commNode->start();
  initUi();
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
void MainWindow::updateRobotPose(RobotPose pose) {
  m_roboItem->updateRobotPose(pose);
  QPointF pos;
  pos.setX(pose.x);
  pos.setY(pose.y);
  QPointF scenePose = m_roboItem->mapToScene(pos);
  m_roboImg->updatePose(pose);
  m_roboImg->setPos(scenePose.x(), scenePose.y());
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
  setWindowFlags(Qt::CustomizeWindowHint);  // 去掉标题栏
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
  if (event->button() == Qt::LeftButton)  // 判断左键是否按下
  {
    left_pressed_ = true;
    left_pressed_point_ = event->pos();
    last_pressed_point_ = event->pos();
    curpos_ = countFlag(event->pos(), countRow(event->pos()));
  }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event) {
  Q_UNUSED(event);
  if (left_pressed_) left_pressed_ = false;
  setCursor(Qt::ArrowCursor);
  curpos_ = 0;
}

void MainWindow::mouseMoveEvent(QMouseEvent *event) {
  Q_UNUSED(event);
  if (this->isFullScreen()) return;  // 窗口铺满全屏，直接返回，不做任何操作
  // if (left_pressed_)
  //   move(event->pos() - left_pressed_point_ + pos());  // 移动当前窗口

  int poss = countFlag(event->pos(), countRow(event->pos()));
  setCursorType(poss);
  std::cout << "curpos_:" << curpos_ << " pos:" << poss << std::endl;
  if (left_pressed_) {
    QPoint ptemp = event->pos() - left_pressed_point_;
    if (curpos_ == 22)  // 按下窗体中心 移动窗口
    {
      ptemp = ptemp + pos();
      move(ptemp);
    } else {
      QRect wid = geometry();
      switch (curpos_)  // 窗体四周 改变窗口的大小
      {
        case 11:
          wid.setTopLeft(wid.topLeft() + ptemp);
          break;  // 左上角
        case 13:
          wid.setTopRight(wid.topRight() + ptemp);
          break;  // 右上角
        case 31:
          wid.setBottomLeft(wid.bottomLeft() + ptemp);
          break;  // 左下角
        case 33:
          wid.setBottomRight(wid.bottomRight() + ptemp);
          break;  // 右下角
        case 12:
          wid.setTop(wid.top() + ptemp.y());
          break;  // 中上角
        case 21:
          wid.setLeft(wid.left() + ptemp.x());
          break;  // 中左角
        case 23:
          wid.setRight(wid.right() + ptemp.x());
          break;  // 中右角
        case 32:
          wid.setBottom(wid.bottom() + ptemp.y());
          break;  // 中下角
      }
      setGeometry(wid);
    }
    last_pressed_point_ = event->globalPos();  // 更新位置
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
      QApplication::restoreOverrideCursor();  // 恢复鼠标指针性状
      break;
  }
}