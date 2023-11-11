#pragma once
#include "basic/algorithm.h"
#include "basic/point_type.h"
#include "widgets/joystick.h"
#include <QCalendarWidget>
#include <QCheckBox>
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
#include <QVBoxLayout>
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
using namespace basic;
class SpeedCtrlWidget : public QWidget {
  Q_OBJECT
private:
  QCheckBox *checkBox_use_all_;
  JoyStick *joyStick_widget_;
  QSlider *horizontalSlider_raw_;
  QSlider *horizontalSlider_linear_;
signals:
  void signalControlSpeed(const RobotSpeed &speed);
private slots:
  void slotSpeedControl() {
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

    float x = moveBindings[key][0];
    float y = moveBindings[key][1];
    float z = moveBindings[key][2];
    float th = moveBindings[key][3];
    emit signalControlSpeed(RobotSpeed(x * liner, y * liner, th * turn));
  }
  void slotJoyStickKeyChange(int value) {
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
    emit signalControlSpeed(RobotSpeed(x * liner, y * liner, th * turn));
  }

public:
  SpeedCtrlWidget(QWidget *parent = 0) : QWidget(parent) {
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
    horizontalLayout_18->setObjectName(
        QString::fromUtf8("horizontalLayout_18"));
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
    horizontalLayout_19->setObjectName(
        QString::fromUtf8("horizontalLayout_19"));
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
    pushButton_back->setShortcut(
        QApplication::translate("Widget", ",", nullptr));
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
    connect(pushButton_backr, SIGNAL(clicked()), this,
            SLOT(slotSpeedControl()));

    horizontalLayout_19->addWidget(pushButton_backr);

    verticalLayout_cmd_btn->addLayout(horizontalLayout_19);

    QWidget *cmdCtrlWidget = new QWidget();
    cmdCtrlWidget->setLayout(verticalLayout_cmd_btn);

    QTabWidget *tabWidget = new QTabWidget;

    tabWidget->addTab(cmdCtrlWidget, "CmdCtrl");
    verticalLayout_speed_ctrl->addWidget(tabWidget);

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
    horizontalLayout_20->setObjectName(
        QString::fromUtf8("horizontalLayout_20"));
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
    connect(horizontalSlider_raw_, &QSlider::valueChanged,
            [label_raw](qreal value) {
              label_raw->setText(QString::number(rad2deg(value * 0.01)) +
                                 " deg/s");
            });
    horizontalLayout_20->addWidget(label_raw);

    verticalLayout_speed_ctrl->addLayout(horizontalLayout_20);

    // linear anglur
    QHBoxLayout *horizontalLayout_21 = new QHBoxLayout();
    horizontalLayout_21->setObjectName(
        QString::fromUtf8("horizontalLayout_21"));
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
    connect(btn_stop, &QPushButton::clicked,
            [this]() { emit signalControlSpeed(RobotSpeed()); });
    horizontalLayout_stop_button->addStretch();
    horizontalLayout_stop_button->addWidget(btn_stop);
    horizontalLayout_stop_button->addStretch();
    verticalLayout_speed_ctrl->addLayout(horizontalLayout_stop_button);

    QHBoxLayout *horizontalLayout_23 = new QHBoxLayout();
    horizontalLayout_23->setObjectName(
        QString::fromUtf8("horizontalLayout_23"));
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

    this->setLayout(verticalLayout_speed_ctrl);
  }

  ~SpeedCtrlWidget() {}
};
