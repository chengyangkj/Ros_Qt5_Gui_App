#include "widgets/status_bar.h"

StatusBarWidget::StatusBarWidget(QWidget *parent) : QWidget(parent) {

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

  horizontalLayout_status->addWidget(label_power_);
  this->setLayout(horizontalLayout_status);

  SlotSetBatteryStatus(0, 0);
  QTimer *timer_current_time = new QTimer;
  timer_current_time->setInterval(100);
  timer_current_time->start();
  QObject::connect(timer_current_time, &QTimer::timeout, [=]() {
    label_time->setText(QDateTime::currentDateTime().toString("  hh:mm:ss  "));
  });
}

void StatusBarWidget::SlotSetBatteryStatus(double percent, double voltage) {
  battery_bar_->setValue(percent);
  label_power_->setText(QString::number(voltage, 'f', 2) + "V");
}