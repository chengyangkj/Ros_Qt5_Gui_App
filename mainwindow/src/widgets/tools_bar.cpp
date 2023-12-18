#include "widgets/tools_bar.h"

ToolsBarWidget::ToolsBarWidget(QWidget *parent) : QWidget(parent) {
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
  set_pos_btn->setText("Reloc(重定位)");
  horizontalLayout_tools->addWidget(set_pos_btn);

  QPushButton *set_goal_btn = new QPushButton();
  set_goal_btn->setText("Set Nav Goal(导航到目标点)");
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

  QPushButton *focus_robot_btn = new QPushButton();
  focus_robot_btn->setObjectName(QString::fromUtf8(" focus_robot_btn"));
  focus_robot_btn->setText("Focus Robot");
  focus_robot_btn->setStyleSheet(
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
  icon6.addFile(QString::fromUtf8("://images/crosshair.svg"), QSize(),
                QIcon::Normal, QIcon::Off);
  focus_robot_btn->setIcon(icon6);

  horizontalLayout_tools->addWidget(focus_robot_btn);
  this->setLayout(horizontalLayout_tools);

  connect(set_pos_btn, SIGNAL(clicked()), this, SIGNAL(SignalSetRelocPose()));
  connect(set_goal_btn, SIGNAL(clicked()), this, SIGNAL(SignalSetNavPose()));
  connect(focus_robot_btn, &QPushButton::clicked, [this, focus_robot_btn]() {
    if (focus_robot_btn->text() == "Focus Robot") {
      focus_robot_btn->setText("Stop Focus Robot");
      emit SignalFocusRobot(true);
    } else {
      focus_robot_btn->setText("Focus Robot");
      emit SignalFocusRobot(false);
    }
  });
}
