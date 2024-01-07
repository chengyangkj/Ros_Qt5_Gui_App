#pragma once
#include "algorithm.h"
#include "point_type.h"
#include "widgets/joystick.h"
#include <QCalendarWidget>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QGraphicsItem>
#include <QHBoxLayout>
#include <QtWidgets/QListWidget>
using namespace basic;
class NavGoalWidget : public QWidget {
  Q_OBJECT
public:
  enum HandleResult { kSend = 0, kRemove = 1, kCancel = 2 };

private:
  QDoubleSpinBox *spinBox_x_;
  QDoubleSpinBox *spinBox_y_;
  QDoubleSpinBox *spinBox_theta_;
signals:
  void SignalPoseChanged(const RobotPose &pose);
  void SignalHandleOver(const HandleResult &flag, const RobotPose &pose);
public slots:
  void SetPose(const RobotPose &pose);
private slots:
  void SlotUpdateValue(double);

public:
  NavGoalWidget(QWidget *parent = 0);
  ~NavGoalWidget() {}
};
