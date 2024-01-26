#pragma once
#include <QCalendarWidget>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QGraphicsItem>
#include <QHBoxLayout>
#include <QtWidgets/QListWidget>
#include "algorithm.h"
#include "point_type.h"
#include "widgets/joystick.h"
using namespace basic;
class SetPoseWidget : public QWidget {
  Q_OBJECT
 private:
  QDoubleSpinBox *spinBox_x_;
  QDoubleSpinBox *spinBox_y_;
  QDoubleSpinBox *spinBox_theta_;
 signals:
  void SignalPoseChanged(const RobotPose &pose);
  void SignalHandleOver(const bool &is_submit, const RobotPose &pose);
 public slots:
  void SetPose(const RobotPose &pose);
 private slots:
  void SlotUpdateValue(double);

 public:
  SetPoseWidget(QWidget *parent = 0);
  ~SetPoseWidget() {}
};
