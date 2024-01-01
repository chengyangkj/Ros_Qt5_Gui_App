#include "widgets/set_pose_widget.h"

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
SetPoseWidget::SetPoseWidget(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setSpacing(0);
  QHBoxLayout *layout_x = new QHBoxLayout();
  layout_x->setSpacing(0);
  QLabel *label_x = new QLabel("X:");
  label_x->setMaximumSize(15, 30);
  spinBox_x_ = new QDoubleSpinBox();
  spinBox_x_->setRange(-10000, 10000);
  spinBox_x_->setSingleStep(0.1);
  layout_x->addWidget(label_x);
  layout_x->addWidget(spinBox_x_);
  QHBoxLayout *layout_y = new QHBoxLayout();
  layout_y->setSpacing(0);

  QLabel *label_y = new QLabel("Y:");
  label_y->setMaximumSize(15, 30);
  spinBox_y_ = new QDoubleSpinBox();
  spinBox_y_->setRange(-10000, 10000);
  spinBox_y_->setSingleStep(0.1);
  layout_y->addWidget(label_y);
  layout_y->addWidget(spinBox_y_);
  QHBoxLayout *layout_z = new QHBoxLayout();
  layout_z->setSpacing(0);

  QLabel *label_z = new QLabel("theta:");
  spinBox_theta_ = new QDoubleSpinBox();
  spinBox_theta_->setRange(-180, 180);
  spinBox_theta_->setSingleStep(1);
  layout_z->addWidget(label_z);
  layout_z->addWidget(spinBox_theta_);
  label_z->setMaximumSize(40, 30);
  QHBoxLayout *layout_button = new QHBoxLayout();
  layout_button->setSpacing(0);
  QPushButton *button_ok = new QPushButton("OK");
  QPushButton *button_cancel = new QPushButton("Cancel");
  layout_button->addWidget(button_ok);
  layout_button->addWidget(button_cancel);
  layout->addLayout(layout_x);
  layout->addLayout(layout_y);
  layout->addLayout(layout_z);
  layout->addLayout(layout_button);
  //   QPalette pal = childWidget->palette();
  this->setLayout(layout);
  connect(spinBox_x_, SIGNAL(valueChanged(double)), this,
          SLOT(SlotUpdateValue(double)));
  connect(spinBox_y_, SIGNAL(valueChanged(double)), this,
          SLOT(SlotUpdateValue(double)));
  connect(spinBox_theta_, SIGNAL(valueChanged(double)), this,
          SLOT(SlotUpdateValue(double)));
  connect(button_ok, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(true,
                          RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())));
  });
  connect(button_cancel, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(false,
                          RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())));
  });
}
void SetPoseWidget::SlotUpdateValue(double value) {
  emit SignalPoseChanged(RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                   deg2rad(spinBox_theta_->value())));
}
void SetPoseWidget::SetPose(const RobotPose &pose) {
  spinBox_x_->blockSignals(true);
  spinBox_y_->blockSignals(true);
  spinBox_theta_->blockSignals(true);
  spinBox_x_->setValue(pose.x);
  spinBox_y_->setValue(pose.y);
  spinBox_theta_->setValue(rad2deg(pose.theta));

  spinBox_x_->blockSignals(false);
  spinBox_y_->blockSignals(false);
  spinBox_theta_->blockSignals(false);
}