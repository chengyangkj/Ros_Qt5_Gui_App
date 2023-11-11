#include "widgets/set_pose_widget.h"

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
SetPoseWidget::SetPoseWidget(QWidget *parent) : QWidget(parent) {

  QVBoxLayout *layout = new QVBoxLayout(this);
  QHBoxLayout *layout_x = new QHBoxLayout();
  QLabel *label_x = new QLabel("X:");
  spinBox_x_ = new QDoubleSpinBox();
  spinBox_x_->setRange(-10000, 10000);
  layout_x->addWidget(label_x);
  layout_x->addWidget(spinBox_x_);
  QHBoxLayout *layout_y = new QHBoxLayout();
  QLabel *label_y = new QLabel("Y:");
  spinBox_y_ = new QDoubleSpinBox();
  spinBox_y_->setRange(-10000, 10000);
  layout_y->addWidget(label_y);
  layout_y->addWidget(spinBox_y_);
  QHBoxLayout *layout_z = new QHBoxLayout();
  QLabel *label_z = new QLabel("theta:");
  spinBox_theta_ = new QDoubleSpinBox();
  spinBox_theta_->setRange(-10000, 10000);
  layout_z->addWidget(label_z);
  layout_z->addWidget(spinBox_theta_);

  QHBoxLayout *layout_button = new QHBoxLayout();
  QPushButton *button_ok = new QPushButton("OK");
  QPushButton *button_cancel = new QPushButton("Cancel");
  layout_button->addWidget(button_ok);
  layout_button->addWidget(button_cancel);
  layout->addLayout(layout_x);
  layout->addLayout(layout_y);
  layout->addLayout(layout_z);
  layout->addLayout(layout_button);

  this->setLayout(layout);

  connect(button_ok, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(true, RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())));
  });
  connect(button_cancel, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(false,
                          RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())));
  });
}
void SetPoseWidget::SetPose(const RobotPose &pose) {
  spinBox_x_->setValue(pose.x);
  spinBox_y_->setValue(pose.y);
  spinBox_theta_->setValue(rad2deg(pose.theta));
}