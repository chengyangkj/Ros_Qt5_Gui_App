#include "widgets/nav_goal_widget.h"

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
NavGoalWidget::NavGoalWidget(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setSpacing(0);
  QHBoxLayout *layout_name = new QHBoxLayout();
  layout_name->setSpacing(0);
  QLabel *label_name = new QLabel("Name:");
  label_name->setMinimumSize(15, 30);
  lineEdit_name_ = new QLineEdit();
  layout_name->addWidget(label_name);
  layout_name->addWidget(lineEdit_name_);
  layout->addLayout(layout_name);

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
  label_z->setMinimumSize(15, 30);
  label_z->setMaximumSize(40, 30);
  QVBoxLayout *layout_button = new QVBoxLayout();
  layout_button->setSpacing(0);
  QPushButton *button_send = new QPushButton("Move");
  QPushButton *button_remove = new QPushButton("Delete");
  QPushButton *button_cancel = new QPushButton("Close");
  layout_button->addWidget(button_send);
  layout_button->addWidget(button_remove);
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
  connect(button_send, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(HandleResult::kSend,
                          RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())));
  });
  connect(button_cancel, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(HandleResult::kCancel,
                          RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())));
  });
  connect(button_remove, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(HandleResult::kRemove,
                          RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())));
  });
}
void NavGoalWidget::SlotUpdateValue(double value) {
  emit SignalPoseChanged(RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                   deg2rad(spinBox_theta_->value())));
}
void NavGoalWidget::SetEditEnabled(bool flag) {
  spinBox_x_->setEnabled(flag);
  spinBox_y_->setEnabled(flag);
  spinBox_theta_->setEnabled(flag);
  lineEdit_name_->setEnabled(flag);
}
void NavGoalWidget::SetPose(const PointInfo &info) {
  spinBox_x_->blockSignals(true);
  spinBox_y_->blockSignals(true);
  spinBox_theta_->blockSignals(true);
  spinBox_x_->setValue(info.pose.x);
  spinBox_y_->setValue(info.pose.y);
  spinBox_theta_->setValue(rad2deg(info.pose.theta));
  lineEdit_name_->setText(info.name);
  spinBox_x_->blockSignals(false);
  spinBox_y_->blockSignals(false);
  spinBox_theta_->blockSignals(false);
}