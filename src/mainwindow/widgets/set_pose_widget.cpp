#include "widgets/set_pose_widget.h"
#include <QDebug>
#include <QPainter>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>

void SetPoseWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  
  // 绘制圆角矩形背景
  QRectF rect = this->rect();
  painter.setPen(QPen(QColor(224, 224, 224), 1));
  painter.setBrush(QColor(255, 255, 255));
  painter.drawRoundedRect(rect, 8, 8);
  
  QWidget::paintEvent(event);
}

SetPoseWidget::SetPoseWidget(QWidget *parent) : QWidget(parent) {
  // 设置背景角色，确保在QGraphicsView中背景不透明
  setAutoFillBackground(true);
  setAttribute(Qt::WA_OpaquePaintEvent);
  
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->setSpacing(8);
  layout->setContentsMargins(12, 12, 12, 12);
  
  // 设置现代化样式
  setStyleSheet(R"(
    QWidget {
      background-color: transparent;
      font-family: 'Segoe UI', Arial, sans-serif;
      font-size: 12px;
    }
    QLabel {
      color: #333333;
      font-weight: 500;
      border: none;
      background: transparent;
    }
    QDoubleSpinBox {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      padding: 4px 6px;
      background-color: #fafafa;
      color: #333333;
      font-size: 11px;
      min-height: 20px;
      max-height: 24px;
    }
    QDoubleSpinBox:focus {
      border: 2px solid #4a90e2;
      background-color: #ffffff;
    }
    QDoubleSpinBox:disabled {
      background-color: #f5f5f5;
      color: #999999;
      border: 1px solid #e0e0e0;
    }
    QPushButton {
      border: none;
      border-radius: 4px;
      padding: 6px 12px;
      font-weight: 500;
      font-size: 11px;
      color: #ffffff;
      background-color: #4a90e2;
      min-height: 24px;
      max-height: 28px;
    }
    QPushButton:hover {
      background-color: #357abd;
    }
    QPushButton:pressed {
      background-color: #2d5aa0;
    }
    QPushButton:disabled {
      background-color: #cccccc;
      color: #999999;
    }
    QPushButton#okButton {
      background-color: #27ae60;
    }
    QPushButton#okButton:hover {
      background-color: #229954;
    }
    QPushButton#cancelButton {
      background-color: #95a5a6;
    }
    QPushButton#cancelButton:hover {
      background-color: #7f8c8d;
    }
  )");
  
  // 设置调色板确保背景不透明
  QPalette pal = palette();
  pal.setColor(QPalette::Window, QColor(255, 255, 255));
  pal.setColor(QPalette::Base, QColor(255, 255, 255));
  setPalette(pal);
  
  QHBoxLayout *layout_x = new QHBoxLayout();
  layout_x->setSpacing(6);
  QLabel *label_x = new QLabel("X:");
  label_x->setMinimumSize(20, 20);
  spinBox_x_ = new QDoubleSpinBox();
  spinBox_x_->setRange(-10000, 10000);
  spinBox_x_->setSingleStep(0.1);
  layout_x->addWidget(label_x);
  layout_x->addWidget(spinBox_x_);
  
  QHBoxLayout *layout_y = new QHBoxLayout();
  layout_y->setSpacing(6);
  QLabel *label_y = new QLabel("Y:");
  label_y->setMinimumSize(20, 20);
  spinBox_y_ = new QDoubleSpinBox();
  spinBox_y_->setRange(-10000, 10000);
  spinBox_y_->setSingleStep(0.1);
  layout_y->addWidget(label_y);
  layout_y->addWidget(spinBox_y_);
  
  QHBoxLayout *layout_z = new QHBoxLayout();
  layout_z->setSpacing(6);
  QLabel *label_z = new QLabel("角度:");
  label_z->setMinimumSize(40, 20);
  spinBox_theta_ = new QDoubleSpinBox();
  spinBox_theta_->setRange(-180, 180);
  spinBox_theta_->setSingleStep(1);
  layout_z->addWidget(label_z);
  layout_z->addWidget(spinBox_theta_);
  
  QHBoxLayout *layout_button = new QHBoxLayout();
  layout_button->setSpacing(6);
  QPushButton *button_ok = new QPushButton("确定");
  QPushButton *button_cancel = new QPushButton("取消");
  
  // 设置按钮对象名以便样式表识别
  button_ok->setObjectName("okButton");
  button_cancel->setObjectName("cancelButton");
  
  layout_button->addWidget(button_ok);
  layout_button->addWidget(button_cancel);
  
  layout->addLayout(layout_x);
  layout->addLayout(layout_y);
  layout->addLayout(layout_z);
  layout->addSpacing(4);
  layout->addLayout(layout_button);
  
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