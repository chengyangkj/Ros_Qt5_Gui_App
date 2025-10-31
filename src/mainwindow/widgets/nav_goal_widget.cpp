#include "widgets/nav_goal_widget.h"

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QDebug>
#include <QPainter>
#include <QShortcut>

void NavGoalWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  
  // 绘制圆角矩形背景
  QRectF rect = this->rect();
  painter.setPen(QPen(QColor(224, 224, 224), 1));
  painter.setBrush(QColor(255, 255, 255));
  painter.drawRoundedRect(rect, 8, 8);
  
  QWidget::paintEvent(event);
}

NavGoalWidget::NavGoalWidget(QWidget *parent) : QWidget(parent) {
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
    QLineEdit, QDoubleSpinBox {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      padding: 4px 6px;
      background-color: #fafafa;
      color: #333333;
      font-size: 11px;
      min-height: 20px;
      max-height: 24px;
    }
    QLineEdit:focus, QDoubleSpinBox:focus {
      border: 2px solid #4a90e2;
      background-color: #ffffff;
    }
    QLineEdit:disabled, QDoubleSpinBox:disabled {
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
    QPushButton#deleteButton {
      background-color: #e74c3c;
    }
    QPushButton#deleteButton:hover {
      background-color: #c0392b;
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
  
  QHBoxLayout *layout_name = new QHBoxLayout();
  layout_name->setSpacing(6);
  QLabel *label_name = new QLabel("name:");
  label_name->setMinimumSize(40, 20);
  lineEdit_name_ = new QLineEdit();
  lineEdit_name_->setReadOnly(true);
  button_edit_name_ = new QPushButton("编辑名称");
  layout_name->addWidget(label_name);
  layout_name->addWidget(lineEdit_name_);
  layout->addLayout(layout_name);

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
  QLabel *label_z = new QLabel("theta:");
  label_z->setMinimumSize(40, 20);
  spinBox_theta_ = new QDoubleSpinBox();
  spinBox_theta_->setRange(-180, 180);
  spinBox_theta_->setSingleStep(1);
  layout_z->addWidget(label_z);
  layout_z->addWidget(spinBox_theta_);
  
  QVBoxLayout *layout_button = new QVBoxLayout();
  layout_button->setSpacing(4);
  button_send_ = new QPushButton("Go To Point");
  button_multi_point_nav_ = new QPushButton("Go Through Points");
  button_remove_ = new QPushButton("Remove Point");
  button_cancel_ = new QPushButton("Close");
  
  // 设置按钮对象名以便样式表识别
  button_remove_->setObjectName("deleteButton");
  button_cancel_->setObjectName("cancelButton");
  
  // 为删除按钮添加快捷键
  button_remove_->setShortcut(QKeySequence::Delete);
  button_remove_->setToolTip("删除点位 (Delete/Backspace)");
  
  layout_button->addWidget(button_edit_name_);
  layout_button->addWidget(button_send_);
  layout_button->addWidget(button_multi_point_nav_);
  layout_button->addWidget(button_remove_);
  layout_button->addWidget(button_cancel_);
  
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
  
  connect(button_edit_name_, &QPushButton::clicked, [this]() {
    if (lineEdit_name_->isReadOnly()) {
      original_name_ = lineEdit_name_->text();
      lineEdit_name_->setReadOnly(false);
      lineEdit_name_->setFocus();
      button_edit_name_->setText("保存名称");
    } else {
      QString new_name = lineEdit_name_->text();
      if (new_name != original_name_) {
        emit SignalHandleOver(HandleResult::kChangeName,
                              RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                       deg2rad(spinBox_theta_->value())),
                              new_name);
      }
      lineEdit_name_->setReadOnly(true);
      button_edit_name_->setText("编辑名称");
    }
  });
  
  connect(button_send_, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(HandleResult::kSend,
                          RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())),
                          lineEdit_name_->text());
  });
  connect(button_multi_point_nav_, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(HandleResult::kMultiPointNav,
                          RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())),
                          lineEdit_name_->text());
  });

  connect(button_cancel_, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(HandleResult::kCancel,
                          RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())),
                          lineEdit_name_->text());
  });
  connect(button_remove_, &QPushButton::clicked, [this]() {
    emit SignalHandleOver(HandleResult::kRemove,
                          RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                    deg2rad(spinBox_theta_->value())),  
                          lineEdit_name_->text());
  });

}
void NavGoalWidget::SlotUpdateValue(double value) {
  emit SignalPoseChanged(RobotPose(spinBox_x_->value(), spinBox_y_->value(),
                                   deg2rad(spinBox_theta_->value())));
}
void NavGoalWidget::SetEditMode(bool flag) {
  spinBox_x_->setEnabled(flag);
  spinBox_y_->setEnabled(flag);
  spinBox_theta_->setEnabled(flag);
  lineEdit_name_->setEnabled(flag);
  button_remove_->setEnabled(flag);
  button_edit_name_->setVisible(flag);
  button_send_->setVisible(!flag);
}

void NavGoalWidget::SetPose(const PointInfo &info) {
  if(IsAnyControlBeingEdited()){
    return;
  }
  spinBox_x_->blockSignals(true);
  spinBox_y_->blockSignals(true);
  spinBox_theta_->blockSignals(true);
  lineEdit_name_->blockSignals(true);
  spinBox_x_->setValue(info.pose.x);
  spinBox_y_->setValue(info.pose.y);
  spinBox_theta_->setValue(rad2deg(info.pose.theta));
  lineEdit_name_->setText(info.name);
  original_name_ = info.name;
  lineEdit_name_->setReadOnly(true);
  button_edit_name_->setText("编辑名称");
  spinBox_x_->blockSignals(false);
  spinBox_y_->blockSignals(false);
  spinBox_theta_->blockSignals(false);
  lineEdit_name_->blockSignals(false);
}

bool NavGoalWidget::IsAnyControlBeingEdited() const {
  // 检查是否有任何输入控件正在获得焦点
  return spinBox_x_->hasFocus() || 
         spinBox_y_->hasFocus() || 
         spinBox_theta_->hasFocus() || 
         lineEdit_name_->hasFocus();
}