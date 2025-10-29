#include "widgets/topology_route_widget.h"
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDebug>
#include <QPainter>
#include <QDoubleSpinBox>

void TopologyRouteWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  
  // 绘制圆角矩形背景
  QRectF rect = this->rect();
  painter.setPen(QPen(QColor(224, 224, 224), 1));
  painter.setBrush(QColor(255, 255, 255));
  painter.drawRoundedRect(rect, 8, 8);
  
  QWidget::paintEvent(event);
}

TopologyRouteWidget::TopologyRouteWidget(QWidget *parent) : QWidget(parent) {
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
    QLineEdit, QComboBox {
      border: 1px solid #d0d0d0;
      border-radius: 4px;
      padding: 4px 6px;
      background-color: #fafafa;
      color: #333333;
      font-size: 11px;
      min-height: 20px;
      max-height: 24px;
    }
    QLineEdit:focus, QComboBox:focus {
      border: 2px solid #4a90e2;
      background-color: #ffffff;
    }
    QLineEdit:disabled, QComboBox:disabled {
      background-color: #f5f5f5;
      color: #999999;
      border: 1px solid #e0e0e0;
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
    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
      border: none;
      background-color: #f0f0f0;
      width: 16px;
      border-radius: 2px;
    }
    QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
      background-color: #e0e0e0;
    }
    QDoubleSpinBox::up-button:pressed, QDoubleSpinBox::down-button:pressed {
      background-color: #d0d0d0;
    }
    QDoubleSpinBox::up-arrow {
      image: none;
      border-left: 4px solid transparent;
      border-right: 4px solid transparent;
      border-bottom: 4px solid #666666;
    }
    QDoubleSpinBox::down-arrow {
      image: none;
      border-left: 4px solid transparent;
      border-right: 4px solid transparent;
      border-top: 4px solid #666666;
    }
    QComboBox::drop-down {
      border: none;
      width: 20px;
    }
    QComboBox::down-arrow {
      image: none;
      border-left: 5px solid transparent;
      border-right: 5px solid transparent;
      border-top: 5px solid #666666;
      margin-right: 5px;
    }
    QComboBox:disabled::down-arrow {
      border-top-color: #cccccc;
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
  
  // 路径名称（只读）
  QHBoxLayout *layout_name = new QHBoxLayout();
  layout_name->setSpacing(6);
  QLabel *label_name = new QLabel("路径名:");
  label_name->setMinimumSize(50, 20);
  lineEdit_route_name_ = new QLineEdit();
  lineEdit_route_name_->setReadOnly(true); // 路径名不可修改
  layout_name->addWidget(label_name);
  layout_name->addWidget(lineEdit_route_name_);
  layout->addLayout(layout_name);
  
  // 控制器类型
  QHBoxLayout *layout_controller = new QHBoxLayout();
  layout_controller->setSpacing(6);
  QLabel *label_controller = new QLabel("控制器:");
  label_controller->setMinimumSize(50, 20);
  comboBox_controller_ = new QComboBox();
  layout_controller->addWidget(label_controller);
  layout_controller->addWidget(comboBox_controller_);
  layout->addLayout(layout_controller);
  
  // 速度限制
  QHBoxLayout *layout_speed = new QHBoxLayout();
  layout_speed->setSpacing(6);
  QLabel *label_speed = new QLabel("速度限制:");
  label_speed->setMinimumSize(50, 20);
  spinBox_speed_limit_ = new QDoubleSpinBox();
  spinBox_speed_limit_->setRange(0.1, 10.0);  // 设置范围0.1到10.0
  spinBox_speed_limit_->setValue(1.0);  // 默认值
  spinBox_speed_limit_->setSuffix(" m/s");  // 添加单位后缀
  spinBox_speed_limit_->setDecimals(2);  // 显示两位小数
  spinBox_speed_limit_->setSingleStep(0.1);  // 步进值0.1
  layout_speed->addWidget(label_speed);
  layout_speed->addWidget(spinBox_speed_limit_);
  layout->addLayout(layout_speed);
  
  // 按钮区域
  QVBoxLayout *layout_button = new QVBoxLayout();
  layout_button->setSpacing(4);
  button_delete_ = new QPushButton("删除路径");
  button_cancel_ = new QPushButton("关闭");
  
  // 设置按钮对象名以便样式表识别
  button_delete_->setObjectName("deleteButton");
  button_cancel_->setObjectName("cancelButton");
  
  // 为删除按钮添加快捷键
  button_delete_->setShortcut(QKeySequence::Delete);
  button_delete_->setToolTip("删除路径 (Delete/Backspace)");
  
  layout_button->addWidget(button_delete_);
  layout_button->addWidget(button_cancel_);
  layout->addSpacing(4);
  layout->addLayout(layout_button);
  
  this->setLayout(layout);
  
  // 连接信号槽
  connect(comboBox_controller_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &TopologyRouteWidget::SlotUpdateValue);
  
  connect(spinBox_speed_limit_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &TopologyRouteWidget::SlotUpdateValue);
  
  connect(button_delete_, &QPushButton::clicked, [this]() {
    RouteInfo info;
    info.route_name = lineEdit_route_name_->text();
    info.controller = comboBox_controller_->currentText().toStdString();
    
    // 验证速度限制输入
    double speed_limit = spinBox_speed_limit_->value();
    info.speed_limit = (speed_limit > 0) ? speed_limit : 1.0;
    
    emit SignalHandleOver(HandleResult::kDelete, info);
  });
  
  connect(button_cancel_, &QPushButton::clicked, [this]() {
    RouteInfo info;
    info.route_name = lineEdit_route_name_->text();
    info.controller = comboBox_controller_->currentText().toStdString();
    
    // 验证速度限制输入
    double speed_limit = spinBox_speed_limit_->value();
    info.speed_limit = (speed_limit > 0) ? speed_limit : 1.0;
    
    emit SignalHandleOver(HandleResult::kCancel, info);
  });
}

void TopologyRouteWidget::SetEditMode(bool is_edit) {
  comboBox_controller_->setEnabled(is_edit);
  spinBox_speed_limit_->setEnabled(is_edit);
  button_delete_->setEnabled(is_edit);
}

void TopologyRouteWidget::SetSupportControllers(const std::set<std::string> &controllers) {
  comboBox_controller_->clear();
  for (const auto &controller : controllers) {
    comboBox_controller_->addItem(controller.c_str());
    std::cout << "add controller:" << controller << std::endl;
  }
  
}

void TopologyRouteWidget::SlotUpdateValue() {
  RouteInfo info;
  info.route_name = lineEdit_route_name_->text();
  info.controller = comboBox_controller_->currentText().toStdString();
  
  // 验证速度限制输入
  double speed_limit = spinBox_speed_limit_->value();
  if (speed_limit > 0 && speed_limit <= 10.0) {  // 限制在0.1到10.0之间
    info.speed_limit = speed_limit;
  } else {
    // 如果输入无效，使用默认值
    info.speed_limit = 1.0;
    spinBox_speed_limit_->setValue(1.0);
  }
  
  emit SignalRouteInfoChanged(info);
}

void TopologyRouteWidget::SetRouteInfo(const RouteInfo &info) {
  if(IsAnyControlBeingEdited()){
    return;
  }
  lineEdit_route_name_->blockSignals(true);
  comboBox_controller_->blockSignals(true);
  spinBox_speed_limit_->blockSignals(true);
  
  lineEdit_route_name_->setText(info.route_name);
  
  // 设置控制器
  int controller_index = comboBox_controller_->findText(info.controller.c_str());
  if (controller_index >= 0) {
    comboBox_controller_->setCurrentIndex(controller_index);
  }
  
  // 设置速度限制
  spinBox_speed_limit_->setValue(info.speed_limit);
  
  lineEdit_route_name_->blockSignals(false);
  comboBox_controller_->blockSignals(false);
  spinBox_speed_limit_->blockSignals(false);
} 

bool TopologyRouteWidget::IsAnyControlBeingEdited() const {
  // 检查是否有任何输入控件正在获得焦点
  return comboBox_controller_->hasFocus() || 
         spinBox_speed_limit_->hasFocus();
} 