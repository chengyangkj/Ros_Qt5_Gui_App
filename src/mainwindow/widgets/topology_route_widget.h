#pragma once
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QWidget>
#include <QDoubleSpinBox>
#include "map/topology_map.h"

class TopologyRouteWidget : public QWidget {
  Q_OBJECT
 public:
  enum HandleResult { 
    kDelete = 0,
    kCancel = 1
  };
  
  struct RouteInfo {
    QString route_name;
    std::string controller;
    std::string goal_checker;
    double speed_limit = 1.0;  // 默认最大速度1.0
  };

 protected:
  void paintEvent(QPaintEvent *event) override;

 private:
  QLineEdit *lineEdit_route_name_;
  QComboBox *comboBox_controller_;
  QComboBox *comboBox_goal_checker_;
  QDoubleSpinBox *spinBox_speed_limit_;  // 速度限制数字输入框
  QPushButton *button_delete_;
  QPushButton *button_cancel_;
  
  // 检查是否有控件正在被编辑
  bool IsAnyControlBeingEdited() const;
  
 signals:
  void SignalRouteInfoChanged(const RouteInfo &info);
  void SignalHandleOver(const HandleResult &flag, const RouteInfo &info);
  
 public slots:
  void SetRouteInfo(const RouteInfo &info);
  void SetEditMode(bool is_edit);
  void SetSupportControllers(const std::set<std::string> &controllers);
  void SetSupportGoalCheckers(const std::set<std::string> &goal_checkers);
  
 private slots:
  void SlotUpdateValue();
  
 public:
  TopologyRouteWidget(QWidget *parent = 0);
  ~TopologyRouteWidget() {}
}; 