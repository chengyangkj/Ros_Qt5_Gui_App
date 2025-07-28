#pragma once
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QWidget>
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
  };

 protected:
  void paintEvent(QPaintEvent *event) override;

 private:
  QLineEdit *lineEdit_route_name_;
  QComboBox *comboBox_controller_;
  QPushButton *button_delete_;
  QPushButton *button_cancel_;
  
 signals:
  void SignalRouteInfoChanged(const RouteInfo &info);
  void SignalHandleOver(const HandleResult &flag, const RouteInfo &info);
  
 public slots:
  void SetRouteInfo(const RouteInfo &info);
  void SetEditMode(bool is_edit);
  void SetSupportControllers(const std::vector<std::string> &controllers);
  
 private slots:
  void SlotUpdateValue();
  
 public:
  TopologyRouteWidget(QWidget *parent = 0);
  ~TopologyRouteWidget() {}
}; 