#pragma once
#include <QCalendarWidget>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QGraphicsItem>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QtWidgets/QListWidget>
#include "algorithm.h"
#include "point_type.h"
#include "widgets/joystick.h"
using namespace basic;
class NavGoalWidget : public QWidget {
  Q_OBJECT
 public:
  enum HandleResult { kSend = 0,
                      kRemove = 1,
                      kCancel = 2,
                      kChangeName = 3,
                      kMultiPointNav = 4 };
  struct PointInfo {
    RobotPose pose;
    QString name;
  };

 protected:
  void paintEvent(QPaintEvent *event) override;

 private:
  QDoubleSpinBox *spinBox_x_;
  QDoubleSpinBox *spinBox_y_;
  QDoubleSpinBox *spinBox_theta_;
  QLineEdit *lineEdit_name_;
  QPushButton *button_edit_name_;
  QPushButton *button_send_;
  QPushButton *button_multi_point_nav_;
  QPushButton *button_remove_;
  QPushButton *button_cancel_;
  QString original_name_;
  
  // 检查是否有控件正在被编辑
  bool IsAnyControlBeingEdited() const;
  
 signals:
  void SignalPoseChanged(const RobotPose &pose);
  void SignalHandleOver(const HandleResult &flag, const RobotPose &pose,const QString &name);
  void SignalPointNameChanged(const QString &name);
 public slots:
  void SetPose(const PointInfo &pose);
  void SetEditMode(bool is_edit);
 private slots:
  void SlotUpdateValue(double);

 public:
  NavGoalWidget(QWidget *parent = 0);
  ~NavGoalWidget() {}
};
