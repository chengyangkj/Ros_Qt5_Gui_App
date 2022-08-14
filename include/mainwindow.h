#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "dashboard.h"
#include "rclcomm.h"
#include "roboItem.h"
#include "ui_mainwindow.h"
#include "QTimer"
#include <QDateTime>
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  rclcomm *commNode;
 public slots:
  void onRecvData(QString);

 private:
  void initUi();
  void closeEvent(QCloseEvent *event);  // Overloaded function
  void setCurrentMenu(QPushButton *cur_btn);

 private:
  Ui::MainWindowDesign *ui;
  QGraphicsScene *m_qGraphicScene = nullptr;
  roboItem *m_roboItem = nullptr;
  DashBoard *m_speedDashBoard;
  QTimer *m_timerCurrentTime;
};
#endif  // MAINWINDOW_H
