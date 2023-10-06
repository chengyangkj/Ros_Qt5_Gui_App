/*
 * @Author: chengyangkj chengyangkj@qq.com
 * @Date: 2023-10-06 07:12:50
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-06 07:40:29
 * @FilePath: /ROS2_Qt5_Gui_App/src/mainwindow.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "mainwindow.h"

#include "ui_mainwindow.h"

#include <QCalendarWidget>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QTableWidget>
#include <QToolBar>
#include <QTreeView>
#include <QWidgetAction>

#include "DockAreaTabBar.h"
#include "DockAreaTitleBar.h"
#include "DockAreaWidget.h"
#include "DockComponentsFactory.h"
#include "FloatingDockContainer.h"

using namespace ads;

CMainWindow::CMainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::CMainWindow) {
  setupUi();
}

CMainWindow::~CMainWindow() { delete ui; }
void CMainWindow::setupUi() {
  ui->setupUi(this);
  CDockManager::setConfigFlag(CDockManager::OpaqueSplitterResize, true);
  CDockManager::setConfigFlag(CDockManager::XmlCompressionEnabled, false);
  CDockManager::setConfigFlag(CDockManager::FocusHighlighting, true);
  dock_manager_ = new CDockManager(this);

  QVBoxLayout *central_layout = new QVBoxLayout();

  horizontalLayout_tools_ = new QHBoxLayout();
  horizontalLayout_tools_->setSpacing(0);
  horizontalLayout_tools_->setObjectName(
      QString::fromUtf8("horizontalLayout_tools_"));

  set_pos_btn_ = new QPushButton();
  set_pos_btn_->setObjectName(QString::fromUtf8("set_pos_btn_"));
  set_pos_btn_->setMinimumSize(QSize(0, 25));
  set_pos_btn_->setMaximumSize(QSize(16777215, 16777215));
  set_pos_btn_->setStyleSheet(
      QString::fromUtf8("QPushButton:hover{\n"
                        "background-color:rgb(186, 189, 182);\n"
                        "border-bottom:2px solid rgb(67, 154, 246);\n"
                        "}\n"
                        "QPushButton:checked{\n"
                        "background-color:cyan;\n"
                        "border-bottom:2px solid white \n"
                        "}\n"
                        "QPushButton:pressed{\n"
                        "background-color:rgb(67, 154, 246)\n"
                        "}\n"
                        "QPushButton{\n"
                        "background-color:rgb(238, 238, 236);\n"
                        "border:none; \n"
                        "padding:0px 0px 0px 0px;\n"
                        "margin:0px 0px 0px 0px;\n"
                        "}"));
  QIcon icon4;
  icon4.addFile(QString::fromUtf8(":/images/classes/SetInitialPose.png"),
                QSize(), QIcon::Normal, QIcon::Off);
  set_pos_btn_->setIcon(icon4);
  set_pos_btn_->setText("重定位");
  horizontalLayout_tools_->addWidget(set_pos_btn_);

  set_goal_btn_ = new QPushButton();
  set_goal_btn_->setText("发布目标点");
  set_goal_btn_->setObjectName(QString::fromUtf8("set_goal_btn_"));
  set_goal_btn_->setMinimumSize(QSize(0, 25));
  set_goal_btn_->setStyleSheet(
      QString::fromUtf8("QPushButton:hover{\n"
                        "background-color:rgb(186, 189, 182);\n"
                        "border-bottom:2px solid rgb(67, 154, 246);\n"
                        "}\n"
                        "QPushButton:checked{\n"
                        "background-color:cyan;\n"
                        "border-bottom:2px solid white \n"
                        "}\n"
                        "QPushButton:pressed{\n"
                        "background-color:rgb(67, 154, 246)\n"
                        "}\n"
                        "QPushButton{\n"
                        "background-color:rgb(238, 238, 236);\n"
                        "border:none; \n"
                        "padding:0px 0px 0px 0px;\n"
                        "margin:0px 0px 0px 0px;\n"
                        "}"));
  QIcon icon5;
  icon5.addFile(QString::fromUtf8(":/images/classes/SetGoal.png"), QSize(),
                QIcon::Normal, QIcon::Off);
  set_goal_btn_->setIcon(icon5);

  horizontalLayout_tools_->addWidget(set_goal_btn_);

  set_mutil_goal_btn_ = new QPushButton();
  set_mutil_goal_btn_->setObjectName(QString::fromUtf8("set_mutil_goal_btn_"));
  set_mutil_goal_btn_->setText("发布多目标点");
  set_mutil_goal_btn_->setStyleSheet(
      QString::fromUtf8("QPushButton:hover{\n"
                        "background-color:rgb(186, 189, 182);\n"
                        "border-bottom:2px solid rgb(67, 154, 246);\n"
                        "}\n"
                        "QPushButton:checked{\n"
                        "background-color:cyan;\n"
                        "border-bottom:2px solid white \n"
                        "}\n"
                        "QPushButton:pressed{\n"
                        "background-color:rgb(67, 154, 246)\n"
                        "}\n"
                        "QPushButton{\n"
                        "background-color:rgb(238, 238, 236);\n"
                        "border:none; \n"
                        "padding:0px 0px 0px 0px;\n"
                        "margin:0px 0px 0px 0px;\n"
                        "}"));
  QIcon icon6;
  icon6.addFile(QString::fromUtf8("://images/mutil_pose.png"), QSize(),
                QIcon::Normal, QIcon::Off);
  set_mutil_goal_btn_->setIcon(icon6);

  horizontalLayout_tools_->addWidget(set_mutil_goal_btn_);

  QGraphicsView *graphicsView = new QGraphicsView();
  display_manager_ = new Display::DisplayManager(graphicsView);

  // Set central widget

  QWidget *center_widget = new QWidget();
  central_layout->addLayout(horizontalLayout_tools_);
  central_layout->addWidget(graphicsView);
  center_widget->setLayout(central_layout);
  CDockWidget *CentralDockWidget = new CDockWidget("CentralWidget");
  CentralDockWidget->setWidget(center_widget);
  auto *CentralDockArea = dock_manager_->setCentralWidget(CentralDockWidget);
  CentralDockArea->setAllowedAreas(DockWidgetArea::OuterDockAreas);
}
void CMainWindow::savePerspective() {
  QString PerspectiveName =
      QInputDialog::getText(this, "Save Perspective", "Enter unique name:");
  if (PerspectiveName.isEmpty()) {
    return;
  }

  dock_manager_->addPerspective(PerspectiveName);
  QSignalBlocker Blocker(PerspectiveComboBox);
  PerspectiveComboBox->clear();
  PerspectiveComboBox->addItems(dock_manager_->perspectiveNames());
  PerspectiveComboBox->setCurrentText(PerspectiveName);
}

//============================================================================
void CMainWindow::closeEvent(QCloseEvent *event) {
  // Delete dock manager here to delete all floating widgets. This ensures
  // that all top level windows of the dock manager are properly closed
  dock_manager_->deleteLater();
  QMainWindow::closeEvent(event);
}
