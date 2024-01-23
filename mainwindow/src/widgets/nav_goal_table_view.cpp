#include "widgets/nav_goal_table_view.h"
#include "algorithm.h"
#include "logger/logger.h"
#include <QComboBox>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QtConcurrent>
NavGoalTableView::NavGoalTableView(QWidget *_parent_widget)
    : QTableView(_parent_widget) {
  table_model_ = new QStandardItemModel();
  setModel(table_model_);
  QStringList table_h_headers;
  table_h_headers << "点位名"
                  << "任务状态"
                  << "删除"
                  << "运行";
  QHeaderView *headerView = new QHeaderView(Qt::Horizontal);
  headerView->setSectionResizeMode(QHeaderView::ResizeToContents);
  headerView->setSelectionBehavior(QAbstractItemView::SelectRows);
  headerView->setCascadingSectionResizes(false);
  setSelectionBehavior(QAbstractItemView::SelectRows);
  setSelectionMode(QAbstractItemView::SingleSelection);
  this->setHorizontalHeader(headerView);
  // 添加数据模型
  table_model_->setHorizontalHeaderLabels(table_h_headers);
  connect(table_model_, &QStandardItemModel::itemChanged, this,
          &NavGoalTableView::onItemChanged);
}

NavGoalTableView::~NavGoalTableView() {}

void NavGoalTableView::onItemChanged(QStandardItem *item) {
  if (item->column() == 0) {
    qDebug() << "点位名: " << item->text();
  } else if (item->column() == 2) {
    qDebug() << "任务状态: " << item->checkState();
  }
}
void NavGoalTableView::UpdateTopologyMap(const TopologyMap &_topology_map) {
  topologyMap_ = _topology_map;
}
void NavGoalTableView::UpdateSelectPoint(const TopologyMap::PointInfo &point) {
  if (!this->isEnabled())
    return;
  QModelIndexList selectedIndexes = selectionModel()->selectedRows();
  if (selectedIndexes.size() == 1) {
    QModelIndex modelIndex = model()->index(selectedIndexes[0].row(), 0);
    QWidget *widget = indexWidget(modelIndex);

    if (widget) {
      QComboBox *comboBox = static_cast<QComboBox *>(widget);
      comboBox->setCurrentText(point.name.c_str());
    }
  } else {
    QWidget *widget =
        indexWidget(model()->index(table_model_->rowCount() - 1, 0));
    if (widget) {
      QComboBox *comboBox = static_cast<QComboBox *>(widget);
      comboBox->setCurrentText(point.name.c_str());
    }
  }
}
void NavGoalTableView::AddItem() {

  QComboBox *comboBox = new QComboBox();
  for (auto point : topologyMap_.points) {
    comboBox->addItem(point.name.c_str());
  }
  QLabel *label_status = new QLabel("None");
  QPushButton *button_remove = new QPushButton("Delete");
  QPushButton *button_run = new QPushButton("Run");
  int row = table_model_->rowCount();

  connect(button_remove, &QPushButton::clicked, [this, row]() {
    QModelIndexList selectedIndexes = selectionModel()->selectedRows();
    if (selectedIndexes.size() == 1) {
      table_model_->removeRow(selectedIndexes[0].row());
    }
  });
  table_model_->insertRow(row);

  setIndexWidget(table_model_->index(row, 0), comboBox);
  setIndexWidget(table_model_->index(row, 1), label_status);
  setIndexWidget(table_model_->index(row, 2), button_remove);
  setIndexWidget(table_model_->index(row, 3), button_run);
}
void NavGoalTableView::StartTaskChain() {
  is_task_chain_running_ = true;
  QtConcurrent::run([this]() {
    for (int row = 0; row < table_model_->rowCount(); ++row) {

      QComboBox *comboBoxName =
          static_cast<QComboBox *>(indexWidget(model()->index(row, 0)));
      QLabel *label_status =
          static_cast<QLabel *>(indexWidget(model()->index(row, 1)));
      label_status->setText("Running");
      TopologyMap::PointInfo point =
          topologyMap_.GetPoint(comboBoxName->currentText().toStdString());
      RobotPose target_pose = point.ToRobotPose();
      emit signalSendNavGoal(target_pose);
      RobotPose diff = absoluteDifference(target_pose, robot_pose_);
      while (diff.mod() > 0.1 || diff.theta > deg2rad(5)) {
        LOG_INFO("Task chain is running diff:" << diff);
        diff = absoluteDifference(target_pose, robot_pose_);
        QThread::msleep(100);
      }
      label_status->setText("Finish");
      if (!is_task_chain_running_) {
        emit signalTaskFinish();
        LOG_INFO("Task chain is stopped");
        return;
      }
    }
  });
}
void NavGoalTableView::StopTaskChain() {
  if (is_task_chain_running_) {
    is_task_chain_running_ = false;
  }
}
void NavGoalTableView::UpdateRobotPose(const RobotPose &pose) {
  robot_pose_ = pose;
}
