#include "widgets/nav_goal_table_view.h"
#include <QComboBox>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
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
  QModelIndexList selectedIndexes = selectionModel()->selectedRows();
  if (selectedIndexes.size() == 1) {
    QModelIndex modelIndex = model()->index(selectedIndexes[0].row(), 0);
    QWidget *widget = indexWidget(modelIndex);

    if (widget) {
      QComboBox *comboBox = static_cast<QComboBox *>(widget);
      comboBox->setCurrentText(point.name.c_str());
    }
  }
}
void NavGoalTableView::AddItem() {
  std::cout << "add item" << std::endl;

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
