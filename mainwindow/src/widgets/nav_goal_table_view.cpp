#include "widgets/nav_goal_table_view.h"
#include <QHeaderView>
NavGoalTableView::NavGoalTableView(QWidget *_parent_widget)
    : QTableView(_parent_widget) {
  db_table_model_ = new QStandardItemModel();
  setModel(db_table_model_);
  QStringList table_h_headers;
  table_h_headers << "点位名"
                  << "点位坐标"
                  << "任务类型"
                  << "任务状态";
  QHeaderView *header = new QHeaderView(Qt::Horizontal);
  header->setSectionResizeMode(QHeaderView::Interactive);
  header->setCascadingSectionResizes(false);
  this->setHorizontalHeader(header);
  // 添加数据模型
  db_table_model_->setHorizontalHeaderLabels(table_h_headers);
}

NavGoalTableView::~NavGoalTableView() {}