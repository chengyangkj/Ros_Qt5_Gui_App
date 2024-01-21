#include "config/topology_map.h"
#include <QDebug>
#include <QHeaderView>
#include <QPainter>
#include <QStandardItemModel>
#include <QTableView>
class NavGoalTableView : public QTableView {
  Q_OBJECT
public:
  explicit NavGoalTableView(QWidget *_parent_widget = nullptr);
  ~NavGoalTableView() override;

private:
  QStandardItemModel *table_model_;
  TopologyMap topologyMap_;
public slots:
  void UpdateTopologyMap(const TopologyMap &_topology_map);
  void AddItem();
  void UpdateSelectPoint(const TopologyMap::PointInfo &);

private:
  void onItemChanged(QStandardItem *item);
};
