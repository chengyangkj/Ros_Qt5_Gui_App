#include <QDebug>
#include <QHeaderView>
#include <QPainter>
#include <QStandardItemModel>
#include <QTableView>
#include <mutex>
#include "config/task_chain.h"
#include "config/topology_map.h"
using namespace basic;
class NavGoalTableView : public QTableView {
  Q_OBJECT
 public:
  explicit NavGoalTableView(QWidget *_parent_widget = nullptr);
  ~NavGoalTableView() override;

 private:
  QStandardItemModel *table_model_;
  TopologyMap topologyMap_;
  RobotPose robot_pose_;
  std::atomic<bool> is_task_chain_running_;
  TaskChain task_chain_;
 public slots:
  void UpdateTopologyMap(const TopologyMap &_topology_map);
  void AddItem();
  void UpdateSelectPoint(const TopologyMap::PointInfo &);
  void StartTaskChain(bool is_loop);
  void StopTaskChain();
  void UpdateRobotPose(const RobotPose &pose);
  bool LoadTaskChain(const std::string &name);
  bool SaveTaskChain(const std::string &name);
 signals:
  void signalSendNavGoal(const RobotPose &pose);
  void signalTaskFinish();

 private:
  void onItemChanged(QStandardItem *item);
};
