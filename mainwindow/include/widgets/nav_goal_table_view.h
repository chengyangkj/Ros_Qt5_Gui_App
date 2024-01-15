#include <QStandardItemModel>
#include <QTableView>

class NavGoalTableView : public QTableView {
public:
  explicit NavGoalTableView(QWidget *_parent_widget = nullptr);
  ~NavGoalTableView() override;

private:
  QStandardItemModel *db_table_model_;
};