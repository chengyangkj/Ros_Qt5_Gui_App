#pragma once
#include <display/virtual_display.h>
#include <QCursor>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <memory>
#include <vector>
#include "map/topology_map.h"

#include "display/point_shape.h"
#include "display/topology_line.h"
#include "widgets/nav_goal_widget.h"
#include "widgets/set_pose_widget.h"
#include "widgets/topology_route_widget.h"
#include "display/manager/map_edit_command.h"

namespace Display {

enum MapEditMode {
  kStopEdit = 0,
  kMoveCursor,        // 正常编辑模式
  kErase,         // 橡皮擦模式
  kDrawLine,      // 绘制线段模式
  kAddPoint,      // 添加点位模式
  kRegion,        // 添加区域模式
  kDrawWithPen,   //画笔
  kLinkTopology,  // 拓扑图
};

class DisplayManager;
class DisplayOccMap;
class SceneManager : public QGraphicsScene {
  Q_OBJECT

 private:
  SetPoseWidget *set_nav_pose_widget_;
  QGraphicsView *view_ptr_;
  Display::VirtualDisplay *curr_handle_display_{nullptr};
  DisplayManager *display_manager_;
  std::unique_ptr<NavGoalWidget> nav_goal_widget_;
  std::unique_ptr<TopologyRouteWidget> topology_route_widget_;

  TopologyMap topology_map_;
  MapEditMode current_mode_;
  bool right_pressed_{false};
  bool left_pressed_{false};
  double pen_range_{0.5};
  QCursor eraser_cursor_;
  QCursor move_cursor_;
  QCursor line_cursor_;
  QCursor nav_goal_cursor_;
  QCursor rect_cursor_;
  QCursor region_cursor_;
  QCursor pen_cursor_;
  
  // 拓扑连接相关
  QString first_selected_point_;
  bool is_linking_mode_{false};
  std::vector<TopologyLine*> topology_lines_;
  TopologyLine* selected_topology_line_{nullptr};
  
  // 预览线段相关
  TopologyLine* preview_line_{nullptr};
  QPointF first_point_pos_;
  bool is_drawing_line_{false};
  QPointF line_start_pose_;
  
  // 点位移动跟踪（用于撤销）
  std::map<std::string, TopologyMap::PointInfo> point_move_start_positions_;
  
  // 擦除/绘制操作跟踪（用于撤销）
  bool is_erase_operation_active_{false};
  bool is_draw_point_operation_active_{false};
  bool is_draw_line_operation_active_{false};
  QRectF erase_operation_region_;
  QImage erase_operation_saved_image_;
  QRectF draw_point_operation_region_;
  QImage draw_point_operation_saved_image_;
  QImage draw_line_operation_saved_image_;
  DisplayOccMap* current_map_ptr_{nullptr};
  
  // 撤销/重做系统
  std::vector<std::unique_ptr<MapEditCommand>> command_history_;
  size_t command_history_index_{0};
  static constexpr size_t kMaxHistorySize = 50;
  
  signals:
  void signalTopologyMapUpdate(const TopologyMap &map);
  void signalCurrentSelectPointChanged(const TopologyMap::PointInfo &);
  void signalEditMapModeChanged(MapEditMode mode);
 public slots:
  void SetEditMapMode(MapEditMode mode);
  void SetToolRange(double range);
  double GetEraserRange() const { return pen_range_; }
  double GetPenRange() const { return pen_range_; }

 public:
  SceneManager(QObject *parent = nullptr);
  virtual ~SceneManager();
  void Init(QGraphicsView *view_ptr, DisplayManager *manager);
  void AddOneNavPoint();
  void AddPointAtRobotPosition();
  void OpenTopologyMap(const std::string &file_path);
  void UpdateTopologyMap(const TopologyMap &topology_map);
  TopologyMap GetTopologyMap() { return topology_map_; }

 private:
  void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
  void wheelEvent(QGraphicsSceneWheelEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void blindNavGoalWidget(Display::VirtualDisplay *, bool is_edit = false);
  void updateNavGoalWidgetPose( Display::VirtualDisplay *, bool is_move = true);
  void blindTopologyRouteWidget(TopologyLine* line, bool is_edit = false);
  std::string generatePointName(const std::string &prefix);
  void eraseScenePointRange(const QPointF &, double);
  void drawScenePointRange(const QPointF &, double);
  void setEraseCursor();
  void setPenCursor();
  void drawPoint(const QPointF &);
  void SetPointMoveEnable(bool is_enable);
  void cleanupTopologyDisplays(const std::vector<std::string> &point_names);
  PointShape* createTopologyPointDisplay(const TopologyMap::PointInfo &point_info);
  
  // 拓扑连接相关方法
  void handleTopologyLinking(const QString &point_name);
  void createTopologyLine(const QString &from, const QString &to);
  void clearTopologyLineSelection();
  void deleteSelectedTopologyLine();
  std::string generateRouteId(const QString &from, const QString &to);
  TopologyLine* findTopologyLine(const QString &route_id);
  void loadTopologyRoutes();
  void updateAllTopologyLinesStatus();
  
  // 撤销/重做相关
  void PushCommand(std::unique_ptr<MapEditCommand> command);
  void Undo();
  void Redo();
  void ClearCommandHistory();
  
  // 供命令类访问的友元声明
  friend class EraseCommand;
  friend class DrawPointCommand;
  friend class DrawLineCommand;
  friend class AddPointCommand;
  friend class RemovePointCommand;
  friend class AddTopologyLineCommand;
  friend class RemoveTopologyLineCommand;
  friend class UpdatePointCommand;
  friend class UpdatePointNameCommand;
};
}  // namespace Display