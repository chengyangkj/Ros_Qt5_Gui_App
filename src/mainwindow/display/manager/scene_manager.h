#pragma once
#include <display/virtual_display.h>
#include <QCursor>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include "map/topology_map.h"

#include "display/point_shape.h"
#include "display/topology_line.h"
#include "widgets/nav_goal_widget.h"
#include "widgets/set_pose_widget.h"
namespace Display {

enum MapEditMode {
  kStop = 0,
  kNormal,        // 正常编辑模式
  kErase,         // 橡皮擦模式
  kDrawLine,      // 绘制线段模式
  kAddPoint,      // 添加点位模式
  kRegion,        // 添加区域模式
  kDrawWithPen,   //画笔
  kLinkTopology,  // 拓扑图
};

class DisplayManager;
class SceneManager : public QGraphicsScene {
  Q_OBJECT

 private:
  SetPoseWidget *set_nav_pose_widget_;
  QGraphicsView *view_ptr_;
  Display::VirtualDisplay *curr_handle_display_{nullptr};
  DisplayManager *display_manager_;
  NavGoalWidget *nav_goal_widget_;

  TopologyMap topology_map_;
  MapEditMode current_mode_;
  bool right_pressed_{false};
  bool left_pressed_{false};
  double eraser_range_{3};
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
  
 signals:
  void signalTopologyMapUpdate(const TopologyMap &map);
  void signalCurrentSelectPointChanged(const TopologyMap::PointInfo &);
 public slots:
  void SetEditMapMode(MapEditMode mode);
  void SaveTopologyMap(const std::string &file_path);

 public:
  SceneManager(QObject *parent = nullptr);
  virtual ~SceneManager();
  void Init(QGraphicsView *view_ptr, DisplayManager *manager);
  void AddOneNavPoint();
  void LoadTopologyMap();
  void OpenTopologyMap(const std::string &file_path);

 private:
  void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
  void wheelEvent(QGraphicsSceneWheelEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void saveTopologyMap();
  void blindNavGoalWidget(Display::VirtualDisplay *);
  void updateNavGoalWidgetPose( Display::VirtualDisplay *, bool is_move = true);
  std::string generatePointName(const std::string &prefix);
  void eraseScenePointRange(const QPointF &, double);
  void setEraseCursor();
  void drawPoint(const QPointF &);
  void SetPointMoveEnable(bool is_enable);
  void cleanupTopologyDisplays(const std::vector<std::string> &point_names);
  PointShape* createTopologyPointDisplay(const TopologyMap::PointInfo &point_info);
  
  // 拓扑连接相关方法
  void handleTopologyLinking(const QString &point_name);
  void createTopologyLine(const QString &from, const QString &to);
  void updateTopologyLinePositions();
  void clearTopologyLineSelection();
  void deleteSelectedTopologyLine();
  std::string generateRouteId(const QString &from, const QString &to);
  TopologyLine* findTopologyLine(const QString &route_id);
  void loadTopologyRoutes();
  void updateAllTopologyLinesBidirectionalStatus();
};
}  // namespace Display