/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-29 14:21:31
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 09:31:36
 * @FilePath:
 * ////include/display/display_manager.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H
#include "algorithm.h"
#include "display_cost_map.h"
#include "display_occ_map.h"
#include "display_path.h"
#include "factory_display.h"
#include "laser_points.h"
#include "msg/msg_info.h"
#include "point_shape.h"
#include "widgets/set_pose_widget.h"
#include <Eigen/Dense>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include <QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <any>
#include <functional>
#include <map>
#define DISPLAY_ROBOT ToString(MsgId::kRobotPose)
#define DISPLAY_MAP ToString(MsgId::kOccupancyMap)
#define DISPLAY_LOCAL_COST_MAP ToString(MsgId::kLocalCostMap)
#define DISPLAY_GLOBAL_COST_MAP ToString(MsgId::kGlobalCostMap)
#define DISPLAY_GLOBAL_PATH ToString(MsgId::kGlobalPath)
#define DISPLAY_LOCAL_PATH ToString(MsgId::kLocalPath)
#define DISPLAY_LASER ToString(MsgId::kLaserScan)
#define DISPLAY_PARTICLE "Particle"
#define DISPLAY_REGION "Region"
#define DISPLAY_TAG "Tag"
#define DISPLAY_GOAL "GoalPose"

// group
#define GROUP_MAP "Group_Map"
namespace Display {
class DisplayManager : public QObject {
  Q_OBJECT
private:
  std::map<std::string, std::any> display_map_;

  RobotPose robot_pose_{0, 0, 0};
  RobotPose robot_pose_goal_{0, 0, 0};
  OccupancyMap map_data_;
  std::string focus_display_;
  RobotPose local_cost_world_pose_;
  CostMap local_cost_map_;
  double global_scal_value_ = 1;
  bool is_reloc_mode_{false};
  QGraphicsView *graphics_view_ptr_;
  SetPoseWidget *set_reloc_pose_widget_;
  SetPoseWidget *set_nav_pose_widget_;
signals:
  void cursorPosMap(QPointF);
  void signalPub2DPose(const RobotPose &pose);
  void signalPub2DGoal(const RobotPose &pose);
public slots:
  void updateScaled(double value);
  void SetRelocPose();
  void SetNavPose();
  void slotRobotScenePoseChanged(const RobotPose &pose);
  void slotNavGoalScenePoseChanged(const RobotPose &pose);
  void slotSetRobotPose(const RobotPose &pose);
  void slotSetNavPose(const RobotPose &pose);
  void UpdateTopicData(const MsgId &id, const std::any &data);

private:
  void FocusDisplay(const std::string &display_name);
  void InitUi();
  std::vector<Point> transLaserPoint(const std::vector<Point> &point);
  QPushButton *btn_move_focus_;

public:
  DisplayManager(QGraphicsView *viewer);
  ~DisplayManager();
  VirtualDisplay *GetDisplay(const std::string &name);
  QPointF wordPose2Scene(const QPointF &point);
  RobotPose wordPose2Scene(const RobotPose &point);
  QPointF wordPose2Map(const QPointF &pose);
  RobotPose wordPose2Map(const RobotPose &pose);
  bool UpdateDisplay(const std::string &display_name, const std::any &data);
  void UpdateRobotPose(const RobotPose &pose);
  bool SetDisplayConfig(const std::string &config_name, const std::any &data);
  void SetRelocMode(bool is_move);
  void SetNavGoalMode(bool is_start);

  void SetFocusOn(const std::string &display_name) {
    focus_display_ = display_name;
  }
};

} // namespace Display

#endif
