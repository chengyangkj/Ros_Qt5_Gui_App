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
#pragma once
#include "algorithm.h"

#include <Eigen/Dense>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include <QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <any>
#include <functional>
#include <map>
#include "config/config_manager.h"
#include "config/topology_map.h"
#include "display/manager/view_manager.h"
#include "display_cost_map.h"
#include "display_factory.h"
#include "display_occ_map.h"
#include "display_path.h"
#include "laser_points.h"
#include "point_shape.h"
#include "robot_shape.h"
#include "widgets/set_pose_widget.h"
// group
#define GROUP_MAP "Group_Map"
namespace Display {

class SceneManager;
class DisplayManager : public QObject {
  Q_OBJECT
 public:
 private:
  std::map<std::string, std::any> display_map_;

  RobotPose robot_pose_{0, 0, 0};
  RobotPose robot_pose_goal_{0, 0, 0};
  OccupancyMap map_data_;
  std::string focus_display_;
  RobotPose local_cost_world_pose_;
  OccupancyMap local_cost_map_;
  double global_scal_value_ = 1;
  bool is_reloc_mode_{false};
  ViewManager *graphics_view_ptr_;
  SetPoseWidget *set_reloc_pose_widget_;
  SceneManager *scene_manager_ptr_;
  bool init_flag_{false};
 signals:
  void cursorPosMap(QPointF);
  void signalPub2DPose(const RobotPose &pose);
  void signalPub2DGoal(const RobotPose &pose);
  void signalTopologyMapUpdate(const TopologyMap &map);
  void signalCurrentSelectPointChanged(const TopologyMap::PointInfo &);
  void signalPubMap(const OccupancyMap &map);
 public slots:
  void updateScaled(double value);
  void StartReloc();
  void SetEditMapMode(MapEditMode mode);
  void AddOneNavPoint();
  void slotRobotScenePoseChanged(const RobotPose &pose);
  void slotSetRobotPose(const RobotPose &pose);
  void UpdateTopicData(const MsgId &id, const std::any &data);
  void FocusDisplay(const std::string &display_type);
  void SaveMap(const std::string &save_path);
  void OpenMap(const std::string &save_path);
  void SetScaleBig();
  void SetScaleSmall();

 private:
  void InitUi();
  std::vector<Point> transLaserPoint(const std::vector<Point> &point);
  QPushButton *btn_move_focus_;

 public:
  DisplayManager();
  ~DisplayManager();
  QGraphicsView *GetViewPtr() { return graphics_view_ptr_; }
  VirtualDisplay *GetDisplay(const std::string &name);
  QPointF wordPose2Scene(const QPointF &point);
  RobotPose wordPose2Scene(const RobotPose &point);
  QPointF wordPose2Map(const QPointF &pose);
  RobotPose wordPose2Map(const RobotPose &pose);
  RobotPose mapPose2Word(const RobotPose &pose);
  RobotPose scenePoseToWord(const RobotPose &pose);
  RobotPose scenePoseToMap(const RobotPose &pose);
  bool UpdateDisplay(const std::string &display_type, const std::any &data);
  void UpdateRobotPose(const RobotPose &pose);
  bool SetDisplayConfig(const std::string &config_name, const std::any &data);
  void SetRelocMode(bool is_move);
  void SetNavGoalMode(bool is_start);
  OccupancyMap &GetMap();
  void UpdateMap(OccupancyMap &);
  RobotPose GetRobotPose() { return robot_pose_; }
  void SetFocusOn(const std::string &display_type) {
    focus_display_ = display_type;
  }
  void FocusOnMapCenter();
};

}  // namespace Display
