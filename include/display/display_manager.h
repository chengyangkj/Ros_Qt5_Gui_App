/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-03-29 14:21:31
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-07-27 13:45:27
 * @FilePath:
 * /hontai/src/tools/localizationViewer/include/display/display_manager.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H
#include <Eigen/Dense>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPushButton>
#include <any>
#include <functional>
#include <map>

#include "display_tag.h"
#include "laser_points.h"
#include "particle_points.h"
#include "point_shape.h"
#include "region.h"
#include "robot_map.h"
#define DISPLAY_ROBOT "Robot"
#define DISPLAY_MAP "OccupyMap"
#define DISPLAY_COST_MAP "CostMap"
#define DISPLAY_GLOBAL_COST_MAP "GlobalMap"
#define DISPLAY_GLOBAL_PATH "GlobalPath"
#define DISPLAY_LOCAL_PATH "LocalPath"
#define DISPLAY_LASER "LaserScan"
#define DISPLAY_PARTICLE "Particle"
#define DISPLAY_REGION "Region"
#define DISPLAY_TAG "Tag"
namespace Display {
class DisplayManager : public QObject {
  Q_OBJECT
 private:
  std::map<std::string, std::any> display_map_;
  QGraphicsScene* scene_ptr_;
  QGraphicsView* viewer_ptr_;
  std::function<VirtualDisplay::FactoryDisplay*()> DisplayInstance;
  Eigen::Vector3f robot_pose_{0, 0, 0};
  Eigen::Vector3f robot_pose_scene_;
  OccupancyMap map_data_;
  std::string focus_display_;
 signals:
  void cursorPosMap(QPointF);
  void cursorPosScene(QPointF);
  void robotPoseMap(Eigen::Vector3f pose);
  void signalPub2DPose(QPointF, QPointF);
  void signalPub2DGoal(QPointF, QPointF);
 public slots:
  void updateCoordinateSystem();
  void updateScaled(double value);
  void slotDisplayUpdated(std::string display_name);
  void slotDisplaySetScaled(std::string display_name, double value);
  void slotUpdateCursorPose(std::string, QPointF);
  void slotDisplayScenePoseChanged(std::string, QPointF);
  void start2DPose();
  void start2DGoal();

 private:
  Eigen::Vector2f transWord2Scene(const Eigen::Vector2f& point);
  void FocusDisplay(std::string display_name);
  void InitUi();
  QPushButton* btn_move_focus_;

 public:
  DisplayManager(QGraphicsView* viewer);
  ~DisplayManager();
  VirtualDisplay* GetDisplay(const std::string& name);
  bool UpdateDisplay(const std::string& display_name, const std::any& data);
  void UpdateRobotPose(const Eigen::Vector3f& pose);
  bool SetDisplayConfig(const std::string& config_name, const std::any& data);
  Eigen::Vector3f GetMapPoseInScene(const Eigen::Vector3f& pose);
  void SetMoveRobot(bool is_move);
  void SetFocusOn(std::string display_name) { focus_display_ = display_name; }
};

}  // namespace Display

#endif
