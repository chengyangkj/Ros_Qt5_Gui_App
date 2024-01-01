// 1,图元坐标系 scenPose 对应所有图层的外部全局坐标系
// 2, 图层坐标系 每个图层的单独坐标系
// 3, 占栅格地图坐标系 occPose
// 4,机器人全局地图坐标系 wordPose
#include "display/display_manager.h"
#include "algorithm.h"
#include "logger/logger.h"
#include <Eigen/Eigen>
#include <QOpenGLWidget>

#include <fstream>
namespace Display {

DisplayManager::DisplayManager() {
  graphics_view_ptr_ = new QGraphicsView();
  FactoryDisplay::Instance()->Init(graphics_view_ptr_);

  //------------------------------------start display instace (register
  // display)-----------------------------
  (new DisplayOccMap(DISPLAY_MAP, 1));
  (new DisplayCostMap(DISPLAY_GLOBAL_COST_MAP, 2, DISPLAY_MAP));

  (new DisplayCostMap(DISPLAY_LOCAL_COST_MAP, 3, DISPLAY_MAP));
  (new PointShape(PointShape::ePointType::kRobot, DISPLAY_ROBOT, 7,
                  DISPLAY_MAP))
      ->SetRotateEnable(true);
  (new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL, 8,
                  DISPLAY_MAP))
      ->SetRotateEnable(true)
      ->SetMoveEnable(true)
      ->setVisible(false);
  new LaserPoints(DISPLAY_LASER, 2, DISPLAY_MAP);
  new DisplayPath(DISPLAY_GLOBAL_PATH, 6, DISPLAY_MAP);
  new DisplayPath(DISPLAY_LOCAL_PATH, 6, DISPLAY_MAP);

  // defalut display config

  SetDisplayConfig(DISPLAY_GLOBAL_PATH + "/Color", Color(0, 0, 255));
  SetDisplayConfig(DISPLAY_LOCAL_PATH + "/Color", Color(0, 255, 0));

  // connection

  connect(GetDisplay(DISPLAY_ROBOT),
          SIGNAL(signalPoseUpdate(const RobotPose &)), this,
          SLOT(slotRobotScenePoseChanged(const RobotPose &)));
  connect(GetDisplay(DISPLAY_GOAL), SIGNAL(signalPoseUpdate(const RobotPose &)),
          this, SLOT(slotNavGoalScenePoseChanged(const RobotPose &)));
  // 设置默认地图图层响应鼠标事件
  FactoryDisplay::Instance()->SetMoveEnable(DISPLAY_MAP);
  InitUi();
}
void DisplayManager::UpdateTopicData(const MsgId &id, const std::any &data) {
  UpdateDisplay(ToString(id), data);
}
void DisplayManager::slotSetRobotPose(const RobotPose &pose) {
  FactoryDisplay::Instance()->SetMoveEnable(DISPLAY_ROBOT, false);
  UpdateRobotPose(pose);
  // enable move after 300ms
  QTimer::singleShot(300, [this]() {
    FactoryDisplay::Instance()->SetMoveEnable(DISPLAY_ROBOT, true);
  });
}
void DisplayManager::slotSetNavPose(const RobotPose &pose) {
  FactoryDisplay::Instance()->SetMoveEnable(DISPLAY_GOAL, false);
  GetDisplay(DISPLAY_GOAL)->UpdateDisplay(wordPose2Map(pose));
  // enable move after 300ms
  QTimer::singleShot(300, [this]() {
    FactoryDisplay::Instance()->SetMoveEnable(DISPLAY_GOAL, true);
  });
}
void DisplayManager::slotRobotScenePoseChanged(const RobotPose &pose) {
  if (is_reloc_mode_) {
    QPointF occ_pose =
        GetDisplay(DISPLAY_MAP)->mapFromScene(QPointF(pose.x, pose.y));
    double x, y;
    map_data_.ScenePose2xy(occ_pose.x(), occ_pose.y(), x, y);
    // 更新坐标
    robot_pose_.x = x;
    robot_pose_.y = y;
    robot_pose_.theta = pose.theta;
    set_reloc_pose_widget_->SetPose(
        RobotPose(robot_pose_.x, robot_pose_.y, robot_pose_.theta));
  }
}
void DisplayManager::slotNavGoalScenePoseChanged(const RobotPose &pose) {
  QPointF occ_pose =
      GetDisplay(DISPLAY_MAP)->mapFromScene(QPointF(pose.x, pose.y));
  double x, y;
  map_data_.ScenePose2xy(occ_pose.x(), occ_pose.y(), x, y);
  robot_pose_goal_.x = x;
  robot_pose_goal_.y = y;
  robot_pose_goal_.theta = pose.theta;
  set_nav_pose_widget_->SetPose(RobotPose(
      robot_pose_goal_.x, robot_pose_goal_.y, robot_pose_goal_.theta));
}
void DisplayManager::InitUi() {
  set_reloc_pose_widget_ = new SetPoseWidget(graphics_view_ptr_);
  set_reloc_pose_widget_->hide();
  connect(set_reloc_pose_widget_, &SetPoseWidget::SignalHandleOver,
          [this](const bool &is_submit, const RobotPose &pose) {
            SetRelocMode(false);
            if (is_submit) {
              emit signalPub2DPose(pose);
            }
          });
  connect(set_reloc_pose_widget_, SIGNAL(SignalPoseChanged(const RobotPose &)),
          this, SLOT(slotSetRobotPose(const RobotPose &)));

  set_nav_pose_widget_ = new SetPoseWidget(graphics_view_ptr_);
  set_nav_pose_widget_->hide();
  connect(set_nav_pose_widget_, &SetPoseWidget::SignalHandleOver,
          [this](const bool &is_submit, const RobotPose &pose) {
            SetNavGoalMode(false);
            if (is_submit) {
              emit signalPub2DGoal(pose);
            }
          });
  connect(set_nav_pose_widget_, SIGNAL(SignalPoseChanged(const RobotPose &)),
          this, SLOT(slotSetNavPose(const RobotPose &)));
}
DisplayManager::~DisplayManager() {}

bool DisplayManager::SetDisplayConfig(const std::string &config_name,
                                      const std::any &data) {
  QString q_config_name = QString::fromStdString(config_name);
  auto config_list = q_config_name.split("/");
  if (config_list.empty() || config_list.size() != 2) {
    return false;
  }
  VirtualDisplay *display = GetDisplay(config_list[0].toStdString());
  if (!display) {
    std::cout << "error current display not fi csxnd:"
              << config_list[0].toStdString() << " config_name:" << config_name
              << std::endl;
    return false;
  }
  // 设置图层是否响应鼠标事件
  if (config_list[1] == "MouseEvent") {
    bool is_response;
    GetAnyData(bool, data, is_response);
    display->SetMoveEnable(is_response);
    std::cout << "config:" << config_name << " res:" << is_response
              << std::endl;
  }
  return display->SetDisplayConfig(config_list[1].toStdString(), data);
}
bool DisplayManager::UpdateDisplay(const std::string &display_name,
                                   const std::any &data) {
  // std::cout << "update display:" << display_name << std::endl;/
  VirtualDisplay *display = GetDisplay(display_name);
  if (!display) {
    // std::cout << "error current display not find on update:" << display_name
    //           << std::endl;
    return false;
  }
  if (display_name == DISPLAY_MAP) {
    display->UpdateDisplay(data);
    GetAnyData(OccupancyMap, data, map_data_);

    // 所有图层更新地图数据
    for (auto [name, display] :
         FactoryDisplay::Instance()->GetTotalDisplayMap()) {
      display->UpdateMap(map_data_);
    }
  } else if (display_name == DISPLAY_ROBOT) {
    //重定位时屏蔽位置更新
    if (!is_reloc_mode_) {
      GetAnyData(RobotPose, data, robot_pose_);
      UpdateRobotPose(robot_pose_);
    }
  } else if (display_name == DISPLAY_LASER) {
    LaserScan laser_scan;
    GetAnyData(LaserScan, data, laser_scan)
        // 点坐标转换为图元坐标系下

        laser_scan.data = transLaserPoint(laser_scan.data);

    display->UpdateDisplay(laser_scan);
  } else if (display_name == DISPLAY_GLOBAL_PATH ||
             display_name == DISPLAY_LOCAL_PATH) {
    // 激光坐标转换为地图的图元坐标
    RobotPath path_data;
    RobotPath path_data_trans;
    GetAnyData(RobotPath, data, path_data);
    for (auto one_points : path_data) {
      // std::cout << "location:" << one_laser.first << std::endl;
      // 转换为图元坐标系
      double x, y;
      map_data_.xy2ScenePose(one_points.x, one_points.y, x, y);
      path_data_trans.push_back(Point(x, y));
    }
    display->UpdateDisplay(path_data_trans);
  } else {
    display->UpdateDisplay(data);
  }
  return true;
}
/**
 * @description:坐标系转换为图元坐标系
 * @return {*}
 */
std::vector<Point>
DisplayManager::transLaserPoint(const std::vector<Point> &point) {
  // point为车身坐标系下的坐标 需要根据当前机器人坐标转换为map
  std::vector<Point> res;
  for (auto one_point : point) {
    //根据机器人坐标转换为map坐标系下
    basic::RobotPose map_pose = basic::absoluteSum(
        basic::RobotPose(robot_pose_.x, robot_pose_.y, robot_pose_.theta),
        basic::RobotPose(one_point.x, one_point.y, 0));
    // 转换为图元坐标系
    double x, y;
    map_data_.xy2ScenePose(map_pose.x, map_pose.y, x, y);
    res.push_back(Point(x, y));
  }
  return res;
}

/**
 * @description: 更新机器人在世界坐标系下的坐标
 * @param {Vector3f&} pose x y theta
 * @return {*}
 */
void DisplayManager::UpdateRobotPose(const RobotPose &pose) {
  robot_pose_ = pose;
  GetDisplay(DISPLAY_ROBOT)->UpdateDisplay(wordPose2Map(pose));
}

void DisplayManager::updateScaled(double value) {
  FactoryDisplay::Instance()->SetDisplayScaled(DISPLAY_LASER, value);
}
void DisplayManager::SetRelocMode(bool is_start) {
  is_reloc_mode_ = is_start;
  if (is_start) {
    set_reloc_pose_widget_->SetPose(
        RobotPose(robot_pose_.x, robot_pose_.y, robot_pose_.theta));
    set_reloc_pose_widget_->show();
  } else {
    set_reloc_pose_widget_->hide();
  }
  FactoryDisplay::Instance()->SetMoveEnable(DISPLAY_ROBOT, is_start);
}
void DisplayManager::SetNavGoalMode(bool is_start) {
  if (is_start) {
    slotSetNavPose(absoluteSum(robot_pose_,RobotPose(1,0,0)));
    GetDisplay(DISPLAY_GOAL)->setVisible(true);
    GetDisplay(DISPLAY_GOAL)->SetMoveEnable(true);
    set_nav_pose_widget_->show();
    FocusDisplay(DISPLAY_GOAL);
    QTimer::singleShot(100, this, [=]() { FocusDisplay(""); });
  } else {
    GetDisplay(DISPLAY_GOAL)->setVisible(false);
    set_nav_pose_widget_->hide();
  }
}
void DisplayManager::FocusDisplay(const std::string &display_name) {
  FactoryDisplay::Instance()->FocusDisplay(display_name);
}

/**
 * @description: 世界坐标系点转为全局scene坐标
 * @param {Vector2f&} point 传入的点坐标
 * @return {*}
 */
RobotPose DisplayManager::wordPose2Scene(const RobotPose &point) {
  // xy在栅格地图上的图元坐标
  double x, y;
  map_data_.xy2ScenePose(point.x, point.y, x, y);
  // xy在map图层上的坐标
  QPointF pose = FactoryDisplay::Instance()
                     ->GetDisplay(DISPLAY_MAP)
                     ->PoseToScene(QPointF(x, y));

  RobotPose res;
  res.x = pose.x();
  res.y = pose.y();
  res.theta = point.theta;
  return res;
}
/**
 * @description: 世界坐标系点转为以map图层为基准坐标的图元坐标
 * @param {Vector2f&} point 传入的点坐标
 * @return {*}
 */
QPointF DisplayManager::wordPose2Scene(const QPointF &point) {
  // xy在栅格地图上的图元坐标
  double x, y;
  map_data_.xy2ScenePose(point.x(), point.y(), x, y);
  return FactoryDisplay::Instance()
      ->GetDisplay(DISPLAY_MAP)
      ->PoseToScene(QPointF(x, y));
}
RobotPose DisplayManager::wordPose2Map(const RobotPose &pose) {
  RobotPose ret = pose;
  double x, y;
  map_data_.xy2ScenePose(pose.x, pose.y, x, y);
  ret.x = x;
  ret.y = y;
  return ret;
}
QPointF DisplayManager::wordPose2Map(const QPointF &pose) {
  QPointF ret;
  double x, y;
  map_data_.xy2ScenePose(pose.x(), pose.y(), x, y);
  ret.setX(x);
  ret.setY(y);
  return ret;
}
RobotPose DisplayManager::mapPose2Word(const RobotPose &pose) {
  RobotPose ret = pose;
  double x, y;
  map_data_.ScenePose2xy(pose.x, pose.y, x, y);
  ret.x = x;
  ret.y = y;
  return ret;
}
VirtualDisplay *DisplayManager::GetDisplay(const std::string &name) {
  return FactoryDisplay::Instance()->GetDisplay(name);
}
void DisplayManager::SetRelocPose() {
  if (!set_reloc_pose_widget_->isVisible()) {
    SetRelocMode(true);
  }
}
void DisplayManager::SetNavPose() {
  if (!set_nav_pose_widget_->isVisible()) {
    SetNavGoalMode(true);
  }
}

} // namespace Display