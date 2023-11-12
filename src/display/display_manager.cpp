// 1,图元坐标系 scenPose 对应所有图层的外部全局坐标系
// 2, 图层坐标系 每个图层的单独坐标系
// 3, 占栅格地图坐标系 occPose
// 4,机器人全局地图坐标系 wordPose
#include "display/display_manager.h"
#include "basic/algorithm.h"
#include "common/logger/logger.h"
#include <Eigen/Eigen>
#include <fstream>
namespace Display {

DisplayManager::DisplayManager(QGraphicsView *viewer)
    : graphics_view_ptr_(viewer) {
  FactoryDisplay::Instance()->Init(viewer);

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
  new ParticlePoints(DISPLAY_PARTICLE, 4, DISPLAY_MAP);
  new Region(DISPLAY_REGION, 3);
  new DisplayTag(DISPLAY_TAG, 4);
  new DisplayPath(DISPLAY_GLOBAL_PATH, 6, DISPLAY_MAP);
  new DisplayPath(DISPLAY_LOCAL_PATH, 6, DISPLAY_MAP);

  // defalut display config

  SetDisplayConfig(DISPLAY_GLOBAL_PATH "/Color", Display::Color(0, 0, 255));
  SetDisplayConfig(DISPLAY_LOCAL_PATH "/Color", Display::Color(0, 255, 0));

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
void DisplayManager::slotSetRobotPose(const RobotPose &pose) {
  FactoryDisplay::Instance()->SetMoveEnable(DISPLAY_ROBOT, false);
  UpdateRobotPose(Eigen::Vector3f(pose.x, pose.y, pose.theta));
  // enable move after 300ms
  QTimer::singleShot(300, [this]() {
    FactoryDisplay::Instance()->SetMoveEnable(DISPLAY_ROBOT, true);
  });
}
void DisplayManager::slotSetNavPose(const RobotPose &pose) {
  FactoryDisplay::Instance()->SetMoveEnable(DISPLAY_GOAL, false);
  GetDisplay(DISPLAY_GOAL)
      ->UpdateDisplay(
          wordPose2Map(Eigen::Vector3f(pose.x, pose.y, pose.theta)));
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
    map_data_.occPose2xy(occ_pose.x(), occ_pose.y(), x, y);
    // 更新坐标
    robot_pose_[0] = x;
    robot_pose_[1] = y;
    robot_pose_[2] =  pose.theta;

    set_reloc_pose_widget_->SetPose(
        RobotPose(robot_pose_[0], robot_pose_[1], robot_pose_[2]));
  }
}
void DisplayManager::slotNavGoalScenePoseChanged(const RobotPose &pose) {
  QPointF occ_pose =
      GetDisplay(DISPLAY_MAP)->mapFromScene(QPointF(pose.x, pose.y));
  double x, y;
  map_data_.occPose2xy(occ_pose.x(), occ_pose.y(), x, y);
  robot_pose_goal_[0] = x;
  robot_pose_goal_[1] = y;
  robot_pose_goal_[2] = pose.theta;
  set_nav_pose_widget_->SetPose(
      RobotPose(robot_pose_goal_[0], robot_pose_goal_[1], robot_pose_goal_[2]));
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
      GetAnyData(Pose3f, data, robot_pose_);
      UpdateRobotPose(robot_pose_);
    }
  } else if (display_name == DISPLAY_LASER) {
    Display::LaserDataMap laser_data_map, laser_data_scene;
    GetAnyData(Display::LaserDataMap, data, laser_data_map)
        // 点坐标转换为图元坐标系下
        for (auto one_laser : laser_data_map) {
      laser_data_scene[one_laser.first] = transLaserPoint(one_laser.second);
    }

    display->UpdateDisplay(laser_data_scene);
  } else if (display_name == DISPLAY_PARTICLE) {
    // 激光坐标转换为地图的图元坐标
    Display::ParticlePointsType particles;
    Display::ParticlePointsType particles_tans;
    GetAnyData(Display::ParticlePointsType, data, particles);
    for (auto one_points : particles) {
      // std::cout << "location:" << one_laser.first << std::endl;
      // 转换为图元坐标系
      double x, y;
      map_data_.xy2occPose(one_points[0], one_points[1], x, y);
      particles_tans.push_back(Eigen::Vector3f(x, y, one_points[2]));
    }
    display->UpdateDisplay(particles_tans);
  } else if (display_name == DISPLAY_GLOBAL_PATH ||
             display_name == DISPLAY_LOCAL_PATH) {
    // 激光坐标转换为地图的图元坐标
    Display::PathData path_data;
    Display::PathData path_data_trans;
    GetAnyData(Display::PathData, data, path_data);
    for (auto one_points : path_data) {
      // std::cout << "location:" << one_laser.first << std::endl;
      // 转换为图元坐标系
      double x, y;
      map_data_.xy2occPose(one_points[0], one_points[1], x, y);
      path_data_trans.push_back(Display::Point2f(x, y));
    }
    display->UpdateDisplay(path_data_trans);
  } else if (display_name == DISPLAY_REGION) {
    Display::RegionDataMap region_data;
    Display::RegionDataMap region_tans;
    GetAnyData(Display::RegionDataMap, data, region_data);
    for (auto [region_name, region] : region_data) {
      std::vector<Display::RangeVec> range_ve;
      for (auto one_region : region) {
        Display::RangeVec ivec;
        double xmin, ymin, xmax, ymax;
        map_data_.xy2occPose(one_region[0], one_region[1], xmin, ymin);
        map_data_.xy2occPose(one_region[2], one_region[3], xmax, ymax);
        ivec[0] = xmin;
        ivec[1] = ymin;
        ivec[2] = xmax;
        ivec[3] = ymax;
        range_ve.push_back(ivec);
      }
      region_tans[region_name] = range_ve;
    }
    display->UpdateDisplay(region_tans);
  } else if (display_name == DISPLAY_LOCAL_COST_MAP) {
    if (data.type() == typeid(RobotPose)) {
      GetAnyData(RobotPose, data, local_cost_world_pose_);
    } else if (data.type() == typeid(CostMap)) {
      GetAnyData(CostMap, data, local_cost_map_);
      display->UpdateDisplay(data);
    } else {
      display->UpdateDisplay(data);
    }
    FactoryDisplay::Instance()->SetDisplayPoseInParent(
        DISPLAY_LOCAL_COST_MAP,
        wordPose2Map(Eigen::Vector3f(local_cost_world_pose_.x,
                                     local_cost_world_pose_.y,
                                     local_cost_world_pose_.theta)));
  } else {
    display->UpdateDisplay(data);
  }
  return true;
}
/**
 * @description:坐标系转换为图元坐标系
 * @return {*}
 */
std::vector<Eigen::Vector2f>
DisplayManager::transLaserPoint(const std::vector<Eigen::Vector2f> &point) {
  // point为车身坐标系下的坐标 需要根据当前机器人坐标转换为map
  std::vector<Eigen::Vector2f> res;
  for (auto one_point : point) {
    //根据机器人坐标转换为map坐标系下
    basic::RobotPose map_pose = basic::absoluteSum(
        basic::RobotPose(robot_pose_[0], robot_pose_[1], robot_pose_[2]),
        basic::RobotPose(one_point[0], one_point[1], 0));
    // 转换为图元坐标系
    double x, y;
    map_data_.xy2occPose(map_pose.x, map_pose.y, x, y);
    res.push_back(Eigen::Vector2f(x, y));
  }
  return res;
}

/**
 * @description: 更新机器人在世界坐标系下的坐标
 * @param {Vector3f&} pose x y theta
 * @return {*}
 */
void DisplayManager::UpdateRobotPose(const Eigen::Vector3f &pose) {
  // FocusDisplay(DISPLAY_ROBOT);
  emit DisplayRobotPoseWorld(pose);
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
        RobotPose(robot_pose_[0], robot_pose_[1], robot_pose_[2]));
    set_reloc_pose_widget_->show();
  } else {
    set_reloc_pose_widget_->hide();
  }
  FactoryDisplay::Instance()->SetMoveEnable(DISPLAY_ROBOT, is_start);
}
void DisplayManager::SetNavGoalMode(bool is_start) {
  if (is_start) {
    slotSetNavPose(RobotPose(0, 0, 0));
    GetDisplay(DISPLAY_GOAL)->setVisible(true);
    GetDisplay(DISPLAY_GOAL)->SetMoveEnable(true);
    set_nav_pose_widget_->show();
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
Eigen::Vector3f DisplayManager::wordPose2Scene(const Eigen::Vector3f &point) {
  // xy在栅格地图上的图元坐标
  double x, y;
  map_data_.xy2occPose(point[0], point[1], x, y);
  // xy在map图层上的坐标
  QPointF pose = FactoryDisplay::Instance()
                     ->GetDisplay(DISPLAY_MAP)
                     ->PoseToScene(QPointF(x, y));

  Eigen::Vector3f res;
  res[0] = pose.x();
  res[1] = pose.y();
  res[2] = point[2];
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
  map_data_.xy2occPose(point.x(), point.y(), x, y);
  return FactoryDisplay::Instance()
      ->GetDisplay(DISPLAY_MAP)
      ->PoseToScene(QPointF(x, y));
}
/**
 * @description: 世界坐标系点转为以map图层为栅格地图坐标系
 * @param {Vector2f&} point 传入的点坐标
 * @return {*}
 */
Eigen::Vector3f DisplayManager::wordPose2Map(const Eigen::Vector3f &pose) {
  Eigen::Vector3f ret;
  double x, y;
  map_data_.xy2occPose(pose[0], pose[1], x, y);
  ret[0] = x;
  ret[1] = y;
  ret[2] = pose[2];
  return ret;
}
QPointF DisplayManager::wordPose2Map(const QPointF &pose) {
  QPointF ret;
  double x, y;
  map_data_.xy2occPose(pose.x(), pose.y(), x, y);
  ret.setX(x);
  ret.setY(y);
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