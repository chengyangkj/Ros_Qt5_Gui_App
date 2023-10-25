/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-03-29 14:21:31
 * @LastEditors: chengyangkj chengyangkj@qq.com
 * @LastEditTime: 2023-10-15 09:50:01
 * @FilePath:
 * ////src/display/display_manager.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// 1,图元坐标系 scenPose 对应所有图层的外部全局坐标系
// 2, 图层坐标系 每个图层的单独坐标系
// 3, 占栅格地图坐标系 occPose
// 4,机器人全局地图坐标系 wordPose
#include "display/display_manager.h"
#include "common/logger/logger.h"
#include <Eigen/Eigen>
#include <fstream>

namespace Display {

DisplayManager::DisplayManager(QGraphicsView *viewer) {
  FactoryDisplay::Instance()->Init(viewer);

  //------------------------------------start display instace (register
  // display)-----------------------------
  (new RobotMap(RobotMap::MapType::kOccupyMap, DISPLAY_MAP, 1));
  new RobotMap(RobotMap::MapType::kCostMap, DISPLAY_GLOBAL_COST_MAP, 2,
               DISPLAY_MAP);
  (new RobotMap(RobotMap::MapType::kCostMap, DISPLAY_LOCAL_COST_MAP, 3,
                DISPLAY_MAP));
  (new PointShape(PointShape::ePointType::kRobot, DISPLAY_ROBOT, 7,
                  DISPLAY_MAP))
      ->SetRotateEnable(true);
  new LaserPoints(DISPLAY_LASER, 2, DISPLAY_MAP);
  new ParticlePoints(DISPLAY_PARTICLE, 4, DISPLAY_MAP);
  new Region(DISPLAY_REGION, 3);
  new DisplayTag(DISPLAY_TAG, 4);
  new DisplayPath(DISPLAY_GLOBAL_PATH, 6, DISPLAY_MAP);
  new DisplayPath(DISPLAY_LOCAL_PATH, 6, DISPLAY_MAP);
  (new PointShape(PointShape::ePointType::kNavGoal, DISPLAY_GOAL, 8,
                  DISPLAY_MAP))
      ->SetRotateEnable(true)
      ->SetMouseEventEnable(true)
      ->setVisible(false);

  connect(GetDisplay(DISPLAY_GOAL),
          SIGNAL(signalPointScenePoseUpdate(Eigen::Vector3f)), this,
          SLOT(slotUpdateNavGoalScenePose(Eigen::Vector3f)));
  // defalut display config

  SetDisplayConfig(DISPLAY_GLOBAL_PATH "/Color", Display::Color(0, 0, 255));
  SetDisplayConfig(DISPLAY_LOCAL_PATH "/Color", Display::Color(0, 255, 0));

  // connection

  connect(GetDisplay(DISPLAY_ROBOT),
          SIGNAL(signalPointScenePoseUpdate(Eigen::Vector3f)), this,
          SLOT(slotUpdateRobotScenePose(Eigen::Vector3f)));

  // 设置默认地图图层响应鼠标事件
  FactoryDisplay::Instance()->SetEnableMosuleEvent(DISPLAY_MAP);
  InitUi();
}
void DisplayManager::slotUpdateRobotScenePose(Eigen::Vector3f pose) {
  if (is_reloc_mode_) {
    QPointF occ_pose =
        GetDisplay(DISPLAY_MAP)->mapFromScene(QPointF(pose[0], pose[1]));
    double x, y;
    map_data_.occPose2xy(occ_pose.x(), occ_pose.y(), x, y);
    Eigen::Vector3f robot_pose_new;
    // 更新坐标
    robot_pose_new[0] = x;
    robot_pose_new[1] = y;
    robot_pose_new[2] = robot_pose_reloc_init_[2] - deg2rad(pose[2]);
    UpdateRobotPose(robot_pose_new);
  }
}
void DisplayManager::slotUpdateNavGoalScenePose(Eigen::Vector3f pose) {
  QPointF occ_pose =
      GetDisplay(DISPLAY_MAP)->mapFromScene(QPointF(pose[0], pose[1]));
  double x, y;
  map_data_.occPose2xy(occ_pose.x(), occ_pose.y(), x, y);
  robot_pose_goal_[0] = x;
  robot_pose_goal_[1] = y;
  robot_pose_goal_[2] = 0 - deg2rad(pose[2]);
  // FactoryDisplay::Instance()->SetDisplayPoseInParent(
  //     DISPLAY_GOAL, DISPLAY_MAP,
  //     Eigen::Vector3f(occ_pose.x(), occ_pose.y(), 0));
}
void DisplayManager::InitUi() {
  // 跟随车体移动的按钮
}
DisplayManager::~DisplayManager() {}

void DisplayManager::slotUpdateCursorPose(std::string name, QPointF pose) {
  if (name == DISPLAY_MAP) {
    int scene_x = pose.x();
    int scene_y = pose.y();
    emit cursorPosScene(QPointF(scene_x, scene_y));
    double x, y;
    map_data_.occPose2xy(scene_x, scene_y, x, y);
    emit cursorPosMap(QPointF(x, y));
  }
}
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
    display->SetEnableMosuleEvent(is_response);
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
      int x, y;
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
      int x, y;
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
        int xmin, ymin, xmax, ymax;
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
    int x, y;
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
  emit DisplayRobotPoseWorld(pose);
  robot_pose_ = pose;
  // 地图图层 更新机器人图元坐标
  robot_pose_scene_ = wordPose2Scene(pose);

  if (!is_reloc_mode_) {
    GetDisplay(DISPLAY_ROBOT)->UpdateDisplay(robot_pose_);
    FactoryDisplay::Instance()->SetDisplayPoseInParent(DISPLAY_ROBOT,
                                                       wordPose2Map(pose));
  }
}

void DisplayManager::updateScaled(double value) {
  FactoryDisplay::Instance()->SetDisplayScaled(DISPLAY_LASER, value);
}
void DisplayManager::SetMoveRobot(bool is_move) {
  is_reloc_mode_ = is_move;
  if (is_move) {
    robot_pose_reloc_init_ = robot_pose_;
  } else {
    emit signalPub2DPose(robot_pose_);
  }
  FactoryDisplay::Instance()->SetEnableMosuleEvent(DISPLAY_ROBOT, is_move);
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
  int x, y;
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
  int x, y;
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
  int x, y;
  map_data_.xy2occPose(pose[0], pose[1], x, y);
  ret[0] = x;
  ret[1] = y;
  ret[2] = pose[2];
  return ret;
}
QPointF DisplayManager::wordPose2Map(const QPointF &pose) {
  QPointF ret;
  int x, y;
  map_data_.xy2occPose(pose.x(), pose.y(), x, y);
  ret.setX(x);
  ret.setY(y);
  return ret;
}
VirtualDisplay *DisplayManager::GetDisplay(const std::string &name) {
  return FactoryDisplay::Instance()->GetDisplay(name);
}
void DisplayManager::start2DPose(const bool &is_start) {
  SetMoveRobot(is_start);
}
void DisplayManager::start2DGoal(const bool &is_start) {
  if (is_start) {
    GetDisplay(DISPLAY_GOAL)->setPos(wordPose2Map(QPointF(0, 0)));
    GetDisplay(DISPLAY_GOAL)->setVisible(true);
  } else {
    GetDisplay(DISPLAY_GOAL)->setVisible(false);
    emit signalPub2DGoal(robot_pose_goal_);
  }
}

} // namespace Display