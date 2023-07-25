/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-03-29 14:21:31
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-05-17 16:31:13
 * @FilePath:
 * /hontai/src/tools/localizationViewer/src/display/display_manager.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "display/display_manager.h"

#include <Eigen/Eigen>
DisplayManager::DisplayManager(QGraphicsView* viewer)
    : viewer_ptr_(viewer),
      DisplayInstance(VirtualDisplay::FactoryDisplay::Instance) {
  // 初始化场景类
  scene_ptr_ = new QGraphicsScene();
  scene_ptr_->clear();
  // 初始化item
  viewer_ptr_->setScene(scene_ptr_);
  viewer_ptr_->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  new RobotMap(RobotMap::MapType::kOccupyMap, DISPLAY_MAP, 1);
  new PointShape(PointShape::ePointType::kRobot, DISPLAY_ROBOT, 5);
  new LaserPoints(DISPLAY_LASER, 2);
  new ParticlePoints(DISPLAY_PARTICLE, 4);
  new Region(DISPLAY_REGION, 3);
  new DisplayTag(DISPLAY_TAG, 4);
  for (auto [name, display] : DisplayInstance()->GetDisplayMap()) {
    scene_ptr_->addItem(display);
    // //图层更新时间
    connect(display, SIGNAL(displayUpdated(std::string)), this,
            SLOT(slotDisplayUpdated(std::string)));
    // scene 坐标改变事件
    connect(display, SIGNAL(scenePoseChanged(std::string, QPointF)), this,
            SLOT(slotDisplayScenePoseChanged(std::string, QPointF)));
    // 图层放大缩小事件
    connect(display, SIGNAL(displaySetScaled(std::string, double)), this,
            SLOT(slotDisplaySetScaled(std::string, double)));
    connect(display, SIGNAL(updateCursorPose(std::string, QPointF)), this,
            SLOT(slotUpdateCursorPose(std::string, QPointF)));
  }
  // 设置默认地图图层响应鼠标事件
  DisplayInstance()->SetMouseHandleDisplay(DISPLAY_MAP);
  std::cout << "display size:" << DisplayInstance()->GetDisplaySize()
            << std::endl;
  InitUi();
}
void DisplayManager::InitUi() {
  // 跟随车体移动的按钮
  QPushButton* btn_move_focus_ = new QPushButton(viewer_ptr_);
  btn_move_focus_->resize(32, 32);
  btn_move_focus_->setStyleSheet(
      "background-image:url(://images/robot_track_on.png);");
  btn_move_focus_->setFlat(true);
}
DisplayManager::~DisplayManager() {}
void DisplayManager::slotDisplayUpdated(std::string display_name) {
  // 不响应主图层的事件
  if (DisplayInstance()->GetMouseHandleDisplay() != display_name) return;
  // 其他所有图层update
  for (auto [name, display] : DisplayInstance()->GetDisplayMap()) {
    if (name != display_name) {
      display->Update();
    }
  }
  updateCoordinateSystem();
}
/**
 * @description: 图层在图元坐标系下的坐标改变的事件
 * @return {*}
 */
void DisplayManager::slotDisplayScenePoseChanged(std::string display_name,
                                                 QPointF pose) {
  // 不响应主图层的事件
  if (DisplayInstance()->GetMouseHandleDisplay() != display_name) return;
  if (display_name == DISPLAY_ROBOT) {
    // 机器人的图元坐标转世界坐标
    QPointF map_scene_pose = GetDisplay(DISPLAY_MAP)->mapFromScene(pose);
    double x, y;
    map_data_.scene2xy(map_scene_pose.x(), map_scene_pose.y(), x, y);
    Eigen::Vector3f robot_pose_new;
    // 更新坐标
    robot_pose_new[0] = x;
    robot_pose_new[1] = y;
    robot_pose_new[2] = robot_pose_[2];
    UpdateRobotPose(robot_pose_new);
  }
}
void DisplayManager::slotUpdateCursorPose(std::string name, QPointF pose) {
  if (name == DISPLAY_MAP) {
    int scene_x = pose.x();
    int scene_y = pose.y();
    emit cursorPosScene(QPointF(scene_x, scene_y));
    double x, y;
    map_data_.scene2xy(scene_x, scene_y, x, y);
    emit cursorPosMap(QPointF(x, y));
  }
}
void DisplayManager::slotDisplaySetScaled(std::string display_name,
                                          double value) {
  // 只响应主图层的事件
  if (DisplayInstance()->GetMouseHandleDisplay() != display_name) return;
  // 其他所有图层scaled
  for (auto [name, display] : DisplayInstance()->GetDisplayMap()) {
    if (name != display_name) {
      display->SetScaled(value);
    }
  }
  // std::cout << "scale:" << value << std::endl;
  // 地图0 0点在view 的坐标
  // QPointF map_zero_view_scene_pose =
  //     DisplayInstance()->GetDisplay(DISPLAY_MAP)->mapToScene(0, 0);
  // DisplayInstance()->SetDisplayScenePose(DISPLAY_TAG,
  //                                        map_zero_view_scene_pose * value);
}
bool DisplayManager::SetDisplayConfig(const std::string& config_name,
                                      const std::any& data) {
  QString q_config_name = QString::fromStdString(config_name);
  auto config_list = q_config_name.split("/");
  if (config_list.empty() || config_list.size() != 2) {
    return false;
  }
  VirtualDisplay* display = GetDisplay(config_list[0].toStdString());
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
    display->SetResposeMouseEvent(is_response);
    std::cout << "config:" << config_name << " res:" << is_response
              << std::endl;
  }
  return display->SetDisplayConfig(config_list[1].toStdString(), data);
}
bool DisplayManager::UpdateDisplay(const std::string& display_name,
                                   const std::any& data) {
  VirtualDisplay* display = GetDisplay(display_name);
  if (!display) {
    std::cout << "error current display not find on update:" << display_name
              << std::endl;
    return false;
  }
  if (display_name == DISPLAY_MAP) {
    display->UpdateDisplay(data);
    GetAnyData(OccupancyMap, data, map_data_);
    GetDisplay(DISPLAY_LASER)->UpdateDisplay(data);
    // 所有图层更新地图数据
    for (auto [name, display] : DisplayInstance()->GetDisplayMap()) {
      display->UpdateMap(map_data_);
    }
  } else if (display_name == DISPLAY_ROBOT) {
    display->UpdateDisplay(data);
    GetAnyData(Eigen::Vector3f, data, robot_pose_);
    UpdateRobotPose(robot_pose_);

  } else if (display_name == DISPLAY_LASER) {
    display->UpdateDisplay(map_data_);
    display->UpdateDisplay(robot_pose_);
    display->UpdateDisplay(data);
  } else if (display_name == DISPLAY_PARTICLE) {
    // 激光坐标转换为地图的图元坐标
    Display::ParticlePointsType particles;
    Display::ParticlePointsType particles_tans;
    GetAnyData(Display::ParticlePointsType, data, particles);
    for (auto one_points : particles) {
      // std::cout << "location:" << one_laser.first << std::endl;
      // 转换为图元坐标系
      int x, y;
      map_data_.xy2scene(one_points[0], one_points[1], x, y);
      particles_tans.push_back(Eigen::Vector3f(x, y, one_points[2]));
    }
    display->UpdateDisplay(particles_tans);
  } else if (display_name == DISPLAY_REGION) {
    Display::RegionDataMap region_data;
    Display::RegionDataMap region_tans;
    GetAnyData(Display::RegionDataMap, data, region_data);
    for (auto [region_name, region] : region_data) {
      std::vector<Display::RangeVec> range_ve;
      for (auto one_region : region) {
        Display::RangeVec ivec;
        int xmin, ymin, xmax, ymax;
        map_data_.xy2scene(one_region[0], one_region[1], xmin, ymin);
        map_data_.xy2scene(one_region[2], one_region[3], xmax, ymax);
        ivec[0] = xmin;
        ivec[1] = ymin;
        ivec[2] = xmax;
        ivec[3] = ymax;
        range_ve.push_back(ivec);
      }
      region_tans[region_name] = range_ve;
    }
    display->UpdateDisplay(region_tans);
  } else {
    display->UpdateDisplay(data);
  }
  FocusDisplay(focus_display_);
  return true;
}
/**
 * @description: 传入一个世界坐标,得到该坐标的在GriphicsScene的图元坐标
 * @param {Vector3f&} pose
 * @return {*}
 */
Eigen::Vector3f DisplayManager::GetMapPoseInScene(const Eigen::Vector3f& pose) {
  int x, y;
  map_data_.xy2scene(pose[0], pose[1], x, y);
  QPointF scene_pose = GetDisplay(DISPLAY_MAP)->mapToScene(x, y);
  Eigen::Vector3f scene_pose_e;
  scene_pose_e[0] = scene_pose.x();
  scene_pose_e[1] = scene_pose.y();
  scene_pose_e[2] = pose[2];
  return scene_pose_e;
}
/**
 * @description: 更新机器人在世界坐标系下的坐标
 * @param {Vector3f&} pose x y theta
 * @return {*}
 */
void DisplayManager::UpdateRobotPose(const Eigen::Vector3f& pose) {
  emit robotPoseMap(pose);
  robot_pose_ = pose;
  // 地图图层 更新机器人图元坐标
  int x, y;
  map_data_.xy2scene(robot_pose_[0], robot_pose_[1], x, y);
  robot_pose_scene_[0] = x;
  robot_pose_scene_[1] = y;
  robot_pose_scene_[2] = robot_pose_[2];
  GetDisplay(DISPLAY_LASER)->UpdateDisplay(pose);
  GetDisplay(DISPLAY_MAP)->SetDisplayConfig("RobotPose", robot_pose_scene_);

  // 更新图元之间的坐标系
  updateCoordinateSystem();
  // 设置移动时的跟随焦点
}
void DisplayManager::updateScaled(double value) {
  DisplayInstance()->SetDisplayScaled(DISPLAY_LASER, value);
}
void DisplayManager::SetMoveRobot(bool is_move) {
  if (is_move) {
    DisplayInstance()->SetMouseHandleDisplay(DISPLAY_ROBOT);
  } else {
    DisplayInstance()->SetMouseHandleDisplay(DISPLAY_MAP);
  }
}
void DisplayManager::FocusDisplay(std::string display_name) {
  auto display = GetDisplay(display_name);
  if (display != nullptr) {
    viewer_ptr_->centerOn(display);
  }
}
/**
 * @description: 更新图层间的坐标系关系
 * @return {*}
 */
void DisplayManager::updateCoordinateSystem() {
  // Robot
  {
    auto scene_pose =
        transWord2Scene(Eigen::Vector2f(robot_pose_[0], robot_pose_[1]));
    QPointF pose = QPointF(scene_pose[0], scene_pose[1]);
    DisplayInstance()->SetDisplayScenePose(DISPLAY_ROBOT, pose);

    // 地图0 0点在view 的坐标
    QPointF map_zero_view_scene_pose =
        DisplayInstance()->GetDisplay(DISPLAY_MAP)->mapToScene(0, 0);
    // 图层对齐
    for (auto [name, display] : DisplayInstance()->GetDisplayMap()) {
      if (name == DISPLAY_ROBOT) continue;
      DisplayInstance()->SetDisplayScenePose(name, map_zero_view_scene_pose);
    }
  }
}

/**
 * @description: 世界坐标系点转为QGriaphicsScence的图层坐标系
 * @param {Vector2f&} point 传入的点坐标
 * @return {*}
 */
Eigen::Vector2f DisplayManager::transWord2Scene(const Eigen::Vector2f& point) {
  int x, y;
  map_data_.xy2scene(point[0], point[1], x, y);
  QPointF pose = DisplayInstance()->GetDisplay(DISPLAY_MAP)->mapToScene(x, y);
  Eigen::Vector2f res;
  res[0] = pose.x();
  res[1] = pose.y();
  return res;
}
VirtualDisplay* DisplayManager::GetDisplay(const std::string& name) {
  return DisplayInstance()->GetDisplay(name);
}
