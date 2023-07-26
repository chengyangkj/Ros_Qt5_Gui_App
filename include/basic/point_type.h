/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-07-26 10:07:32
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-07-26 16:30:38
 * @FilePath: /ROS2_Qt5_Gui_App/include/basic/point_type.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef POINT_TYPE_H
#define POINT_TYPE_H
#include "point.h"

namespace basic {
typedef OrientedPoint RobotPose;
struct RobotSpeed {
  double vx{0};
  double vy{0};
  double w{0};
};
struct RobotState : public RobotPose, RobotSpeed {};
// 激光点云数据结构
struct LaserScan {
  LaserScan() = default;
  LaserScan(int i, std::vector<Point> d) : id(i), data(d) {}
  int id;                   // 激光ID
  std::vector<Point> data;  // 点坐标
  void push_back(Point p) { data.push_back(p); }
  void clear() { data.clear(); }
};
// 机器人路径 由点集组成
struct RobotPath {
  RobotPath() = default;
  RobotPath(std::vector<Point>&& p) : data(std::move(p)) {}
  void push_back(Point p) { data.push_back(p); }
  void clear() { data.clear(); }

  std::vector<Point> data;
};

}  // namespace basic

#endif