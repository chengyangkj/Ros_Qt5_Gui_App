/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-27 14:47:24
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-07-28 10:21:19
 * @FilePath: /ros_qt5_gui_app/src/channel/ros1/rosnode.cpp
 * @Description: ros1通讯类
 */
#include "channel/ros1/rosnode.h"
rosnode::rosnode(/* args */) {
  ros::init(init_argc, init_argv, "ros_qt5_gui_app",
            ros::init_options::AnonymousName);
}

rosnode::~rosnode() {}
void rosnode::run() {}
void rosnode::PubRelocPose(const RobotPose &pose) {}
void rosnode::PubNavGoal(const RobotPose &pose) {}