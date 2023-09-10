/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-07-27 14:47:24
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-07-28 10:21:19
 * @FilePath: /ROS2_Qt5_Gui_App/src/channel/ros1/rosnode.cpp
 * @Description: ros1通讯类
 */
#include "channel/ros1/rosnode.h"
rosnode::rosnode(/* args */) {
  ros::init(init_argc, init_argv, "ros_qt5_gui_app",
            ros::init_options::AnonymousName);
}

rosnode::~rosnode() {}
void rosnode::run() {}
void rosnode::pub2DPose(QPointF start, QPointF end) {}
void rosnode::pub2DGoal(QPointF start, QPointF end) {}