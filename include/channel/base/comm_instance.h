/*
 * @Author: chengyang chengyangkj@outlook.com
 * @Date: 2023-07-27 09:58:26
 * @LastEditors: chengyang chengyangkj@outlook.com
 * @LastEditTime: 2023-07-27 14:41:19
 * @FilePath: /ROS2_Qt5_Gui_App/include/channel/base/comm_instance.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

// 这两个宏在cmakelist中设置
#ifdef ROS2_NODE
#define COMM_NODE_NAME rclcomm
#include "channel/ros2/rclcomm.h"
#elif ROS1_NODE
#define COMM_NODE_NAME rosnode
#include "channel/ros1/rosnode.h"
#endif

class CommInstance {
 private:
  /* data */
 public:
  CommInstance(/* args */) {}
  ~CommInstance() {}
  static VirtualCommNode* Instance() {
    static auto* node = new COMM_NODE_NAME();
    return node;
  }
};
