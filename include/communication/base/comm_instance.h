/*
 * @Author: chengyang cyjiang@robovision.cn
 * @Date: 2023-07-27 09:58:26
 * @LastEditors: chengyang cyjiang@robovision.cn
 * @LastEditTime: 2023-07-27 11:07:13
 * @FilePath: /ROS2_Qt5_Gui_App/include/communication/base/comm_instance.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#ifndef ROS2
#define ROS2
#endif

#ifdef ROS2
#define COMM_NODE_NAME rclcomm
#include "communication/rclcomm.h"
#else
#define COMM_NODE_NAME "rosnode"
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
