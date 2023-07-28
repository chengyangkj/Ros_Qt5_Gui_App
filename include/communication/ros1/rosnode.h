#pragma once
#include <ros/ros.h>

#include "communication/base/virtual_comm_node.h"
class rosnode : public VirtualCommNode {
  Q_OBJECT
 private:
  /* data */
 public:
  rosnode(/* args */);
  ~rosnode();
  void pub2DPose(QPointF start, QPointF end) override;
  void pub2DGoal(QPointF start, QPointF end) override;
  void run() override;

 private:
  int init_argc;
  char **init_argv;
};