#pragma once
#include <ros/ros.h>

#include "channel/base/virtual_comm_node.h"
class rosnode : public VirtualCommNode {
  Q_OBJECT
private:
  /* data */
public:
  rosnode(/* args */);
  ~rosnode() override;

  void run() override;
public slots:
  void pub2DPose(Eigen::Vector3f pose) override;
  void pub2DGoal(Eigen::Vector3f pose) override;

private:
  int init_argc;
  char **init_argv;
};